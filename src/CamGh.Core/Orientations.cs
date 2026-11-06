namespace CamGh.Core;

/// <summary>Candidate tool directions and visibility-driven selection for 3+2 (indexed) machining.</summary>
public static class OrientationPlanner
{
    /// <summary>
    /// Evenly spread candidate tool axes (Fibonacci sphere) plus the five box faces, limited to axes with
    /// Z ≥ minAxisZ. Axes point from the cutter tip towards the spindle, so +Z machines from above and
    /// horizontal axes machine the sides. Directions from below are excluded by default (table/fixture).
    /// </summary>
    public static List<Vec3> Candidates(int count = 64, double minAxisZ = -1e-9)
    {
        var dirs = new List<Vec3> { new(0, 0, 1), new(1, 0, 0), new(-1, 0, 0), new(0, 1, 0), new(0, -1, 0) };
        double golden = Math.PI * (3 - Math.Sqrt(5));
        for (int i = 0; i < count; i++)
        {
            double z = 1 - 2.0 * (i + 0.5) / count, rad = Math.Sqrt(Math.Max(0, 1 - z * z)), th = golden * i;
            var d = new Vec3(rad * Math.Cos(th), rad * Math.Sin(th), z);
            if (d.Z < minAxisZ) continue;
            if (dirs.Any(e => e.Dot(d) > Math.Cos(Math.PI / 180 * 8))) continue;
            dirs.Add(d);
        }
        return dirs;
    }

    /// <summary>Highest part surface (+offset) per local cell; -∞ where the part does not project.</summary>
    public static double[,] UpperEnvelope(TriMesh localMesh, double x0, double y0, double h, int nx, int ny, double offset)
    {
        var map = new double[nx, ny];
        for (int i = 0; i < nx; i++) for (int j = 0; j < ny; j++) map[i, j] = double.NegativeInfinity;
        for (int f = 0; f < localMesh.Count; f++)
        {
            var a = localMesh.A(f); var b = localMesh.B(f); var c = localMesh.C(f);
            double den = (b.Y - c.Y) * (a.X - c.X) + (c.X - b.X) * (a.Y - c.Y);
            int i0 = Math.Max(0, (int)Math.Floor((Math.Min(a.X, Math.Min(b.X, c.X)) - x0) / h)), i1 = Math.Min(nx - 1, (int)Math.Ceiling((Math.Max(a.X, Math.Max(b.X, c.X)) - x0) / h));
            int j0 = Math.Max(0, (int)Math.Floor((Math.Min(a.Y, Math.Min(b.Y, c.Y)) - y0) / h)), j1 = Math.Min(ny - 1, (int)Math.Ceiling((Math.Max(a.Y, Math.Max(b.Y, c.Y)) - y0) / h));
            if (Math.Abs(den) < 1e-14)
            {
                // Edge-on triangle: still contributes its highest point to the cells it crosses (walls).
                double zt = Math.Max(a.Z, Math.Max(b.Z, c.Z)) + offset;
                for (int i = i0; i <= i1; i++) for (int j = j0; j <= j1; j++)
                    if (SegmentNearCell(a, b, c, x0 + i * h, y0 + j * h, h)) map[i, j] = Math.Max(map[i, j], zt);
                continue;
            }
            for (int i = i0; i <= i1; i++)
                for (int j = j0; j <= j1; j++)
                {
                    double x = x0 + i * h, y = y0 + j * h;
                    double u = ((b.Y - c.Y) * (x - c.X) + (c.X - b.X) * (y - c.Y)) / den;
                    double v = ((c.Y - a.Y) * (x - c.X) + (a.X - c.X) * (y - c.Y)) / den, w = 1 - u - v;
                    if (u < -1e-9 || v < -1e-9 || w < -1e-9) continue;
                    double z = u * a.Z + v * b.Z + w * c.Z + offset;
                    if (z > map[i, j]) map[i, j] = z;
                }
        }
        return map;
    }

    private static bool SegmentNearCell(Vec3 a, Vec3 b, Vec3 c, double x, double y, double h)
    {
        double best = double.MaxValue;
        foreach (var (p, q) in new[] { (a, b), (b, c), (c, a) })
        {
            double dx = q.X - p.X, dy = q.Y - p.Y, den = dx * dx + dy * dy;
            double t = den < 1e-24 ? 0 : Math.Clamp(((x - p.X) * dx + (y - p.Y) * dy) / den, 0, 1);
            double ex = x - (p.X + t * dx), ey = y - (p.Y + t * dy); best = Math.Min(best, ex * ex + ey * ey);
        }
        return best <= 0.5 * h * h;
    }

    /// <summary>Lowest point of a cutter (tip disc + body of the given length) for a tip at world height tipZ.</summary>
    public static double LowestToolZ(double tipZ, Vec3 axis, double radius, double length)
    {
        var w = axis.Normalized();
        double horiz = Math.Sqrt(Math.Max(0, 1 - w.Z * w.Z));
        return tipZ + Math.Min(0, length * w.Z) - radius * horiz;
    }

    /// <summary>
    /// Visibility scoring for roughing directions. Part envelopes are precomputed once per candidate; each call to
    /// <see cref="RoughingCandidates.Best"/> scores the candidates against the CURRENT stock: the remaining
    /// material that lies above the part (plus allowance) along that axis, within tool reach and above the
    /// fixture. Planning one orientation, cutting it in the stock and scoring again captures the terraces and
    /// shadows the real paths leave, instead of an idealised removal.
    /// </summary>
    public sealed class RoughingCandidates
    {
        private readonly List<(Vec3 Axis, Frame Frame, StockBounds Bounds, int Nx, int Ny, double[,] Env)> _c;
        private readonly double _hc, _radius; private readonly int _stride; private readonly bool[] _used;

        public RoughingCandidates(TriMesh part, TriDexelStock stock, double radius, double allowance, int candidateCount = 64)
        {
            var box = stock.Bounds;
            double extent = Math.Max(box.MaxX - box.MinX, Math.Max(box.MaxY - box.MinY, box.MaxZ - box.MinZ));
            _hc = Math.Max(Math.Max(stock.Resolution * 1.5, radius * 0.5), extent / 120); _radius = radius;
            _stride = Math.Max(1, (int)Math.Round(_hc / stock.Resolution / 1.5));
            _c = Candidates(candidateCount).AsParallel().AsOrdered().Select(d =>
            {
                var frame = Frame.FromAxis(d);
                var lb = frame.LocalBounds(box, _hc);
                int nx = (int)Math.Ceiling((lb.MaxX - lb.MinX) / _hc) + 1, ny = (int)Math.Ceiling((lb.MaxY - lb.MinY) / _hc) + 1;
                return (d, frame, lb, nx, ny, UpperEnvelope(part.Transformed(frame), lb.MinX, lb.MinY, _hc, nx, ny, allowance));
            }).ToList();
            _used = new bool[_c.Count];
        }

        /// <summary>Best unused axis for the current stock and its estimated removable volume (null when none).</summary>
        public (Vec3 Axis, double Gain)? Best(TriDexelStock stock, double reach, double fixtureZ)
        {
            int best = -1; double bestGain = 0;
            for (int k = 0; k < _c.Count; k++)
            {
                if (_used[k]) continue;
                var c = _c[k];
                double gain = stock.VolumeAbove(c.Frame, Removable(c.Frame, c.Env, c.Bounds, _hc, c.Nx, c.Ny, stock.MaxLocal(c.Frame) - reach, fixtureZ, _radius), _hc, _stride);
                if (gain > bestGain) { bestGain = gain; best = k; }
            }
            if (best < 0) return null;
            _used[best] = true;
            return (_c[best].Axis, bestGain);
        }
    }

    private static Func<double, double, double, bool> Removable(Frame frame, double[,] env, StockBounds lb, double hc, int nx, int ny, double floorW, double fixtureZ, double radius)
    {
        return (x, y, w) =>
        {
            if (w < floorW) return false;
            int i = (int)Math.Round((x - lb.MinX) / hc), j = (int)Math.Round((y - lb.MinY) / hc);
            // Material at the map border is treated as covered by the neighbouring part height (conservative).
            double limit = i >= 0 && j >= 0 && i < nx && j < ny ? env[i, j] : double.NegativeInfinity;
            if (w <= limit) return false;
            // Material below the fixture plane is never machinable (clamping base / table).
            return frame.ToWorld(x, y, w).Z >= fixtureZ;
        };
    }
}
