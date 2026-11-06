namespace CamGh.Core;

/// <summary>
/// Tri-dexel in-process stock. Three families of parallel rays (along X, Y and Z) each store the sorted
/// material intervals they pass through. Unlike a height field it represents undercuts, through-holes and
/// material left on any side, so several tool orientations can share one stock state.
/// Removal uses exact ray intersections with the swept cutter (stadium prisms and capsules), so a cut never
/// removes more than the cutter volume; the only approximation is the ray spacing.
/// </summary>
public sealed class TriDexelStock
{
    public StockBounds Bounds { get; }
    public double Resolution { get; }
    private readonly double[] _min, _max;
    private readonly int[][] _n;              // _n[f] = {nu, nv}
    private readonly double[][] _step;        // _step[f] = {hu, hv}
    private readonly double[]?[][] _rays;     // _rays[f][iu*nv+iv] = {lo0,hi0,lo1,hi1,...} or null when empty

    public TriDexelStock(StockBounds box, double resolution)
    {
        if (box.MaxX <= box.MinX || box.MaxY <= box.MinY || box.MaxZ <= box.MinZ) throw new ArgumentException("Stock bounds must have positive dimensions.");
        if (!(resolution > 0)) throw new ArgumentOutOfRangeException(nameof(resolution));
        Bounds = box;
        _min = new[] { box.MinX, box.MinY, box.MinZ }; _max = new[] { box.MaxX, box.MaxY, box.MaxZ };
        // Grow the spacing until the three families stay below ~3M rays in total.
        double h = resolution;
        while (RayCount(h) > 3_000_000) h *= 1.2;
        Resolution = h;
        _n = new int[3][]; _step = new double[3][]; _rays = new double[]?[3][];
        for (int f = 0; f < 3; f++)
        {
            int u = (f + 1) % 3, v = (f + 2) % 3;
            int nu = Math.Max(1, (int)Math.Ceiling((_max[u] - _min[u]) / h)), nv = Math.Max(1, (int)Math.Ceiling((_max[v] - _min[v]) / h));
            _n[f] = new[] { nu, nv };
            _step[f] = new[] { (_max[u] - _min[u]) / nu, (_max[v] - _min[v]) / nv };
            _rays[f] = new double[]?[nu * nv];
            for (int i = 0; i < nu * nv; i++) _rays[f][i] = new[] { _min[f], _max[f] };
        }
    }

    private TriDexelStock(TriDexelStock other)
    {
        Bounds = other.Bounds; Resolution = other.Resolution; _min = other._min; _max = other._max; _n = other._n; _step = other._step;
        _rays = new double[]?[3][];
        for (int f = 0; f < 3; f++) _rays[f] = other._rays[f].Select(r => r?.ToArray()).ToArray();
    }

    public TriDexelStock Clone() => new(this);

    private long RayCount(double h)
    {
        long nx = (long)Math.Ceiling((_max[0] - _min[0]) / h), ny = (long)Math.Ceiling((_max[1] - _min[1]) / h), nz = (long)Math.Ceiling((_max[2] - _min[2]) / h);
        return ny * nz + nz * nx + nx * ny;
    }

    private Vec3 RayBase(int f, int iu, int iv)
    {
        int u = (f + 1) % 3, v = (f + 2) % 3;
        var p = new double[3];
        p[u] = _min[u] + (iu + 0.5) * _step[f][0];
        p[v] = _min[v] + (iv + 0.5) * _step[f][1];
        return new Vec3(p[0], p[1], p[2]);
    }

    /// <summary>Remaining material volume, integrated over the Z ray family.</summary>
    public double Volume()
    {
        double sum = 0, cell = _step[2][0] * _step[2][1];
        foreach (var r in _rays[2]) if (r is not null) for (int k = 0; k < r.Length; k += 2) sum += r[k + 1] - r[k];
        return sum * cell;
    }

    /// <summary>Point classification using the nearest Z ray.</summary>
    public bool IsMaterial(Vec3 p)
    {
        int iu = (int)Math.Floor((p.X - _min[0]) / _step[2][0]), iv = (int)Math.Floor((p.Y - _min[1]) / _step[2][1]);
        if (iu < 0 || iv < 0 || iu >= _n[2][0] || iv >= _n[2][1]) return false;
        var r = _rays[2][iu * _n[2][1] + iv];
        if (r is null) return false;
        for (int k = 0; k < r.Length; k += 2) if (p.Z >= r[k] && p.Z <= r[k + 1]) return true;
        return false;
    }

    // ------------------------------------------------------------------ removal

    /// <summary>
    /// Removes the volume swept by a cutter whose tip moves from a to b with a fixed tool axis.
    /// The cutter is a flat or ball end mill of the given radius with a cylindrical body of the given length.
    /// </summary>
    public void SubtractSweep(Vec3 a, Vec3 b, Vec3 axis, ToolShape shape, double radius, double length)
    {
        var w = axis.Normalized(); var d = b - a; double axial = d.Dot(w); var lateral = d - w * axial;
        if (Math.Abs(axial) <= 1e-7 * Math.Max(1, d.Length))
        {
            SubtractPlanar(a, b - w * axial, w, shape, radius, length);
            return;
        }
        if (lateral.Length <= 1e-7 * Math.Max(1, d.Length))
        {
            // Pure axial plunge/retract: the static cutter at the lower tip, lengthened by the travel.
            var low = axial < 0 ? b : a;
            SubtractPlanar(low, low, w, shape, radius, length + Math.Abs(axial));
            return;
        }
        // Ramps and general moves: chain static cutters; spacing well under one ray pitch.
        int n = Math.Max(1, (int)Math.Ceiling(d.Length / (0.5 * Resolution)));
        for (int i = 0; i <= n; i++) { var p = a + d * ((double)i / n); SubtractPlanar(p, p, w, shape, radius, length); }
    }

    private static readonly object[] Stripes = Enumerable.Range(0, 8192).Select(_ => new object()).ToArray();
    private double _removedLength;   // accumulated by SubtractPlanar on ray family 0 while measuring

    /// <summary>Same as <see cref="SubtractSweep"/>, returning the volume (mm³) the move actually removed.</summary>
    public double SubtractSweepMeasured(Vec3 a, Vec3 b, Vec3 axis, ToolShape shape, double radius, double length)
    {
        _removedLength = 0;
        SubtractSweep(a, b, axis, shape, radius, length);
        return _removedLength * _step[0][0] * _step[0][1];
    }

    /// <summary>Subtracts many independent paths using all cores (order does not matter for the final stock).</summary>
    public void SubtractPaths(IReadOnlyList<IReadOnlyList<Point3>> paths, IReadOnlyList<Vec3> axes, ToolShape shape, double radius, double length)
    {
        Parallel.For(0, paths.Count, i => SubtractPath(paths[i], axes.Count == 1 ? axes[0] : axes[i], shape, radius, length));
    }

    public void SubtractPath(IReadOnlyList<Point3> path, Vec3 axis, ToolShape shape, double radius, double length)
    {
        if (path.Count == 1) { var p = Vec3.From(path[0]); SubtractSweep(p, p, axis, shape, radius, length); return; }
        for (int i = 0; i + 1 < path.Count; i++) SubtractSweep(Vec3.From(path[i]), Vec3.From(path[i + 1]), axis, shape, radius, length);
    }

    private void SubtractPlanar(Vec3 a, Vec3 b, Vec3 w, ToolShape shape, double r, double length)
    {
        // Only the part of the tool body that can reach the stock matters: clamp its length to the stock's extent along the axis.
        double maxProj = 0; for (int k = 0; k < 3; k++) maxProj += (w[k] >= 0 ? _max[k] : _min[k]) * w[k];
        double reach = maxProj - a.Dot(w) + 1e-6;
        if (reach <= 0) return;
        if (reach < length) length = reach;
        var tf = Frame.FromAxis(w);
        var la = tf.ToLocal(a); var lb = tf.ToLocal(b);
        var cutter = new SweptCutter(tf, la.X, la.Y, lb.X, lb.Y, la.Z, r, length, shape, a + w * r, b + w * r);
        // World bounding box of the swept volume.
        var top = w * length;
        Span<double> lo = stackalloc double[3], hi = stackalloc double[3];
        for (int k = 0; k < 3; k++)
        {
            double a0 = a[k], b0 = b[k], t = top[k];
            lo[k] = Math.Min(Math.Min(a0, b0), Math.Min(a0 + t, b0 + t)) - r; hi[k] = Math.Max(Math.Max(a0, b0), Math.Max(a0 + t, b0 + t)) + r;
        }
        for (int f = 0; f < 3; f++)
        {
            int u = (f + 1) % 3, v = (f + 2) % 3;
            int iu0 = Math.Max(0, (int)Math.Floor((lo[u] - _min[u]) / _step[f][0] - 0.5)), iu1 = Math.Min(_n[f][0] - 1, (int)Math.Ceiling((hi[u] - _min[u]) / _step[f][0] - 0.5));
            int iv0 = Math.Max(0, (int)Math.Floor((lo[v] - _min[v]) / _step[f][1] - 0.5)), iv1 = Math.Min(_n[f][1] - 1, (int)Math.Ceiling((hi[v] - _min[v]) / _step[f][1] - 0.5));
            if (iu0 > iu1 || iv0 > iv1) continue;
            var e = Vec3.Axis(f); int nv = _n[f][1];
            var rays = _rays[f];
            for (int iu = iu0; iu <= iu1; iu++)
                for (int iv = iv0; iv <= iv1; iv++)
                {
                    int idx = iu * nv + iv; var cur = rays[idx];
                    if (cur is null) continue;
                    // Cheap rejection: the cutter's extent along this ray must overlap some remaining material interval.
                    double lof = lo[f], hif = hi[f]; bool overlap = false;
                    for (int k = 0; k < cur.Length; k += 2) if (cur[k + 1] >= lof && cur[k] <= hif) { overlap = true; break; }
                    if (!overlap) continue;
                    var o = RayBase(f, iu, iv);
                    if (!cutter.Intersect(o, e, out double t0, out double t1)) continue;
                    if (t1 < cur[0] || t0 > cur[^1]) continue;
                    // Rays are replaced, never edited in place, so a short striped lock makes concurrent subtraction safe
                    // (removal commutes: the result is the union of everything swept, whatever the order).
                    lock (Stripes[(idx * 3 + f) & (Stripes.Length - 1)])
                    {
                        cur = rays[idx];
                        if (cur is null) continue;
                        var next = SubtractInterval(cur, t0, t1);
                        if (f == 0) _removedLength += Span(cur) - (next is null ? 0 : Span(next));
                        rays[idx] = next;
                    }
                }
        }
    }

    private static double Span(double[] r) { double s = 0; for (int k = 0; k < r.Length; k += 2) s += r[k + 1] - r[k]; return s; }

    private static double[]? SubtractInterval(double[] cur, double t0, double t1)
    {
        if (cur.Length == 2)
        {
            double l = cur[0], h = cur[1];
            if (t1 <= l || t0 >= h) return cur;
            bool left = t0 > l + 1e-9, right = t1 < h - 1e-9;
            if (left && right) return new[] { l, t0, t1, h };
            if (left) return new[] { l, t0 };
            if (right) return new[] { t1, h };
            return null;
        }
        var res = new List<double>(cur.Length + 2);
        for (int k = 0; k < cur.Length; k += 2)
        {
            double lo = cur[k], hi = cur[k + 1];
            if (t1 <= lo || t0 >= hi) { res.Add(lo); res.Add(hi); continue; }
            if (t0 > lo + 1e-9) { res.Add(lo); res.Add(t0); }
            if (t1 < hi - 1e-9) { res.Add(t1); res.Add(hi); }
        }
        return res.Count == 0 ? null : res.ToArray();
    }

    /// <summary>Exact line intersection with a flat or ball cutter swept along a segment perpendicular to its axis.</summary>
    private readonly struct SweptCutter
    {
        private readonly Frame _f; private readonly double _ax, _ay, _bx, _by, _w0, _r, _len; private readonly ToolShape _shape; private readonly Vec3 _c0, _c1;
        public SweptCutter(Frame f, double ax, double ay, double bx, double by, double w0, double r, double len, ToolShape shape, Vec3 c0, Vec3 c1)
        { _f = f; _ax = ax; _ay = ay; _bx = bx; _by = by; _w0 = w0; _r = r; _len = Math.Max(len, shape == ToolShape.Ball ? r : 1e-9); _shape = shape; _c0 = c0; _c1 = c1; }

        public bool Intersect(Vec3 o, Vec3 e, out double t0, out double t1)
        {
            var lo = _f.ToLocal(o); var le = _f.ToLocal(e);
            double prismBottom = _shape == ToolShape.Ball ? _w0 + _r : _w0;
            bool hit = StadiumPrism(lo, le, prismBottom, _w0 + _len, out t0, out t1);
            if (_shape == ToolShape.Ball && Capsule(o, e, _c0, _c1, _r, out double c0, out double c1))
            {
                if (hit) { t0 = Math.Min(t0, c0); t1 = Math.Max(t1, c1); } else { t0 = c0; t1 = c1; hit = true; }
            }
            return hit;
        }

        private bool StadiumPrism(Vec3 o, Vec3 e, double wLo, double wHi, out double t0, out double t1)
        {
            t0 = double.NegativeInfinity; t1 = double.PositiveInfinity;
            if (Math.Abs(e.Z) < 1e-12) { if (o.Z < wLo || o.Z > wHi) return false; }
            else { double s0 = (wLo - o.Z) / e.Z, s1 = (wHi - o.Z) / e.Z; t0 = Math.Min(s0, s1); t1 = Math.Max(s0, s1); }
            if (!Stadium2D(o.X, o.Y, e.X, e.Y, out double q0, out double q1)) return false;
            t0 = Math.Max(t0, q0); t1 = Math.Min(t1, q1);
            return t1 > t0;
        }

        private bool Stadium2D(double px, double py, double fx, double fy, out double t0, out double t1)
        {
            t0 = double.PositiveInfinity; t1 = double.NegativeInfinity;
            double r2 = _r * _r, ff = fx * fx + fy * fy;
            if (ff < 1e-24)
            {
                // Line parallel to the tool axis: inside test on the 2D stadium.
                if (DistToSegment2(px, py) > r2) return false;
                t0 = double.NegativeInfinity; t1 = double.PositiveInfinity; return true;
            }
            bool any = false;
            void Disc(double cx, double cy, ref double lo, ref double hi)
            {
                double dx = px - cx, dy = py - cy, b = dx * fx + dy * fy, c = dx * dx + dy * dy - r2, disc = b * b - ff * c;
                if (disc < 0) return;
                double s = Math.Sqrt(disc); lo = Math.Min(lo, (-b - s) / ff); hi = Math.Max(hi, (-b + s) / ff); any = true;
            }
            Disc(_ax, _ay, ref t0, ref t1); Disc(_bx, _by, ref t0, ref t1);
            double sx = _bx - _ax, sy = _by - _ay, L = Math.Sqrt(sx * sx + sy * sy);
            if (L > 1e-12)
            {
                sx /= L; sy /= L; double nx = -sy, ny = sx;
                // Slab 0 <= (q-A)·s <= L and |(q-A)·n| <= r, each linear in t.
                double lo = double.NegativeInfinity, hi = double.PositiveInfinity;
                bool ok = Slab((px - _ax) * sx + (py - _ay) * sy, fx * sx + fy * sy, 0, L, ref lo, ref hi)
                       && Slab((px - _ax) * nx + (py - _ay) * ny, fx * nx + fy * ny, -_r, _r, ref lo, ref hi);
                if (ok && hi > lo) { t0 = Math.Min(t0, lo); t1 = Math.Max(t1, hi); any = true; }
            }
            return any && t1 > t0;
        }

        private static bool Slab(double p, double d, double lo, double hi, ref double t0, ref double t1)
        {
            if (Math.Abs(d) < 1e-15) return p >= lo && p <= hi;
            double a = (lo - p) / d, b = (hi - p) / d;
            t0 = Math.Max(t0, Math.Min(a, b)); t1 = Math.Min(t1, Math.Max(a, b)); return t1 > t0;
        }

        private double DistToSegment2(double px, double py)
        {
            double sx = _bx - _ax, sy = _by - _ay, den = sx * sx + sy * sy;
            double t = den < 1e-24 ? 0 : Math.Clamp(((px - _ax) * sx + (py - _ay) * sy) / den, 0, 1);
            double dx = px - (_ax + sx * t), dy = py - (_ay + sy * t); return dx * dx + dy * dy;
        }

        private static bool Capsule(Vec3 o, Vec3 e, Vec3 c0, Vec3 c1, double r, out double t0, out double t1)
        {
            t0 = double.PositiveInfinity; t1 = double.NegativeInfinity; bool any = false;
            double ee = e.Dot(e);
            void Sphere(Vec3 c, ref double lo, ref double hi)
            {
                var d = o - c; double b = d.Dot(e), cc = d.Dot(d) - r * r, disc = b * b - ee * cc;
                if (disc < 0) return;
                double s = Math.Sqrt(disc); lo = Math.Min(lo, (-b - s) / ee); hi = Math.Max(hi, (-b + s) / ee); any = true;
            }
            Sphere(c0, ref t0, ref t1); Sphere(c1, ref t0, ref t1);
            var axis = c1 - c0; double L = axis.Length;
            if (L > 1e-12)
            {
                var s = axis * (1 / L); var d = o - c0;
                var dp = d - s * d.Dot(s); var ep = e - s * e.Dot(s);
                double a = ep.Dot(ep), b = dp.Dot(ep), c = dp.Dot(dp) - r * r;
                double lo = double.NegativeInfinity, hi = double.PositiveInfinity; bool ok = true;
                if (a < 1e-24) ok = c <= 0;
                else
                {
                    double disc = b * b - a * c;
                    if (disc < 0) ok = false; else { double q = Math.Sqrt(disc); lo = (-b - q) / a; hi = (-b + q) / a; }
                }
                if (ok) ok = Slab(d.Dot(s), e.Dot(s), 0, L, ref lo, ref hi);
                if (ok && hi > lo) { t0 = Math.Min(t0, lo); t1 = Math.Max(t1, hi); any = true; }
            }
            return any && t1 > t0;
        }
    }

    // ------------------------------------------------------------------ directional queries

    /// <summary>
    /// Highest remaining material along -W for each cell of a local grid (cell (i,j) centred at x0+i*h, y0+j*h).
    /// Cells without material are -∞. The estimate is conservative (never lower than the true top).
    /// </summary>
    public double[,] TopMap(Frame frame, double x0, double y0, double h, int nx, int ny)
    {
        var map = new double[nx, ny];
        for (int i = 0; i < nx; i++) for (int j = 0; j < ny; j++) map[i, j] = double.NegativeInfinity;
        int f = BestFamily(frame.W);
        var e = Vec3.Axis(f); var le = frame.ToLocal(e);
        double along = Math.Sqrt(Math.Max(0, 1 - le.Z * le.Z));
        double rayPitch = Math.Max(_step[f][0], _step[f][1]);
        double splat = 0.75 * rayPitch + 0.5 * h;
        double bump = 0.71 * rayPitch * along;           // material prism width seen across a slanted ray
        int reach = (int)Math.Ceiling(splat / h);
        int nu = _n[f][0], nv = _n[f][1];
        var locks = new object[Math.Max(1, nx)];
        for (int i = 0; i < locks.Length; i++) locks[i] = new object();
        Parallel.For(0, nu, iu =>
        {
            for (int iv = 0; iv < nv; iv++)
            {
                var r = _rays[f][iu * nv + iv]; if (r is null) continue;
                var o = RayBase(f, iu, iv);
                for (int k = 0; k < r.Length; k += 2)
                {
                    double len = r[k + 1] - r[k];
                    int samples = along < 1e-9 ? 1 : Math.Max(1, (int)Math.Ceiling(len * along / (0.5 * h)));
                    for (int s = 0; s <= samples; s++)
                    {
                        double t = along < 1e-9 ? (le.Z >= 0 ? r[k + 1] : r[k]) : r[k] + len * s / samples;
                        var l = frame.ToLocal(o + e * t); double wv = l.Z + bump;
                        int ci = (int)Math.Round((l.X - x0) / h), cj = (int)Math.Round((l.Y - y0) / h);
                        for (int di = -reach; di <= reach; di++)
                        {
                            int ii = ci + di; if (ii < 0 || ii >= nx) continue;
                            lock (locks[ii])
                                for (int dj = -reach; dj <= reach; dj++)
                                {
                                    int jj = cj + dj; if (jj < 0 || jj >= ny) continue;
                                    double dx = x0 + ii * h - l.X, dy = y0 + jj * h - l.Y;
                                    if (dx * dx + dy * dy > splat * splat) continue;
                                    if (wv > map[ii, jj]) map[ii, jj] = wv;
                                }
                        }
                    }
                }
            }
        });
        return map;
    }

    private static int BestFamily(Vec3 w)
    {
        double ax = Math.Abs(w.X), ay = Math.Abs(w.Y), az = Math.Abs(w.Z);
        return ax >= ay && ax >= az ? 0 : ay >= az ? 1 : 2;
    }

    /// <summary>Largest local W of any remaining material (interval end points are exact for straight rays).</summary>
    public double MaxLocal(Frame frame)
    {
        double best = double.NegativeInfinity;
        for (int f = 0; f < 3; f++)
        {
            var e = Vec3.Axis(f); int nv = _n[f][1];
            for (int idx = 0; idx < _rays[f].Length; idx++)
            {
                var r = _rays[f][idx]; if (r is null) continue;
                var o = RayBase(f, idx / nv, idx % nv);
                for (int k = 0; k < r.Length; k++) best = Math.Max(best, frame.ToLocal(o + e * r[k]).Z);
            }
        }
        return best;
    }

    /// <summary>
    /// Approximate volume above a removal limit in the given frame: material whose local W exceeds
    /// limit(localX, localY) is counted. Used to rank candidate tool directions.
    /// </summary>
    public double VolumeAbove(Frame frame, Func<double, double, double, bool> removable, double sampleStep, int rayStride)
    {
        int f = 2, nu = _n[f][0], nv = _n[f][1];
        double cell = _step[f][0] * _step[f][1] * rayStride * rayStride;
        double total = 0; object gate = new();
        Parallel.For(0, (nu + rayStride - 1) / rayStride, ku =>
        {
            int iu = ku * rayStride; double local = 0;
            for (int iv = 0; iv < nv; iv += rayStride)
            {
                var r = _rays[f][iu * nv + iv]; if (r is null) continue;
                var o = RayBase(f, iu, iv);
                for (int k = 0; k < r.Length; k += 2)
                {
                    double len = r[k + 1] - r[k]; int n = Math.Max(1, (int)Math.Ceiling(len / sampleStep)); double ds = len / n;
                    for (int s = 0; s < n; s++)
                    {
                        var p = new Vec3(o.X, o.Y, r[k] + (s + 0.5) * ds);
                        var l = frame.ToLocal(p);
                        if (removable(l.X, l.Y, l.Z) && p.Z >= _min[2] - 1) local += ds;
                    }
                }
            }
            lock (gate) total += local;
        });
        return total * cell;
    }

    /// <summary>Simulation helper: removes every sampled point the predicate marks removable (all three families).</summary>
    public void RemoveWhere(Frame frame, Func<double, double, double, bool> removable, double sampleStep)
    {
        for (int f = 0; f < 3; f++)
        {
            int ff = f, nv = _n[f][1]; var e = Vec3.Axis(f);
            Parallel.For(0, _rays[ff].Length, idx =>
            {
                var r = _rays[ff][idx]; if (r is null) return;
                var o = RayBase(ff, idx / nv, idx % nv);
                var res = new List<double>(); bool changed = false;
                for (int k = 0; k < r.Length; k += 2)
                {
                    double len = r[k + 1] - r[k]; int n = Math.Max(1, (int)Math.Ceiling(len / sampleStep)); double ds = len / n;
                    double runStart = double.NaN;
                    for (int s = 0; s < n; s++)
                    {
                        double t = r[k] + (s + 0.5) * ds; var l = frame.ToLocal(o + e * t);
                        bool keep = !removable(l.X, l.Y, l.Z);
                        if (!keep) changed = true;
                        if (keep && double.IsNaN(runStart)) runStart = r[k] + s * ds;
                        if (!keep && !double.IsNaN(runStart)) { res.Add(runStart); res.Add(r[k] + s * ds); runStart = double.NaN; }
                    }
                    if (!double.IsNaN(runStart)) { res.Add(runStart); res.Add(r[k + 1]); }
                }
                if (changed) _rays[ff][idx] = res.Count == 0 ? null : res.ToArray();
            });
        }
    }

    /// <summary>Remaining material as line segments along Z (every stride-th ray) for previews.</summary>
    public List<List<Point3>> PreviewSegments(int maxSegments)
    {
        int f = 2, nu = _n[f][0], nv = _n[f][1];
        int count = _rays[f].Sum(r => r is null ? 0 : r.Length / 2);
        int stride = 1;
        while (count / (stride * stride) > maxSegments && stride < 64) stride++;
        var result = new List<List<Point3>>();
        for (int iu = 0; iu < nu; iu += stride)
            for (int iv = 0; iv < nv; iv += stride)
            {
                var r = _rays[f][iu * nv + iv]; if (r is null) continue;
                var o = RayBase(f, iu, iv);
                for (int k = 0; k < r.Length; k += 2) result.Add(new List<Point3> { new(o.X, o.Y, r[k]), new(o.X, o.Y, r[k + 1]) });
            }
        return result;
    }

    // ------------------------------------------------------------------ surface mesh (for previews and simulation)

    /// <summary>
    /// Closed surface mesh of the remaining material (surface nets on the ray grid). Grid points sit on the ray
    /// crossings, so every grid edge lies on a ray of its own direction and the surface crossing along it is the exact
    /// interval end point stored in that ray; vertices are the average of the crossings around each cell.
    /// stride ≥ 1 skips rays to keep the grid below maxCells.
    /// </summary>
    public (float[] Positions, int[] Indices) ToMesh(long maxCells = 4_000_000)
    {
        int nx = _n[2][0], ny = _n[2][1], nz = _n[0][1];
        double hx = _step[2][0], hy = _step[2][1], hz = _step[0][1];
        int s = 1; while ((long)Math.Ceiling(nx / (double)s) * (long)Math.Ceiling(ny / (double)s) * (long)Math.Ceiling(nz / (double)s) > maxCells) s++;
        int NX = (nx + s - 1) / s, NY = (ny + s - 1) / s, NZ = (nz + s - 1) / s;
        // Point coordinates on padded index ranges -1..N (padding lies just outside the stock).
        double X(int I) => I < 0 ? _min[0] - 0.5 * hx : I >= NX ? _max[0] + 0.5 * hx : _min[0] + (I * s + 0.5) * hx;
        double Y(int J) => J < 0 ? _min[1] - 0.5 * hy : J >= NY ? _max[1] + 0.5 * hy : _min[1] + (J * s + 0.5) * hy;
        double Z(int K) => K < 0 ? _min[2] - 0.5 * hz : K >= NZ ? _max[2] + 0.5 * hz : _min[2] + (K * s + 0.5) * hz;
        int PX = NX + 2, PY = NY + 2, PZ = NZ + 2;
        var occ = new bool[(long)PX * PY * PZ];
        long P(int I, int J, int K) => ((long)(I + 1) * PY + (J + 1)) * PZ + (K + 1);
        Parallel.For(0, NX, I =>
        {
            for (int J = 0; J < NY; J++)
            {
                var r = _rays[2][(I * s) * _n[2][1] + J * s]; if (r is null) continue;
                for (int K = 0; K < NZ; K++)
                {
                    double z = Z(K);
                    for (int q = 0; q < r.Length; q += 2) if (z >= r[q] && z <= r[q + 1]) { occ[P(I, J, K)] = true; break; }
                }
            }
        });
        bool O(int I, int J, int K) => occ[P(I, J, K)];
        // Exact crossing between two grid points along an axis, from the ray of that axis through them.
        double Cross(double[]? ray, double a, double b)
        {
            double mid = 0.5 * (a + b), best = mid, bestD = double.MaxValue;
            if (ray is not null)
                foreach (var t in ray) if (t >= Math.Min(a, b) - 1e-9 && t <= Math.Max(a, b) + 1e-9 && Math.Abs(t - mid) < bestD) { bestD = Math.Abs(t - mid); best = t; }
            return best;
        }
        double[]? RayX(int J, int K) => J < 0 || K < 0 || J >= NY || K >= NZ ? null : _rays[0][(J * s) * _n[0][1] + K * s];
        double[]? RayY(int K, int I) => K < 0 || I < 0 || K >= NZ || I >= NX ? null : _rays[1][(K * s) * _n[1][1] + I * s];
        double[]? RayZ(int I, int J) => I < 0 || J < 0 || I >= NX || J >= NY ? null : _rays[2][(I * s) * _n[2][1] + J * s];

        // Cell (I,J,K) spans points I..I+1 etc., I in -1..N-1; stored at (I+1, J+1, K+1).
        int CX = NX + 1, CY = NY + 1, CZ = NZ + 1;
        var cellIndex = new int[(long)CX * CY * CZ];
        long C(int I, int J, int K) => ((long)(I + 1) * CY + (J + 1)) * CZ + (K + 1);
        var verts = new List<float>();
        int count = 0;
        for (int I = -1; I < NX; I++)
            for (int J = -1; J < NY; J++)
                for (int K = -1; K < NZ; K++)
                {
                    bool c0 = O(I, J, K), any = false, all = true;
                    for (int d = 0; d < 8; d++) { bool v = O(I + (d & 1), J + ((d >> 1) & 1), K + ((d >> 2) & 1)); any |= v; all &= v; }
                    if (!any || all) { cellIndex[C(I, J, K)] = -1; continue; }
                    double sx = 0, sy = 0, sz = 0; int n = 0;
                    for (int dj = 0; dj < 2; dj++) for (int dk = 0; dk < 2; dk++)
                        if (O(I, J + dj, K + dk) != O(I + 1, J + dj, K + dk)) { sx += Cross(RayX(J + dj, K + dk), X(I), X(I + 1)); sy += Y(J + dj); sz += Z(K + dk); n++; }
                    for (int di = 0; di < 2; di++) for (int dk = 0; dk < 2; dk++)
                        if (O(I + di, J, K + dk) != O(I + di, J + 1, K + dk)) { sx += X(I + di); sy += Cross(RayY(K + dk, I + di), Y(J), Y(J + 1)); sz += Z(K + dk); n++; }
                    for (int di = 0; di < 2; di++) for (int dj = 0; dj < 2; dj++)
                        if (O(I + di, J + dj, K) != O(I + di, J + dj, K + 1)) { sx += X(I + di); sy += Y(J + dj); sz += Cross(RayZ(I + di, J + dj), Z(K), Z(K + 1)); n++; }
                    cellIndex[C(I, J, K)] = count++;
                    verts.Add((float)(sx / n)); verts.Add((float)(sy / n)); verts.Add((float)(sz / n));
                    _ = c0;
                }
        var idx = new List<int>();
        void Quad(int a, int b, int c, int d, bool flip)
        {
            if (a < 0 || b < 0 || c < 0 || d < 0) return;
            if (!flip) { idx.Add(a); idx.Add(b); idx.Add(c); idx.Add(a); idx.Add(c); idx.Add(d); }
            else { idx.Add(a); idx.Add(c); idx.Add(b); idx.Add(a); idx.Add(d); idx.Add(c); }
        }
        int Cell(int I, int J, int K) => cellIndex[C(I, J, K)];
        for (int I = -1; I < NX; I++)
            for (int J = 0; J < NY; J++)
                for (int K = 0; K < NZ; K++)
                {
                    bool a = O(I, J, K), b = O(I + 1, J, K);
                    if (a != b) Quad(Cell(I, J - 1, K - 1), Cell(I, J, K - 1), Cell(I, J, K), Cell(I, J - 1, K), !a);
                }
        for (int I = 0; I < NX; I++)
            for (int J = -1; J < NY; J++)
                for (int K = 0; K < NZ; K++)
                {
                    bool a = O(I, J, K), b = O(I, J + 1, K);
                    if (a != b) Quad(Cell(I - 1, J, K - 1), Cell(I - 1, J, K), Cell(I, J, K), Cell(I, J, K - 1), !a);
                }
        for (int I = 0; I < NX; I++)
            for (int J = 0; J < NY; J++)
                for (int K = -1; K < NZ; K++)
                {
                    bool a = O(I, J, K), b = O(I, J, K + 1);
                    if (a != b) Quad(Cell(I - 1, J - 1, K), Cell(I, J - 1, K), Cell(I, J, K), Cell(I - 1, J, K), !a);
                }
        return (verts.ToArray(), idx.ToArray());
    }
}
