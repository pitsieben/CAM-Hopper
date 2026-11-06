namespace CamGh.Core;

/// <summary>
/// Classic drop-cutter in a local tool frame (tool axis +Z). For a cutter located above (x, y) it returns the
/// lowest tip height at which the cutter touches but does not penetrate any triangle: the maximum over vertex,
/// edge and facet contact heights. Because every triangle within reach is tested, the result is gouge-free with
/// respect to the whole mesh, not just the face being machined.
/// </summary>
public sealed class DropCutter
{
    private readonly TriangleBins _bins;
    private readonly TriMesh _m;
    private readonly ToolShape _shape;
    private readonly double _r;

    public DropCutter(TriangleBins bins, ToolShape shape, double radius)
    {
        if (!(radius > 0)) throw new ArgumentOutOfRangeException(nameof(radius));
        _bins = bins; _m = bins.Mesh; _shape = shape; _r = radius;
    }

    public double Radius => _r;

    /// <summary>Returns the tip Z, or -∞ when nothing lies under the cutter. contact = face index that set the height.</summary>
    public double Drop(double x, double y, int[] stamp, ref int stampValue, out int contact)
    {
        double best = double.NegativeInfinity; int bestFace = -1;
        double r = _r;
        _bins.Query(x - r, x + r, y - r, y + r, stamp, ref stampValue, f =>
        {
            double z = _shape == ToolShape.Ball ? BallTriangle(f, x, y) : FlatTriangle(f, x, y);
            if (z > best) { best = z; bestFace = f; }
        });
        contact = bestFace;
        return best;
    }

    // ---------------------------------------------------------------- ball end (sphere centre = tip + r)

    private double BallTriangle(int f, double x, double y)
    {
        var a = _m.A(f); var b = _m.B(f); var c = _m.C(f); double r = _r, best = double.NegativeInfinity;
        // Facet: centre sits on the plane offset by r; contact point = centre - r*n must be inside the triangle.
        var n = _m.Normals[f];
        if (n.Z > 1e-9)
        {
            double cz = a.Z + (r - n.X * (x - a.X) - n.Y * (y - a.Y) - n.Z * 0) / n.Z;
            // n·(centre - a) = r  →  n.X(x-a.X) + n.Y(y-a.Y) + n.Z(cz-a.Z) = r
            var cc = new Vec3(x - r * n.X, y - r * n.Y, cz - r * n.Z);
            if (InsideXY(a, b, c, cc.X, cc.Y)) best = Math.Max(best, cz - r);
        }
        best = Math.Max(best, BallVertex(a, x, y)); best = Math.Max(best, BallVertex(b, x, y)); best = Math.Max(best, BallVertex(c, x, y));
        best = Math.Max(best, BallEdge(a, b, x, y)); best = Math.Max(best, BallEdge(b, c, x, y)); best = Math.Max(best, BallEdge(c, a, x, y));
        return best;
    }

    private double BallVertex(Vec3 p, double x, double y)
    {
        double d2 = (p.X - x) * (p.X - x) + (p.Y - y) * (p.Y - y), r2 = _r * _r;
        return d2 > r2 ? double.NegativeInfinity : p.Z + Math.Sqrt(r2 - d2) - _r;
    }

    private double BallEdge(Vec3 a, Vec3 b, double x, double y)
    {
        double ex = b.X - a.X, ey = b.Y - a.Y, L = Math.Sqrt(ex * ex + ey * ey);
        if (L < 1e-12) return double.NegativeInfinity;               // vertical edge: vertex tests cover it
        double ux = ex / L, uy = ey / L;
        double s0 = (x - a.X) * ux + (y - a.Y) * uy;                 // centre position along the edge (xy)
        double dperp = Math.Abs(-(x - a.X) * uy + (y - a.Y) * ux);
        if (dperp >= _r) return double.NegativeInfinity;
        double rr = Math.Sqrt(_r * _r - dperp * dperp);              // circle radius in the vertical plane of the edge
        double m = (b.Z - a.Z) / L, k = Math.Sqrt(1 + m * m);
        double sc = s0 + rr * m / k;                                  // contact location along the edge
        if (sc < 0 || sc > L) return double.NegativeInfinity;        // end points are handled by vertex tests
        double centreZ = a.Z + m * s0 + rr * k;
        return centreZ - _r;
    }

    // ---------------------------------------------------------------- flat end (tip disc of radius r)

    private double FlatTriangle(int f, double x, double y)
    {
        var a = _m.A(f); var b = _m.B(f); var c = _m.C(f); double best = double.NegativeInfinity, r = _r;
        var n = _m.Normals[f];
        if (n.Z > 1e-9)
        {
            double gx = -n.X / n.Z, gy = -n.Y / n.Z, g = Math.Sqrt(gx * gx + gy * gy);
            // Highest plane point under the disc is on the rim in the uphill direction (or the centre if level).
            double px = x, py = y;
            if (g > 1e-12) { px = x + r * gx / g; py = y + r * gy / g; }
            if (InsideXY(a, b, c, px, py)) best = a.Z + gx * (px - a.X) + gy * (py - a.Y);
        }
        foreach (var p in new[] { a, b, c })
            if ((p.X - x) * (p.X - x) + (p.Y - y) * (p.Y - y) <= r * r) best = Math.Max(best, p.Z);
        best = Math.Max(best, FlatEdge(a, b, x, y)); best = Math.Max(best, FlatEdge(b, c, x, y)); best = Math.Max(best, FlatEdge(c, a, x, y));
        return best;
    }

    private double FlatEdge(Vec3 a, Vec3 b, double x, double y)
    {
        // Edge points within the disc form one parameter interval; z is linear so the maximum is at its ends.
        double dx = b.X - a.X, dy = b.Y - a.Y, fx = a.X - x, fy = a.Y - y;
        double A = dx * dx + dy * dy; if (A < 1e-24) return double.NegativeInfinity;
        double B = fx * dx + fy * dy, C = fx * fx + fy * fy - _r * _r, disc = B * B - A * C;
        if (disc < 0) return double.NegativeInfinity;
        double s = Math.Sqrt(disc), t0 = (-B - s) / A, t1 = (-B + s) / A, best = double.NegativeInfinity;
        foreach (var t in new[] { t0, t1 }) if (t >= 0 && t <= 1) best = Math.Max(best, a.Z + t * (b.Z - a.Z));
        return best;
    }

    private static bool InsideXY(Vec3 a, Vec3 b, Vec3 c, double x, double y)
    {
        double den = (b.Y - c.Y) * (a.X - c.X) + (c.X - b.X) * (a.Y - c.Y);
        if (Math.Abs(den) < 1e-14) return false;
        double u = ((b.Y - c.Y) * (x - c.X) + (c.X - b.X) * (y - c.Y)) / den;
        double v = ((c.Y - a.Y) * (x - c.X) + (a.X - c.X) * (y - c.Y)) / den;
        return u >= -1e-10 && v >= -1e-10 && 1 - u - v >= -1e-10;
    }
}
