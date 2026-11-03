using System.Text.Json.Serialization;

namespace CamGh.Core;

public sealed record MeshInput(double[][] Vertices, int[][] Faces);
public sealed record ParallelRequest(MeshInput Mesh, double LaneSpacing, double DirectionX = 1, double DirectionY = 0, double EdgeInset = 0, double ChordTolerance = 0.05);
public sealed record Point3(double X, double Y, double Z);
public sealed record ParallelResult(List<List<Point3>> Paths, QualityReport Quality, string Method);
public sealed record QualityReport(int PathCount, int PointCount, double TotalCutLength, double MeanSegmentLength, double SegmentLengthCoefficientOfVariation, double MinimumZ, double MaximumZ);

/// <summary>Projected parallel scanlines over a triangulated height surface. Mesh-plane intersections are computed exactly per triangle; this is not a geodesic or iso-scallop algorithm.</summary>
public static class ParallelPlanner
{
    private sealed record Segment(Point3 A, Point3 B, double U);
    public static ParallelResult Generate(ParallelRequest request)
    {
        if (request.LaneSpacing <= 0 || !double.IsFinite(request.LaneSpacing)) throw new ArgumentOutOfRangeException(nameof(request.LaneSpacing));
        var mesh = request.Mesh;
        if (mesh.Vertices.Length < 3 || mesh.Faces.Length == 0) throw new ArgumentException("A triangulated mesh is required.");
        var dLen = Math.Sqrt(request.DirectionX * request.DirectionX + request.DirectionY * request.DirectionY);
        if (dLen < 1e-12) throw new ArgumentException("Scan direction must be non-zero.");
        double dx = request.DirectionX / dLen, dy = request.DirectionY / dLen;
        // u is the along-lane coordinate; v is the across-lane coordinate.
        double ux = dx, uy = dy, vx = -dy, vy = dx;
        var points = mesh.Vertices.Select(v => new Point3(v[0], v[1], v[2])).ToArray();
        double minV = points.Min(p => p.X * vx + p.Y * vy) + request.EdgeInset;
        double maxV = points.Max(p => p.X * vx + p.Y * vy) - request.EdgeInset;
        var paths = new List<List<Point3>>();
        int laneCount = Math.Max(1, (int)Math.Floor((maxV - minV) / request.LaneSpacing) + 1);
        for (int lane = 0; lane < laneCount; lane++)
        {
            double v = minV + lane * request.LaneSpacing;
            var segments = new List<Segment>();
            foreach (var face in mesh.Faces)
            {
                if (face.Length < 3) continue;
                var tri = new[] { points[face[0]], points[face[1]], points[face[2]] };
                var hits = new List<Point3>(2);
                for (int e = 0; e < 3; e++)
                {
                    var a = tri[e]; var b = tri[(e + 1) % 3];
                    double va = a.X * vx + a.Y * vy - v, vb = b.X * vx + b.Y * vy - v;
                    if (Math.Abs(va) < 1e-9) AddUnique(hits, a);
                    if ((va < 0 && vb > 0) || (va > 0 && vb < 0))
                    {
                        double t = va / (va - vb);
                        AddUnique(hits, new Point3(a.X + t * (b.X - a.X), a.Y + t * (b.Y - a.Y), a.Z + t * (b.Z - a.Z)));
                    }
                }
                if (hits.Count >= 2)
                {
                    var ordered = hits.OrderBy(p => p.X * ux + p.Y * uy).ToArray();
                    var a = ordered[0]; var b = ordered[^1];
                    if (Distance(a, b) > 1e-8) segments.Add(new Segment(a, b, (a.X * ux + a.Y * uy + b.X * ux + b.Y * uy) * .5));
                }
            }
            // Join triangle fragments by endpoint proximity. Disconnected islands remain separate paths.
            segments.Sort((a,b) => a.U.CompareTo(b.U));
            var joined = new List<Point3>();
            foreach (var s in segments)
            {
                if (joined.Count == 0) { joined.Add(s.A); joined.Add(s.B); continue; }
                if (Distance(joined[^1], s.A) <= Math.Max(request.ChordTolerance, 1e-5)) joined.Add(s.B);
                else { AddPath(paths, joined); joined = new List<Point3>{s.A,s.B}; }
            }
            AddPath(paths, joined);
        }
        return new ParallelResult(paths, Measure(paths), "Projected parallel triangle-mesh scanlines; fixed XY lane pitch, linear triangle interpolation.");
    }
    private static void AddUnique(List<Point3> xs, Point3 p) { if (xs.All(q => Distance(p,q) > 1e-8)) xs.Add(p); }
    private static double Distance(Point3 a, Point3 b) => Math.Sqrt((a.X-b.X)*(a.X-b.X)+(a.Y-b.Y)*(a.Y-b.Y)+(a.Z-b.Z)*(a.Z-b.Z));
    private static void AddPath(List<List<Point3>> paths, List<Point3> p) { if (p.Count >= 2) paths.Add(p); }
    private static QualityReport Measure(List<List<Point3>> paths)
    {
        var lengths = paths.SelectMany(p => p.Zip(p.Skip(1), Distance)).ToArray();
        if (lengths.Length == 0) return new(0,0,0,0,0,0,0);
        double mean = lengths.Average(), sd = Math.Sqrt(lengths.Average(x => (x-mean)*(x-mean)));
        var zs = paths.SelectMany(p => p).Select(p => p.Z).ToArray();
        return new(paths.Count, paths.Sum(p => p.Count), lengths.Sum(), mean, mean == 0 ? 0 : sd/mean, zs.Min(), zs.Max());
    }
}

/// <summary>Analytic ball-end cusp step estimate on a plane; research hook, not yet curvature-adaptive 3D spacing.</summary>
public static class ScallopMath
{
    public static double PlanarBallEndStep(double toolRadius, double targetScallop)
    {
        if (toolRadius <= 0 || targetScallop <= 0 || targetScallop >= toolRadius) throw new ArgumentOutOfRangeException(nameof(targetScallop));
        return 2 * Math.Sqrt(2 * targetScallop * toolRadius - targetScallop * targetScallop);
    }
}
