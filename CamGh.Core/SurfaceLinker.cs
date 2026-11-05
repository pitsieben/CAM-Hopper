namespace CamGh.Core;

/// <summary>
/// Stay-down links between neighbouring finishing passes: instead of retracting to the clearance plane after
/// every pass, the tool is carried straight to the next pass along the part, riding just above the surface
/// (drop-cutter height of the real tool, so the link can never gouge). Links that would be long, or that would
/// have to climb far above their end points, are refused and the caller retracts as usual.
/// </summary>
internal sealed class SurfaceLinker
{
    private readonly Frame _frame;
    private readonly DropCutter _cutter;
    private readonly int[] _stamp;
    private int _stampValue;
    private readonly double _leave, _maxLink, _maxRise, _step;

    public SurfaceLinker(MeshInput mesh, Vec3 axis, ToolDefinition tool, double leave)
    {
        _frame = Frame.FromAxis(axis);
        var local = TriMesh.From(mesh).Transformed(_frame);
        var bins = new TriangleBins(local, Math.Max(tool.Radius, local.MeanEdgeLength()));
        _leave = Math.Max(0, leave);
        _cutter = new DropCutter(bins, tool.Shape, tool.Shape == ToolShape.Ball ? tool.Radius + _leave : tool.Radius);
        _stamp = bins.NewStamp();
        _maxLink = 4 * tool.Diameter;
        _maxRise = 2 * tool.Diameter;
        _step = Math.Max(0.25, tool.Radius * 0.25);
    }

    /// <summary>Intermediate points from a to b (excluding both), or null when the link must retract instead.</summary>
    public List<Point3>? Link(Point3 a, Point3 b)
    {
        var la = _frame.ToLocal(Vec3.From(a)); var lb = _frame.ToLocal(Vec3.From(b));
        double d = (lb - la).Length;
        if (d > _maxLink) return null;
        int n = Math.Max(1, (int)Math.Ceiling(d / _step));
        double ceiling = Math.Max(la.Z, lb.Z) + _maxRise;
        var pts = new List<Point3>();
        for (int i = 1; i < n; i++)
        {
            double t = (double)i / n, x = la.X + (lb.X - la.X) * t, y = la.Y + (lb.Y - la.Y) * t, z = la.Z + (lb.Z - la.Z) * t;
            double floor = _cutter.Drop(x, y, _stamp, ref _stampValue, out _);
            if (double.IsFinite(floor)) z = Math.Max(z, floor + _leave);
            if (z > ceiling) return null;
            pts.Add(_frame.ToWorld(new Vec3(x, y, z)).ToPoint());
        }
        // The end points themselves must not sit below the surface the cutter would ride on.
        return pts;
    }

    /// <summary>Greedy nearest-end ordering that may reverse passes, so neighbouring lanes become a zigzag.</summary>
    public static List<List<Point3>> OrderZigzag(List<List<Point3>> cuts)
    {
        var rest = cuts.Where(p => p.Count >= 2).ToList();
        if (rest.Count < 3) return rest;
        var result = new List<List<Point3>> { rest[0] }; rest.RemoveAt(0);
        double D2(Point3 a, Point3 b) => (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z);
        while (rest.Count > 0)
        {
            var tail = result[^1][^1]; int best = 0; bool rev = false; double bd = double.MaxValue;
            for (int i = 0; i < rest.Count; i++)
            {
                double ds = D2(tail, rest[i][0]), de = D2(tail, rest[i][^1]);
                if (ds < bd) { bd = ds; best = i; rev = false; }
                if (de < bd) { bd = de; best = i; rev = true; }
            }
            var next = rest[best]; rest.RemoveAt(best);
            if (rev) { next = new List<Point3>(next); next.Reverse(); }
            result.Add(next);
        }
        return result;
    }
}
