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

    /// <summary>Greedy nearest-end ordering that may reverse passes, so neighbouring lanes become a zigzag.
    /// Uses a uniform grid over the pass end points, so it stays fast with hundreds of thousands of passes.</summary>
    public static List<List<Point3>> OrderZigzag(List<List<Point3>> cuts)
    {
        var all = cuts.Where(p => p.Count >= 2).ToList();
        int n = all.Count;
        if (n < 3) return all;
        double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
        foreach (var p in all) foreach (var q in new[] { p[0], p[^1] })
        { minX = Math.Min(minX, q.X); minY = Math.Min(minY, q.Y); minZ = Math.Min(minZ, q.Z); maxX = Math.Max(maxX, q.X); maxY = Math.Max(maxY, q.Y); maxZ = Math.Max(maxZ, q.Z); }
        double vol = Math.Max(1e-6, (maxX - minX + 1) * (maxY - minY + 1) * (maxZ - minZ + 1));
        double cell = Math.Max(0.5, Math.Cbrt(vol / (2.0 * n)) * 2);
        long Key(int ix, int iy, int iz) => ((long)ix & 0x1FFFFF) | (((long)iy & 0x1FFFFF) << 21) | (((long)iz & 0x1FFFFF) << 42);
        int Ix(double v, double lo) => (int)Math.Floor((v - lo) / cell);
        var grid = new Dictionary<long, List<int>>();
        for (int i = 0; i < n; i++)
            for (int e = 0; e < 2; e++)
            {
                var q = e == 0 ? all[i][0] : all[i][^1];
                long k = Key(Ix(q.X, minX), Ix(q.Y, minY), Ix(q.Z, minZ));
                if (!grid.TryGetValue(k, out var l)) grid[k] = l = new List<int>();
                l.Add(i * 2 + e);
            }
        var used = new bool[n];
        var result = new List<List<Point3>>(n);
        int cur = 0; bool curRev = false; used[0] = true; result.Add(all[0]);
        int usedCount = 1;
        double D2(Point3 a, Point3 b) => (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z);
        while (usedCount < n)
        {
            var tail = result[^1][^1];
            int cx = Ix(tail.X, minX), cy = Ix(tail.Y, minY), cz = Ix(tail.Z, minZ);
            int bestId = -1; double bd = double.MaxValue;
            for (int r = 0; r < 400; r++)
            {
                for (int dx = -r; dx <= r; dx++) for (int dy = -r; dy <= r; dy++) for (int dz = -r; dz <= r; dz++)
                {
                    if (Math.Max(Math.Abs(dx), Math.Max(Math.Abs(dy), Math.Abs(dz))) != r) continue;
                    if (!grid.TryGetValue(Key(cx + dx, cy + dy, cz + dz), out var list)) continue;
                    for (int li = list.Count - 1; li >= 0; li--)
                    {
                        int id = list[li], pi = id >> 1;
                        if (used[pi]) { list[li] = list[^1]; list.RemoveAt(list.Count - 1); continue; }
                        var q = (id & 1) == 0 ? all[pi][0] : all[pi][^1];
                        double d = D2(tail, q);
                        if (d < bd) { bd = d; bestId = id; }
                    }
                }
                if (bestId >= 0 && Math.Sqrt(bd) <= r * cell) break;
            }
            if (bestId < 0)   // far away: scan whatever is left
                for (int i = 0; i < n; i++) if (!used[i])
                    foreach (int e in new[] { 0, 1 })
                    { var q = e == 0 ? all[i][0] : all[i][^1]; double d = D2(tail, q); if (d < bd) { bd = d; bestId = i * 2 + e; } }
            int pidx = bestId >> 1; used[pidx] = true; usedCount++;
            var next = all[pidx];
            if ((bestId & 1) == 1) { next = new List<Point3>(next); next.Reverse(); }
            result.Add(next);
        }
        return result;
    }
}
