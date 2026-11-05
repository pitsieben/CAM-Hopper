namespace CamGh.Core;

/// <summary>Cutting paths machined with one fixed tool axis (3+2). Axis points from the tip towards the spindle.</summary>
public sealed record OrientedPaths(Vec3 Axis, List<List<Point3>> Paths, double RemovedVolume, string Note);

public sealed record MultiAxisRoughResult(List<OrientedPaths> Groups, double StartVolume, double EndVolume, string Method);

/// <summary>
/// R1 roughing: several fixed tool orientations (3+2). For each axis the part and the shared tri-dexel stock
/// are expressed in that tool frame and the existing stock-aware Z-level engine generates layers there. Its
/// height field is seeded with the top of the material that is still left in 3D, so later orientations only
/// cut what earlier ones could not reach. Each orientation's paths are subtracted from the 3D stock before
/// the next orientation is planned.
/// </summary>
public static class MultiAxisRoughPlanner
{
    public static MultiAxisRoughResult Generate(MultiAxisRoughingCommand cmd, TriDexelStock stock)
    {
        var tool = cmd.Tool; double r = tool.Radius;
        double stepdown = tool.Diameter * cmd.Parameters.StepdownPercent / 100.0;
        double stepover = tool.Diameter * cmd.Parameters.StepoverPercent / 100.0;
        double leave = cmd.Parameters.StockToLeave;
        double reach = cmd.ToolReach > 0 ? cmd.ToolReach : 4 * tool.Diameter;
        double fixtureZ = cmd.FixtureZ ?? stock.Bounds.MinZ;
        var part = TriMesh.From(cmd.Mesh);
        double startVolume = stock.Volume();

        var groups = new List<OrientedPaths>();
        string selection;
        if (cmd.Directions is { Count: > 0 })
        {
            var axes = cmd.Directions.Select(d => Vec3.From(d).Normalized()).Where(d => d.Length > 0.5).ToList();
            if (axes.Count == 0) throw new ArgumentException($"{cmd.Name}: all given directions are zero vectors.");
            foreach (var axis in axes) groups.Add(Rough(cmd, stock, axis, reach, fixtureZ));
            selection = $"{axes.Count} user direction(s)";
        }
        else
        {
            // Plan → cut → re-score: each new axis is chosen against the stock that is really left.
            var candidates = new OrientationPlanner.RoughingCandidates(part, stock, r, leave);
            double minGain = 0.005 * startVolume;
            for (int round = 0; round < Math.Max(1, cmd.MaxOrientations); round++)
            {
                var best = candidates.Best(stock, reach, fixtureZ);
                if (best is null || best.Value.Gain < minGain) break;
                var group = Rough(cmd, stock, best.Value.Axis, reach, fixtureZ);
                if (group.Paths.Count > 0) groups.Add(group);
            }
            if (groups.Count == 0) throw new ArgumentException($"{cmd.Name}: no tool direction can reach the remaining material (check reach and fixture height).");
            selection = $"{groups.Count} direction(s) chosen by visibility against the remaining stock";
        }
        double endVolume = stock.Volume();
        return new MultiAxisRoughResult(groups.Where(g => g.Paths.Count > 0).ToList(), startVolume, endVolume,
            $"3+2 multi-orientation {(cmd.Strategy == RoughingStrategy.SurfaceOffset ? "surface-offset" : "Z-level")} roughing, {selection}; tri-dexel stock at {stock.Resolution:g} mm; reach {reach:g} mm; fixture Z {fixtureZ:g}.");
    }

    private static OrientedPaths Rough(MultiAxisRoughingCommand cmd, TriDexelStock stock, Vec3 axis, double reach, double fixtureZ) =>
        cmd.Strategy == RoughingStrategy.ZLevel ? RoughOne(cmd, stock, axis, reach, fixtureZ) : RoughOffset(cmd, stock, axis, reach, fixtureZ);

    /// <summary>
    /// Surface-offset roughing for one tool axis. Passes are flat levels one stepdown apart, each floored by the
    /// part surface offset by the stock to leave: z = max(level, floor(x, y)). The floor is an exact drop-cutter of
    /// a cutter enlarged by the allowance onto the part mesh (the finishing machinery), so wherever a pass reaches the
    /// part it follows that offset surface rather than a grid, and once the levels pass below the part the tool
    /// has left a uniform shell instead of Z-level stairs. Engagement stays within one stepdown: material above a
    /// level was removed by the level before it. Paths are zigzag lanes; each move is subtracted from the shared
    /// tri-dexel stock as it is planned, and moves that remove nothing (air) are dropped, so later passes and later
    /// orientations only cut what is really left.
    /// </summary>
    private static OrientedPaths RoughOffset(MultiAxisRoughingCommand cmd, TriDexelStock stock, Vec3 axis, double reach, double fixtureZ)
    {
        var tool = cmd.Tool; double r = tool.Radius;
        double ap = tool.Diameter * cmd.Parameters.StepdownPercent / 100.0;
        double ae = tool.Diameter * cmd.Parameters.StepoverPercent / 100.0;
        double leave = Math.Max(0, cmd.Parameters.StockToLeave);
        var frame = Frame.FromAxis(axis);
        var local = TriMesh.From(frame.Transform(cmd.Mesh));
        var bins = new TriangleBins(local, Math.Max(2.0, r));
        var dc = new DropCutter(bins, tool.Shape, r + leave);
        var lb = frame.LocalBounds(stock.Bounds, r + 1);
        double stockBottom = frame.LocalBounds(stock.Bounds, 0).MinZ;
        double zTop = stock.MaxLocal(frame);
        double s = cmd.GridResolution > 0 ? cmd.GridResolution : Math.Clamp(r * 0.1, 0.3, 1.0);
        int nx = (int)Math.Ceiling((lb.MaxX - lb.MinX) / s) + 1;
        int lanes = Math.Max(1, (int)Math.Ceiling((lb.MaxY - lb.MinY) / ae) + 1);
        double y0 = 0.5 * (lb.MinY + lb.MaxY) - 0.5 * (lanes - 1) * ae;
        double X(int i) => lb.MinX + i * s;
        double Y(int j) => y0 + j * ae;

        // Offset floor under every lane sample: the tip may not go lower without coming closer than `leave`.
        var floor = new double[lanes][];
        Parallel.For(0, lanes, j =>
        {
            var stamp = bins.NewStamp(); int sv = 0; var row = new double[nx];
            for (int i = 0; i < nx; i++) { double z = dc.Drop(X(i), Y(j), stamp, ref sv, out _); row[i] = double.IsNegativeInfinity(z) ? z : z + leave; }
            floor[j] = row;
        });
        bool Allowed(double x, double y, double z) => OrientationPlanner.LowestToolZ(frame.ToWorld(x, y, z).Z, frame.W, r, reach) >= fixtureZ - 1e-9;
        Vec3 W(double x, double y, double z) => frame.ToWorld(x, y, z);
        // Material probe inside the cutter: centre and eight rim points at 0.8 r, at three heights up to one stepdown.
        var ring = Enumerable.Range(0, 8).Select(k => (Math.Cos(k * Math.PI / 4) * 0.8 * r, Math.Sin(k * Math.PI / 4) * 0.8 * r)).Prepend((0.0, 0.0)).ToArray();
        double[] heights = { 0.15, 0.5 * ap, ap };
        bool Probe(double x, double y, double z)
        {
            foreach (var hgt in heights) foreach (var (dx, dy) in ring) if (stock.IsMaterial(W(x + dx, y + dy, z + hgt))) return true;
            return false;
        }
        int chunk = Math.Max(1, (int)Math.Round(0.25 * r / s));
        double linkMax = 3 * tool.Diameter;            // farther than this, retract and reposition instead
        const double airVolume = 0.01;                 // mm³: a move removing less than this is cutting air
        double airGapMax = 3 * tool.Diameter;          // shorter air stretches inside a lane are fed through

        // Air pre-test: the highest remaining material within the cutter's footprint (a square of half-width r,
        // which contains the disk) from the conservative stock top map. If that is not above the tip, the move
        // cannot cut, so it is not simulated. Refreshed every level, because each level changes the stock.
        int mapNy = (int)Math.Ceiling((lb.MaxY - lb.MinY) / s) + 1;
        int half = (int)Math.Ceiling(r / s);
        double[,] FootTop()
        {
            var top = stock.TopMap(frame, lb.MinX, lb.MinY, s, nx, mapNy);
            return SquareMax(SquareMax(top, nx, mapNy, half, alongX: true), nx, mapNy, half, alongX: false);
        }

        var paths = new List<List<Point3>>(); int levels = 0; double removed = 0;
        var linkStamp = bins.NewStamp(); int linkSv = 0;
        for (double level = zTop - ap; level > stockBottom - ap - 1e-9; level -= ap)
        {
            levels++;
            var foot = FootTop();
            var runs = new List<List<Point3>>();
            for (int j = 0; j < lanes; j++)
            {
                bool forward = (j % 2) == 0; double y = Y(j); var fl = floor[j];
                int row = Math.Clamp((int)Math.Round((y - lb.MinY) / s), 0, mapNy - 1);
                List<Point3>? run = null; var air = new List<Point3>(); double airLen = 0;
                void Close() { if (run is { Count: >= 2 }) runs.Add(run); run = null; air.Clear(); airLen = 0; }
                // Simulate in chunks of a few samples: the chord of a chunk deviates from the sampled path by well
                // under 0.1 mm, far finer than the cut/air decision needs, and it cuts the stock simulation cost
                // several-fold. The emitted path keeps every sample, so its shape is unaffected.
                for (int step = 0; step + 1 < nx; step += chunk)
                {
                    int last = Math.Min(nx - 1, step + chunk);
                    int Idx(int k) => forward ? k : nx - 1 - k;
                    int ia = Idx(step), ib = Idx(last);
                    double za = Math.Max(level, fl[ia]), zb = Math.Max(level, fl[ib]);
                    bool ok = true;
                    for (int k = step; k <= last && ok; k++) ok = Allowed(X(Idx(k)), y, Math.Max(level, fl[Idx(k)]));
                    if (!ok) { Close(); continue; }
                    double zLow = za, zHigh = za;
                    for (int k = step + 1; k <= last; k++) { double z = Math.Max(level, fl[Idx(k)]); zLow = Math.Min(zLow, z); zHigh = Math.Max(zHigh, z); }
                    bool maybe = false;
                    for (int k = step; k <= last && !maybe; k++) maybe = foot[Idx(k), row] > zLow;
                    // Tighter probe: any material in the lower part of the cutter at the chunk's ends or middle?
                    // Skipping a move here leaves both the path and the stock untouched, so the simulation stays exact.
                    if (maybe) maybe = Probe(X(ia), y, za) || Probe(X(ib), y, zb) || Probe(0.5 * (X(ia) + X(ib)), y, 0.5 * (za + zb));
                    double cut = maybe ? stock.SubtractSweepMeasured(W(X(ia), y, za), W(X(ib), y, zb), frame.W, tool.Shape, r, reach) : 0;
                    var pts = new List<Point3>(last - step);
                    for (int k = step + 1; k <= last; k++) pts.Add(new Point3(X(Idx(k)), y, Math.Max(level, fl[Idx(k)])));
                    if (cut > airVolume)
                    {
                        removed += cut;
                        if (run is null) run = new List<Point3> { new(X(ia), y, za) };
                        else if (air.Count > 0) { if (airLen <= airGapMax) run.AddRange(air); else { var restart = air[^1]; Close(); run = new List<Point3> { restart }; } }
                        air.Clear(); airLen = 0;
                        run.AddRange(pts);
                    }
                    else if (run is not null) { air.AddRange(pts); airLen += s * pts.Count; }
                }
                Close();
            }
            // Order the level's runs nearest-first (a run may be reversed) and join each to the previous one with a
            // straight link lifted over the offset floor, so a level becomes a few long paths instead of many strips.
            var left = new List<List<Point3>>(runs);
            List<Point3>? current = null;
            while (left.Count > 0)
            {
                int pick = 0; bool reverse = false;
                if (current is not null)
                {
                    double best = double.MaxValue; var end = current[^1];
                    for (int k = 0; k < left.Count; k++)
                    {
                        double d0 = Dist2(end, left[k][0]), d1 = Dist2(end, left[k][^1]);
                        if (d0 < best) { best = d0; pick = k; reverse = false; }
                        if (d1 < best) { best = d1; pick = k; reverse = true; }
                    }
                }
                var next = left[pick]; left.RemoveAt(pick);
                if (reverse) next.Reverse();
                if (current is not null && TryLink(current[^1], next[0], out var link)) { current.AddRange(link); current.AddRange(next); }
                else { current = next; paths.Add(current); }
            }
        }
        bool TryLink(Point3 a, Point3 b, out List<Point3> link)
        {
            link = new List<Point3>();
            double dx = b.X - a.X, dy = b.Y - a.Y, dist = Math.Sqrt(dx * dx + dy * dy);
            if (dist > linkMax || Math.Abs(b.Z - a.Z) > ap) return false;
            int n = Math.Max(1, (int)Math.Ceiling(dist / s));
            var prev = a;
            for (int k = 1; k < n; k++)
            {
                double t = (double)k / n, x = a.X + dx * t, y = a.Y + dy * t;
                double f = dc.Drop(x, y, linkStamp, ref linkSv, out _); f = double.IsNegativeInfinity(f) ? f : f + leave;
                double z = Math.Max(a.Z + (b.Z - a.Z) * t, f);
                if (!Allowed(x, y, z)) return false;
                var p = new Point3(x, y, z); link.Add(p);
                removed += stock.SubtractSweepMeasured(W(prev.X, prev.Y, prev.Z), W(x, y, z), frame.W, tool.Shape, r, reach); prev = p;
            }
            removed += stock.SubtractSweepMeasured(W(prev.X, prev.Y, prev.Z), W(b.X, b.Y, b.Z), frame.W, tool.Shape, r, reach);
            return true;
        }

        var world = paths.Where(p => p.Count >= 2).Select(p => CamJobPlanner.Simplify(p.Select(frame.ToWorld).ToList(), 0.01)).ToList();
        return new OrientedPaths(frame.W, world, removed,
            $"Axis ({frame.W.X:0.###}, {frame.W.Y:0.###}, {frame.W.Z:0.###}): {levels} level(s), {world.Count} path(s), removed {removed / 1000:0.#} cm³ (surface offset, {lanes} lanes at {ae:0.#} mm, samples {s:0.##} mm).");
    }

    private static double Dist2(Point3 a, Point3 b) => (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z);

    /// <summary>Running maximum over a window of ±half cells along one grid axis (monotonic deque, O(n)).</summary>
    private static double[,] SquareMax(double[,] src, int nx, int ny, int half, bool alongX)
    {
        var dst = new double[nx, ny];
        int lines = alongX ? ny : nx, len = alongX ? nx : ny;
        Parallel.For(0, lines, line =>
        {
            double V(int k) => alongX ? src[k, line] : src[line, k];
            var dq = new int[len]; int head = 0, tail = 0;
            for (int k = 0, add = 0; k < len; k++)
            {
                int hi = Math.Min(len - 1, k + half), lo = k - half;
                for (; add <= hi; add++) { while (tail > head && V(dq[tail - 1]) <= V(add)) tail--; dq[tail++] = add; }
                while (dq[head] < lo) head++;
                double m = V(dq[head]);
                if (alongX) dst[k, line] = m; else dst[line, k] = m;
            }
        });
        return dst;
    }

    private static OrientedPaths RoughOne(MultiAxisRoughingCommand cmd, TriDexelStock stock, Vec3 axis, double reach, double fixtureZ)
    {
        var tool = cmd.Tool; double r = tool.Radius;
        double stepdown = tool.Diameter * cmd.Parameters.StepdownPercent / 100.0;
        double stepover = tool.Diameter * cmd.Parameters.StepoverPercent / 100.0;
        var frame = Frame.FromAxis(axis);
        // Default layer grid: at most 1 mm. A 2 mm grid (the old default for a 16 mm cutter) both coarsened the
        // cut contours into visible staircases and left about twice the skin over the part for finishing.
        double h = cmd.GridResolution > 0 ? cmd.GridResolution : Math.Clamp(r * 0.15, 0.5, 1.0);
        var lb = frame.LocalBounds(stock.Bounds, r + 3 * h + 1);
        while ((Math.Ceiling((lb.MaxX - lb.MinX) / h) + 1) * (Math.Ceiling((lb.MaxY - lb.MinY) / h) + 1) > 1_500_000) h *= 1.2;
        lb = frame.LocalBounds(stock.Bounds, r + 3 * h + 1);
        var ctx = new RoughingContext
        {
            InitialRemaining = (sb, hh, nx, ny) => stock.TopMap(frame, sb.MinX, sb.MinY, hh, nx, ny),
            Reach = reach,
            FullAccess = true,
            RasterPatternsOnly = true,
            TipAllowed = (x, y, z) => OrientationPlanner.LowestToolZ(frame.ToWorld(x, y, z).Z, frame.W, r, reach) >= fixtureZ - 1e-9,
        };
        var request = new HorizontalRoughRequest(frame.Transform(cmd.Mesh), lb, r, stepdown, stepover, cmd.Parameters.StockToLeave, h, cmd.Parameters.Pattern, tool.Shape);
        var local = HorizontalRoughPlanner.Generate(request, ctx);
        // Points on straight runs carry no information; dropping them shrinks programs and robot files.
        var world = local.Paths.Where(p => p.Count >= 2).Select(p => CamJobPlanner.Simplify(p.Select(frame.ToWorld).ToList(), 0.005)).ToList();
        double before = stock.Volume();
        foreach (var path in world) stock.SubtractPath(path, frame.W, tool.Shape, r, reach);
        double removed = Math.Max(0, before - stock.Volume());
        return new OrientedPaths(frame.W, world, removed,
            $"Axis ({frame.W.X:0.###}, {frame.W.Y:0.###}, {frame.W.Z:0.###}): {local.ZLevels} layer(s), {world.Count} path(s), removed {removed / 1000:0.#} cm³ (grid {h:0.##} mm).");
    }
}
