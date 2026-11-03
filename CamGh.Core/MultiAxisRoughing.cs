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
            foreach (var axis in axes) groups.Add(RoughOne(cmd, stock, axis, reach, fixtureZ));
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
                var group = RoughOne(cmd, stock, best.Value.Axis, reach, fixtureZ);
                if (group.Paths.Count > 0) groups.Add(group);
            }
            if (groups.Count == 0) throw new ArgumentException($"{cmd.Name}: no tool direction can reach the remaining material (check reach and fixture height).");
            selection = $"{groups.Count} direction(s) chosen by visibility against the remaining stock";
        }
        double endVolume = stock.Volume();
        return new MultiAxisRoughResult(groups.Where(g => g.Paths.Count > 0).ToList(), startVolume, endVolume,
            $"3+2 multi-orientation Z-level roughing, {selection}; tri-dexel stock at {stock.Resolution:g} mm; reach {reach:g} mm; fixture Z {fixtureZ:g}.");
    }

    private static OrientedPaths RoughOne(MultiAxisRoughingCommand cmd, TriDexelStock stock, Vec3 axis, double reach, double fixtureZ)
    {
        var tool = cmd.Tool; double r = tool.Radius;
        double stepdown = tool.Diameter * cmd.Parameters.StepdownPercent / 100.0;
        double stepover = tool.Diameter * cmd.Parameters.StepoverPercent / 100.0;
        var frame = Frame.FromAxis(axis);
        double h = cmd.GridResolution > 0 ? cmd.GridResolution : Math.Clamp(r * 0.3, 0.4, 2.0);
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
