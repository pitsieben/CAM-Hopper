using System.Text.Json.Serialization;

namespace CamGh.Core;

public enum ToolShape { Flat, Ball }
public enum CutPattern { Offset = 0, OffsetSpiral = 1, Linear = 2, Spiral = 3, Radial = 4, HighSpeed = 5 }
public enum EngageMove { Plunge, Ramp, Helix }
public enum RetractMove { Vertical, Ramp, StayDown }
public enum ClearanceMode { Automatic, Manual }

public sealed record ToolDefinition(double Diameter, ToolShape Shape, int Flutes, double Rpm)
{
    public double Radius => Diameter / 2;
    public double CuttingSpeedMetresPerMinute => Math.PI * Diameter * Rpm / 1000.0;
}

public sealed record CutParameters(
    double StepoverPercent,
    double StepdownPercent,
    CutPattern Pattern,
    EngageMove Engage,
    RetractMove Retract,
    ClearanceMode ClearanceMode,
    double ClearanceOffset,
    double StockToLeave = 0.5);

public sealed record StockDefinition(StockBounds Bounds);
public sealed record MachineSetup(string Units = "Millimetres", double SafeStartZ = 0, string Name = "Default 3-axis");

[JsonPolymorphic(TypeDiscriminatorPropertyName = "$type")]
[JsonDerivedType(typeof(HorizontalRoughingCommand), "horizontal-roughing")]
[JsonDerivedType(typeof(ParallelFinishingCommand), "parallel-finishing")]
[JsonDerivedType(typeof(MultiAxisRoughingCommand), "multi-axis-roughing")]
[JsonDerivedType(typeof(RegionFinishingCommand), "region-finishing")]
public abstract record CamCommand(string Name, MeshInput Mesh, ToolDefinition Tool, CutParameters Parameters);

public sealed record HorizontalRoughingCommand(
    string Name,
    MeshInput Mesh,
    ToolDefinition Tool,
    CutParameters Parameters,
    double GridResolution = 0)
    : CamCommand(Name, Mesh, Tool, Parameters);

public sealed record ParallelFinishingCommand(
    string Name,
    MeshInput Mesh,
    ToolDefinition Tool,
    CutParameters Parameters,
    double DirectionX = 1,
    double DirectionY = 0,
    double EdgeInset = 0)
    : CamCommand(Name, Mesh, Tool, Parameters);

/// <summary>
/// R1: 3+2 roughing from several fixed tool axes against a shared 3D (tri-dexel) stock.
/// Directions are tool axes pointing from the cutter tip towards the spindle; empty = choose by visibility.
/// </summary>
public sealed record MultiAxisRoughingCommand(
    string Name,
    MeshInput Mesh,
    ToolDefinition Tool,
    CutParameters Parameters,
    List<Point3>? Directions = null,
    int MaxOrientations = 6,
    double ToolReach = 0,
    double? FixtureZ = null,
    double GridResolution = 0,
    double StockResolution = 0)
    : CamCommand(Name, Mesh, Tool, Parameters);

/// <summary>
/// F1: region-based 3+2 finishing. Patches come from accessibility-constrained k-means on face normals and position
/// (PositionWeight 0 = normals only); BorderSmoothing is the curvature-flow radius in mm (0 = off) — features smaller
/// than it are removed and borders run smoothly through faces. Lanes are gouge-free drop-cutter samples.
/// </summary>
public sealed record RegionFinishingCommand(
    string Name,
    MeshInput Mesh,
    ToolDefinition Tool,
    CutParameters Parameters,
    List<Point3>? Directions = null,
    int MaxOrientations = 6,
    double MaxWallAngleDegrees = 75,
    double SampleStep = 0,
    int OverlapRings = 1,
    double BorderSmoothing = 16.0,
    double PositionWeight = 0.5)
    : CamCommand(Name, Mesh, Tool, Parameters);

public sealed record CamJobRequest(List<CamCommand> Commands, StockDefinition Stock, MachineSetup Setup);

/// <summary>PathAxes holds one tool axis per path (tip → spindle); null means +Z for every path.</summary>
/// <summary>Region finishing also returns FaceLabels / FaceTilt (per input face, -1 = unreachable) and PatchAxes (per patch label).</summary>
public sealed record OperationResult(string Name, string Type, List<List<Point3>> Paths, QualityReport Quality, string Message, double SpindleRpm = 0, double CuttingSpeedMetresPerMinute = 0, List<Point3>? PathAxes = null,
    List<int>? FaceLabels = null, List<double>? FaceTilt = null, List<Point3>? PatchAxes = null, List<double[]>? PatchTriangles = null);

/// <summary>
/// Paths are linked motions (cuts + clearance moves); MotionAxes holds the fixed tool axis of each motion.
/// StockPreview is the remaining tri-dexel stock as Z-direction line segments.
/// </summary>
public sealed record CamJobResult(List<List<Point3>> Paths, List<OperationResult> Operations, double ClearanceZ, string Summary,
    List<Point3>? MotionAxes = null, List<List<Point3>>? StockPreview = null, double RemainingStockVolume = 0, double StartStockVolume = 0,
    List<int>? MotionOperations = null);

public static class CamJobPlanner
{
    private static readonly Point3 ZAxis = new(0, 0, 1);

    public static CamJobResult Generate(CamJobRequest job) => Generate(job, null);

    /// <summary>Plans the job. When snapshots is given it receives a copy of the 3D stock before the first operation and after each one.</summary>
    public static CamJobResult Generate(CamJobRequest job, List<TriDexelStock>? snapshots)
    {
        if (job.Commands.Count == 0) throw new ArgumentException("At least one CAM command is required.");
        var b = job.Stock.Bounds;
        double extent = Math.Max(b.MaxX - b.MinX, Math.Max(b.MaxY - b.MinY, b.MaxZ - b.MinZ));
        double requested = job.Commands.OfType<MultiAxisRoughingCommand>().Select(c => c.StockResolution).Where(x => x > 0).DefaultIfEmpty(0).Min();
        // One 3D stock is shared by every operation so later orientations only cut what is left.
        var stock = new TriDexelStock(b, requested > 0 ? requested : Math.Max(extent / 250.0, 0.25));
        double startVolume = stock.Volume();
        snapshots?.Add(stock.Clone());
        var operations = new List<OperationResult>();
        foreach (var command in job.Commands)
        {
            if (operations.Count > 0) snapshots?.Add(stock.Clone());
            Validate(command);
            switch (command)
            {
                case HorizontalRoughingCommand rough:
                {
                    var result = HorizontalRoughPlanner.Generate(new HorizontalRoughRequest(
                        rough.Mesh, job.Stock.Bounds, rough.Tool.Radius,
                        rough.Tool.Diameter * rough.Parameters.StepdownPercent / 100.0,
                        rough.Tool.Diameter * rough.Parameters.StepoverPercent / 100.0,
                        rough.Parameters.StockToLeave, rough.GridResolution, rough.Parameters.Pattern, rough.Tool.Shape));
                    UpdateStock(stock, result.Paths, new Vec3(0, 0, 1), rough);
                    var retractNote = rough.Parameters.Retract == RetractMove.StayDown
                        ? "Safe links stay down through legal cutter-center space; retracts are used only between disconnected regions."
                        : $"Safe links stay down where possible; {rough.Parameters.Retract} retracts are used where regions cannot be legally linked.";
                    var note = $"{result.Method} Generated {result.ZLevels} Z-level(s) and {result.Paths.Count} cutting path(s). {retractNote} Engage mode is not yet applied.";
                    operations.Add(new OperationResult(rough.Name, "Horizontal Roughing", result.Paths, result.Quality, note,
                        rough.Tool.Rpm, rough.Tool.CuttingSpeedMetresPerMinute, Enumerable.Repeat(ZAxis, result.Paths.Count).ToList()));
                    break;
                }
                case ParallelFinishingCommand finish:
                {
                    var result = ParallelPlanner.Generate(new ParallelRequest(
                        finish.Mesh,
                        finish.Tool.Diameter * finish.Parameters.StepoverPercent / 100.0,
                        finish.DirectionX, finish.DirectionY, finish.EdgeInset));
                    SubtractFinish(stock, result.Paths, null, finish);
                    var retractNote = finish.Parameters.Retract == RetractMove.StayDown
                        ? "StayDown requested; safe clearance links used because in-process stock is not tracked."
                        : $"{finish.Parameters.Retract} retracts generated at segment ends.";
                    var note = $"{result.Method} Finishing pattern executed: Linear; requested cut pattern applies to horizontal roughing only. {retractNote} Engage mode is not yet applied.";
                    operations.Add(new OperationResult(finish.Name, "Parallel Finishing", result.Paths, result.Quality, note,
                        finish.Tool.Rpm, finish.Tool.CuttingSpeedMetresPerMinute, Enumerable.Repeat(ZAxis, result.Paths.Count).ToList()));
                    break;
                }
                case MultiAxisRoughingCommand multi:
                {
                    var result = MultiAxisRoughPlanner.Generate(multi, stock);
                    var paths = result.Groups.SelectMany(g => g.Paths).ToList();
                    var axes = result.Groups.SelectMany(g => g.Paths.Select(_ => g.Axis.ToPoint())).ToList();
                    var note = $"{result.Method} Stock {result.StartVolume / 1000:0.#} → {result.EndVolume / 1000:0.#} cm³. " + string.Join(" ", result.Groups.Select(g => g.Note))
                        + " Links between orientations are clearance moves only; check reorientation moves in robot simulation. Engage mode is not yet applied.";
                    operations.Add(new OperationResult(multi.Name, "Multi-Axis Roughing (3+2)", paths, Measure(paths), note,
                        multi.Tool.Rpm, multi.Tool.CuttingSpeedMetresPerMinute, axes));
                    break;
                }
                case RegionFinishingCommand region:
                {
                    var result = RegionFinishPlanner.Generate(region);
                    var paths = result.Groups.SelectMany(g => g.Paths).ToList();
                    var axes = result.Groups.SelectMany(g => g.Paths.Select(_ => g.Axis.ToPoint())).ToList();
                    SubtractFinish(stock, paths, axes, region);
                    var note = $"{result.Method} Contacted {result.CoveredAreaFraction:P1} of the surface area; {result.UnreachableAreaFraction:P1} is not reachable by any chosen axis. "
                        + string.Join(" ", result.Groups.Select(g => g.Note));
                    operations.Add(new OperationResult(region.Name, "Region Finishing (3+2)", paths, Measure(paths), note,
                        region.Tool.Rpm, region.Tool.CuttingSpeedMetresPerMinute, axes,
                        result.FaceLabels.ToList(), result.FaceTilt.Select(t => Math.Round(t, 2)).ToList(), result.PatchAxes.Select(a => a.ToPoint()).ToList(), result.PatchTriangles));
                    break;
                }
                default: throw new NotSupportedException($"Unsupported CAM command {command.GetType().Name}.");
            }
        }
        snapshots?.Add(stock.Clone());
        var cuttingPaths = operations.SelectMany(o => o.Paths).ToList();
        var clearance = job.Commands.Max(c => c.Parameters.ClearanceMode == ClearanceMode.Automatic
            ? job.Stock.Bounds.MaxZ + c.Parameters.ClearanceOffset
            : c.Parameters.ClearanceOffset);
        if (clearance <= job.Stock.Bounds.MaxZ)
            throw new ArgumentException($"Clearance Z {clearance:g} must be above stock top Z {job.Stock.Bounds.MaxZ:g}.");
        var motions = new List<List<Point3>>(); var motionAxes = new List<Point3>(); var motionOps = new List<int>();
        for (int i = 0; i < operations.Count; i++)
        {
            var op = operations[i]; var parameters = job.Commands[i].Parameters;
            double offset = parameters.ClearanceMode == ClearanceMode.Automatic ? parameters.ClearanceOffset : Math.Max(parameters.ClearanceOffset - b.MaxZ, 5);
            int start = 0;
            while (start < op.Paths.Count)
            {
                var axis = AxisOf(op, start); int end = start + 1;
                while (end < op.Paths.Count && SameAxis(AxisOf(op, end), axis)) end++;
                var w = Vec3.From(axis).Normalized();
                // Vertical work keeps the absolute clearance plane; other axes clear the stock box by the offset.
                double clearW = w.Z > 0.999999 ? clearance : Frame.Corners(b).Max(c => c.Dot(w)) + offset;
                var motion = BuildClearanceMotion(op.Paths.GetRange(start, end - start), w, clearW, parameters.Retract);
                if (motion.Count > 0) { motions.Add(motion); motionAxes.Add(w.ToPoint()); motionOps.Add(i); }
                start = end;
            }
        }
        double remaining = stock.Volume();
        return new CamJobResult(motions, operations, clearance,
            $"{operations.Count} operation(s), {cuttingPaths.Count} cutting segment(s), {motions.Count} linked motion path(s) including clearance retracts and rapids; clearance Z {clearance:g} {job.Setup.Units} for vertical work. Remaining stock {remaining / 1000:0.#} of {startVolume / 1000:0.#} cm³ (tri-dexel {stock.Resolution:g} mm).",
            motionAxes, stock.PreviewSegments(20_000), remaining, startVolume, motionOps);
    }

    private static Point3 AxisOf(OperationResult op, int index) => op.PathAxes is { } a && index < a.Count ? a[index] : ZAxis;
    private static bool SameAxis(Point3 a, Point3 b) => Vec3.From(a).Normalized().Dot(Vec3.From(b).Normalized()) > 1 - 1e-9;

    /// <summary>Removes roughing cuts from the shared stock. The cutter body is modelled one stepdown deep, which
    /// is enough for top-down layers and errs towards keeping material (later operations cut air, never crash).</summary>
    private static void UpdateStock(TriDexelStock stock, List<List<Point3>> paths, Vec3 axis, CamCommand c)
    {
        double length = Math.Max(2 * c.Tool.Diameter * c.Parameters.StepdownPercent / 100.0, c.Tool.Radius) + c.Tool.Radius;
        foreach (var p in paths) stock.SubtractPath(Simplify(p, 0.005), axis, c.Tool.Shape, c.Tool.Radius, length);
    }

    /// <summary>Removes finishing lanes from the shared stock so the simulation shows what each finishing pass takes off.
    /// The cutter is modelled three diameters long along each lane's own axis.</summary>
    private static void SubtractFinish(TriDexelStock stock, List<List<Point3>> paths, List<Point3>? axes, CamCommand c)
    {
        double length = 3 * c.Tool.Diameter;
        for (int i = 0; i < paths.Count; i++)
        {
            var axis = axes is not null && i < axes.Count ? Vec3.From(axes[i]).Normalized() : new Vec3(0, 0, 1);
            stock.SubtractPath(Simplify(paths[i], 0.005), axis, c.Tool.Shape, c.Tool.Radius, length);
        }
    }

    /// <summary>Drops points that lie within tolerance of the line through their neighbours.</summary>
    public static List<Point3> Simplify(List<Point3> path, double tolerance)
    {
        if (path.Count < 3) return path;
        var result = new List<Point3> { path[0] };
        for (int i = 1; i < path.Count - 1; i++)
        {
            var a = Vec3.From(result[^1]); var p = Vec3.From(path[i]); var c = Vec3.From(path[i + 1]);
            var d = c - a; double len = d.Length;
            if (len < 1e-12) continue;
            double dist = (p - a).Cross(d).Length / len;
            if (dist > tolerance) result.Add(path[i]);
        }
        result.Add(path[^1]);
        return result;
    }

    private static List<Point3> BuildClearanceMotion(List<List<Point3>> cuts, Vec3 axis, double clearanceW, RetractMove retract)
    {
        Point3 Lift(Point3 p) { var v = Vec3.From(p); return (v + axis * (clearanceW - v.Dot(axis))).ToPoint(); }
        var motion = new List<Point3>();
        foreach (var cut in cuts.Where(p => p.Count >= 2))
        {
            var start = cut[0]; var end = cut[^1];
            if (motion.Count == 0)
            {
                motion.Add(Lift(start));
                motion.Add(start);
            }
            else
            {
                var previous = motion[^1];
                motion.Add(Lift(previous));
                motion.Add(Lift(start));
                motion.Add(start);
            }
            motion.AddRange(cut.Skip(1));
            var retractFrom = cut.Count > 1 ? cut[^2] : end;
            motion.Add(retract == RetractMove.Ramp && cut.Count > 1 ? Lift(retractFrom) : Lift(end));
        }
        return motion;
    }

    private static QualityReport Measure(List<List<Point3>> paths)
    {
        double D(Point3 a, Point3 b) => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));
        var lens = paths.SelectMany(p => p.Zip(p.Skip(1), D)).ToArray();
        if (lens.Length == 0) return new(0, 0, 0, 0, 0, 0, 0);
        double mean = lens.Average(), sd = Math.Sqrt(lens.Average(x => (x - mean) * (x - mean)));
        var zs = paths.SelectMany(p => p).Select(p => p.Z).ToArray();
        return new(paths.Count, paths.Sum(p => p.Count), lens.Sum(), mean, mean == 0 ? 0 : sd / mean, zs.Min(), zs.Max());
    }

    private static void Validate(CamCommand command)
    {
        if (command.Tool.Diameter <= 0 || command.Tool.Rpm <= 0 || command.Tool.Flutes < 1)
            throw new ArgumentException($"{command.Name}: tool diameter/RPM/flutes are invalid.");
        if (command.Parameters.StepoverPercent <= 0 || command.Parameters.StepdownPercent <= 0)
            throw new ArgumentException($"{command.Name}: stepover and stepdown percentages must be positive.");
    }
}
