using System.Diagnostics;
using CamGh.Core;

// Test part: an upright wavy torus (through-hole along Y, undercuts on every side) standing in a wooden block.
const double R = 50, A0 = 22, CZ = 120, SZ = 1.5;
static double Minor(double u) => A0 * (1 + 0.25 * Math.Sin(3 * u));
var verts = new List<double[]>(); var faces = new List<int[]>();
int NU = 240, NV = 60;
for (int i = 0; i < NU; i++)
    for (int j = 0; j < NV; j++)
    {
        double u = 2 * Math.PI * i / NU, v = 2 * Math.PI * j / NV, a = Minor(u);
        double ring = R + a * Math.Cos(v);
        verts.Add(new[] { ring * Math.Cos(u), a * Math.Sin(v), CZ + SZ * ring * Math.Sin(u) });
    }
for (int i = 0; i < NU; i++)
    for (int j = 0; j < NV; j++)
    {
        int a = i * NV + j, b = ((i + 1) % NU) * NV + j, c = ((i + 1) % NU) * NV + (j + 1) % NV, d = i * NV + (j + 1) % NV;
        faces.Add(new[] { a, b, c }); faces.Add(new[] { a, c, d });
    }
var mesh = new MeshInput(verts.ToArray(), faces.ToArray());
var box = new StockBounds(-85, -35, 0, 85, 35, 240);
var part = TriMesh.From(mesh);
Console.WriteLine($"Part: {part.Count} triangles, closed={part.IsClosed()}, volume={part.SignedVolume() / 1000:0.0} cm³");

bool InsideDeep(Vec3 p, double fraction)
{
    double xz = Math.Sqrt(p.X * p.X + Math.Pow((p.Z - CZ) / SZ, 2)); double u = Math.Atan2((p.Z - CZ) / SZ, p.X);
    double q = Math.Sqrt(Math.Pow(xz - R, 2) + p.Y * p.Y);
    return q < fraction * Minor(u) * 0.66;      // well inside even after the vertical stretch
}

int failures = 0;
void Check(bool ok, string what) { Console.WriteLine((ok ? "  PASS " : "  FAIL ") + what); if (!ok) failures++; }

// Rhino meshes are often unwelded: give every face its own vertices and expect the welded mesh to be closed again.
{
    var split = new MeshInput(faces.SelectMany(f => f.Select(i => verts[i])).ToArray(), Enumerable.Range(0, faces.Count).Select(k => new[] { 3 * k, 3 * k + 1, 3 * k + 2 }).ToArray());
    var w = TriMesh.From(split);
    Check(w.IsClosed() && w.Vertices.Length == verts.Count && Math.Abs(w.SignedVolume() - part.SignedVolume()) < 1, "unwelded input is welded back into the same closed solid");
}

// ------------------------------------------------------------------ step 1: tri-dexel stock unit checks
{
    var s = new TriDexelStock(new StockBounds(0, 0, 0, 100, 100, 50), 1.0);
    double v0 = s.Volume();
    Check(Math.Abs(v0 - 100 * 100 * 50) < 1, $"box volume {v0:0}");
    // Flat Ø10 slot 50 mm long, 5 mm deep from the top: analytic volume = (50*10 + π*25) * 5
    s.SubtractSweep(new Vec3(25, 50, 45), new Vec3(75, 50, 45), new Vec3(0, 0, 1), ToolShape.Flat, 5, 20);
    double removed = v0 - s.Volume(), expected = (50 * 10 + Math.PI * 25) * 5;
    Check(Math.Abs(removed - expected) / expected < 0.05, $"flat slot removal {removed:0} vs analytic {expected:0} mm³");
    Check(!s.IsMaterial(new Vec3(50, 50, 47)) && s.IsMaterial(new Vec3(50, 50, 44)) && s.IsMaterial(new Vec3(50, 57, 47)), "slot interior empty, floor and walls intact");
    // Side cut with a horizontal axis: ball Ø10 along Y at x=100 face, 3 mm into the wall
    double before = s.Volume();
    s.SubtractSweep(new Vec3(97, 20, 25), new Vec3(97, 80, 25), new Vec3(1, 0, 0), ToolShape.Ball, 5, 30);
    Check(before - s.Volume() > 0 && !s.IsMaterial(new Vec3(99, 50, 25)) && s.IsMaterial(new Vec3(96, 50, 25)), "side cut along a horizontal axis removes only the cutter volume");
    // Dashboard stock surface: watertight (every edge used an even number of times) and enclosing the dexel volume.
    var (mp, mi) = s.ToMesh();
    var use = new Dictionary<(int, int), int>(); double mv = 0;
    for (int t = 0; t < mi.Length; t += 3)
    {
        for (int k = 0; k < 3; k++) { int a = mi[t + k], b2 = mi[t + (k + 1) % 3]; var key = a < b2 ? (a, b2) : (b2, a); use[key] = use.GetValueOrDefault(key) + 1; }
        Vec3 P(int i) => new(mp[3 * i], mp[3 * i + 1], mp[3 * i + 2]);
        mv += P(mi[t]).Dot(P(mi[t + 1]).Cross(P(mi[t + 2]))) / 6;
    }
    Check(use.Values.All(c => c % 2 == 0) && Math.Abs(Math.Abs(mv) - s.Volume()) / s.Volume() < 0.01, $"stock surface mesh is watertight, volume {Math.Abs(mv):0} vs {s.Volume():0} mm³");
    var top = s.TopMap(Frame.FromAxis(new Vec3(0, 0, 1)), 0, 0, 2, 51, 51);
    Check(top[25, 25] <= 45 + 2.5 && top[25, 25] >= 44.9 && top[5, 5] >= 49.9, $"top map follows the slot floor ({top[25, 25]:0.00}) and stock top ({top[5, 5]:0.00})");
}

// ------------------------------------------------------------------ step 2: multi-orientation roughing
var roughTool = new ToolDefinition(12, ToolShape.Flat, 2, 18000);
var roughCut = new CutParameters(50, 50, CutPattern.Offset, EngageMove.Plunge, RetractMove.Vertical, ClearanceMode.Automatic, 10, 1.0);
var rough = new MultiAxisRoughingCommand("Rough 3+2", mesh, roughTool, roughCut, null, 6, 0, 0, 0, 1.0);
var stock = new TriDexelStock(box, 1.0);
var sw = Stopwatch.StartNew();
var rr = MultiAxisRoughPlanner.Generate(rough, stock);
Console.WriteLine($"Roughing: {sw.Elapsed.TotalSeconds:0.0}s, {rr.Groups.Count} orientation(s), stock {rr.StartVolume / 1000:0} → {rr.EndVolume / 1000:0} cm³");
foreach (var g in rr.Groups) Console.WriteLine("   " + g.Note);
Check(rr.Groups.Count >= 3, $"several orientations used ({rr.Groups.Count})");
Check(rr.EndVolume < rr.StartVolume * 0.6, $"removed {(1 - rr.EndVolume / rr.StartVolume):P0} of the block");
// Gouge check: drop a flat cutter in each group's frame; every CL point must sit at or above contact + allowance.
{
    int checkedPts = 0, bad = 0; double worst = 0;
    foreach (var g in rr.Groups)
    {
        var f = Frame.FromAxis(g.Axis); var local = part.Transformed(f);
        var bins = new TriangleBins(local, 6); var dc = new DropCutter(bins, ToolShape.Flat, roughTool.Radius);
        var stamp = bins.NewStamp(); int sv = 0;
        foreach (var path in g.Paths)
            for (int k = 0; k < path.Count; k += 3)
            {
                var l = f.ToLocal(Vec3.From(path[k]));
                double z = dc.Drop(l.X, l.Y, stamp, ref sv, out _);
                if (!double.IsFinite(z)) continue;
                checkedPts++;
                double gap = l.Z - (z + roughCut.StockToLeave);
                worst = Math.Min(worst, gap);
                if (gap < -0.35) bad++;      // allowance tolerance: roughing grid is 1.8 mm
            }
    }
    Check(bad == 0, $"roughing never cuts into part+allowance ({checkedPts} CL points, worst {worst:0.00} mm)");
}
// Material preservation: points deep inside the part must still be material in the stock.
{
    var rnd = new Random(1); int inside = 0, lost = 0;
    for (int i = 0; i < 20000; i++)
    {
        var p = new Vec3(box.MinX + rnd.NextDouble() * (box.MaxX - box.MinX), box.MinY + rnd.NextDouble() * (box.MaxY - box.MinY), box.MinZ + rnd.NextDouble() * (box.MaxZ - box.MinZ));
        if (!InsideDeep(p, 0.9)) continue;
        inside++; if (!stock.IsMaterial(p)) lost++;
    }
    Check(lost == 0, $"no part material removed from stock ({inside} interior samples)");
}

// ------------------------------------------------------------------ step 3: region finishing
var finTool = new ToolDefinition(6, ToolShape.Ball, 2, 20000);
var finCut = new CutParameters(15, 50, CutPattern.Linear, EngageMove.Plunge, RetractMove.Vertical, ClearanceMode.Automatic, 10, 0);
var fin = new RegionFinishingCommand("Finish 3+2", mesh, finTool, finCut, null, 6, 75, 0, 1);
sw.Restart();
var fr = RegionFinishPlanner.Generate(fin);
Console.WriteLine($"Finishing: {sw.Elapsed.TotalSeconds:0.0}s, {fr.Groups.Count} region(s), contacted {fr.CoveredAreaFraction:P1}, unreachable {fr.UnreachableAreaFraction:P1}");
foreach (var g in fr.Groups) Console.WriteLine("   " + g.Note);
Check(fr.CoveredAreaFraction > 0.8, "most of the surface is contacted");
// Independent gouge check: ball centre (tip + r*axis) distance to the mesh must be >= r.
{
    var bins = new TriangleBins(part, 6); double worst = double.MaxValue; int n = 0;
    foreach (var g in fr.Groups)
        foreach (var path in g.Paths)
            for (int k = 0; k < path.Count; k += 2)
            {
                var c = Vec3.From(path[k]) + g.Axis * finTool.Radius; double best = double.MaxValue;
                // Brute force over all faces near the centre (bins are XY only, so test a generous box and all z).
                for (int f = 0; f < part.Count; f++)
                {
                    var a = part.A(f);
                    if (Math.Abs(a.X - c.X) > 10 || Math.Abs(a.Y - c.Y) > 10 || Math.Abs(a.Z - c.Z) > 10) continue;
                    best = Math.Min(best, PointTriangleDistance(c, part.A(f), part.B(f), part.C(f)));
                }
                worst = Math.Min(worst, best); n++;
                if (n > 4000) goto done;
            }
    done:
    Check(worst >= finTool.Radius - 0.01, $"ball finishing is gouge-free on {n} sampled CL points (closest centre {worst:0.000} mm, radius {finTool.Radius})");
}

// ------------------------------------------------------------------ patch segmentation (k-means on normals + border smoothing)
{
    double wall = 75, cosWall = Math.Cos(wall * Math.PI / 180), eps = Math.Max(1e-3, part.MeanEdgeLength() * 0.05);
    sw.Restart();
    var raw = PatchSegmenter.Segment(part, wall, 8, 0, 16 * 9, null);
    var seg = PatchSegmenter.Segment(part, wall, 8, 16, 16 * 9, null, 0.5);
    Console.WriteLine($"Segmentation: {sw.Elapsed.TotalSeconds:0.0}s for both runs; {seg.Axes.Count} patches; {seg.Log}");
    Console.WriteLine($"   smoothing off: mean tilt {raw.MeanTiltDegrees:0.0}°, max {raw.MaxTiltDegrees:0.0}°, border {raw.BorderLength:0} mm");
    Console.WriteLine($"   smoothing 16 mm + position 0.5: mean tilt {seg.MeanTiltDegrees:0.0}°, max {seg.MaxTiltDegrees:0.0}°, border {seg.BorderLength:0} mm (before smoothing {seg.BorderLengthBeforeSmoothing:0})");
    // Previous F1 assignment for comparison: greedy area cover over candidates + most-head-on + 3 majority passes.
    var cands = OrientationPlanner.Candidates(64); var acc = cands.Select(d => RegionFinishPlanner.Accessible(part, d, cosWall, eps)).ToList();
    var chosen = new List<int>(); var cov = new bool[part.Count];
    for (int k = 0; k < 6; k++)
    {
        int best = -1; double bg = 0;
        for (int c = 0; c < cands.Count; c++) { if (chosen.Contains(c)) continue; double g = 0; for (int f = 0; f < part.Count; f++) if (acc[c][f] && !cov[f]) g += part.Areas[f]; if (g > bg) { bg = g; best = c; } }
        if (best < 0 || bg < 0.002 * part.Areas.Sum()) break; chosen.Add(best); for (int f = 0; f < part.Count; f++) cov[f] |= acc[best][f];
    }
    var oldLabels = PatchSegmenter.Assign(part, chosen.Select(c => cands[c]).ToList(), chosen.Select(c => acc[c]).ToList());
    double oldTilt = 0, area = 0;
    for (int f = 0; f < part.Count; f++) if (oldLabels[f] >= 0) { oldTilt += Math.Acos(Math.Clamp(part.Normals[f].Dot(cands[chosen[oldLabels[f]]]), -1, 1)) * 180 / Math.PI * part.Areas[f]; area += part.Areas[f]; }
    Console.WriteLine($"   previous greedy F1: {chosen.Count} patches, mean tilt {oldTilt / area:0.0}°, border {PatchSegmenter.BorderLength(part, oldLabels):0} mm (before its majority filter)");
    bool admissible = true;
    for (int f = 0; f < part.Count; f++)
    {
        int c = seg.Labels[f]; if (c < 0) continue;
        if (part.Normals[f].Dot(seg.Axes[c]) < cosWall - 1e-9 || !seg.Admissible[c][f]) admissible = false;
    }
    Check(admissible, "every patch face is within the wall limit and in line of sight of its axis");
    Check(seg.Axes.All(a => a.Z >= -1e-9), "no patch axis points below the horizon");
    Check(seg.BorderLength < raw.BorderLength * 0.9, $"border smoothing shortens patch borders ({raw.BorderLength:0} → {seg.BorderLength:0} mm)");
    Check(raw.MeanTiltDegrees <= oldTilt / area, $"k-means tilt before smoothing ({raw.MeanTiltDegrees:0.0}°) no worse than greedy ({oldTilt / area:0.0}°)");
    Check(seg.UnreachableAreaFraction < 0.01, $"unreachable area {seg.UnreachableAreaFraction:P2}");
}

// ------------------------------------------------------------------ full job through the planner (JSON round trip)
{
    var job = new CamJobRequest(new List<CamCommand> { rough, fin }, new StockDefinition(box), new MachineSetup());
    var json = System.Text.Json.JsonSerializer.Serialize(job);
    var back = System.Text.Json.JsonSerializer.Deserialize<CamJobRequest>(json)!;
    sw.Restart();
    var res = CamJobPlanner.Generate(back);
    var outJson = System.Text.Json.JsonSerializer.Serialize(res);
    Console.WriteLine($"Job: {sw.Elapsed.TotalSeconds:0.0}s, {res.Paths.Count} motions, axes {res.MotionAxes?.Count}, preview {res.StockPreview?.Count} segments, result JSON {outJson.Length / 1e6:0.0} MB");
    Console.WriteLine("   " + res.Summary);
    Check(res.MotionAxes!.Count == res.Paths.Count, "one tool axis per motion");
    // Every clearance point of an oriented motion must lie outside the stock box (in its frame).
    bool clearOk = true;
    for (int m = 0; m < res.Paths.Count; m++)
    {
        var w = Vec3.From(res.MotionAxes[m]); double top = Frame.Corners(box).Max(c => c.Dot(w));
        if (Vec3.From(res.Paths[m][0]).Dot(w) < top - 1e-6) clearOk = false;
    }
    Check(clearOk, "each motion starts above the stock along its own tool axis");
}

Console.WriteLine(failures == 0 ? "ALL CHECKS PASSED" : $"{failures} CHECK(S) FAILED");
return failures == 0 ? 0 : 1;

static double PointTriangleDistance(Vec3 p, Vec3 a, Vec3 b, Vec3 c)
{
    // Ericson, Real-Time Collision Detection 5.1.5
    var ab = b - a; var ac = c - a; var ap = p - a;
    double d1 = ab.Dot(ap), d2 = ac.Dot(ap); if (d1 <= 0 && d2 <= 0) return (p - a).Length;
    var bp = p - b; double d3 = ab.Dot(bp), d4 = ac.Dot(bp); if (d3 >= 0 && d4 <= d3) return (p - b).Length;
    double vc = d1 * d4 - d3 * d2; if (vc <= 0 && d1 >= 0 && d3 <= 0) { double v = d1 / (d1 - d3); return (p - (a + ab * v)).Length; }
    var cp = p - c; double d5 = ab.Dot(cp), d6 = ac.Dot(cp); if (d6 >= 0 && d5 <= d6) return (p - c).Length;
    double vb = d5 * d2 - d1 * d6; if (vb <= 0 && d2 >= 0 && d6 <= 0) { double w = d2 / (d2 - d6); return (p - (a + ac * w)).Length; }
    double va = d3 * d6 - d5 * d4; if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { double w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); return (p - (b + (c - b) * w)).Length; }
    double denom = 1 / (va + vb + vc); double vv = vb * denom, ww = vc * denom;
    return (p - (a + ab * vv + ac * ww)).Length;
}
