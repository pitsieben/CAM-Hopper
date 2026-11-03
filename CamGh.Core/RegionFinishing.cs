namespace CamGh.Core;

/// <summary>FaceLabels and FaceTilt are indexed like the input mesh faces; -1 marks faces no patch can reach (or dropped degenerate faces).</summary>
public sealed record RegionFinishResult(List<OrientedPaths> Groups, double CoveredAreaFraction, double UnreachableAreaFraction, string Method,
    int[] FaceLabels, double[] FaceTilt, List<Vec3> PatchAxes, PatchSegmentation Segmentation, List<double[]> PatchTriangles);

/// <summary>
/// F1 finishing: split the part surface into patches that one fixed tool axis can reach, then machine each patch
/// with gouge-free parallel lanes from that axis (3+2).
/// <list type="number">
/// <item>Patches: <see cref="PatchSegmenter"/> — spherical k-means on face normals restricted to faces within the
/// wall-angle limit and in line of sight of the spindle, k grown until the surface is covered and then merged down,
/// borders relaxed at a physical radius and small islands removed. Directions may also be given.</item>
/// <item>Each patch is grown by a few face rings (only where admissible) so neighbouring patches overlap.</item>
/// <item>Lanes run along the patch's long direction; each sample is a drop-cutter against the WHOLE part, so the
/// cutter never gouges neighbouring patches; a sample is kept when its contact face is in the patch.</item>
/// </list>
/// </summary>
public static class RegionFinishPlanner
{
    public static RegionFinishResult Generate(RegionFinishingCommand cmd)
    {
        var tool = cmd.Tool; double r = tool.Radius;
        double stepover = tool.Diameter * cmd.Parameters.StepoverPercent / 100.0;
        if (!(stepover > 0)) throw new ArgumentException($"{cmd.Name}: stepover must be positive.");
        double leave = Math.Max(0, cmd.Parameters.StockToLeave);
        double step = cmd.SampleStep > 0 ? cmd.SampleStep : Math.Clamp(Math.Min(stepover, r * 0.25), 0.1, 1.0);
        double cosWall = Math.Cos(Math.Clamp(cmd.MaxWallAngleDegrees, 5, 89) * Math.PI / 180);
        var part = TriMesh.From(cmd.Mesh);
        double totalArea = part.Areas.Sum();
        double eps = Math.Max(1e-3, part.MeanEdgeLength() * 0.05);

        // ---- patches: accessibility-constrained spherical k-means on face normals + border smoothing
        var given = cmd.Directions is { Count: > 0 } ? cmd.Directions.Select(Vec3.From).ToList() : null;
        double minIsland = Math.Max(16 * r * r, 0.0005 * totalArea);
        var seg = PatchSegmenter.Segment(part, cmd.MaxWallAngleDegrees, Math.Max(1, cmd.MaxOrientations), Math.Max(0, cmd.BorderSmoothing), minIsland, given,
            Math.Max(0, cmd.PositionWeight));
        var region = seg.Labels; var nb = part.Neighbours();
        double unreachable = seg.UnreachableAreaFraction;

        // ---- paths per region
        var groups = new List<OrientedPaths>();
        var touched = new bool[part.Count];
        for (int c = 0; c < seg.Axes.Count; c++)
        {
            var axis = seg.Axes[c];
            // Footprint: the patch's faces grown by the overlap rings plus one (smooth borders may cross into neighbours).
            var allowed = new bool[part.Count];
            for (int f = 0; f < part.Count; f++) allowed[f] = region[f] == c;
            for (int ring = 0; ring < Math.Max(0, cmd.OverlapRings) + 1; ring++)
            {
                var grow = (bool[])allowed.Clone();
                for (int f = 0; f < part.Count; f++) if (allowed[f]) foreach (var g in nb[f]) if (seg.Admissible[c][g]) grow[g] = true;
                allowed = grow;
            }
            if (!allowed.Any(x => x)) continue;
            // Membership at the exact contact point: inside the smooth border, or within the overlap margin of it.
            // Without smoothing (no field) the face rings define the overlap as before.
            var ringFaces = allowed;
            if (seg.Field is null)
            {
                ringFaces = new bool[part.Count];
                for (int f = 0; f < part.Count; f++) ringFaces[f] = region[f] == c;
                for (int ring = 0; ring < Math.Max(0, cmd.OverlapRings); ring++)
                {
                    var grow = (bool[])ringFaces.Clone();
                    for (int f = 0; f < part.Count; f++) if (ringFaces[f]) foreach (var g in nb[f]) if (seg.Admissible[c][g]) grow[g] = true;
                    ringFaces = grow;
                }
            }
            double overlap = 0.25 * Math.Max(0, cmd.OverlapRings);
            int patch = c; var rf = ringFaces;
            bool Accept(int face, double u, double v, double w) =>
                seg.Field is null ? rf[face] : seg.Margin(face, u, v, w, patch) >= -overlap;
            var paths = RegionLanes(part, axis, allowed, tool.Shape, r, leave, stepover, step, touched, Accept);
            int faces = allowed.Count(x => x);
            groups.Add(new OrientedPaths(axis, paths, 0,
                $"Patch {c + 1}, axis ({axis.X:0.###}, {axis.Y:0.###}, {axis.Z:0.###}): {faces} face(s), {paths.Count} lane path(s)."));
        }
        double coveredArea = Enumerable.Range(0, part.Count).Where(f => touched[f]).Sum(f => part.Areas[f]) / totalArea;
        int inputFaces = cmd.Mesh.Faces.Length;
        var faceLabels = Enumerable.Repeat(-1, inputFaces).ToArray(); var faceTilt = Enumerable.Repeat(-1.0, inputFaces).ToArray();
        for (int f = 0; f < part.Count; f++) { faceLabels[part.SourceFace[f]] = region[f]; faceTilt[part.SourceFace[f]] = seg.TiltDegrees[f]; }
        // Preview: faces split where the smooth borders cross them; one flat xyz list per patch, unreachable last.
        var preview = Enumerable.Range(0, seg.Axes.Count + 1).Select(_ => new List<double>()).ToList();
        foreach (var t in PatchSegmenter.Refine(part, seg, 2, false))
        {
            var list = preview[t.Label < 0 ? seg.Axes.Count : t.Label];
            foreach (var p in new[] { t.A, t.B, t.C }) { list.Add(Math.Round(p.X, 4)); list.Add(Math.Round(p.Y, 4)); list.Add(Math.Round(p.Z, 4)); }
        }
        return new RegionFinishResult(groups.Where(g => g.Paths.Count > 0).ToList(), coveredArea, unreachable,
            $"3+2 region finishing: {seg.Axes.Count} patch(es) from k-means on face normals and position (position weight {cmd.PositionWeight:g}; wall limit {cmd.MaxWallAngleDegrees:g}°, line of sight to the spindle), "
            + $"mean tilt {seg.MeanTiltDegrees:0.#}°, max {seg.MaxTiltDegrees:0.#}°; patch borders {seg.BorderLengthBeforeSmoothing:0} mm along mesh edges → {seg.BorderLength:0} mm smooth; "
            + $"stepover {stepover:g} mm, sample step {step:g} mm, gouge-free drop-cutter against the whole part. ({seg.Log})",
            faceLabels, faceTilt, seg.Axes, seg, preview.Select(l => l.ToArray()).ToList());
    }

    /// <summary>Face reachable from axis d: facing the axis within the wall limit and not shadowed by the part.</summary>
    public static bool[] Accessible(TriMesh part, Vec3 d, double cosWall, double eps)
    {
        var frame = Frame.FromAxis(d);
        var local = part.Transformed(frame);
        var bins = new TriangleBins(local, Math.Max(local.MeanEdgeLength() * 2, 1e-3));
        var result = new bool[part.Count];
        for (int f = 0; f < part.Count; f++)
        {
            if (part.Normals[f].Dot(frame.W) < cosWall) continue;
            var c = local.Centroid(f);
            result[f] = !bins.IsOccludedAbove(c, eps, f);
        }
        return result;
    }

    private static List<List<Point3>> RegionLanes(TriMesh part, Vec3 axis, bool[] allowed, ToolShape shape, double r, double leave,
        double stepover, double step, bool[] touched, Func<int, double, double, double, bool> accept)
    {
        var frame = Frame.FromAxis(axis);
        var local = part.Transformed(frame);
        var bins = new TriangleBins(local, Math.Max(r, local.MeanEdgeLength()));
        // A ball offset by the allowance: drop a ball of radius r+leave, then report the tip of the real ball.
        var cutter = new DropCutter(bins, shape, shape == ToolShape.Ball ? r + leave : r);

        // Lane direction = principal direction of the region's footprint (fewest lanes, longest runs).
        var pts = new List<Vec3>();
        for (int f = 0; f < local.Count; f++) if (allowed[f]) { pts.Add(local.A(f)); pts.Add(local.B(f)); pts.Add(local.C(f)); }
        double mx = pts.Average(p => p.X), my = pts.Average(p => p.Y), sxx = 0, syy = 0, sxy = 0;
        foreach (var p in pts) { sxx += (p.X - mx) * (p.X - mx); syy += (p.Y - my) * (p.Y - my); sxy += (p.X - mx) * (p.Y - my); }
        double ang = 0.5 * Math.Atan2(2 * sxy, sxx - syy), ux = Math.Cos(ang), uy = Math.Sin(ang), vx = -uy, vy = ux;
        double amin = double.MaxValue, amax = double.MinValue, bmin = double.MaxValue, bmax = double.MinValue;
        foreach (var p in pts)
        {
            double a = p.X * ux + p.Y * uy, b = p.X * vx + p.Y * vy;
            amin = Math.Min(amin, a); amax = Math.Max(amax, a); bmin = Math.Min(bmin, b); bmax = Math.Max(bmax, b);
        }
        amin -= r; amax += r;
        int lanes = Math.Max(1, (int)Math.Ceiling((bmax - bmin) / stepover) + 1);
        int samples = Math.Max(2, (int)Math.Ceiling((amax - amin) / step) + 1);
        var laneRuns = new List<List<Point3>>[lanes];
        var hitFaces = new List<int>[lanes];
        Parallel.For(0, lanes, () => (Stamp: bins.NewStamp(), Value: 0), (lane, _, state) =>
        {
            double b = bmin + Math.Min(lane * stepover, bmax - bmin);
            var runs = new List<List<Point3>>(); var run = new List<Point3>(); var faces = new List<int>();
            int stampValue = state.Value;
            for (int s = 0; s < samples; s++)
            {
                double a = amin + s * step;
                double x = a * ux + b * vx, y = a * uy + b * vy;
                double z = cutter.Drop(x, y, state.Stamp, ref stampValue, out int face);
                bool keep = face >= 0 && double.IsFinite(z) && allowed[face];
                if (keep)
                {
                    // Contact point = point of the contact face closest to the cutter's reference point (ball centre / tip).
                    var reference = new Vec3(x, y, shape == ToolShape.Ball ? z + cutter.Radius : z);
                    var (bu, bv, bw) = TriMath.ClosestBarycentric(reference, local.A(face), local.B(face), local.C(face));
                    keep = accept(face, bu, bv, bw);
                }
                if (keep)
                {
                    // Ball: the (r+leave) ball's centre is the real ball's centre, so the real tip is leave higher.
                    // Flat: allowance is applied along the axis (approximate on sloped faces).
                    z += leave;
                    run.Add(frame.ToWorld(x, y, z).ToPoint()); faces.Add(face);
                }
                else if (run.Count > 0) { if (run.Count >= 2) runs.Add(run); run = new List<Point3>(); }
            }
            if (run.Count >= 2) runs.Add(run);
            // Alternate lane direction (zigzag) to shorten the links between lanes.
            if (lane % 2 == 1) { runs.Reverse(); foreach (var rr in runs) rr.Reverse(); }
            laneRuns[lane] = runs; hitFaces[lane] = faces;
            state.Value = stampValue;
            return state;
        }, _ => { });
        foreach (var faces in hitFaces) if (faces is not null) foreach (var f in faces) touched[f] = true;
        return laneRuns.Where(l => l is not null).SelectMany(l => l).Select(p => CamJobPlanner.Simplify(p, 0.002)).ToList();
    }
}
