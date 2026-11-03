namespace CamGh.Core;

/// <summary>A triangle of the border-refined patch mesh: label -1 = unreachable, Face = source face in the part mesh.</summary>
public readonly record struct RefinedTri(Vec3 A, Vec3 B, Vec3 C, int Label, int Face);

/// <summary>
/// Face → patch labels (-1 = no admissible axis), one tool axis per patch, quality statistics and the smooth label
/// field. Field[patch][vertex] is the diffused indicator of the patch; interpolated across a triangle, its arg-max
/// gives patch borders that cut through faces instead of following mesh edges (null = labels are per face).
/// </summary>
public sealed record PatchSegmentation(
    int[] Labels,
    List<Vec3> Axes,
    List<bool[]> Admissible,
    double[] TiltDegrees,
    double MeanTiltDegrees,
    double MaxTiltDegrees,
    double UnreachableAreaFraction,
    double BorderLength,
    double BorderLengthBeforeSmoothing,
    string Log,
    TriMesh Mesh,
    double[][]? Field)
{
    /// <summary>Patch at barycentric point (u,v,w) of a face, restricted to patches admissible for that face.</summary>
    public int LabelAt(int face, double u, double v, double w) => PatchSegmenter.LabelAtPoint(Mesh, Field, Admissible, Labels, face, u, v, w);

    /// <summary>
    /// Field of the patch minus the strongest other admissible patch at a barycentric point (positive inside the patch,
    /// zero on its border). -∞ when the patch is not admissible for the face. Without a field: +1 inside, -1 outside.
    /// </summary>
    public double Margin(int face, double u, double v, double w, int patch)
    {
        if (!Admissible[patch][face]) return double.NegativeInfinity;
        if (Field is null) return Labels[face] == patch ? 1 : -1;
        int a = Mesh.Tris[3 * face], b = Mesh.Tris[3 * face + 1], c = Mesh.Tris[3 * face + 2];
        double own = Field[patch][a] * u + Field[patch][b] * v + Field[patch][c] * w, other = 0;
        for (int p = 0; p < Field.Length; p++)
            if (p != patch && Admissible[p][face]) other = Math.Max(other, Field[p][a] * u + Field[p][b] * v + Field[p][c] * w);
        return own - other;
    }
}

/// <summary>
/// Splits a part surface into machining patches, each finished from one fixed tool axis.
/// <list type="number">
/// <item><b>Admissibility.</b> A face may belong to an axis only when its normal is within the wall-angle limit of
/// the axis and the line from the face towards the spindle is not blocked by the part. Axes never point below the
/// horizon (robot above the table).</item>
/// <item><b>Joint k-means on normal and position.</b> A face goes to the admissible patch with the lowest cost
/// (1 − cos tilt) + positionWeight · (distance to the patch centre / half the part diagonal)². Each patch axis moves to
/// the area-weighted mean normal of its faces and its centre to their mean position (Lloyd iterations; an axis move is
/// shortened if it would lose coverage). Position weight 0 clusters on normals only; larger values give compact,
/// less fragmented patches.</item>
/// <item><b>Automatic k.</b> Axes are added until the surface is covered, then redundant patches are merged.</item>
/// <item><b>Border smoothing = curvature flow.</b> Merriman–Bence–Osher threshold dynamics on the mesh: each patch
/// indicator is diffused with the cotangent heat kernel and every vertex takes the strongest admissible patch, repeated
/// a few times. This moves every patch border by its curvature, the same thing relaxing the border polylines with
/// edge-length (shortening) goals does, but without tracking border topology: junctions, closed loops and vanishing
/// patches are handled automatically, and the result does not depend on how the mesh is tessellated. Features smaller
/// than the smoothing radius disappear.</item>
/// <item><b>Sub-face borders.</b> The final diffused indicators are kept as a per-vertex field; interpolating it inside
/// each triangle places borders between mesh edges, so they are smooth curves instead of staircases.</item>
/// </list>
/// </summary>
public static class PatchSegmenter
{
    private const int MbIterations = 6;

    public static PatchSegmentation Segment(TriMesh part, double maxWallDegrees, int maxPatches, double smoothingRadius, double minIslandArea,
        IReadOnlyList<Vec3>? fixedAxes = null, double positionWeight = 0)
    {
        double cosWall = Math.Cos(Math.Clamp(maxWallDegrees, 5, 89) * Math.PI / 180);
        double eps = Math.Max(1e-3, part.MeanEdgeLength() * 0.05);
        double total = part.Areas.Sum();
        int n = part.Count;
        var log = new List<string>();
        bool[] Acc(Vec3 d) => RegionFinishPlanner.Accessible(part, d, cosWall, eps);
        var (bmin, bmax) = part.Bounds();
        double halfDiag = Math.Max(1e-9, 0.5 * (bmax - bmin).Length);
        var cen = new Vec3[n]; for (int f = 0; f < n; f++) cen[f] = part.Centroid(f);
        double wp = Math.Max(0, positionWeight);

        var axes = new List<Vec3>(); var acc = new List<bool[]>(); var centers = new List<Vec3>();
        bool fixedMode = fixedAxes is { Count: > 0 };
        if (fixedMode)
        {
            foreach (var d in fixedAxes!) { var u = d.Normalized(); if (u.Length > 0.5) { axes.Add(u); acc.Add(Acc(u)); centers.Add(new Vec3(0, 0, 0)); } }
            UpdateCenters(part, cen, Assign(part, axes, acc), centers);
            for (int it = 0; it < 8 && wp > 0; it++) UpdateCenters(part, cen, AssignW(part, cen, axes, centers, acc, wp, halfDiag), centers);
            log.Add($"{axes.Count} given axis/axes");
        }
        else
        {
            var candidates = OrientationPlanner.Candidates(96);
            var candAcc = new bool[candidates.Count][];
            Parallel.For(0, candidates.Count, k => candAcc[k] = Acc(candidates[k]));
            for (int guard = 0; guard < 4 * Math.Max(1, maxPatches); guard++)
            {
                if (axes.Count >= Math.Max(1, maxPatches)) break;
                var served = Served(acc, n);
                int best = -1; double bestGain = 0;
                for (int k = 0; k < candidates.Count; k++)
                {
                    double g = 0; for (int f = 0; f < n; f++) if (!served[f] && candAcc[k][f]) g += part.Areas[f];
                    if (g > bestGain) { bestGain = g; best = k; }
                }
                if (best < 0 || bestGain < 0.002 * total) break;
                var sum = new Vec3(0, 0, 0);
                for (int f = 0; f < n; f++) if (!served[f] && candAcc[best][f]) sum += cen[f] * part.Areas[f];
                centers.Add(sum * (1.0 / bestGain));
                axes.Add(candidates[best]); acc.Add(candAcc[best]);
                int iterations = Lloyd(part, cen, axes, centers, acc, Acc, wp, halfDiag);
                log.Add($"k={axes.Count}: {iterations} iteration(s), unreached {UnservedArea(part, acc) / total:P1}");
            }
            int before = axes.Count;
            MergeRedundant(part, axes, acc, centers);
            if (axes.Count < before) log.Add($"merged {before - axes.Count} redundant patch(es)");
        }
        if (axes.Count == 0) throw new ArgumentException("No tool axis can reach the part surface.");

        var labels = AssignW(part, cen, axes, centers, acc, wp, halfDiag);
        double borderBefore = BorderLength(part, labels);
        RemoveIslands(part, axes, acc, labels, minIslandArea);

        double[][]? field = null;
        if (smoothingRadius > 0)
        {
            labels = SmoothBordersMbo(part, axes, acc, labels, smoothingRadius, out field, out string note);
            log.Add(note);
        }

        // Patches emptied by smoothing/merging are dropped; labels and field rows are renumbered densely.
        var used = Enumerable.Range(0, axes.Count).Where(c => labels.Any(l => l == c)).ToList();
        var remap = Enumerable.Repeat(-1, axes.Count).ToArray();
        for (int i = 0; i < used.Count; i++) remap[used[i]] = i;
        for (int f = 0; f < n; f++) if (labels[f] >= 0) labels[f] = remap[labels[f]];
        axes = used.Select(c => axes[c]).ToList(); acc = used.Select(c => acc[c]).ToList();
        if (field is not null) field = used.Select(c => field[c]).ToArray();

        var tilt = new double[n]; double sumTilt = 0, sumArea = 0, maxTilt = 0, unreachable = 0;
        for (int f = 0; f < n; f++)
        {
            if (labels[f] < 0) { tilt[f] = -1; unreachable += part.Areas[f]; continue; }
            double t = Math.Acos(Math.Clamp(part.Normals[f].Dot(axes[labels[f]]), -1, 1)) * 180 / Math.PI;
            tilt[f] = t; sumTilt += t * part.Areas[f]; sumArea += part.Areas[f]; maxTilt = Math.Max(maxTilt, t);
        }
        var seg = new PatchSegmentation(labels, axes, acc, tilt, sumArea > 0 ? sumTilt / sumArea : 0, maxTilt, unreachable / total,
            0, borderBefore, string.Join("; ", log), part, field);
        return seg with { BorderLength = field is null ? BorderLength(part, labels) : SmoothBorderLength(seg) };
    }

    // ------------------------------------------------------------------ helpers

    private static bool[] Served(List<bool[]> acc, int n)
    {
        var s = new bool[n];
        foreach (var a in acc) for (int f = 0; f < n; f++) s[f] |= a[f];
        return s;
    }

    private static double UnservedArea(TriMesh part, List<bool[]> acc)
    {
        var s = Served(acc, part.Count); double u = 0;
        for (int f = 0; f < part.Count; f++) if (!s[f]) u += part.Areas[f];
        return u;
    }

    private static double ServedArea(TriMesh part, int[] labels)
    {
        double s = 0; for (int f = 0; f < part.Count; f++) if (labels[f] >= 0) s += part.Areas[f]; return s;
    }

    /// <summary>Most head-on admissible axis per face (normals only), -1 when none.</summary>
    public static int[] Assign(TriMesh part, List<Vec3> axes, List<bool[]> acc)
    {
        var labels = new int[part.Count];
        for (int f = 0; f < part.Count; f++)
        {
            labels[f] = -1; double best = double.NegativeInfinity;
            for (int c = 0; c < axes.Count; c++)
            {
                if (!acc[c][f]) continue;
                double s = part.Normals[f].Dot(axes[c]);
                if (s > best) { best = s; labels[f] = c; }
            }
        }
        return labels;
    }

    /// <summary>Lowest-cost admissible patch per face: (1 − cos tilt) + wp · (|centroid − centre| / halfDiag)².</summary>
    private static int[] AssignW(TriMesh part, Vec3[] cen, List<Vec3> axes, List<Vec3> centers, List<bool[]> acc, double wp, double halfDiag)
    {
        var labels = new int[part.Count]; double inv = 1.0 / (halfDiag * halfDiag);
        Parallel.For(0, part.Count, f =>
        {
            int best = -1; double bestCost = double.PositiveInfinity;
            for (int c = 0; c < axes.Count; c++)
            {
                if (!acc[c][f]) continue;
                double cost = 1 - part.Normals[f].Dot(axes[c]);
                if (wp > 0) { var d = cen[f] - centers[c]; cost += wp * d.Dot(d) * inv; }
                if (cost < bestCost) { bestCost = cost; best = c; }
            }
            labels[f] = best;
        });
        return labels;
    }

    private static double UpdateCenters(TriMesh part, Vec3[] cen, int[] labels, List<Vec3> centers)
    {
        var sum = new Vec3[centers.Count]; var area = new double[centers.Count];
        for (int f = 0; f < part.Count; f++) if (labels[f] >= 0) { sum[labels[f]] += cen[f] * part.Areas[f]; area[labels[f]] += part.Areas[f]; }
        double maxMove = 0;
        for (int c = 0; c < centers.Count; c++)
        {
            if (area[c] <= 0) continue;
            var m = sum[c] * (1.0 / area[c]); maxMove = Math.Max(maxMove, (m - centers[c]).Length); centers[c] = m;
        }
        return maxMove;
    }

    /// <summary>
    /// Constrained Lloyd iterations on (normal, position). Axis moves are shortened (1, ½, ¼) if the full move
    /// would leave surface unreachable, so tilt drops without trading away coverage; centres move freely.
    /// </summary>
    private static int Lloyd(TriMesh part, Vec3[] cen, List<Vec3> axes, List<Vec3> centers, List<bool[]> acc,
        Func<Vec3, bool[]> accessible, double wp, double halfDiag)
    {
        int it = 0;
        for (; it < 25; it++)
        {
            var labels = AssignW(part, cen, axes, centers, acc, wp, halfDiag);
            double servedBefore = ServedArea(part, labels);
            var target = new Vec3?[axes.Count];
            for (int c = 0; c < axes.Count; c++)
            {
                var sum = new Vec3(0, 0, 0);
                for (int f = 0; f < part.Count; f++) if (labels[f] == c) sum += part.Normals[f] * part.Areas[f];
                if (sum.Length < 1e-12) continue;
                var d = sum.Normalized();
                if (d.Z < 0) { d = new Vec3(d.X, d.Y, 0); if (d.Length < 1e-6) continue; d = d.Normalized(); }
                if (Math.Acos(Math.Clamp(d.Dot(axes[c]), -1, 1)) > 0.25 * Math.PI / 180) target[c] = d;
            }
            double centerMove = UpdateCenters(part, cen, labels, centers);
            if (target.All(t => t is null)) { if (wp <= 0 || centerMove < 0.002 * halfDiag) break; continue; }
            bool accepted = false;
            foreach (double fraction in new[] { 1.0, 0.5, 0.25 })
            {
                var moved = axes.Select((a, c) => target[c] is { } t ? (a * (1 - fraction) + t * fraction).Normalized() : a).ToList();
                var movedAcc = new bool[axes.Count][];
                Parallel.For(0, axes.Count, c => movedAcc[c] = target[c] is null ? acc[c] : accessible(moved[c]));
                var trial = movedAcc.ToList();
                if (ServedArea(part, Assign(part, moved, trial)) < servedBefore - 1e-6 * Math.Max(1, servedBefore)) continue;
                for (int c = 0; c < axes.Count; c++) { axes[c] = moved[c]; acc[c] = trial[c]; }
                accepted = true; break;
            }
            if (!accepted) break;
        }
        return it;
    }

    /// <summary>Drops patches (smallest first) whose every face is also admissible for another remaining axis.</summary>
    private static void MergeRedundant(TriMesh part, List<Vec3> axes, List<bool[]> acc, List<Vec3> centers)
    {
        bool changed = true;
        while (changed && axes.Count > 1)
        {
            changed = false;
            var labels = Assign(part, axes, acc);
            var order = Enumerable.Range(0, axes.Count).OrderBy(c => part.Areas.Where((_, f) => labels[f] == c).Sum()).ToList();
            foreach (int c in order)
            {
                bool redundant = true;
                for (int f = 0; f < part.Count && redundant; f++)
                {
                    if (labels[f] != c) continue;
                    bool other = false;
                    for (int k = 0; k < axes.Count && !other; k++) if (k != c && acc[k][f]) other = true;
                    redundant = other;
                }
                if (!redundant) continue;
                axes.RemoveAt(c); acc.RemoveAt(c); centers.RemoveAt(c); changed = true; break;
            }
        }
    }

    // ------------------------------------------------------------------ border smoothing (curvature flow)

    /// <summary>
    /// MBO threshold dynamics with the cotangent heat kernel. Returns smoothed face labels (arg-max of the final
    /// field at each face centroid, restricted to patches admissible for the face) and the per-vertex field.
    /// radius: features smaller than about this size vanish (curve-shortening time T = radius² / 2 split over the iterations).
    /// </summary>
    private static int[] SmoothBordersMbo(TriMesh part, List<Vec3> axes, List<bool[]> acc, int[] faceLabels, double radius,
        out double[][] field, out string note)
    {
        int k = axes.Count, nv = part.Vertices.Length, n = part.Count;
        var diffusion = new MeshDiffusion(part);
        double h = diffusion.MeanSpacing();
        double tau = radius * radius / (2.0 * MbIterations);
        bool clamped = false;
        if (tau < 1.5 * h * h) { tau = 1.5 * h * h; clamped = true; }   // below ~h² the flow stagnates on the mesh

        // Vertex label = patch holding most incident area. A patch is excluded at a vertex when most of the incident
        // area is inadmissible for it; the exact per-face test is applied again when borders are evaluated inside faces.
        var votes = new double[k][]; var bad = new bool[k][]; var badArea = new double[k][]; var vertexArea = new double[nv];
        for (int c = 0; c < k; c++) { votes[c] = new double[nv]; bad[c] = new bool[nv]; badArea[c] = new double[nv]; }
        for (int f = 0; f < n; f++)
        {
            for (int e = 0; e < 3; e++)
            {
                int v = part.Tris[3 * f + e]; vertexArea[v] += part.Areas[f];
                if (faceLabels[f] >= 0) votes[faceLabels[f]][v] += part.Areas[f];
                for (int c = 0; c < k; c++) if (!acc[c][f]) badArea[c][v] += part.Areas[f];
            }
        }
        for (int c = 0; c < k; c++) for (int v = 0; v < nv; v++) bad[c][v] = badArea[c][v] > 0.5 * vertexArea[v];
        var label = new int[nv];
        for (int v = 0; v < nv; v++)
        {
            int best = -1; double bv = 0;
            for (int c = 0; c < k; c++) if (votes[c][v] > bv) { bv = votes[c][v]; best = c; }
            label[v] = best;
        }

        int totalCg = 0, cgCalls = 0, iterationsRun = 0;
        for (int iter = 0; iter < MbIterations; iter++)
        {
            var phi = DiffuseIndicators(diffusion, label, k, tau, ref totalCg, ref cgCalls);
            int changes = 0;
            for (int v = 0; v < nv; v++)
            {
                int cur = label[v]; if (cur < 0) continue;
                int best = cur; double bestVal = bad[cur][v] ? -1 : phi[cur][v] + 1e-9;   // small bias keeps ties stable
                for (int c = 0; c < k; c++)
                {
                    if (bad[c][v]) continue;
                    if (phi[c][v] > bestVal) { bestVal = phi[c][v]; best = c; }
                }
                if (best != cur) { label[v] = best; changes++; }
            }
            iterationsRun = iter + 1;
            if (changes == 0) break;
        }
        field = DiffuseIndicators(diffusion, label, k, 0.5 * tau, ref totalCg, ref cgCalls);

        var result = new int[n];
        var fieldCopy = field;
        Parallel.For(0, n, f => result[f] = LabelAtPoint(part, fieldCopy, acc, faceLabels, f, 1.0 / 3, 1.0 / 3, 1.0 / 3));
        note = $"border smoothing: curvature flow radius {radius:g} mm (diffusion time {tau:0.##} × {iterationsRun}), "
            + $"{(cgCalls == 0 ? 0 : totalCg / cgCalls)} CG iterations per solve" + (clamped ? $", radius raised to the mesh scale (≈{h:0.##} mm spacing)" : "");
        return result;
    }

    private static double[][] DiffuseIndicators(MeshDiffusion diffusion, int[] label, int k, double tau, ref int totalCg, ref int calls)
    {
        int nv = label.Length; var phi = new double[k][]; var iterations = new int[k];
        Parallel.For(0, k, c =>
        {
            var u0 = new double[nv]; for (int v = 0; v < nv; v++) u0[v] = label[v] == c ? 1.0 : 0.0;
            phi[c] = new double[nv]; iterations[c] = diffusion.Diffuse(tau, u0, phi[c]);
        });
        totalCg += iterations.Sum(); calls += k;
        return phi;
    }

    /// <summary>Patch at a barycentric point of a face from the smooth field (admissible patches only); falls back to the face label.</summary>
    public static int LabelAtPoint(TriMesh part, double[][]? field, List<bool[]> acc, int[] fallback, int f, double u, double v, double w)
    {
        if (field is null) return fallback[f];
        int a = part.Tris[3 * f], b = part.Tris[3 * f + 1], c = part.Tris[3 * f + 2];
        int best = -1; double bestVal = -1;
        for (int p = 0; p < field.Length; p++)
        {
            if (!acc[p][f]) continue;
            double val = field[p][a] * u + field[p][b] * v + field[p][c] * w;
            if (val > bestVal) { bestVal = val; best = p; }
        }
        return best < 0 || bestVal < 1e-6 ? fallback[f] : best;
    }

    // ------------------------------------------------------------------ islands and border measures

    /// <summary>Connected fragments of a patch below minArea are moved to their best admissible neighbouring patch.</summary>
    private static void RemoveIslands(TriMesh part, List<Vec3> axes, List<bool[]> acc, int[] labels, double minArea)
    {
        var nb = part.NeighbourEdges();
        for (int round = 0; round < 4; round++)
        {
            var comp = Enumerable.Repeat(-1, part.Count).ToArray(); int changed = 0;
            for (int seed = 0; seed < part.Count; seed++)
            {
                if (comp[seed] >= 0 || labels[seed] < 0) continue;
                var members = new List<int>(); var queue = new Queue<int>(); queue.Enqueue(seed); comp[seed] = seed; double area = 0;
                while (queue.Count > 0)
                {
                    int f = queue.Dequeue(); members.Add(f); area += part.Areas[f];
                    foreach (var (g, _) in nb[f]) if (comp[g] < 0 && labels[g] == labels[seed]) { comp[g] = seed; queue.Enqueue(g); }
                }
                if (area >= minArea) continue;
                var border = new Dictionary<int, double>();
                foreach (int f in members) foreach (var (g, len) in nb[f]) if (labels[g] >= 0 && labels[g] != labels[seed]) border[labels[g]] = border.GetValueOrDefault(labels[g]) + len;
                foreach (var (c, _) in border.OrderByDescending(kv => kv.Value))
                {
                    if (!members.All(f => acc[c][f])) continue;
                    foreach (int f in members) labels[f] = c;
                    changed++; break;
                }
            }
            if (changed == 0) break;
        }
    }

    /// <summary>Length of patch borders measured along mesh edges.</summary>
    public static double BorderLength(TriMesh part, int[] labels)
    {
        var nb = part.NeighbourEdges(); double sum = 0;
        for (int f = 0; f < part.Count; f++) foreach (var (g, len) in nb[f]) if (labels[g] != labels[f]) sum += len;
        return sum / 2;
    }

    /// <summary>
    /// Triangles split where the smooth field places a border inside a face, labelled per sub-triangle. Used for
    /// previews (adaptive, only mixed faces) and for measuring the true border length (uniform, mixed faces and their
    /// neighbours, which keeps the edges of neighbouring triangles matched).
    /// </summary>
    public static List<RefinedTri> Refine(TriMesh part, PatchSegmentation seg, int depth, bool uniformRing)
    {
        int n = part.Count; var mixed = new bool[n];
        Parallel.For(0, n, f =>
        {
            int l0 = seg.LabelAt(f, 1, 0, 0), l1 = seg.LabelAt(f, 0, 1, 0), l2 = seg.LabelAt(f, 0, 0, 1), lc = seg.LabelAt(f, 1.0 / 3, 1.0 / 3, 1.0 / 3);
            mixed[f] = !(l0 == l1 && l1 == l2 && l2 == lc);
        });
        var split = (bool[])mixed.Clone();
        if (uniformRing) { var nb = part.Neighbours(); for (int f = 0; f < n; f++) if (mixed[f]) foreach (int g in nb[f]) split[g] = true; }
        var result = new List<RefinedTri>(n + 16 * n / 8);
        for (int f = 0; f < n; f++)
        {
            if (!split[f]) { result.Add(new RefinedTri(part.A(f), part.B(f), part.C(f), seg.LabelAt(f, 1.0 / 3, 1.0 / 3, 1.0 / 3), f)); continue; }
            Subdivide(f, new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1), depth);
        }
        return result;

        Vec3 P(int f, Vec3 b) => part.A(f) * b.X + part.B(f) * b.Y + part.C(f) * b.Z;
        void Subdivide(int f, Vec3 b0, Vec3 b1, Vec3 b2, int d)
        {
            var bc = (b0 + b1 + b2) * (1.0 / 3); int lc = seg.LabelAt(f, bc.X, bc.Y, bc.Z);
            bool cut;
            if (d == 0) cut = false;
            else if (uniformRing) cut = true;
            else cut = !(seg.LabelAt(f, b0.X, b0.Y, b0.Z) == lc && seg.LabelAt(f, b1.X, b1.Y, b1.Z) == lc && seg.LabelAt(f, b2.X, b2.Y, b2.Z) == lc);
            if (!cut) { result.Add(new RefinedTri(P(f, b0), P(f, b1), P(f, b2), lc, f)); return; }
            var m01 = (b0 + b1) * 0.5; var m12 = (b1 + b2) * 0.5; var m20 = (b2 + b0) * 0.5;
            Subdivide(f, b0, m01, m20, d - 1); Subdivide(f, m01, b1, m12, d - 1); Subdivide(f, m20, m12, b2, d - 1); Subdivide(f, m01, m12, m20, d - 1);
        }
    }

    /// <summary>
    /// Length of the smooth patch borders: inside every face where two patches meet, the length of the line where their
    /// fields are equal (linear interpolation along the face edges), plus mesh edges between whole faces of different patches.
    /// Unlike summing sub-triangle edges this does not over-count diagonal borders.
    /// </summary>
    public static double SmoothBorderLength(PatchSegmentation seg)
    {
        var part = seg.Mesh; var field = seg.Field; if (field is null) return BorderLength(part, seg.Labels);
        int n = part.Count; var nb = part.NeighbourEdges();
        var mixed = new bool[n]; var faceLabel = new int[n]; double sum = 0;
        for (int f = 0; f < n; f++)
        {
            int lc = seg.LabelAt(f, 1.0 / 3, 1.0 / 3, 1.0 / 3); faceLabel[f] = lc;
            int l0 = seg.LabelAt(f, 1, 0, 0), l1 = seg.LabelAt(f, 0, 1, 0), l2 = seg.LabelAt(f, 0, 0, 1);
            mixed[f] = !(l0 == lc && l1 == lc && l2 == lc);
            if (!mixed[f]) continue;
            var set = new[] { l0, l1, l2, lc }.Where(l => l >= 0).Distinct().ToList();
            if (set.Count < 2) continue;
            double pairSum = 0; int pairs = 0;
            for (int i = 0; i < set.Count; i++)
                for (int j = i + 1; j < set.Count; j++) { pairSum += IsoLength(part, field, f, set[i], set[j]); pairs++; }
            sum += set.Count == 2 ? pairSum : pairSum * 2.0 / set.Count;       // three-way faces: roughly two of the three pair segments are border
        }
        for (int f = 0; f < n; f++)
        {
            if (mixed[f]) continue;
            foreach (var (g, len) in nb[f]) if (!mixed[g] && faceLabel[g] != faceLabel[f]) sum += len / 2;
        }
        return sum;
    }

    private static double IsoLength(TriMesh part, double[][] field, int f, int a, int b)
    {
        var g = new double[3]; var v = new int[3];
        for (int e = 0; e < 3; e++) { v[e] = part.Tris[3 * f + e]; g[e] = field[a][v[e]] - field[b][v[e]]; }
        var hits = new List<Vec3>(2);
        for (int e = 0; e < 3; e++)
        {
            int i = e, j = (e + 1) % 3;
            if ((g[i] > 0) == (g[j] > 0) || g[i] == g[j]) continue;
            double t = g[i] / (g[i] - g[j]);
            hits.Add(part.Vertices[v[i]] + (part.Vertices[v[j]] - part.Vertices[v[i]]) * t);
        }
        return hits.Count == 2 ? (hits[0] - hits[1]).Length : 0;
    }

    /// <summary>Length of the edges shared by refined triangles with different labels (follows sub-triangle edges, so it over-counts diagonal borders).</summary>
    public static double RefinedBorderLength(List<RefinedTri> tris)
    {
        var seen = new Dictionary<(long, long, long, long, long, long), int>(tris.Count * 2); double sum = 0;
        static long Q(double x) => (long)Math.Round(x * 1e4);
        foreach (var t in tris)
        {
            foreach (var (p, q) in new[] { (t.A, t.B), (t.B, t.C), (t.C, t.A) })
            {
                var a = (Q(p.X), Q(p.Y), Q(p.Z)); var b = (Q(q.X), Q(q.Y), Q(q.Z));
                var key = a.Item1 < b.Item1 || (a.Item1 == b.Item1 && (a.Item2 < b.Item2 || (a.Item2 == b.Item2 && a.Item3 <= b.Item3)))
                    ? (a.Item1, a.Item2, a.Item3, b.Item1, b.Item2, b.Item3) : (b.Item1, b.Item2, b.Item3, a.Item1, a.Item2, a.Item3);
                if (seen.TryGetValue(key, out int other)) { if (other != t.Label) sum += (p - q).Length; }
                else seen[key] = t.Label;
            }
        }
        return sum;
    }
}
