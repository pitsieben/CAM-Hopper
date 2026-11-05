namespace CamGh.Core;

public sealed record SceneTool(double Diameter, string Shape, int Flutes, double Rpm, double CuttingSpeed, double Reach);
public sealed record SceneOperation(int Index, string Name, string Type, SceneTool Tool, int Paths, int Points, double CutLength,
    double RemovedVolume, int Orientations, int Patches, string Message);
public sealed record SceneFrame(int AtPoint, double Volume);
public sealed record SceneInfo(bool HasJob, string CreatedUtc, double Seconds, string Summary, StockBounds? Stock, double StartVolume,
    double RemainingVolume, List<SceneOperation> Operations, List<double> SnapshotVolumes, int Motions, int PartTriangles,
    bool FramesReady = false, List<SceneFrame>? Frames = null);

/// <summary>
/// Everything the dashboard shows about the last job: summary data as JSON and geometry as compact little-endian binary
/// blobs (meshes, cutting paths, linked motions with tool axes, the remaining-stock surface before and after each operation).
/// Stock meshes are built on first request and cached.
/// </summary>
public sealed class JobScene
{
    public CamJobRequest Request { get; }
    public CamJobResult Result { get; }
    public IReadOnlyList<TriDexelStock> Snapshots { get; }
    public DateTime CreatedUtc { get; }
    public double Seconds { get; }
    private readonly byte[]?[] _stockMeshes;
    private List<(int AtPoint, TriDexelStock Stock)>? _frames;
    private byte[]?[] _frameMeshes = Array.Empty<byte[]?>();
    private List<SceneFrame>? _frameInfo;
    private readonly object _gate = new();

    public JobScene(CamJobRequest request, CamJobResult result, IReadOnlyList<TriDexelStock> snapshots, double seconds)
    {
        Request = request; Result = result; Snapshots = snapshots; Seconds = seconds; CreatedUtc = DateTime.UtcNow;
        _stockMeshes = new byte[]?[snapshots.Count];
    }

    public SceneInfo Info()
    {
        var volumes = Snapshots.Select(s => Math.Round(s.Volume(), 1)).ToList();
        var ops = new List<SceneOperation>();
        for (int i = 0; i < Result.Operations.Count; i++)
        {
            var op = Result.Operations[i]; var cmd = i < Request.Commands.Count ? Request.Commands[i] : null;
            var t = cmd?.Tool;
            double reach = cmd is MultiAxisRoughingCommand m && m.ToolReach > 0 ? m.ToolReach : (t?.Diameter ?? 0) * 4;
            var tool = t is null ? new SceneTool(0, "Flat", 0, 0, 0, 0) : new SceneTool(t.Diameter, t.Shape.ToString(), t.Flutes, t.Rpm, Math.Round(t.CuttingSpeedMetresPerMinute, 1), reach);
            double removed = i + 1 < volumes.Count ? Math.Max(0, volumes[i] - volumes[i + 1]) : 0;
            int orientations = op.PathAxes is { } a ? a.Select(p => (Math.Round(p.X, 4), Math.Round(p.Y, 4), Math.Round(p.Z, 4))).Distinct().Count() : 1;
            int patches = op.PatchAxes?.Count ?? 0;
            ops.Add(new SceneOperation(i, op.Name, op.Type, tool, op.Quality.PathCount, op.Quality.PointCount, Math.Round(op.Quality.TotalCutLength, 1),
                Math.Round(removed, 1), orientations, patches, op.Message));
        }
        List<SceneFrame>? frames; lock (_gate) frames = _frameInfo;
        return new SceneInfo(true, CreatedUtc.ToString("o"), Math.Round(Seconds, 2), Result.Summary, Request.Stock.Bounds,
            Math.Round(Result.StartStockVolume, 1), Math.Round(Result.RemainingStockVolume, 1), ops, volumes, Result.Paths.Count, PartMesh().Length / 3,
            frames is not null, frames);
    }

    /// <summary>Part mesh of the first operation: int32 vertexCount, int32 triangleCount, float32 xyz…, int32 indices….</summary>
    public byte[] PartMeshBytes()
    {
        var mesh = Request.Commands.Count > 0 ? Request.Commands[0].Mesh : new MeshInput(Array.Empty<double[]>(), Array.Empty<int[]>());
        var positions = mesh.Vertices.SelectMany(v => new[] { (float)v[0], (float)v[1], (float)v[2] }).ToArray();
        var indices = PartMesh();
        return MeshBytes(positions, indices);
    }

    private int[] PartMesh()
    {
        if (Request.Commands.Count == 0) return Array.Empty<int>();
        var mesh = Request.Commands[0].Mesh;
        return mesh.Faces.Where(f => f.Length >= 3).SelectMany(f => new[] { f[0], f[1], f[2] }).ToArray();
    }

    /// <summary>Remaining stock surface: snapshot 0 = before the first operation, i = after operation i-1.</summary>
    public byte[]? StockMeshBytes(int index)
    {
        if (index < 0 || index >= Snapshots.Count) return null;
        lock (_gate)
        {
            if (_stockMeshes[index] is { } cached) return cached;
            var (positions, indices) = Snapshots[index].ToMesh();
            return _stockMeshes[index] = MeshBytes(positions, indices);
        }
    }

    /// <summary>Builds the playback frames (stock copies along the motion timeline). Meant to run in the background.</summary>
    public void BuildFrames(int count = 24)
    {
        if (Snapshots.Count == 0) return;
        var frames = CamJobPlanner.ReplayFrames(Request, Result, Snapshots[0], count);
        var info = frames.Select(f => new SceneFrame(f.AtPoint, Math.Round(f.Stock.Volume(), 1))).ToList();
        lock (_gate) { _frames = frames; _frameMeshes = new byte[]?[frames.Count]; _frameInfo = info; }
    }

    /// <summary>Remaining stock surface at playback frame <paramref name="index"/>; null until frames are built.</summary>
    public byte[]? FrameMeshBytes(int index)
    {
        TriDexelStock stock;
        lock (_gate)
        {
            if (_frames is null || index < 0 || index >= _frames.Count) return null;
            if (_frameMeshes[index] is { } cached) return cached;
            stock = _frames[index].Stock;
        }
        var (positions, indices) = stock.ToMesh();
        var bytes = MeshBytes(positions, indices);
        lock (_gate) _frameMeshes[index] = bytes;
        return bytes;
    }

    /// <summary>Cutting paths of one operation: int32 pathCount, int32 counts…, float32 xyz….</summary>
    public byte[]? PathBytes(int operation)
    {
        if (operation < 0 || operation >= Result.Operations.Count) return null;
        return PolylineBytes(Result.Operations[operation].Paths, null, null);
    }

    /// <summary>Linked motions: int32 count, int32 point counts…, int32 operation index…, float32 axis xyz…, float32 points….</summary>
    public byte[] MotionBytes() => PolylineBytes(Result.Paths, Result.MotionOperations ?? Enumerable.Repeat(0, Result.Paths.Count).ToList(),
        Result.MotionAxes ?? Enumerable.Repeat(new Point3(0, 0, 1), Result.Paths.Count).ToList());

    public static byte[] MeshBytes(float[] positions, int[] indices)
    {
        using var ms = new MemoryStream(8 + positions.Length * 4 + indices.Length * 4);
        using var w = new BinaryWriter(ms);
        w.Write(positions.Length / 3); w.Write(indices.Length / 3);
        foreach (var f in positions) w.Write(f);
        foreach (var i in indices) w.Write(i);
        w.Flush(); return ms.ToArray();
    }

    private static byte[] PolylineBytes(List<List<Point3>> paths, List<int>? ops, List<Point3>? axes)
    {
        using var ms = new MemoryStream();
        using var w = new BinaryWriter(ms);
        w.Write(paths.Count);
        foreach (var p in paths) w.Write(p.Count);
        if (ops is not null) foreach (var o in ops) w.Write(o);
        if (axes is not null) foreach (var a in axes) { w.Write((float)a.X); w.Write((float)a.Y); w.Write((float)a.Z); }
        foreach (var p in paths) foreach (var q in p) { w.Write((float)q.X); w.Write((float)q.Y); w.Write((float)q.Z); }
        w.Flush(); return ms.ToArray();
    }
}
