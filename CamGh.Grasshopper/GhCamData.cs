using CamGh.Core;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace CamGh.Grasshopper;

internal static class GhCamData
{
    public static bool TryGet<T>(IGH_DataAccess da, int index, out T? value) where T : class
    {
        object raw = null!;
        if (!da.GetData(index, ref raw) || raw is null) { value = null; return false; }
        value = (raw as GH_ObjectWrapper)?.Value as T ?? raw as T;
        return value is not null;
    }
    public static List<T> GetList<T>(IGH_DataAccess da, int index) where T : class
    {
        var raw = new List<object>();
        if (!da.GetDataList(index, raw)) return [];
        return raw.Select(x => (x as GH_ObjectWrapper)?.Value as T ?? x as T).OfType<T>().ToList();
    }
    public static MeshInput ToMeshInput(Mesh mesh) => new(
        mesh.Vertices.Select(v => new[] { (double)v.X, (double)v.Y, (double)v.Z }).ToArray(),
        mesh.Faces.Select(f => new[] { f.A, f.B, f.C })
            .Concat(mesh.Faces.Where(f => f.IsQuad).Select(f => new[] { f.A, f.C, f.D })).ToArray());
}
