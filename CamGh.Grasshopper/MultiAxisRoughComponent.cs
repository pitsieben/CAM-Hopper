using CamGh.Core;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace CamGh.Grasshopper;

/// <summary>R1: 3+2 roughing from several fixed tool directions against a shared 3D stock.</summary>
public sealed class MultiAxisRoughComponent : GH_Component
{
    public MultiAxisRoughComponent() : base("Multi-Axis Roughing", "3+2 Rough",
        "Creates a 3+2 roughing command from several fixed tool directions, each one cutting only the material that earlier directions left in the shared 3D stock. Surface offset (default): flat levels that wrap onto the part offset by the stock to leave, so roughing ends in a uniform shell. Z-level: the original contour layers.",
        "CamGh", "Operations") { }
    public override Guid ComponentGuid => new("3b6f0f5e-8d1c-4b0b-9a3e-6c2d55e1a7c4");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.MultiAxisRoughing;
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddMeshParameter("Part Mesh", "M", "Closed triangulated part (the finished shape)", GH_ParamAccess.item);
        p.AddGenericParameter("Tool", "T", "Tool object from Tool component", GH_ParamAccess.item);
        p.AddGenericParameter("Cut Parameters", "C", "Cut Parameters object (stepover, stepdown, pattern, stock to leave)", GH_ParamAccess.item);
        p.AddVectorParameter("Directions", "D", "Optional tool axes, pointing from the cutter tip towards the spindle (e.g. +Z machines from above). Empty = choose automatically by visibility.", GH_ParamAccess.list);
        p.AddIntegerParameter("Max Orientations", "N", "Upper limit for automatically chosen directions", GH_ParamAccess.item, 6);
        p.AddNumberParameter("Tool Reach", "R", "Usable stick-out below the highest remaining material for each direction; 0 = 4 × tool diameter", GH_ParamAccess.item, 0.0);
        p.AddNumberParameter("Fixture Z", "F", "World Z of the clamping base/table; no tool part goes below it and material below it is kept. Empty = stock bottom.", GH_ParamAccess.item);
        p.AddNumberParameter("Grid Resolution", "G", "Layer height-field pitch in each tool frame; 0 = automatic", GH_ParamAccess.item, 0.0);
        p.AddNumberParameter("Stock Resolution", "S", "Tri-dexel ray spacing of the 3D stock model; 0 = automatic", GH_ParamAccess.item, 0.0);
        p.AddIntegerParameter("Strategy", "St", "0 = surface offset (levels follow the part offset by the stock to leave, ending in a uniform shell); 1 = Z-level contours", GH_ParamAccess.item, 0);
        p[3].Optional = true; p[6].Optional = true; p[9].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Command", "C", "Multi-axis roughing command; connect to CAM Core", GH_ParamAccess.item);
        p.AddTextParameter("Summary", "S", "Declared operation settings", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        Mesh? mesh = null; int maxCount = 6, strategy = 0; double reach = 0, grid = 0, stockRes = 0, fixture = double.NaN;
        if (!da.GetData(0, ref mesh) || mesh is null || !GhCamData.TryGet(da, 1, out ToolDefinition? tool) || tool is null || !GhCamData.TryGet(da, 2, out CutParameters? cut) || cut is null) return;
        var dirs = new List<Vector3d>(); da.GetDataList(3, dirs);
        da.GetData(4, ref maxCount); da.GetData(5, ref reach); da.GetData(6, ref fixture); da.GetData(7, ref grid); da.GetData(8, ref stockRes); da.GetData(9, ref strategy);
        if (maxCount < 1 || reach < 0 || grid < 0 || stockRes < 0) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Max Orientations must be ≥ 1; reach and resolutions cannot be negative."); return; }
        if (strategy is < 0 or > 1) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Strategy must be 0 (surface offset) or 1 (Z-level)."); return; }
        int triangles = mesh.Faces.TriangleCount + 2 * mesh.Faces.QuadCount;
        if (triangles < 20000)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, $"The part mesh has {triangles:N0} triangles. Toolpaths follow its facets, so a coarse mesh shows up as kinks in the paths; for a sculpted part, 30,000+ triangles is a good target.");
        if (!mesh.IsClosed) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "The part mesh is not closed; visibility and the material check assume a closed solid.");
        var directions = dirs.Where(d => d.IsValid && !d.IsZero).Select(d => { d.Unitize(); return new Point3(d.X, d.Y, d.Z); }).ToList();
        var command = new MultiAxisRoughingCommand("Multi-Axis Roughing", GhCamData.ToMeshInput(mesh), tool, cut,
            directions.Count > 0 ? directions : null, maxCount, reach, double.IsFinite(fixture) ? fixture : null, grid, stockRes, (RoughingStrategy)strategy);
        da.SetData(0, command);
        da.SetData(1, $"{(strategy == 0 ? "Surface offset" : "Z-level " + cut.Pattern)}; {tool.Shape} Ø{tool.Diameter:g}; stepdown {cut.StepdownPercent:g}%; stepover {cut.StepoverPercent:g}%; "
            + (directions.Count > 0 ? $"{directions.Count} given direction(s)" : $"up to {maxCount} automatic direction(s)")
            + $"; reach {(reach > 0 ? reach : 4 * tool.Diameter):g} mm");
    }
}
