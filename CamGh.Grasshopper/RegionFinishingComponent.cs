using CamGh.Core;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace CamGh.Grasshopper;

/// <summary>F1: region-based 3+2 finishing.</summary>
public sealed class RegionFinishingComponent : GH_Component
{
    public RegionFinishingComponent() : base("Region Finishing", "3+2 Finish",
        "Creates a 3+2 finishing command: the part surface is split into patches by k-means clustering of face normals (respecting wall angle and line of sight), patch borders are smoothed, and each patch gets gouge-free parallel lanes from its own tool direction.",
        "CamGh", "Operations") { }
    public override Guid ComponentGuid => new("8e2d3c71-5f4a-4a0e-b1d9-2f7c6a9b0e13");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.RegionFinishing;
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddMeshParameter("Part Mesh", "M", "Closed triangulated part (the finished shape)", GH_ParamAccess.item);
        p.AddGenericParameter("Tool", "T", "Tool object from Tool component (ball end recommended)", GH_ParamAccess.item);
        p.AddGenericParameter("Cut Parameters", "C", "Cut Parameters object; stepover % sets the lane spacing, stock to leave is kept on the surface", GH_ParamAccess.item);
        p.AddVectorParameter("Directions", "D", "Optional tool axes (tip → spindle). Empty = choose automatically to cover the surface.", GH_ParamAccess.list);
        p.AddIntegerParameter("Max Orientations", "N", "Upper limit for the number of patches (tool directions)", GH_ParamAccess.item, 6);
        p.AddNumberParameter("Max Wall Angle", "W", "Largest angle in degrees between a face normal and its patch axis. Lower values give more patches with lower tool tilt (the number of patches follows from this limit).", GH_ParamAccess.item, 75.0);
        p.AddNumberParameter("Sample Step", "St", "Spacing of drop-cutter samples along each lane; 0 = automatic", GH_ParamAccess.item, 0.0);
        p.AddIntegerParameter("Overlap Rings", "O", "Rings of neighbouring faces added to each region so neighbouring regions overlap", GH_ParamAccess.item, 1);
        p.AddNumberParameter("Border Smoothing", "B", "Curvature-flow radius in mm: patch borders are shortened and straightened like relaxed curves, features smaller than this disappear, and borders run smoothly through faces instead of along mesh edges (0 = off).", GH_ParamAccess.item, 16.0);
        p.AddNumberParameter("Position Weight", "PW", "Weight of surface position against normal direction in the clustering: 0 = normals only (patches may be scattered), higher = more compact patches at slightly higher tool tilt.", GH_ParamAccess.item, 0.5);
        p[3].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Command", "C", "Region finishing command; connect to CAM Core", GH_ParamAccess.item);
        p.AddTextParameter("Summary", "S", "Declared operation settings", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        Mesh? mesh = null; int maxCount = 6, rings = 1; double wall = 75, step = 0, smoothing = 16, positionWeight = 0.5;
        if (!da.GetData(0, ref mesh) || mesh is null || !GhCamData.TryGet(da, 1, out ToolDefinition? tool) || tool is null || !GhCamData.TryGet(da, 2, out CutParameters? cut) || cut is null) return;
        var dirs = new List<Vector3d>(); da.GetDataList(3, dirs);
        da.GetData(4, ref maxCount); da.GetData(5, ref wall); da.GetData(6, ref step); da.GetData(7, ref rings); da.GetData(8, ref smoothing); da.GetData(9, ref positionWeight);
        if (maxCount < 1 || wall <= 0 || wall >= 90 || step < 0 || rings < 0 || smoothing < 0 || positionWeight < 0) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Max Orientations ≥ 1, wall angle between 0 and 90°; step, rings, smoothing and position weight non-negative."); return; }
        if (tool.Shape != ToolShape.Ball) AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Flat cutters leave steps on curved surfaces; a ball end is recommended for finishing.");
        var directions = dirs.Where(d => d.IsValid && !d.IsZero).Select(d => { d.Unitize(); return new Point3(d.X, d.Y, d.Z); }).ToList();
        var command = new RegionFinishingCommand("Region Finishing", GhCamData.ToMeshInput(mesh), tool, cut,
            directions.Count > 0 ? directions : null, maxCount, wall, step, rings, smoothing, positionWeight);
        da.SetData(0, command);
        da.SetData(1, $"{tool.Shape} Ø{tool.Diameter:g}; lane spacing {tool.Diameter * cut.StepoverPercent / 100:g} mm; wall ≤ {wall:g}°; "
            + (directions.Count > 0 ? $"{directions.Count} given direction(s)" : $"up to {maxCount} automatic direction(s)"));
    }
}
