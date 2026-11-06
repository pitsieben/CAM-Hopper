using CamGh.Core;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace CamGh.Grasshopper;

public sealed class HorizontalRoughComponent : GH_Component
{
    public HorizontalRoughComponent() : base("Horizontal Roughing", "H-Rough", "Creates a horizontal-roughing command for CAM Core.", "CamGh", "Operations") { }
    public override Guid ComponentGuid => new("e7fd72e4-29ab-4c58-a0eb-a379aa3f165b");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.HorizontalRoughing;
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddMeshParameter("Part Mesh", "M", "Triangulated target surface", GH_ParamAccess.item);
        p.AddGenericParameter("Tool", "T", "Tool object from Tool component", GH_ParamAccess.item);
        p.AddGenericParameter("Cut Parameters", "C", "Cut Parameters object", GH_ParamAccess.item);
        p.AddNumberParameter("Grid Resolution", "G", "Height-field sampling pitch; zero chooses automatically", GH_ParamAccess.item, 0.0);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Command", "C", "Horizontal roughing command; connect to CAM Core", GH_ParamAccess.item);
        p.AddTextParameter("Summary", "S", "Declared operation settings", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        Mesh? mesh=null; double grid=0;
        if(!da.GetData(0,ref mesh)||mesh is null || !GhCamData.TryGet(da,1,out ToolDefinition? tool) || tool is null || !GhCamData.TryGet(da,2,out CutParameters? cut) || cut is null)return;
        da.GetData(3,ref grid);
        try
        {
            var command=new HorizontalRoughingCommand("Horizontal Roughing",GhCamData.ToMeshInput(mesh),tool,cut,grid);
            da.SetData(0,command);
            da.SetData(1,$"{tool.Shape} Ø{tool.Diameter:g}; stepdown {cut.StepdownPercent:g}%; stepover {cut.StepoverPercent:g}%; {cut.Pattern}");
        }
        catch(Exception ex){AddRuntimeMessage(GH_RuntimeMessageLevel.Error,ex.Message);}
    }
}
