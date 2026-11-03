using CamGh.Core;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace CamGh.Grasshopper;

public sealed class ParallelFinishingComponent : GH_Component
{
    public ParallelFinishingComponent() : base("Parallel Finishing", "P-Finish", "Creates a parallel-finishing command for CAM Core.", "CamGh", "Operations") { }
    public override Guid ComponentGuid => new("a0f6a6e4-9f32-454e-9078-ca7d7a8f6aa9");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.ParallelFinishing;
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddMeshParameter("Surface Mesh", "M", "Triangulated target surface", GH_ParamAccess.item);
        p.AddGenericParameter("Tool", "T", "Tool object from Tool component", GH_ParamAccess.item);
        p.AddGenericParameter("Cut Parameters", "C", "Cut Parameters object", GH_ParamAccess.item);
        p.AddVectorParameter("Direction", "D", "XY scan direction", GH_ParamAccess.item, Vector3d.XAxis);
        p.AddNumberParameter("Edge Inset", "I", "Inset from projected mesh bounds", GH_ParamAccess.item, 0.0);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Command", "C", "Parallel-finishing command; connect to CAM Core", GH_ParamAccess.item);
        p.AddTextParameter("Summary", "S", "Declared operation settings", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        Mesh? mesh=null;Vector3d direction=Vector3d.XAxis;double inset=0;
        if(!da.GetData(0,ref mesh)||mesh is null||!GhCamData.TryGet(da,1,out ToolDefinition? tool)||tool is null||!GhCamData.TryGet(da,2,out CutParameters? cut)||cut is null)return;
        da.GetData(3,ref direction);da.GetData(4,ref inset);
        if(direction.IsZero){AddRuntimeMessage(GH_RuntimeMessageLevel.Error,"Direction must be non-zero.");return;}
        var command=new ParallelFinishingCommand("Parallel Finishing",GhCamData.ToMeshInput(mesh),tool,cut,direction.X,direction.Y,inset);
        da.SetData(0,command);da.SetData(1,$"{tool.Shape} Ø{tool.Diameter:g}; {cut.StepoverPercent:g}% stepover; direction ({direction.X:g}, {direction.Y:g}); {cut.Pattern}");
    }
}
