using CamGh.Core;
using Grasshopper.Kernel;

namespace CamGh.Grasshopper;

public sealed class ToolComponent : GH_Component
{
    public ToolComponent() : base("Tool", "Tool", "Defines a milling cutter and its cutting-speed metadata.", "CamGh", "Setup") { }
    public override Guid ComponentGuid => new("4703f8fe-0c26-4f80-a337-6c0f9bca4c1b");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.Tool;
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("Diameter", "D", "Tool cutting diameter", GH_ParamAccess.item, 6.0);
        p.AddTextParameter("Shape", "S", "Flat or Ball", GH_ParamAccess.item, "Flat");
        p.AddIntegerParameter("Flutes", "F", "Number of flutes", GH_ParamAccess.item, 2);
        p.AddNumberParameter("RPM", "R", "Spindle speed", GH_ParamAccess.item, 18000);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Tool", "T", "Tool object", GH_ParamAccess.item);
        p.AddNumberParameter("Cutting Speed", "Vc", "Peripheral cutting speed in metres/minute", GH_ParamAccess.item);
        p.AddTextParameter("Summary", "S", "Tool metadata", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        double d=6,rpm=18000;int flutes=2;string shape="Flat";
        da.GetData(0,ref d);da.GetData(1,ref shape);da.GetData(2,ref flutes);da.GetData(3,ref rpm);
        if(!Enum.TryParse<ToolShape>(shape,true,out var kind)){AddRuntimeMessage(GH_RuntimeMessageLevel.Error,"Shape must be Flat or Ball.");return;}
        if(d<=0||flutes<1||rpm<=0){AddRuntimeMessage(GH_RuntimeMessageLevel.Error,"Diameter, flutes and RPM must be positive.");return;}
        var tool=new ToolDefinition(d,kind,flutes,rpm);da.SetData(0,tool);da.SetData(1,tool.CuttingSpeedMetresPerMinute);da.SetData(2,$"{kind} Ø{d:g}, {flutes} flute(s), {rpm:g} RPM, Vc {tool.CuttingSpeedMetresPerMinute:g} m/min");
    }
}
