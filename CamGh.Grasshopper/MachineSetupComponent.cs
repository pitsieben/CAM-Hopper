using CamGh.Core;
using Grasshopper.Kernel;

namespace CamGh.Grasshopper;

public sealed class MachineSetupComponent : GH_Component
{
    public MachineSetupComponent() : base("Machine Setup", "Machine", "Defines units and safe start height for CAM Core.", "CamGh", "Setup") { }
    public override Guid ComponentGuid => new("67dee550-42f9-41f1-82f1-e2d3ee4f4a94");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.MachineSetup;
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Name", "N", "Setup label", GH_ParamAccess.item, "Default 3-axis");
        p.AddTextParameter("Units", "U", "Output units", GH_ParamAccess.item, "Millimetres");
        p.AddNumberParameter("Safe Start Z", "Z", "Machine safe start height", GH_ParamAccess.item, 0.0);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Machine Setup", "M", "Machine setup object", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    { string name="Default 3-axis",units="Millimetres";double z=0;da.GetData(0,ref name);da.GetData(1,ref units);da.GetData(2,ref z);da.SetData(0,new MachineSetup(units,z,name)); }
}
