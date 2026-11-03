using CamGh.Core;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace CamGh.Grasshopper;

public sealed class StockComponent : GH_Component
{
    public StockComponent() : base("Stock", "Stock", "Defines axis-aligned starting stock for CAM Core.", "CamGh", "Setup") { }
    public override Guid ComponentGuid => new("59400bbd-d73d-4144-9329-7582585e9d4c");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.Stock;
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddBoxParameter("Stock Box", "B", "Axis-aligned stock boundary", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Stock", "S", "Stock object", GH_ParamAccess.item);
        p.AddTextParameter("Summary", "Info", "Stock bounds", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        Box box=Box.Unset;if(!da.GetData(0,ref box))return;var bb=box.BoundingBox;
        if(!bb.IsValid){AddRuntimeMessage(GH_RuntimeMessageLevel.Error,"Stock box is invalid.");return;}
        var stock=new StockDefinition(new StockBounds(bb.Min.X,bb.Min.Y,bb.Min.Z,bb.Max.X,bb.Max.Y,bb.Max.Z));
        da.SetData(0,stock);da.SetData(1,$"X {bb.Min.X:g}–{bb.Max.X:g}; Y {bb.Min.Y:g}–{bb.Max.Y:g}; Z {bb.Min.Z:g}–{bb.Max.Z:g}");
    }
}
