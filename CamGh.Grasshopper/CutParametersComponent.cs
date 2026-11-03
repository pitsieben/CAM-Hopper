using CamGh.Core;
using Grasshopper.Kernel;

namespace CamGh.Grasshopper;

public sealed class CutParametersComponent : GH_Component
{
    public CutParametersComponent() : base("Cut Parameters", "Cut Params", "Defines reusable cutting, entry, exit, pattern, and clearance settings.", "CamGh", "Setup") { }
    public override Guid ComponentGuid => new("c5ae1a81-5442-4d36-94cb-b334d8761d91");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.CutParameters;
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("Stepover %", "SO", "Percentage of tool diameter", GH_ParamAccess.item, 40.0);
        p.AddNumberParameter("Stepdown %", "SD", "Percentage of tool diameter", GH_ParamAccess.item, 50.0);
        p.AddNumberParameter("Pattern Index", "P", "Connect an integer number slider: 0 Offset, 1 Offset Spiral, 2 Linear, 3 Spiral, 4 Radial, 5 High Speed", GH_ParamAccess.item, 0.0);
        p.AddTextParameter("Engage", "E", "Plunge, Ramp, or Helix", GH_ParamAccess.item, "Ramp");
        p.AddTextParameter("Retract", "R", "Vertical, Ramp, or StayDown", GH_ParamAccess.item, "Vertical");
        p.AddTextParameter("Clearance Mode", "C", "Automatic or Manual", GH_ParamAccess.item, "Automatic");
        p.AddNumberParameter("Clearance Offset", "Z", "Automatic: added above stock; Manual: absolute clearance Z", GH_ParamAccess.item, 10.0);
        p.AddNumberParameter("Stock to Leave", "A", "Material allowance for roughing", GH_ParamAccess.item, 0.5);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Cut Parameters", "C", "Cut-parameter object", GH_ParamAccess.item);
        p.AddTextParameter("Summary", "S", "Selected settings", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        double so=40,sd=50,patternIndex=0,clearance=10,allowance=.5;string engage="Ramp",retract="Vertical",mode="Automatic";
        da.GetData(0,ref so);da.GetData(1,ref sd);da.GetData(2,ref patternIndex);da.GetData(3,ref engage);da.GetData(4,ref retract);da.GetData(5,ref mode);da.GetData(6,ref clearance);da.GetData(7,ref allowance);
        int patternValue=(int)Math.Round(patternIndex);
        if(Math.Abs(patternIndex-patternValue)>1e-8||patternValue<0||patternValue>5||!Parse(engage,out EngageMove e)||!Parse(retract,out RetractMove r)||!Parse(mode,out ClearanceMode c)||so<=0||sd<=0||clearance<0||allowance<0)
        { AddRuntimeMessage(GH_RuntimeMessageLevel.Error,"Pattern must be an integer from 0 to 5. Check move names and ensure numeric values are non-negative.");return; }
        CutPattern p=(CutPattern)patternValue;
        var result=new CutParameters(so,sd,p,e,r,c,clearance,allowance);da.SetData(0,result);da.SetData(1,$"Pattern {patternValue}: {p}; {so:g}% stepover, {sd:g}% stepdown; {e}/{r}; {c} clearance");
    }
    private static bool Parse<T>(string value,out T result) where T:struct,Enum => Enum.TryParse(value.Replace(" ",""),true,out result);
}
