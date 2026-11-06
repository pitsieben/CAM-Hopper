using CamGh.Core;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace CamGh.Grasshopper;

public sealed class OperationAnalysisComponent : GH_Component
{
    public OperationAnalysisComponent() : base("CAM Analysis", "CAM Analysis", "Summarizes CAM Core operations, spindle and cutting speed, and generated contour/path levels.", "CamGh", "Analysis") { }
    public override Guid ComponentGuid => new("9f75b201-9a78-4cc8-9d3d-16b3c52e5a91");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.Analysis;

    protected override void RegisterInputParams(GH_InputParamManager p)
        => p.AddGenericParameter("Operation Results", "R", "Operation Results output from CAM Core", GH_ParamAccess.list);

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddTextParameter("Report", "T", "Operation summary and path statistics", GH_ParamAccess.list);
        p.AddNumberParameter("RPM", "Rpm", "Spindle speed for each operation", GH_ParamAccess.list);
        p.AddNumberParameter("Cutting Speed", "Vc", "Peripheral cutting speed in metres per minute; feed rate is not part of the current CAM command model", GH_ParamAccess.list);
        p.AddNumberParameter("Path Level Z", "Z", "Mean Z height for each generated path; useful for inspecting contour layers", GH_ParamAccess.tree);
        p.AddPointParameter("Contour / Path Points", "P", "Generated path points grouped by operation and path", GH_ParamAccess.tree);
        p.AddCurveParameter("Contour Lines", "C", "Generated contour/path curves grouped by operation and path", GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var operations=GhCamData.GetList<OperationResult>(da,0);
        if(operations.Count==0)return;
        var reports=new List<string>();var rpms=new List<double>();var cuttingSpeeds=new List<double>();
        var zTree=new GH_Structure<GH_Number>();var pointTree=new GH_Structure<GH_Point>();var curveTree=new GH_Structure<GH_Curve>();
        for(int oi=0;oi<operations.Count;oi++)
        {
            var op=operations[oi];rpms.Add(op.SpindleRpm);cuttingSpeeds.Add(op.CuttingSpeedMetresPerMinute);
            reports.Add($"{op.Name} ({op.Type}): {op.Paths.Count} paths, {op.Quality.PointCount} points, {op.Quality.TotalCutLength:g3} mm cut length, spindle {op.SpindleRpm:g0} RPM, cutting speed {op.CuttingSpeedMetresPerMinute:g4} m/min.");
            for(int pi=0;pi<op.Paths.Count;pi++)
            {
                var path=op.Paths[pi];if(path.Count==0)continue;
                var branch=new GH_Path(oi,pi);zTree.Append(new GH_Number(path.Average(p=>p.Z)),branch);
                var points=path.Select(p=>new Point3d(p.X,p.Y,p.Z)).ToList();
                foreach(var p in points)pointTree.Append(new GH_Point(p),branch);
                if(points.Count>=2)curveTree.Append(new GH_Curve(new PolylineCurve(points)),branch);
            }
        }
        da.SetDataList(0,reports);da.SetDataList(1,rpms);da.SetDataList(2,cuttingSpeeds);da.SetDataTree(3,zTree);da.SetDataTree(4,pointTree);da.SetDataTree(5,curveTree);
    }
}
