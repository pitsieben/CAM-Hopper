using CamGh.Core;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System.Net.Http.Json;

namespace CamGh.Grasshopper;

public sealed class CamGhCoreComponent : GH_Component
{
    private static readonly HttpClient Client = new() { Timeout = TimeSpan.FromMinutes(15) };
    public CamGhCoreComponent() : base("CAM Core", "CAM Core", "Final CAM executor: accepts an ordered list of operation commands and generates the toolpath.", "CamGh", "Core") { }
    public override Guid ComponentGuid => new("217375c5-025f-49b6-a465-f55e4b04b4d5");
    protected override System.Drawing.Bitmap? Icon => ComponentIcons.Core;
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Commands", "C", "One or more ordered CAM operation commands", GH_ParamAccess.list);
        p.AddGenericParameter("Stock", "S", "Stock object from the Stock component", GH_ParamAccess.item);
        p.AddGenericParameter("Machine Setup", "M", "Optional Machine Setup object", GH_ParamAccess.item);
        p[2].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Toolpath Points", "P", "Generated toolpath points; each branch is one continuous path", GH_ParamAccess.tree);
        p.AddGenericParameter("Operation Results", "R", "Per-operation generated path data", GH_ParamAccess.list);
        p.AddTextParameter("Summary", "S", "Job result, clearance height, and path count", GH_ParamAccess.item);
        p.AddTextParameter("Engine", "E", "Worker engine state", GH_ParamAccess.item);
        p.AddVectorParameter("Tool Axes", "A", "Fixed tool axis of each motion branch, pointing from the cutter tip towards the spindle", GH_ParamAccess.tree);
        p.AddPlaneParameter("Tool Planes", "TP", "Robot targets: one plane per toolpath point. Origin = cutter tip, Z = tool direction (spindle → tip, into the material), X = feed direction", GH_ParamAccess.tree);
        p.AddLineParameter("Stock Preview", "SP", "Remaining 3D stock after all operations, drawn as vertical material segments", GH_ParamAccess.list);
        p.AddMeshParameter("Patches", "Pa", "Region Finishing patches, one coloured mesh per patch (branch = operation); faces no patch can reach come last in dark red", GH_ParamAccess.tree);
        p.AddMeshParameter("Tilt Map", "Ti", "Region Finishing tool tilt per face (angle between face normal and its patch axis): green 0°, yellow half the wall limit, red at the wall limit, purple unreachable", GH_ParamAccess.tree);
    }
    /// <summary>True once a job has come back, so Settings opens straight on the simulation tab.</summary>
    internal bool HasResult { get; private set; }
    private bool _busy;

    public override void CreateAttributes() => m_attributes = new CamGhCoreAttributes(this);

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);
        EngineProcess.StatusChanged += OnEngineStatus;
        UpdateMessage();
        _ = EngineProcess.EnsureStartedAsync();           // autostart, like PRC; the window opens from Settings
    }

    public override void RemovedFromDocument(GH_Document document)
    {
        EngineProcess.StatusChanged -= OnEngineStatus;
        base.RemovedFromDocument(document);
    }

    private void OnEngineStatus(string status, string detail) => Rhino.RhinoApp.InvokeOnUiThread(new Action(() =>
    {
        UpdateMessage();
        OnDisplayExpired(true);
    }));

    private void UpdateMessage() => Message = _busy ? "Computing…" : EngineProcess.Status switch
    {
        "Running" => HasResult ? "Engine online · job ready" : "Engine autostarted.\nAwaiting data.",
        "Starting" or "Connecting" => "Engine starting…",
        "Startup failed" or "Stopped" or "Startup error" => "Engine " + EngineProcess.Status.ToLowerInvariant(),
        _ => "Awaiting data"
    };
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var commands = GhCamData.GetList<CamCommand>(da, 0);
        if (commands.Count == 0 || !GhCamData.TryGet(da, 1, out StockDefinition? stock) || stock is null) { HasResult = false; UpdateMessage(); return; }
        GhCamData.TryGet(da, 2, out MachineSetup? setup);
        try
        {
            EngineProcess.EnsureStarted();
            _busy = true; UpdateMessage();
            var req = new CamJobRequest(commands, stock, setup ?? new MachineSetup());
            var response = Client.PostAsJsonAsync("http://127.0.0.1:47831/api/v1/job", req).GetAwaiter().GetResult();
            response.EnsureSuccessStatusCode();
            var result = response.Content.ReadFromJsonAsync<CamJobResult>().GetAwaiter().GetResult()!;
            var pointTree=new GH_Structure<GH_Point>();
            for(int i=0;i<result.Paths.Count;i++)
            {
                var branch=new GH_Path(i);
                foreach(var p in result.Paths[i])pointTree.Append(new GH_Point(new Point3d(p.X,p.Y,p.Z)),branch);
            }
            da.SetDataTree(0,pointTree);
            var axisTree=new GH_Structure<GH_Vector>();var planeTree=new GH_Structure<GH_Plane>();
            for(int i=0;i<result.Paths.Count;i++)
            {
                var a=result.MotionAxes is {} axes&&i<axes.Count?axes[i]:new Point3(0,0,1);
                var axis=new Vector3d(a.X,a.Y,a.Z);axis.Unitize();
                var branch=new GH_Path(i);axisTree.Append(new GH_Vector(axis),branch);
                foreach(var plane in TargetPlanes(result.Paths[i],axis))planeTree.Append(new GH_Plane(plane),branch);
            }
            da.SetDataTree(4,axisTree);da.SetDataTree(5,planeTree);
            var patchTree=new GH_Structure<GH_Mesh>();var tiltTree=new GH_Structure<GH_Mesh>();
            for(int i=0;i<result.Operations.Count&&i<commands.Count;i++)
            {
                if(commands[i] is not RegionFinishingCommand rf||result.Operations[i].FaceLabels is not {} labels)continue;
                var branch=new GH_Path(i);
                int patches=labels.Count==0?0:labels.Max()+1;
                if(result.Operations[i].PatchTriangles is {} tris&&tris.Count==patches+1)
                {
                    // Border-refined preview: faces are split where the smooth borders cross them.
                    for(int c=0;c<patches;c++)patchTree.Append(new GH_Mesh(TriangleMesh(tris[c],PatchColour(c))),branch);
                    if(tris[patches].Length>0)patchTree.Append(new GH_Mesh(TriangleMesh(tris[patches],System.Drawing.Color.FromArgb(120,20,30))),branch);
                }
                else
                {
                    for(int c=0;c<patches;c++)patchTree.Append(new GH_Mesh(FaceMesh(rf.Mesh,f=>labels[f]==c,_=>PatchColour(c))),branch);
                    if(labels.Any(l=>l<0))patchTree.Append(new GH_Mesh(FaceMesh(rf.Mesh,f=>labels[f]<0,_=>System.Drawing.Color.FromArgb(120,20,30))),branch);
                }
                var tilt=result.Operations[i].FaceTilt;
                tiltTree.Append(new GH_Mesh(FaceMesh(rf.Mesh,_=>true,f=>TiltColour(tilt is not null&&f<tilt.Count?tilt[f]:-1,rf.MaxWallAngleDegrees))),branch);
            }
            da.SetDataTree(7,patchTree);da.SetDataTree(8,tiltTree);
            if(result.StockPreview is {} preview)
                da.SetDataList(6,preview.Where(s=>s.Count>=2).Select(s=>new Line(new Point3d(s[0].X,s[0].Y,s[0].Z),new Point3d(s[1].X,s[1].Y,s[1].Z))));
            da.SetDataList(1, result.Operations);
            da.SetData(2, result.Summary);
            da.SetData(3, "Engine online");
            HasResult = true;
        }
        catch (Exception ex) { HasResult = false; AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message); }
        finally { _busy = false; UpdateMessage(); }
    }

    /// <summary>Unwelded mesh of the selected input faces with one colour per face.</summary>
    private static Mesh FaceMesh(MeshInput input,Func<int,bool> include,Func<int,System.Drawing.Color> colour)
    {
        var mesh=new Mesh();
        for(int f=0;f<input.Faces.Length;f++)
        {
            var face=input.Faces[f];if(face.Length<3||!include(f))continue;
            int b=mesh.Vertices.Count;var col=colour(f);
            for(int k=0;k<3;k++){var v=input.Vertices[face[k]];mesh.Vertices.Add(v[0],v[1],v[2]);mesh.VertexColors.Add(col);}
            mesh.Faces.AddFace(b,b+1,b+2);
        }
        mesh.Normals.ComputeNormals();
        return mesh;
    }

    /// <summary>Mesh from a flat xyz list, three points per triangle, one colour.</summary>
    private static Mesh TriangleMesh(double[] xyz,System.Drawing.Color colour)
    {
        var mesh=new Mesh();
        for(int k=0;k+8<xyz.Length;k+=9)
        {
            int b=mesh.Vertices.Count;
            for(int q=0;q<3;q++){mesh.Vertices.Add(xyz[k+3*q],xyz[k+3*q+1],xyz[k+3*q+2]);mesh.VertexColors.Add(colour);}
            mesh.Faces.AddFace(b,b+1,b+2);
        }
        mesh.Normals.ComputeNormals();
        return mesh;
    }

    private static System.Drawing.Color PatchColour(int index)
    {
        // Golden-ratio hue steps keep neighbouring patch numbers visually distinct.
        double h=(index*0.618033988749895)%1.0,s=0.55,v=0.92;
        int i=(int)(h*6);double f=h*6-i,p=v*(1-s),q=v*(1-f*s),t=v*(1-(1-f)*s);
        (double r,double g,double b)=(i%6) switch{0=>(v,t,p),1=>(q,v,p),2=>(p,v,t),3=>(p,q,v),4=>(t,p,v),_=>(v,p,q)};
        return System.Drawing.Color.FromArgb((int)(r*255),(int)(g*255),(int)(b*255));
    }

    private static System.Drawing.Color TiltColour(double tilt,double wall)
    {
        if(tilt<0)return System.Drawing.Color.FromArgb(150,60,170);
        double t=Math.Clamp(tilt/Math.Max(1e-6,wall),0,1);
        // green → yellow → red
        return t<0.5?System.Drawing.Color.FromArgb((int)(60+390*t),200,70):System.Drawing.Color.FromArgb(255,(int)(200-340*(t-0.5)),(int)(70-100*(t-0.5)));
    }

    /// <summary>Planes whose Z points along the tool (into the material) and whose X follows the feed direction.</summary>
    private static IEnumerable<Plane> TargetPlanes(List<Point3> path,Vector3d axis)
    {
        var z=-axis;Vector3d lastX=Vector3d.Unset;
        for(int k=0;k<path.Count;k++)
        {
            var o=new Point3d(path[k].X,path[k].Y,path[k].Z);
            var next=k+1<path.Count?new Point3d(path[k+1].X,path[k+1].Y,path[k+1].Z):o;
            var feed=next-o;feed-=z*(feed*z);
            Vector3d x;
            if(feed.Length>1e-9){feed.Unitize();x=feed;}
            else if(lastX.IsValid)x=lastX;
            else{x=Vector3d.CrossProduct(z,Math.Abs(z.Z)<0.9?Vector3d.ZAxis:Vector3d.XAxis);x.Unitize();}
            lastX=x;
            var y=Vector3d.CrossProduct(z,x);
            yield return new Plane(o,x,y);
        }
    }
}
