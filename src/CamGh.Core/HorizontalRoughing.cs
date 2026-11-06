using Clipper2Lib;

namespace CamGh.Core;

public sealed record StockBounds(double MinX, double MinY, double MinZ, double MaxX, double MaxY, double MaxZ);
public sealed record HorizontalRoughRequest(MeshInput Mesh, StockBounds Stock, double ToolRadius, double Stepdown, double Stepover, double StockToLeave = 0.5, double GridResolution = 0, CutPattern Pattern = CutPattern.Linear, ToolShape ToolShape = ToolShape.Flat);
public sealed record HorizontalRoughResult(List<List<Point3>> Paths, QualityReport Quality, int ZLevels, string Method);

/// <summary>
/// Optional hooks used when the Z-level engine runs inside a rotated tool frame (multi-orientation roughing).
/// All coordinates are local to that frame. When null, the planner behaves exactly as the 3-axis original.
/// </summary>
public sealed class RoughingContext
{
    /// <summary>Builds the initial top-of-remaining-material map for the planner grid (bounds, cell size, nx, ny).</summary>
    public Func<StockBounds, double, int, int, double[,]>? InitialRemaining { get; init; }
    /// <summary>Extra legality test for a cutter tip position (e.g. fixture clearance).</summary>
    public Func<double, double, double, bool>? TipAllowed { get; init; }
    /// <summary>Maximum cutting depth below the highest remaining material (tool reach); infinity = unlimited.</summary>
    public double Reach { get; init; } = double.PositiveInfinity;
    /// <summary>When true the cutter centre may travel over the whole (already expanded) bounds instead of staying inside the stock outline.</summary>
    public bool FullAccess { get; init; }
    /// <summary>Use the raster level-set patterns only (mesh-section offsets assume an open height-field surface).</summary>
    public bool RasterPatternsOnly { get; init; }
}

/// <summary>
/// Layer-based horizontal roughing. A discrete height field tracks in-process stock and conservative
/// cutter clearance. Flat-tool Offset paths use cubic-fitted mesh sections and robust vector polygon
/// offsets; the height field only decides where cuts are needed and validates cutter-center safety.
/// </summary>
public static class HorizontalRoughPlanner
{
    [ThreadStatic] private static double _maxLink;
    private static bool Near(Point3 a, Point3 b) => _maxLink <= 0 || Math.Sqrt(Distance2(a, b)) <= _maxLink;
    private static bool ShortRoute(List<Point3>? route) { if (route is null) return false; double L = 0; for (int i = 0; i + 1 < route.Count; i++) L += Math.Sqrt(Distance2(route[i], route[i + 1])); return _maxLink <= 0 || L <= _maxLink * 1.5; }
    public static HorizontalRoughResult Generate(HorizontalRoughRequest r) => Generate(r, null);

    public static HorizontalRoughResult Generate(HorizontalRoughRequest r, RoughingContext? ctx)
    {
        if (r.ToolRadius <= 0 || r.Stepdown <= 0 || r.Stepover <= 0 || r.StockToLeave < 0) throw new ArgumentOutOfRangeException(nameof(r.ToolRadius), "Tool radius, stepdown and stepover must be positive; stock-to-leave cannot be negative.");
        var s = r.Stock;
        // Links that stay down are only allowed over a short distance (about one tool diameter); farther
        // jumps would be long straight slotting moves across the pocket, so they retract instead.
        _maxLink = Math.Max(2.5 * r.Stepover, 2.2 * r.ToolRadius);
        if (s.MaxX <= s.MinX || s.MaxY <= s.MinY || s.MaxZ <= s.MinZ) throw new ArgumentException("Stock bounds must have positive dimensions.");
        if (r.Mesh.Vertices.Length < 3 || r.Mesh.Faces.Length == 0) throw new ArgumentException("A triangulated target surface is required.");
        double h = r.GridResolution > 0 ? r.GridResolution : Math.Max(0.05, Math.Min(0.2, Math.Min(r.Stepover * 0.2, r.ToolRadius * 0.1)));
        if(r.GridResolution<=0)
        {
            // Keep the default field fine enough to avoid visible grid facets while retaining
            // the existing two-million-cell memory bound on large stock envelopes.
            while((long)(Math.Ceiling((s.MaxX-s.MinX)/h)+1)*(long)(Math.Ceiling((s.MaxY-s.MinY)/h)+1)>2_000_000)h*=1.25;
        }
        int nx = (int)Math.Ceiling((s.MaxX - s.MinX) / h) + 1, ny = (int)Math.Ceiling((s.MaxY - s.MinY) / h) + 1;
        if ((long)nx * ny > 2_000_000) throw new ArgumentException("Height-field grid exceeds two million cells; increase Grid Resolution.");
        var zMap = new double[nx, ny];
        for (int i=0;i<nx;i++) for(int j=0;j<ny;j++) zMap[i,j]=double.NegativeInfinity;
        var verts = r.Mesh.Vertices.Select(v => new Point3(v[0],v[1],v[2])).ToArray();
        // Rasterize each triangle onto the height grid with barycentric interpolation. If surfaces
        // overlap in XY, retain the upper envelope, which is conservative for 3-axis roughing.
        foreach (var face in r.Mesh.Faces)
        {
            if (face.Length < 3) continue;
            RasterTriangle(verts[face[0]],verts[face[1]],verts[face[2]],s,h,nx,ny,zMap);
        }
        var reach=(int)Math.Ceiling(r.ToolRadius/h); var cutterOffsets=new List<(int X,int Y,double Distance)>();
        for(int ox=-reach;ox<=reach;ox++) for(int oy=-reach;oy<=reach;oy++)
        {
            double distance=Math.Sqrt((ox*h)*(ox*h)+(oy*h)*(oy*h));
            if(distance<=r.ToolRadius+1e-9)cutterOffsets.Add((ox,oy,distance));
        }
        // The extra grid-cell margin makes the sampled dilation conservative between vertices.
        int collisionReach=(int)Math.Ceiling((r.ToolRadius+h*Math.Sqrt(2))/h);var collisionOffsets=new List<(int X,int Y,double Distance)>();
        for(int ox=-collisionReach;ox<=collisionReach;ox++)for(int oy=-collisionReach;oy<=collisionReach;oy++)
        {
            double distance=Math.Sqrt((ox*h)*(ox*h)+(oy*h)*(oy*h));if(distance<=r.ToolRadius+h*Math.Sqrt(2)+1e-9)collisionOffsets.Add((ox,oy,Math.Min(distance,r.ToolRadius)));
        }
        var envelope=new double[nx,ny];
        for(int i=0;i<nx;i++) for(int j=0;j<ny;j++)
        {
            double max=double.NegativeInfinity;
            foreach(var o in collisionOffsets)
            {
                int x=i+o.X,y=j+o.Y;
                if(x>=0&&x<nx&&y>=0&&y<ny&&double.IsFinite(zMap[x,y]))
                {
                    double cutterOffset=r.ToolShape==ToolShape.Ball?r.ToolRadius-Math.Sqrt(Math.Max(0,r.ToolRadius*r.ToolRadius-o.Distance*o.Distance)):0;
                    max=Math.Max(max,zMap[x,y]-cutterOffset);
                }
            }
            envelope[i,j]=max;
        }
        var remaining=new double[nx,ny];
        double top=s.MaxZ,floorZ=s.MinZ;
        if(ctx?.InitialRemaining is not null)
        {
            remaining=ctx.InitialRemaining(s,h,nx,ny);
            if(remaining.GetLength(0)!=nx||remaining.GetLength(1)!=ny)throw new InvalidOperationException("Initial remaining map has the wrong size.");
            double highest=double.NegativeInfinity;
            for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)highest=Math.Max(highest,remaining[i,j]);
            if(!double.IsFinite(highest))return new(new List<List<Point3>>(),Measure(new List<List<Point3>>()),0,"No remaining material is visible from this direction.");
            top=Math.Min(s.MaxZ,highest);
            if(double.IsFinite(ctx.Reach))floorZ=Math.Max(floorZ,top-ctx.Reach);
        }
        else for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)remaining[i,j]=s.MaxZ;
        var levels=new List<double>();
        for(double z=top-r.Stepdown;z>floorZ+1e-9;z-=r.Stepdown) levels.Add(z);
        if(levels.Count==0||levels[^1]>floorZ+1e-9)levels.Add(floorZ);
        if(levels.Count>100_000)throw new ArgumentException("Stepdown creates more than 100,000 depth layers; increase Stepdown.");
        var paths=new List<List<Point3>>();
        int laneStride=Math.Max(1,(int)Math.Round(r.Stepover/h));
        for(int li=0;li<levels.Count;li++)
        {
            double z=levels[li];
            var legal=new bool[nx,ny];var needsCut=new bool[nx,ny];
            for(int i=0;i<nx;i++) for(int j=0;j<ny;j++)
            {
                double x=s.MinX+i*h,y=s.MinY+j*h;
                double margin=h*Math.Sqrt(2);
                bool insideStock=ctx?.FullAccess==true||x>=s.MinX+r.ToolRadius+margin&&x<=s.MaxX-r.ToolRadius-margin&&y>=s.MinY+r.ToolRadius+margin&&y<=s.MaxY-r.ToolRadius-margin;
                legal[i,j]=insideStock&&envelope[i,j]+r.StockToLeave<=z+1e-9&&(ctx?.TipAllowed is null||ctx.TipAllowed(x,y,z));
                if(!legal[i,j])continue;
                foreach(var o in cutterOffsets)
                {
                    int sx=i+o.X,sy=j+o.Y;if(sx<0||sy<0||sx>=nx||sy>=ny)continue;
                    double bottom=ToolBottom(z,o.Distance,r.ToolRadius,r.ToolShape);
                    if(remaining[sx,sy]>bottom+1e-8){needsCut[i,j]=true;break;}
                }
            }
            var active=new bool[nx,ny];for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)active[i,j]=legal[i,j]&&needsCut[i,j];
            // The remaining-stock map carries the staircase of the previous level's cut and of the stock grid. Rings built on
            // that edge would copy it, so round the region off first (slightly dilating: cutting a little air is harmless,
            // leaving a ragged wall is not). Specks of one or two cells vanish; finishing removes them.
            if(ctx?.RasterPatternsOnly==true)active=SmoothMask(active,legal,nx,ny);
            bool useMeshOffsets=r.Pattern==CutPattern.Offset&&r.ToolShape==ToolShape.Flat&&ctx?.RasterPatternsOnly!=true;
            var layer=useMeshOffsets
                ?GenerateMeshOffsetPattern(r.Mesh,active,legal,s,h,z,r.ToolRadius,r.StockToLeave,r.Stepover)
                :new List<List<Point3>>();
            bool vectorLayer=layer.Count>0;
            if(!vectorLayer&&!useMeshOffsets)layer=CollapseTinyLoops(GeneratePattern(active,nx,ny,s,h,z,laneStride,r.Pattern,r.ToolRadius),r.ToolRadius);
            if(vectorLayer)layer=CollapseTinyLoops(layer,r.ToolRadius).Where(p=>p.Count>3).ToList();
            layer=vectorLayer?LinkVectorPaths(layer,legal,s,h,z):LinkPaths(layer,legal,s,h,z);
            if(!vectorLayer)layer=CollapseTinyLoops(SmoothPathsWithinSafeArea(layer,legal,s,h),r.ToolRadius);
            List<Point3> transition=new();bool canLinkDown=false;
            if(paths.Count>0&&paths[^1].Count>0&&layer.Count>0)
                canLinkDown=vectorLayer
                    ?TryLinkDownDirect(paths[^1][^1],layer[0],legal,s,h,z,out transition)
                    :TryLinkDown(paths[^1][^1],layer[0],legal,s,h,z,out transition);
            if(canLinkDown)
            {
                UpdateStock(transition,remaining,zMap,s,h,r.ToolRadius,r.StockToLeave,r.ToolShape,cutterOffsets);
                paths[^1].AddRange(transition.Skip(1));layer.RemoveAt(0);
            }
            foreach(var path in layer)UpdateStock(path,remaining,zMap,s,h,r.ToolRadius,r.StockToLeave,r.ToolShape,cutterOffsets);
            paths.AddRange(layer);
        }
        return new(paths,Measure(paths),levels.Count,
            $"Stock-aware Z-level roughing using {r.Pattern} and a {r.ToolShape} cutter. Flat-tool Offset paths use tolerance-bounded cubic mesh sections and robust vector offsets with self-intersection cleanup; the stock field selects remaining cuts and validates cutter-center travel. Grid resolution {h:g} mm. Requires a single-valued upper surface; overlapping XY triangles use their maximum Z.");
    }

    private static double ToolBottom(double tipZ,double distance,double radius,ToolShape shape)
        => shape==ToolShape.Ball?tipZ+radius-Math.Sqrt(Math.Max(0,radius*radius-distance*distance)):tipZ;

    private static void UpdateStock(List<Point3> path,double[,] remaining,double[,] part,StockBounds stock,double h,double radius,double leave,ToolShape shape,List<(int X,int Y,double Distance)> cutterOffsets)
    {
        for(int k=0;k<path.Count;k++)
        {
            var a=path[k];var b=k+1<path.Count?path[k+1]:a;
            int samples=Math.Max(1,(int)Math.Ceiling(Math.Sqrt((b.X-a.X)*(b.X-a.X)+(b.Y-a.Y)*(b.Y-a.Y)+(b.Z-a.Z)*(b.Z-a.Z))/Math.Max(h*.5,1e-6)));
            for(int q=0;q<=samples;q++)
            {
                double t=(double)q/samples,x=a.X+(b.X-a.X)*t,y=a.Y+(b.Y-a.Y)*t,z=a.Z+(b.Z-a.Z)*t;
                int ci=(int)Math.Round((x-stock.MinX)/h),cj=(int)Math.Round((y-stock.MinY)/h);
                foreach(var o in cutterOffsets)
                {
                    int i=ci+o.X,j=cj+o.Y;if(i<0||j<0||i>=remaining.GetLength(0)||j>=remaining.GetLength(1))continue;
                    double floor=double.IsFinite(part[i,j])?part[i,j]+leave:stock.MinZ;
                    double cutBottom=ToolBottom(z,o.Distance,radius,shape);
                    // Only ever lowers the remaining top, so empty (-inf) columns stay empty.
                    remaining[i,j]=Math.Min(remaining[i,j],Math.Max(floor,cutBottom));
                }
            }
        }
    }

    /// <summary>
    /// A one- or two-cell island of material contours into a loop only a few millimetres round. With a cutter
    /// many times that size the loop is just a costly way of visiting one spot, and drawn it reads as a diamond
    /// artifact. Replace every closed loop whose perimeter is under one cutter radius by the straight chord
    /// between its two farthest points: the cutter disk swept along that chord covers the loop, so the same
    /// material is removed, and the chord links into neighbouring paths like any other cut.
    /// </summary>
    private static List<List<Point3>> CollapseTinyLoops(List<List<Point3>> paths,double toolRadius)
    {
        var output=new List<List<Point3>>(paths.Count);
        foreach(var p in paths)
        {
            bool closed=p.Count>3&&Distance2(p[0],p[^1])<1e-12;
            double perimeter=0;for(int i=0;i+1<p.Count;i++)perimeter+=Math.Sqrt(Distance2(p[i],p[i+1]));
            if(!closed||perimeter>=toolRadius){output.Add(p);continue;}
            var best=(A:p[0],B:p[1],D:-1.0);
            for(int i=0;i<p.Count-1;i++)for(int j=i+1;j<p.Count-1;j++){double d=Distance2(p[i],p[j]);if(d>best.D)best=(p[i],p[j],d);}
            output.Add(new List<Point3>{best.A,best.B});
        }
        return output;
    }

    private static List<List<Point3>> LinkPaths(List<List<Point3>> cuts,bool[,] legal,StockBounds stock,double h,double z)
    {
        var output=new List<List<Point3>>();if(cuts.Count==0)return output;
        var current=cuts[0].ToList();
        for(int k=1;k<cuts.Count;k++)
        {
            var next=cuts[k];var route=Near(current[^1],next[0])?FindSafeRoute(current[^1],next[0],legal,stock,h,z):null;
            if(!ShortRoute(route)){if(current.Count>=2)output.Add(current);current=next.ToList();continue;}
            current.AddRange(route!.Skip(1));current.AddRange(next.Skip(1));
        }
        if(current.Count>=2)output.Add(current);return output;
    }

    private static List<List<Point3>> LinkVectorPaths(List<List<Point3>> cuts,bool[,] legal,StockBounds stock,double h,double z)
    {
        var output=new List<List<Point3>>();if(cuts.Count==0)return output;
        // Closed offset loops may start at unrelated mesh vertices. Starting the first
        // contour there can force a long chord across the part when the next contour is
        // linked. Choose the first seam from the closest pair of points on the first two
        // loops, then keep each subsequent link local to the previous loop's seam.
        int firstStart=0,secondStart=0;
        if(cuts.Count>1)
        {
            int n0=ClosedCount(cuts[0]),n1=ClosedCount(cuts[1]);double best=double.PositiveInfinity;
            for(int i=0;i<n0;i++)for(int j=0;j<n1;j++)
            {
                double d=Dist2(cuts[0][i],cuts[1][j]);if(d<best){best=d;firstStart=i;secondStart=j;}
            }
        }
        var current=RotateClosed(cuts[0],firstStart);
        for(int k=1;k<cuts.Count;k++)
        {
            var next=cuts[k];int start;
            if(k==1)start=secondStart;
            else
            {
                int n=ClosedCount(next);double best=double.PositiveInfinity;start=0;
                for(int j=0;j<n;j++){double d=Dist2(current[^1],next[j]);if(d<best){best=d;start=j;}}
            }
            var rotated=RotateClosed(next,start);
            if(Near(current[^1],rotated[0])&&SegmentSafe(current[^1],rotated[0],legal,stock,h))current.AddRange(rotated);
            else
            {
                if(current.Count>=2)output.Add(current);
                current=rotated;
            }
        }
        if(current.Count>=2)output.Add(current);return output;
    }

    private static int ClosedCount(List<Point3> path)
        =>path.Count>1&&Distance2(path[0],path[^1])<1e-12?path.Count-1:path.Count;

    private static List<Point3> RotateClosed(List<Point3> path,int start)
    {
        int n=path.Count>1&&Distance2(path[0],path[^1])<1e-12?path.Count-1:path.Count;
        var rotated=new List<Point3>(n+1);for(int i=0;i<n;i++)rotated.Add(path[(start+i)%n]);
        if(n>0)rotated.Add(rotated[0]);return rotated;
    }

    private static bool TryLinkDownDirect(Point3 previousEnd,List<Point3> nextPath,bool[,] legal,StockBounds stock,double h,double z,out List<Point3> transition)
    {
        var lower=new Point3(previousEnd.X,previousEnd.Y,z);transition=new List<Point3>();
        if(nextPath.Count<2||!Near(lower,nextPath[0])||!IsSafePoint(lower,legal,stock,h)||!IsSegmentSafe(lower,nextPath[0],legal,stock,h))return false;
        transition.Add(previousEnd);transition.Add(lower);
        if(Distance2(lower,nextPath[0])>1e-12)transition.Add(nextPath[0]);
        transition.AddRange(nextPath.Skip(1));return true;
    }

    private static List<List<Point3>> SmoothPathsWithinSafeArea(List<List<Point3>> paths,bool[,] legal,StockBounds stock,double h)
    {
        // Raster contours and grid links inherit the cell staircase of the legality mask. Resample each
        // path to half-cell spacing, low-pass it in arc length (eight binomial passes, sigma ~1.4 cells,
        // which spans a staircase period on shallow slopes), then enforce safety per vertex: any vertex
        // whose neighbouring segments leave the legal cells is pulled back towards its original position,
        // so one tight spot no longer forces the whole path back to its raw staircase.
        var result=new List<List<Point3>>(paths.Count);
        foreach(var source in paths)
        {
            if(source.Count<3){result.Add(source);continue;}
            bool closed=source.Count>3&&Distance2(source[0],source[^1])<1e-12;
            var original=Resample(closed?source.Take(source.Count-1).ToList():source,h*.5,closed);
            int n=original.Count;if(n<5){result.Add(source);continue;}
            var current=original.ToList();
            for(int pass=0;pass<8;pass++)
            {
                var next=current.ToList();
                for(int i=0;i<n;i++)
                {
                    if(!closed&&(i<2||i>n-3))continue;
                    Point3 At(int k)=>current[closed?((k%n)+n)%n:Math.Clamp(k,0,n-1)];
                    var p0=At(i-2);var p1=At(i-1);var p2=At(i);var p3=At(i+1);var p4=At(i+2);
                    double x=(p0.X+4*p1.X+6*p2.X+4*p3.X+p4.X)/16,y=(p0.Y+4*p1.Y+6*p2.Y+4*p3.Y+p4.Y)/16;
                    double dx=x-original[i].X,dy=y-original[i].Y,move=Math.Sqrt(dx*dx+dy*dy),limit=h*1.5;
                    if(move>limit){double k=limit/move;x=original[i].X+dx*k;y=original[i].Y+dy*k;}
                    next[i]=new Point3(x,y,original[i].Z);
                }
                current=next;
            }
            // Local safety repair, judged against the unsmoothed path: a smoothed segment is acceptable if it is
            // legal or if the original segment it replaces was already outside the legal cells (raster contours
            // can sit within half a cell of the mask edge, so an absolute test would reject them before smoothing).
            int segCount=closed?n:n-1;
            var origOk=new bool[segCount];
            for(int i=0;i<segCount;i++)origOk[i]=IsSegmentSafe(original[i],original[(i+1)%n],legal,stock,h);
            bool SegOk(int i)=>!origOk[i]||IsSegmentSafe(current[i],current[(i+1)%n],legal,stock,h);
            for(int attempt=0;attempt<8;attempt++)
            {
                var offending=new HashSet<int>();
                for(int i=0;i<segCount;i++)if(!SegOk(i)){offending.Add(i);offending.Add((i+1)%n);}
                if(offending.Count==0)break;
                foreach(int i in offending)
                    current[i]=attempt==7?original[i]:new Point3((current[i].X+original[i].X)*.5,(current[i].Y+original[i].Y)*.5,original[i].Z);
            }
            bool safe=true;for(int i=0;i<segCount&&safe;i++)safe=SegOk(i);
            if(!safe)current=original.ToList();
            if(closed)current.Add(current[0]);
            result.Add(current);
        }
        return result;
    }

    private static List<Point3> Resample(List<Point3> pts,double spacing,bool closed)
    {
        var src=closed?pts.Append(pts[0]).ToList():pts;var output=new List<Point3>{src[0]};double carry=0;
        for(int i=0;i+1<src.Count;i++)
        {
            var a=src[i];var b=src[i+1];double len=Math.Sqrt(Distance2(a,b));if(len<1e-12)continue;
            double t=spacing-carry;
            while(t<len-1e-9){double u=t/len;output.Add(new Point3(a.X+(b.X-a.X)*u,a.Y+(b.Y-a.Y)*u,a.Z+(b.Z-a.Z)*u));t+=spacing;}
            carry=len-(t-spacing);
        }
        if(closed){if(Distance2(output[^1],output[0])<spacing*spacing*.25)output.RemoveAt(output.Count-1);}
        else if(Distance2(output[^1],src[^1])>1e-12)output.Add(src[^1]);
        return output;
    }

    private static bool TryLinkDown(Point3 previousEnd,List<Point3> nextPath,bool[,] legal,StockBounds stock,double h,double z,out List<Point3> transition)
    {
        var lower=new Point3(previousEnd.X,previousEnd.Y,z);
        transition=new List<Point3>();if(!IsSafePoint(lower,legal,stock,h))return false;
        if(!Near(lower,nextPath[0]))return false;var route=FindSafeRoute(lower,nextPath[0],legal,stock,h,z);if(!ShortRoute(route))return false;
        transition.Add(previousEnd);transition.AddRange(route!);transition.AddRange(nextPath.Skip(1));return true;
    }

    private static List<Point3>? FindSafeRoute(Point3 start,Point3 end,bool[,] legal,StockBounds stock,double h,double z)
    {
        if(IsSegmentSafe(start,end,legal,stock,h))return new(){start,end};
        int nx=legal.GetLength(0),ny=legal.GetLength(1),si=(int)Math.Round((start.X-stock.MinX)/h),sj=(int)Math.Round((start.Y-stock.MinY)/h),ei=(int)Math.Round((end.X-stock.MinX)/h),ej=(int)Math.Round((end.Y-stock.MinY)/h);
        if(!IsCellLegal(si,sj,legal)||!IsCellLegal(ei,ej,legal))return null;
        int count=nx*ny,source=si*ny+sj,target=ei*ny+ej;var cost=Enumerable.Repeat(double.PositiveInfinity,count).ToArray();var parent=Enumerable.Repeat(-1,count).ToArray();var queue=new PriorityQueue<int,double>();
        cost[source]=0;queue.Enqueue(source,Math.Sqrt((si-ei)*(si-ei)+(sj-ej)*(sj-ej)));
        int[] di={-1,-1,-1,0,0,1,1,1},dj={-1,0,1,-1,1,-1,0,1};
        while(queue.TryDequeue(out int node,out double priority))
        {
            int i=node/ny,j=node%ny;double heuristic=Math.Sqrt((i-ei)*(i-ei)+(j-ej)*(j-ej));if(priority>cost[node]+heuristic+1e-9)continue;
            if(node==target)break;
            for(int d=0;d<8;d++)
            {
                int x=i+di[d],y=j+dj[d];if(!IsCellLegal(x,y,legal))continue;
                if(di[d]!=0&&dj[d]!=0&&(!IsCellLegal(i+di[d],j,legal)||!IsCellLegal(i,j+dj[d],legal)))continue;
                int candidate=x*ny+y;double nextCost=cost[node]+(di[d]!=0&&dj[d]!=0?1.41421356237:1);
                if(nextCost>=cost[candidate])continue;cost[candidate]=nextCost;parent[candidate]=node;
                queue.Enqueue(candidate,nextCost+Math.Sqrt((x-ei)*(x-ei)+(y-ej)*(y-ej)));
            }
        }
        if(source!=target&&parent[target]<0)return null;
        var cells=new List<int>();for(int node=target;node>=0;node=parent[node]){cells.Add(node);if(node==source)break;}cells.Reverse();
        var route=new List<Point3>{start};
        foreach(int node in cells){var p=new Point3(stock.MinX+(node/ny)*h,stock.MinY+(node%ny)*h,z);if(Distance2(route[^1],p)>1e-12)route.Add(p);}
        if(Distance2(route[^1],end)>1e-12)route.Add(end);
        for(int k=0;k+1<route.Count;k++)if(!IsSegmentSafe(route[k],route[k+1],legal,stock,h))return null;
        return StringPull(route,legal,stock,h);
    }

    /// <summary>
    /// A* walks cell centres in eight directions, so a raw route is a 45/90-degree staircase. Greedily
    /// replace it with the longest safe straight segments (the "string pulling" of grid path planning):
    /// from each kept point, jump to the farthest later point still reachable in a straight safe line.
    /// </summary>
    private static List<Point3> StringPull(List<Point3> route,bool[,] legal,StockBounds stock,double h)
    {
        if(route.Count<=2)return route;
        var pulled=new List<Point3>{route[0]};int at=0;
        while(at<route.Count-1)
        {
            int next=at+1;
            for(int k=route.Count-1;k>at+1;k--)if(IsSegmentSafe(route[at],route[k],legal,stock,h)){next=k;break;}
            pulled.Add(route[next]);at=next;
        }
        return pulled;
    }

    private static bool IsSafePoint(Point3 p,bool[,] legal,StockBounds stock,double h)
        =>IsCellLegal((int)Math.Round((p.X-stock.MinX)/h),(int)Math.Round((p.Y-stock.MinY)/h),legal);
    private static bool IsCellLegal(int i,int j,bool[,] legal)=>i>=0&&j>=0&&i<legal.GetLength(0)&&j<legal.GetLength(1)&&legal[i,j];
    private static bool IsSegmentSafe(Point3 a,Point3 b,bool[,] legal,StockBounds stock,double h)
    {
        int n=Math.Max(2,(int)Math.Ceiling(Math.Sqrt((b.X-a.X)*(b.X-a.X)+(b.Y-a.Y)*(b.Y-a.Y))/Math.Max(h*.35,1e-6)));
        for(int q=0;q<=n;q++){double t=(double)q/n;int i=(int)Math.Round((a.X+(b.X-a.X)*t-stock.MinX)/h),j=(int)Math.Round((a.Y+(b.Y-a.Y)*t-stock.MinY)/h);if(!IsCellLegal(i,j,legal))return false;}return true;
    }
    private static double Distance2(Point3 a,Point3 b)=>(a.X-b.X)*(a.X-b.X)+(a.Y-b.Y)*(a.Y-b.Y)+(a.Z-b.Z)*(a.Z-b.Z);

    private static List<List<Point3>> GeneratePattern(bool[,] safe,int nx,int ny,StockBounds stock,double h,double z,int stride,CutPattern pattern,double radius)
    {
        return pattern switch
        {
            CutPattern.Offset => OffsetPasses(safe,nx,ny,stock,h,z,stride,false,radius),
            CutPattern.OffsetSpiral => OffsetPasses(safe,nx,ny,stock,h,z,stride,true,radius),
            CutPattern.Spiral => OffsetPasses(safe,nx,ny,stock,h,z,stride,true,radius),
            CutPattern.Radial => Radial(safe,nx,ny,stock,h,z,stride),
            CutPattern.HighSpeed => HighSpeed(safe,nx,ny,stock,h,z,stride,radius),
            _ => Linear(safe,nx,ny,stock,h,z,stride)
        };
    }

    private static List<List<Point3>> Linear(bool[,] safe,int nx,int ny,StockBounds s,double h,double z,int stride)
    {
        var paths=new List<List<Point3>>();
        for(int j=0;j<ny;j+=stride)
        {
            bool reverse=(j/stride)%2==1;var run=new List<Point3>();
            for(int q=0;q<nx;q++)
            {
                int i=reverse?nx-1-q:q;
                if(safe[i,j])run.Add(new Point3(s.MinX+i*h,s.MinY+j*h,z));
                else{if(run.Count>=2)paths.Add(run);run=new List<Point3>();}
            }
            if(run.Count>=2)paths.Add(run);
        }
        return paths;
    }

    private static List<List<Point3>> HighSpeed(bool[,] safe,int nx,int ny,StockBounds s,double h,double z,int stride,double radius)
    {
        // Trochoidal-style raster with small backward loops. Candidate points are retained only
        // when the cutter-centre cell is safe; this is geometric, not a load-adaptive HSM solver.
        var paths=new List<List<Point3>>();double wavelength=Math.Max(4*h,2*radius);double amplitude=Math.Min(radius*.45,wavelength/(2*Math.PI)*1.2);
        for(int j=0;j<ny;j+=stride)
        {
            bool reverse=(j/stride)%2==1;var run=new List<Point3>();double laneY=s.MinY+j*h;double length=(nx-1)*h;int samples=Math.Max(2,(int)Math.Ceiling(length/(h/3)));
            for(int q=0;q<=samples;q++)
            {
                double progress=length*q/samples,phase=2*Math.PI*progress/wavelength;
                double x=reverse?s.MaxX-progress+amplitude*Math.Sin(phase):s.MinX+progress-amplitude*Math.Sin(phase);
                double y=laneY+amplitude*(1-Math.Cos(phase))-amplitude;
                int i=(int)Math.Round((x-s.MinX)/h),k=(int)Math.Round((y-s.MinY)/h);
                if(i>=0&&i<nx&&k>=0&&k<ny&&safe[i,k])run.Add(new Point3(x,y,z));
                else{if(run.Count>=2)paths.Add(run);run=new List<Point3>();}
            }
            if(run.Count>=2)paths.Add(run);
        }
        return paths;
    }

    private static List<List<Point3>> Radial(bool[,] safe,int nx,int ny,StockBounds s,double h,double z,int stride)
    {
        var paths=new List<List<Point3>>();double cx=s.MinX+(nx-1)*h*.5,cy=s.MinY+(ny-1)*h*.5;
        int count=Math.Max(16,(int)Math.Ceiling(2*Math.PI*Math.Max(nx,ny)/Math.Max(1,stride)));
        for(int a=0;a<count;a++)
        {
            double angle=2*Math.PI*a/count,dx=Math.Cos(angle),dy=Math.Sin(angle);var run=new List<Point3>();
            for(double r=0;r<Math.Sqrt(Math.Pow(nx*h,2)+Math.Pow(ny*h,2));r+=h*.5)
            {
                double x=cx+dx*r,y=cy+dy*r;int i=(int)Math.Round((x-s.MinX)/h),j=(int)Math.Round((y-s.MinY)/h);
                if(i>=0&&i<nx&&j>=0&&j<ny&&safe[i,j])run.Add(new Point3(x,y,z));
                else if(run.Count>0){if(run.Count>=2)paths.Add(run);run=new List<Point3>();}
            }
            if(run.Count>=2)paths.Add(run);
        }
        return paths;
    }

    private readonly record struct GridPoint(int X,int Y);
    private readonly record struct GridEdge(GridPoint A,GridPoint B);
    private readonly record struct SectionKey(long X,long Y);
    private readonly record struct SectionEdgeKey(SectionKey A,SectionKey B);
    private sealed record MeshSectionContour(List<Point3> Points,List<(double X,double Y)> OutwardNormals);
    private sealed record MeshSectionEdge(SectionKey A,SectionKey B,Point3 PA,Point3 PB,double NX,double NY);

    private static List<List<Point3>> GenerateMeshOffsetPattern(MeshInput mesh,bool[,] active,bool[,] legal,StockBounds stock,double h,double z,double toolRadius,double stockToLeave,double stepover)
    {
        var contours=MeshContoursAtZ(mesh,z,stock,toolRadius,h);if(contours.Count==0)return StockInsetPaths(active,legal,stock,h,z,toolRadius,stepover);
        const double scale=1_000_000;
        var paths=new List<List<Point3>>();var sourcePaths=new Paths64();double maxOffset=Math.Sqrt(Math.Pow(stock.MaxX-stock.MinX,2)+Math.Pow(stock.MaxY-stock.MinY,2));
        double firstOffset=toolRadius+stockToLeave+2*h;int passLimit=Math.Min(100_000,(int)Math.Ceiling(maxOffset/stepover)+1);
        double smoothMargin=0;
        foreach(var contour in contours)
        {
            double curveTolerance=Math.Max(1e-5,Math.Min(h*.25,Math.Min(stepover*.02,toolRadius*.01)));
            var fitted=FitMeshSectionContour(contour,z,curveTolerance);
            // The slice through a triangle mesh is faceted at the triangle scale and every offset copies that noise.
            // Smooth the slice first (bounded deviation, compensated in the first offset) so the rings come out clean.
            fitted=SmoothContour(fitted,Math.Clamp(toolRadius*.15,.8,2.5),Math.Max(.1,Math.Min(.5,toolRadius*.03)),out double smoothDev);
            smoothMargin=Math.Max(smoothMargin,smoothDev);
            var sourcePath=new Path64();
            foreach(var p in fitted)
            {
                var q=new Point64((long)Math.Round(p.X*scale),(long)Math.Round(p.Y*scale));
                if(sourcePath.Count==0||sourcePath[^1].X!=q.X||sourcePath[^1].Y!=q.Y)sourcePath.Add(q);
            }
            if(sourcePath.Count>1&&sourcePath[0].X==sourcePath[^1].X&&sourcePath[0].Y==sourcePath[^1].Y)sourcePath.RemoveAt(sourcePath.Count-1);
            if(sourcePath.Count<3)continue;
            // Orient every contour so the lower-Z side is on its left. Together, these
            // loops encode outer boundaries and islands/holes for a single offset solve.
            if(!LowSideIsInside(contour))sourcePath=Clipper.ReversePath(sourcePath);
            sourcePaths.Add(sourcePath);
        }
        if(sourcePaths.Count==0)return StockInsetPaths(active,legal,stock,h,z,toolRadius,stepover);
        firstOffset+=smoothMargin;
        int inactivePasses=0;
        for(int pass=0;pass<passLimit;pass++)
        {
            double offset=firstOffset+pass*stepover;
            double curveTolerance=Math.Max(1e-5,Math.Min(h*.25,Math.Min(stepover*.02,toolRadius*.01)));
            Paths64 inset;
            try
            {
                // Negative offset moves each boundary into the low-Z cutting region:
                // outer contours contract while clockwise island boundaries expand.
                inset=Clipper.InflatePaths(sourcePaths,-offset*scale,JoinType.Round,EndType.Polygon,2.0,curveTolerance*scale);
            }
            catch(ClipperLibException){break;}
            if(inset.Count==0)break;
            bool passCut=false;
            foreach(var polygon in inset)
            {
                if(polygon.Count<3)continue;
                var sampled=polygon.Select(p=>new Point3(p.X/scale,p.Y/scale,z)).ToList();
                if(Distance2(sampled[0],sampled[^1])>1e-12)sampled.Add(sampled[0]);
                int candidateActive=0,candidateSamples=0;bool allLegal=true;
                for(int i=0;i+1<sampled.Count&&allLegal;i++)
                {
                    var a=sampled[i];var b=sampled[i+1];int steps=Math.Max(1,(int)Math.Ceiling(Math.Sqrt((b.X-a.X)*(b.X-a.X)+(b.Y-a.Y)*(b.Y-a.Y))/Math.Max(h*.5,1e-6)));
                    for(int q=0;q<=steps;q++)
                    {
                        double t=(double)q/steps;var p=new Point3(a.X+(b.X-a.X)*t,a.Y+(b.Y-a.Y)*t,z);int x=(int)Math.Round((p.X-stock.MinX)/h),y=(int)Math.Round((p.Y-stock.MinY)/h);
                        if(x<0||y<0||x>=legal.GetLength(0)||y>=legal.GetLength(1)||!legal[x,y]){allLegal=false;break;}
                        if(active[x,y])candidateActive++;candidateSamples++;
                    }
                }
                if(!allLegal)continue;
                if(candidateActive>=Math.Max(2,(int)Math.Ceiling(candidateSamples*.02)))
                {paths.Add(sampled);passCut=true;}
            }
            if(passCut)inactivePasses=0;
            else if(++inactivePasses>=2)break;
        }
        return paths.Count>0?paths:StockInsetPaths(active,legal,stock,h,z,toolRadius,stepover);
    }

    /// <summary>Gaussian smoothing of a section polyline (resampled uniformly). Returns the input when it is too short or
    /// would move more than maxDeviation; deviation is the largest distance from the smoothed curve to the original.</summary>
    private static List<Point3> SmoothContour(List<Point3> pts,double sigma,double maxDeviation,out double deviation)
    {
        deviation=0;
        if(pts.Count<8)return pts;
        bool closed=Distance2(pts[0],pts[^1])<1e-8;
        if(!closed)return pts;                       // open (stock-clipped) slices keep their exact shape
        for(int attempt=0;attempt<3;attempt++,sigma*=.5)
        {
            double step=Math.Max(.1,sigma/3);
            double total=0;var cum=new double[pts.Count];
            for(int i=1;i<pts.Count;i++){total+=Math.Sqrt(Distance2(pts[i-1],pts[i]));cum[i]=total;}
            int n=(int)Math.Round(total/step);if(n<24)return pts;
            double ds=total/n;var res=new double[n,2];int seg=0;
            for(int k=0;k<n;k++)
            {
                double t=k*ds;while(seg+2<pts.Count&&cum[seg+1]<t)seg++;
                double len=cum[seg+1]-cum[seg];double u=len<1e-12?0:(t-cum[seg])/len;
                res[k,0]=pts[seg].X+(pts[seg+1].X-pts[seg].X)*u;res[k,1]=pts[seg].Y+(pts[seg+1].Y-pts[seg].Y)*u;
            }
            int half=(int)Math.Ceiling(3*sigma/ds);var w=new double[2*half+1];double wsum=0;
            for(int j=-half;j<=half;j++){w[j+half]=Math.Exp(-.5*(j*ds/sigma)*(j*ds/sigma));wsum+=w[j+half];}
            var outPts=new List<Point3>(n+1);double z=pts[0].Z;
            for(int k=0;k<n;k++)
            {
                double x=0,y=0;
                for(int j=-half;j<=half;j++){int q=((k+j)%n+n)%n;x+=w[j+half]*res[q,0];y+=w[j+half]*res[q,1];}
                outPts.Add(new Point3(x/wsum,y/wsum,z));
            }
            outPts.Add(outPts[0]);
            double dev=0;
            for(int k=0;k<n&&dev<=maxDeviation;k++)
            {
                double best=double.PositiveInfinity;
                // distance to the original polyline (windowed by arc length for speed)
                for(int i=0;i+1<pts.Count;i++)best=Math.Min(best,PointSegmentDistance(outPts[k],pts[i],pts[i+1]));
                dev=Math.Max(dev,best);
            }
            if(dev<=maxDeviation){deviation=dev;return outPts;}
        }
        return pts;
    }

    private static bool LowSideIsInside(MeshSectionContour contour)
    {
        int n=contour.Points.Count-1;double agreement=0;
        if(n<3||contour.OutwardNormals.Count!=n)return true;
        double area=0;for(int i=0;i<n;i++){var a=contour.Points[i];var b=contour.Points[(i+1)%n];area+=a.X*b.Y-b.X*a.Y;}
        double orientation=Math.Sign(area);if(orientation==0)return true;
        for(int i=0;i<n;i++)
        {
            var a=contour.Points[i];var b=contour.Points[(i+1)%n];double dx=b.X-a.X,dy=b.Y-a.Y,len=Math.Sqrt(dx*dx+dy*dy);if(len<1e-12)continue;
            // For a positive (CCW) loop, the interior lies to the left of each edge.
            double ix=-orientation*dy/len,iy=orientation*dx/len;var normal=contour.OutwardNormals[i];agreement+=ix*normal.X+iy*normal.Y;
        }
        return agreement>0;
    }

    private static List<Point3> FitMeshSectionContour(MeshSectionContour contour,double z,double tolerance)
    {
        var fitted=CreateSmoothOffsetContour(contour,0,z,tolerance);
        if(fitted.Count<4)return contour.Points.ToList();
        double maxDeviation=0;int n=contour.Points.Count-1;
        for(int i=0;i+1<fitted.Count;i++)
        {
            var p=fitted[i];double nearest=double.PositiveInfinity;
            for(int j=0;j<n;j++)nearest=Math.Min(nearest,PointSegmentDistance(p,contour.Points[j],contour.Points[j+1]));
            maxDeviation=Math.Max(maxDeviation,nearest);
            if(maxDeviation>tolerance)return contour.Points.ToList();
        }
        return fitted;
    }

    private static double PointSegmentDistance(Point3 p,Point3 a,Point3 b)
    {
        double dx=b.X-a.X,dy=b.Y-a.Y,den=dx*dx+dy*dy;
        double t=den<1e-20?0:Math.Clamp(((p.X-a.X)*dx+(p.Y-a.Y)*dy)/den,0,1);
        double ex=p.X-(a.X+dx*t),ey=p.Y-(a.Y+dy*t);return Math.Sqrt(ex*ex+ey*ey);
    }

    private static List<List<Point3>> StockInsetPaths(bool[,] active,bool[,] legal,StockBounds stock,double h,double z,double toolRadius,double stepover)
    {
        double minX=stock.MinX+toolRadius+2*h,maxX=stock.MaxX-toolRadius-2*h;
        double minY=stock.MinY+toolRadius+2*h,maxY=stock.MaxY-toolRadius-2*h;
        double maxInset=Math.Min(maxX-minX,maxY-minY)*.5;var paths=new List<List<Point3>>();
        if(maxInset<=0)return paths;
        double tolerance=Math.Max(1e-5,Math.Min(stepover*.02,toolRadius*.01));
        for(double inset=0;inset<=maxInset;inset+=stepover)
        {
            double x0=minX+inset,x1=maxX-inset,y0=minY+inset,y1=maxY-inset;
            if(x1-x0<=stepover||y1-y0<=stepover)break;
            double corner=Math.Min(Math.Min(toolRadius,stepover*.35),Math.Min(x1-x0,y1-y0)*.2);
            var path=RoundedRectangle(x0,y0,x1,y1,corner,z,tolerance);bool safe=true;int activeCount=0,sampleCount=0;
            for(int i=0;i+1<path.Count&&safe;i++)
            {
                var a=path[i];var b=path[i+1];int steps=Math.Max(1,(int)Math.Ceiling(Math.Sqrt((b.X-a.X)*(b.X-a.X)+(b.Y-a.Y)*(b.Y-a.Y))/Math.Max(h*.5,1e-6)));
                for(int q=0;q<=steps;q++)
                {
                    double t=(double)q/steps;var p=new Point3(a.X+(b.X-a.X)*t,a.Y+(b.Y-a.Y)*t,z);int iCell=(int)Math.Round((p.X-stock.MinX)/h),jCell=(int)Math.Round((p.Y-stock.MinY)/h);
                    if(iCell<0||jCell<0||iCell>=legal.GetLength(0)||jCell>=legal.GetLength(1)||!legal[iCell,jCell]){safe=false;break;}
                    if(active[iCell,jCell])activeCount++;sampleCount++;
                }
            }
            if(!safe)break;
            if(activeCount>=Math.Max(2,(int)Math.Ceiling(sampleCount*.02)))paths.Add(path);
            else if(paths.Count>0)break;
        }
        return paths;
    }

    private static List<Point3> RoundedRectangle(double x0,double y0,double x1,double y1,double radius,double z,double tolerance)
    {
        if(radius<=1e-9)return new(){new(x0,y0,z),new(x1,y0,z),new(x1,y1,z),new(x0,y1,z),new(x0,y0,z)};
        var result=new List<Point3>();var corners=new[]{(X:x1-radius,Y:y0+radius,A:-Math.PI/2),(X:x1-radius,Y:y1-radius,A:0.0),(X:x0+radius,Y:y1-radius,A:Math.PI/2),(X:x0+radius,Y:y0+radius,A:Math.PI)};
        double maxAngle=2*Math.Acos(Math.Clamp(1-tolerance/radius,-1,1));if(!double.IsFinite(maxAngle)||maxAngle<1e-3)maxAngle=.1;
        foreach(var c in corners)
        {
            int n=Math.Max(2,(int)Math.Ceiling((Math.PI/2)/maxAngle));
            for(int i=0;i<=n;i++){double a=c.A+(Math.PI/2)*i/n;result.Add(new Point3(c.X+radius*Math.Cos(a),c.Y+radius*Math.Sin(a),z));}
        }
        result.Add(result[0]);return result;
    }

    private static List<MeshSectionContour> MeshContoursAtZ(MeshInput mesh,double z,StockBounds stock,double toolRadius,double h)
    {
        const double keyScale=1_000_000;var edges=new List<MeshSectionEdge>();var seen=new HashSet<SectionEdgeKey>();var coordinates=new Dictionary<SectionKey,Point3>();
        SectionKey Key(Point3 p)=>new((long)Math.Round(p.X*keyScale),(long)Math.Round(p.Y*keyScale));
        // Keep each mesh boundary edge with its adjacent triangle. A height slice through
        // an open surface ends at the mesh perimeter; adding the perimeter portions below
        // the slice level closes the low-side machining region without an arbitrary chord.
        var boundary=new Dictionary<(int A,int B),(int Count,int Start,int End,int Opposite)>();
        foreach(var face in mesh.Faces)
        {
            if(face.Length<3)continue;var a=mesh.Vertices[face[0]];var b=mesh.Vertices[face[1]];var c=mesh.Vertices[face[2]];
            if(a.Length<3||b.Length<3||c.Length<3)continue;
            for(int e=0;e<3;e++)
            {
                int ia=face[e],ib=face[(e+1)%3],io=face[(e+2)%3];var key=ia<ib?(ia,ib):(ib,ia);
                if(boundary.TryGetValue(key,out var old))boundary[key]=(old.Count+1,old.Start,old.End,old.Opposite);
                else boundary[key]=(1,ia,ib,io);
            }
            double za=a[2]-z,zb=b[2]-z,zc=c[2]-z;
            if(Math.Abs(za)<1e-10&&Math.Abs(zb)<1e-10&&Math.Abs(zc)<1e-10)continue;
            var tri=new[]{new Point3(a[0],a[1],a[2]),new Point3(b[0],b[1],b[2]),new Point3(c[0],c[1],c[2])};var hit=new List<Point3>(3);
            for(int e=0;e<3;e++)
            {
                var p=tri[e];var q=tri[(e+1)%3];double dp=p.Z-z,dq=q.Z-z;
                if(Math.Abs(dp)<1e-10)AddSectionPoint(hit,new Point3(p.X,p.Y,z));
                if((dp< -1e-10&&dq>1e-10)||(dp>1e-10&&dq< -1e-10))
                {double t=dp/(dp-dq);AddSectionPoint(hit,new Point3(p.X+t*(q.X-p.X),p.Y+t*(q.Y-p.Y),z));}
            }
            if(hit.Count<2)continue;
            var pair=(A:hit[0],B:hit[1],D:0.0);
            for(int i=0;i<hit.Count;i++)for(int j=i+1;j<hit.Count;j++)
            {double d=(hit[i].X-hit[j].X)*(hit[i].X-hit[j].X)+(hit[i].Y-hit[j].Y)*(hit[i].Y-hit[j].Y);if(d>pair.D)pair=(hit[i],hit[j],d);}
            if(pair.D<1e-16)continue;
            double den=(tri[1].X-tri[0].X)*(tri[2].Y-tri[0].Y)-(tri[2].X-tri[0].X)*(tri[1].Y-tri[0].Y);
            if(Math.Abs(den)<1e-14)continue;
            double gx=((tri[1].Z-tri[0].Z)*(tri[2].Y-tri[0].Y)-(tri[2].Z-tri[0].Z)*(tri[1].Y-tri[0].Y))/den;
            double gy=((tri[1].X-tri[0].X)*(tri[2].Z-tri[0].Z)-(tri[2].X-tri[0].X)*(tri[1].Z-tri[0].Z))/den;
            double gl=Math.Sqrt(gx*gx+gy*gy);if(gl<1e-12)continue;
            var ka=Key(pair.A);var kb=Key(pair.B);if(ka.Equals(kb))continue;
            var edgeKey=ka.X<kb.X||ka.X==kb.X&&ka.Y<kb.Y?new SectionEdgeKey(ka,kb):new SectionEdgeKey(kb,ka);
            if(!seen.Add(edgeKey))continue;
            coordinates.TryAdd(ka,pair.A);coordinates.TryAdd(kb,pair.B);
            edges.Add(new MeshSectionEdge(ka,kb,pair.A,pair.B,-gx/gl,-gy/gl));
        }
        foreach(var edge in boundary.Values.Where(e=>e.Count==1))
        {
            var a=mesh.Vertices[edge.Start];var b=mesh.Vertices[edge.End];var opposite=mesh.Vertices[edge.Opposite];
            if(a.Length<3||b.Length<3||opposite.Length<3)continue;
            bool aLow=a[2]<=z+1e-10,bLow=b[2]<=z+1e-10;if(!aLow&&!bLow)continue;
            Point3 pa=new(a[0],a[1],a[2]),pb=new(b[0],b[1],b[2]);
            if(aLow!=bLow)
            {
                double t=(z-pa.Z)/(pb.Z-pa.Z);var cross=new Point3(pa.X+t*(pb.X-pa.X),pa.Y+t*(pb.Y-pa.Y),z);
                if(aLow)pb=cross;else pa=cross;
            }
            var midX=(pa.X+pb.X)*.5;var midY=(pa.Y+pb.Y)*.5;double nx=opposite[0]-midX,ny=opposite[1]-midY,nl=Math.Sqrt(nx*nx+ny*ny);
            if(nl<1e-12)continue;nx/=nl;ny/=nl;
            var ka=Key(pa);var kb=Key(pb);if(ka.Equals(kb))continue;
            var key=ka.X<kb.X||ka.X==kb.X&&ka.Y<kb.Y?new SectionEdgeKey(ka,kb):new SectionEdgeKey(kb,ka);
            if(!seen.Add(key))continue;coordinates.TryAdd(ka,pa);coordinates.TryAdd(kb,pb);
            edges.Add(new MeshSectionEdge(ka,kb,pa,pb,nx,ny));
        }
        var adjacent=new Dictionary<SectionKey,List<int>>();
        for(int i=0;i<edges.Count;i++)
        {
            if(!adjacent.TryGetValue(edges[i].A,out var a))adjacent[edges[i].A]=a=new List<int>();a.Add(i);
            if(!adjacent.TryGetValue(edges[i].B,out var b))adjacent[edges[i].B]=b=new List<int>();b.Add(i);
        }
        var used=new bool[edges.Count];var contours=new List<MeshSectionContour>();
        for(int seed=0;seed<edges.Count;seed++)if(!used[seed])
        {
            var start=edges[seed].A;var at=start;var keys=new List<SectionKey>{start};var normals=new List<(double X,double Y)>();int edge=seed,guard=edges.Count+1;
            while(guard-->0&&!used[edge])
            {
                used[edge]=true;var e=edges[edge];normals.Add((e.NX,e.NY));at=e.A.Equals(at)?e.B:e.A;keys.Add(at);
                if(at.Equals(start))break;
                if(!adjacent.TryGetValue(at,out var next))break;
                edge=next.FirstOrDefault(i=>!used[i],-1);if(edge<0)break;
            }
            if(keys.Count<3)continue;
            var points=keys.Select(k=>coordinates[k]).ToList();
            if(!keys[^1].Equals(keys[0]))
            {
                // Open mesh sections are closed to the cutter-safe interior of the base
                // stock. If stock bounds cannot provide a valid route, close the slice
                // directly so downstream offsets never receive an open polygon.
                var closure=CloseToStockBoundary(points[^1],points[0],stock,toolRadius+h*Math.Sqrt(2),z);
                if(closure.Count<2)closure=new(){points[^1],points[0]};
                points.AddRange(closure.Skip(1));
                normals.AddRange(Enumerable.Repeat((X:0.0,Y:0.0),closure.Count-1));
            }
            else points[^1]=points[0];
            if(points.Count<4||Distance2(points[0],points[^1])>1e-10)continue;
            contours.Add(new MeshSectionContour(points,normals));
        }
        return contours;
    }

    private static List<Point3> CloseToStockBoundary(Point3 from,Point3 to,StockBounds stock,double inset,double z)
    {
        double minX=stock.MinX+inset,maxX=stock.MaxX-inset,minY=stock.MinY+inset,maxY=stock.MaxY-inset;
        double w=maxX-minX,h=maxY-minY;if(w<=0||h<=0)return new();
        double perimeter=2*(w+h);
        (Point3 Point,double U,double D2) Project(Point3 p)
        {
            var candidates=new List<(Point3 Point,double U,double D2)>
            {
                (new(Math.Clamp(p.X,minX,maxX),minY,z),Math.Clamp(p.X-minX,0,w),0),
                (new(maxX,Math.Clamp(p.Y,minY,maxY),z),w+Math.Clamp(p.Y-minY,0,h),0),
                (new(Math.Clamp(p.X,minX,maxX),maxY,z),w+h+maxX-Math.Clamp(p.X,minX,maxX),0),
                (new(minX,Math.Clamp(p.Y,minY,maxY),z),2*w+h+maxY-Math.Clamp(p.Y,minY,maxY),0)
            };
            for(int i=0;i<candidates.Count;i++){var c=candidates[i];double dx=p.X-c.Point.X,dy=p.Y-c.Point.Y;candidates[i]=(c.Point,c.U,dx*dx+dy*dy);}
            return candidates.OrderBy(c=>c.D2).First();
        }
        Point3 At(double u)
        {
            u=((u%perimeter)+perimeter)%perimeter;
            if(u<=w)return new(minX+u,minY,z);u-=w;
            if(u<=h)return new(maxX,minY+u,z);u-=h;
            if(u<=w)return new(maxX-u,maxY,z);u-=w;
            return new(minX,maxY-u,z);
        }
        var a=Project(from);var b=Project(to);double forward=(b.U-a.U+perimeter)%perimeter,reverse=(a.U-b.U+perimeter)%perimeter;
        bool goForward=forward<=reverse;double distance=goForward?forward:reverse;
        var result=new List<Point3>{from};if(Distance2(from,a.Point)>1e-12)result.Add(a.Point);
        double[] corners={0,w,w+h,2*w+h,perimeter};
        if(goForward)
        {
            foreach(double c in corners.Concat(corners.Select(x=>x+perimeter)).Where(c=>c>a.U+1e-9&&c<a.U+distance-1e-9).OrderBy(c=>c))result.Add(At(c));
        }
        else
        {
            foreach(double c in corners.Concat(corners.Select(x=>x-perimeter)).Where(c=>c<a.U-1e-9&&c>a.U-distance+1e-9).OrderByDescending(c=>c))result.Add(At(c));
        }
        if(Distance2(result[^1],b.Point)>1e-12)result.Add(b.Point);
        if(Distance2(result[^1],to)>1e-12)result.Add(to);
        return result;
    }

    private static void AddSectionPoint(List<Point3> points,Point3 p)
    {if(points.All(q=>(p.X-q.X)*(p.X-q.X)+(p.Y-q.Y)*(p.Y-q.Y)>1e-16))points.Add(p);}

    private static List<Point3> CreateSmoothOffsetContour(MeshSectionContour contour,double distance,double z,double tolerance)
    {
        int n=contour.Points.Count-1;if(n<3||contour.OutwardNormals.Count!=n)return new();
        var pointNormals=new (double X,double Y)[n];
        for(int i=0;i<n;i++)
        {
            var a=contour.OutwardNormals[(i+n-1)%n];var b=contour.OutwardNormals[i];double nx=a.X+b.X,ny=a.Y+b.Y,len=Math.Sqrt(nx*nx+ny*ny);
            if(len<1e-10){nx=b.X;ny=b.Y;len=Math.Sqrt(nx*nx+ny*ny);}if(len<1e-10)return new();pointNormals[i]=(nx/len,ny/len);
        }
        Point3 Evaluate(int span,double t)
        {
            int i0=(span+n-1)%n,i1=span%n,i2=(span+1)%n,i3=(span+2)%n;double t2=t*t,t3=t2*t,u=1-t;
            double w0=u*u*u/6,w1=(3*t3-6*t2+4)/6,w2=(-3*t3+3*t2+3*t+1)/6,w3=t3/6;
            var p0=contour.Points[i0];var p1=contour.Points[i1];var p2=contour.Points[i2];var p3=contour.Points[i3];
            double x=w0*p0.X+w1*p1.X+w2*p2.X+w3*p3.X,y=w0*p0.Y+w1*p1.Y+w2*p2.Y+w3*p3.Y;
            double nx=w0*pointNormals[i0].X+w1*pointNormals[i1].X+w2*pointNormals[i2].X+w3*pointNormals[i3].X;
            double ny=w0*pointNormals[i0].Y+w1*pointNormals[i1].Y+w2*pointNormals[i2].Y+w3*pointNormals[i3].Y;double nl=Math.Sqrt(nx*nx+ny*ny);
            if(nl>1e-12){x+=distance*nx/nl;y+=distance*ny/nl;}
            return new Point3(x,y,z);
        }
        double Deviation(Point3 p,Point3 a,Point3 b)
        {
            double dx=b.X-a.X,dy=b.Y-a.Y,den=dx*dx+dy*dy;
            double t=den<1e-20?0:Math.Clamp(((p.X-a.X)*dx+(p.Y-a.Y)*dy)/den,0,1);
            double ex=p.X-(a.X+dx*t),ey=p.Y-(a.Y+dy*t);return Math.Sqrt(ex*ex+ey*ey);
        }
        var output=new List<Point3>();
        void Subdivide(int span,double t0,Point3 a,double t1,Point3 b,int depth)
        {
            double tm=(t0+t1)*.5;var mid=Evaluate(span,tm);
            double tq0=(3*t0+t1)*.25,tq1=(t0+3*t1)*.25;
            var q0=Evaluate(span,tq0);var q1=Evaluate(span,tq1);
            double error=Math.Max(Deviation(mid,a,b),Math.Max(Deviation(q0,a,b),Deviation(q1,a,b)));
            if(error<=tolerance||depth>=14){if(output.Count==0||Distance2(output[^1],b)>1e-18)output.Add(b);return;}
            Subdivide(span,t0,a,tm,mid,depth+1);Subdivide(span,tm,mid,t1,b,depth+1);
        }
        var first=Evaluate(0,0);output.Add(first);
        for(int span=0;span<n;span++)Subdivide(span,0,Evaluate(span,0),1,Evaluate(span,1),0);
        if(output.Count>1)output[^1]=output[0];return output;
    }

    private static MeshSectionContour SimplifyMeshContour(MeshSectionContour contour,double tolerance)
    {
        int n=contour.Points.Count-1;if(n<=8||tolerance<=0)return contour;
        var points=contour.Points.Take(n).ToList();var normals=contour.OutwardNormals.ToList();bool changed=true;
        while(changed&&points.Count>8)
        {
            changed=false;
            for(int i=1;i<points.Count&&points.Count>8;i++)
            {
                int prev=i-1,next=(i+1)%points.Count;var a=points[prev];var b=points[i];var c=points[next];
                double dx=c.X-a.X,dy=c.Y-a.Y,den=dx*dx+dy*dy;
                double t=den<1e-20?0:Math.Clamp(((b.X-a.X)*dx+(b.Y-a.Y)*dy)/den,0,1);
                double ex=b.X-(a.X+dx*t),ey=b.Y-(a.Y+dy*t),deviation=Math.Sqrt(ex*ex+ey*ey);
                var n0=normals[prev];var n1=normals[i];double dot=n0.X*n1.X+n0.Y*n1.Y;
                if(deviation>tolerance||dot<.9)continue;
                double nx=n0.X+n1.X,ny=n0.Y+n1.Y,len=Math.Sqrt(nx*nx+ny*ny);if(len>1e-12)normals[prev]=(nx/len,ny/len);
                points.RemoveAt(i);normals.RemoveAt(i);changed=true;i--;
            }
        }
        points.Add(points[0]);return new MeshSectionContour(points,normals);
    }
    /// <summary>Blurs the inward distance field (normalised over safe cells only) so its level sets are smooth sub-cell
    /// curves instead of copying the staircase of the binary legality mask and the chamfer metric.</summary>
    private static double[,] SmoothDistance(double[,] d,bool[,] safe,int nx,int ny)
    {
        double[,] cur=d;
        for(int pass=0;pass<3;pass++)
        {
            var tmp=new double[nx,ny];var dst=new double[nx,ny];
            // [1 4 6 4 1]/16 along x then y, weights counted only for safe cells
            double[] k={1,4,6,4,1};
            for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)
            {
                if(!safe[i,j]){tmp[i,j]=0;continue;}
                double sum=0,ws=0;for(int o=-2;o<=2;o++){int a=i+o;if(a<0||a>=nx||!safe[a,j])continue;sum+=k[o+2]*cur[a,j];ws+=k[o+2];}
                tmp[i,j]=sum/ws;
            }
            for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)
            {
                if(!safe[i,j]){dst[i,j]=0;continue;}
                double sum=0,ws=0;for(int o=-2;o<=2;o++){int b=j+o;if(b<0||b>=ny||!safe[i,b])continue;sum+=k[o+2]*tmp[i,b];ws+=k[o+2];}
                dst[i,j]=sum/ws;
            }
            cur=dst;
        }
        return cur;
    }

    private static List<List<Point3>> OffsetPasses(bool[,] safe,int nx,int ny,StockBounds s,double h,double z,int stride,bool join,double toolRadius)
    {
        // Generate genuine inward level sets on the source grid instead of shrinking a
        // stride-sized block mask (which made the pitch and shape depend on grid alignment).
        // The chamfer distance is a 8-neighbour approximation to Euclidean distance; its
        // threshold is the requested stepover, so local pass spacing is stable to about one cell.
        double pitch=h*stride;var distance=SmoothDistance(DistanceToUnsafe(safe,nx,ny,h),safe,nx,ny);double maxDistance=0;
        for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)if(safe[i,j])maxDistance=Math.Max(maxDistance,distance[i,j]);
        var paths=new List<List<Point3>>();double firstContour=h*1.25;
        int passGuard=(int)Math.Ceiling(maxDistance/pitch)+1;
        for(int pass=0;pass<=passGuard;pass++)
        {
            double inset=firstContour+pass*pitch;if(inset>maxDistance)break;
            paths.AddRange(Contours(distance,nx,ny,s,h,z,inset).Select(c=>SmoothContour(c,2.5*h,.8*h,out _)));
        }
        // Offset rings can disappear before a small center island has a contour with enough
        // vertices to emit. Park et al. describe these residuals as corner, center, and neck
        // uncut regions, then add clean-up TPEs. Detect the same condition by measuring the
        // actual cutter-disk coverage of the emitted rings and rastering only uncovered cells.
        // This also catches small gaps caused by rasterization and narrow concavities.
        var covered=new bool[nx,ny];
        foreach(var path in paths) MarkSweptCells(path,covered,s,h,toolRadius);
        var residual=new bool[nx,ny];bool hasResidual=false;
        for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)
            if(safe[i,j]&&!covered[i,j]){residual[i,j]=true;hasResidual=true;}
        if(hasResidual)
        {
            // Cleanup lanes are spaced no farther apart than the primary stepover. Keeping
            // their centers inside the original safe mask preserves the cutter/part clearance.
            int cleanupStride=Math.Max(1,Math.Min(stride,(int)Math.Floor(toolRadius/h)));
            var cleanup=Linear(residual,nx,ny,s,h,z,cleanupStride);
            paths.AddRange(cleanup);
        }
        if(!join||paths.Count<2)return paths;
        // Link adjacent inset loops only if every sampled connector cell is still cutter-safe.
        var linked=new List<List<Point3>>();var current=paths[0].ToList();
        for(int k=1;k<paths.Count;k++)
        {
            var next=paths[k];var best=(I:0,J:0,D:double.PositiveInfinity);
            for(int i=0;i<current.Count;i++)for(int j=0;j<next.Count;j++){double d=Dist2(current[i],next[j]);if(d<best.D)best=(i,j,d);}
            if(SegmentSafe(current[best.I],next[best.J],safe,s,h))
            {
                var merged=current.Skip(best.I).Concat(current.Take(best.I+1)).ToList();merged.Add(next[best.J]);merged.AddRange(next.Skip(best.J+1));merged.AddRange(next.Take(best.J+1));current=merged;
            }
            else{linked.Add(current);current=next.ToList();}
        }
        linked.Add(current);return linked;
    }

    private static bool[,] SmoothMask(bool[,] mask,bool[,] legal,int nx,int ny)
    {
        var cur=new double[nx,ny];for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)cur[i,j]=mask[i,j]?1:0;
        double[] k={1,4,6,4,1};
        for(int pass=0;pass<2;pass++)
        {
            var tmp=new double[nx,ny];var dst=new double[nx,ny];
            for(int i=0;i<nx;i++)for(int j=0;j<ny;j++){double sum=0;for(int o=-2;o<=2;o++){int a=Math.Clamp(i+o,0,nx-1);sum+=k[o+2]*cur[a,j];}tmp[i,j]=sum/16;}
            for(int i=0;i<nx;i++)for(int j=0;j<ny;j++){double sum=0;for(int o=-2;o<=2;o++){int b=Math.Clamp(j+o,0,ny-1);sum+=k[o+2]*tmp[i,b];}dst[i,j]=sum/16;}
            cur=dst;
        }
        var result=new bool[nx,ny];
        for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)result[i,j]=legal[i,j]&&cur[i,j]>=.4;
        return result;
    }

    private static double[,] DistanceToUnsafe(bool[,] safe,int nx,int ny,double h)
    {
        const double inf=1e20;int w=nx+2,v=ny+2;var seeds=new double[w,v];
        for(int i=0;i<w;i++)for(int j=0;j<v;j++)seeds[i,j]=inf;
        // A one-cell unsafe frame makes the stock/grid boundary part of the distance field.
        for(int i=0;i<w;i++){seeds[i,0]=0;seeds[i,v-1]=0;}
        for(int j=0;j<v;j++){seeds[0,j]=0;seeds[w-1,j]=0;}
        for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)if(!safe[i,j])seeds[i+1,j+1]=0;
        var horizontal=new double[w,v];var input=new double[Math.Max(w,v)];var output=new double[input.Length];
        for(int j=0;j<v;j++)
        {
            for(int i=0;i<w;i++)input[i]=seeds[i,j];DistanceTransform1D(input,output,w);
            for(int i=0;i<w;i++)horizontal[i,j]=output[i];
        }
        var squared=new double[w,v];
        for(int i=0;i<w;i++)
        {
            for(int j=0;j<v;j++)input[j]=horizontal[i,j];DistanceTransform1D(input,output,v);
            for(int j=0;j<v;j++)squared[i,j]=output[j];
        }
        var distance=new double[nx,ny];
        for(int i=0;i<nx;i++)for(int j=0;j<ny;j++)distance[i,j]=Math.Sqrt(squared[i+1,j+1])*h;
        return distance;
    }

    private static void DistanceTransform1D(double[] f,double[] d,int n)
    {
        var sites=new int[n];var boundaries=new double[n+1];int k=-1;
        for(int q=0;q<n;q++)
        {
            if(f[q]>=1e19)continue;
            double intersection=0;
            while(k>=0)
            {
                int p=sites[k];intersection=((f[q]+(double)q*q)-(f[p]+(double)p*p))/(2.0*(q-p));
                if(intersection>boundaries[k])break;k--;
            }
            k++;sites[k]=q;boundaries[k]=k==0?double.NegativeInfinity:intersection;boundaries[k+1]=double.PositiveInfinity;
        }
        if(k<0){Array.Fill(d,1e20,0,n);return;}
        int siteIndex=0;
        for(int q=0;q<n;q++)
        {
            while(siteIndex<k&&boundaries[siteIndex+1]<q)siteIndex++;
            double delta=q-sites[siteIndex];d[q]=delta*delta+f[sites[siteIndex]];
        }
    }

    private static void MarkSweptCells(List<Point3> path,bool[,] covered,StockBounds s,double h,double radius)
    {
        if(path.Count==0)return;
        int reach=(int)Math.Ceiling(radius/h)+1;
        for(int k=0;k<path.Count;k++)
        {
            var a=path[k];var b=k+1<path.Count?path[k+1]:a;
            int samples=Math.Max(1,(int)Math.Ceiling(Math.Sqrt(Dist2(a,b))/Math.Max(h*.4,1e-8)));
            for(int q=0;q<=samples;q++)
            {
                double t=(double)q/samples,x=a.X+(b.X-a.X)*t,y=a.Y+(b.Y-a.Y)*t;
                int ci=(int)Math.Round((x-s.MinX)/h),cj=(int)Math.Round((y-s.MinY)/h);
                for(int di=-reach;di<=reach;di++)for(int dj=-reach;dj<=reach;dj++)
                {
                    int i=ci+di,j=cj+dj;
                    if(i>=0&&j>=0&&i<covered.GetLength(0)&&j<covered.GetLength(1)&&
                       (di*h)*(di*h)+(dj*h)*(dj*h)<=radius*radius+1e-9)covered[i,j]=true;
                }
            }
        }
    }
    private static List<List<Point3>> Contours(double[,] field,int nx,int ny,StockBounds s,double h,double z,double level)
    {
        // Marching squares and linear edge interpolation remove the axis-aligned staircase
        // introduced by tracing each occupied raster cell as a square.
        const int scale=10000;var edges=new List<GridEdge>();
        GridPoint Crossing(int ax,int ay,int bx,int by)
        {
            double va=field[ax,ay],vb=field[bx,by],t=Math.Abs(vb-va)<1e-12?.5:Math.Clamp((level-va)/(vb-va),0,1);
            return new((int)Math.Round((ax+(bx-ax)*t)*scale),(int)Math.Round((ay+(by-ay)*t)*scale));
        }
        for(int i=0;i<nx-1;i++)for(int j=0;j<ny-1;j++)
        {
            bool bl=field[i,j]>=level,br=field[i+1,j]>=level,tr=field[i+1,j+1]>=level,tl=field[i,j+1]>=level;
            int code=(bl?1:0)|(br?2:0)|(tr?4:0)|(tl?8:0);if(code==0||code==15)continue;
            var p=new GridPoint[4];bool[] crosses={bl!=br,br!=tr,tr!=tl,tl!=bl};
            if(crosses[0])p[0]=Crossing(i,j,i+1,j);
            if(crosses[1])p[1]=Crossing(i+1,j,i+1,j+1);
            if(crosses[2])p[2]=Crossing(i+1,j+1,i,j+1);
            if(crosses[3])p[3]=Crossing(i,j+1,i,j);
            var active=Enumerable.Range(0,4).Where(e=>crosses[e]).ToArray();
            if(active.Length==2)edges.Add(new(p[active[0]],p[active[1]]));
            else if(active.Length==4)
            {
                if(code==5){edges.Add(new(p[0],p[3]));edges.Add(new(p[1],p[2]));}
                else{edges.Add(new(p[0],p[1]));edges.Add(new(p[2],p[3]));}
            }
        }
        var incident=new Dictionary<GridPoint,List<int>>();
        for(int i=0;i<edges.Count;i++)
        {
            if(!incident.TryGetValue(edges[i].A,out var a))incident[edges[i].A]=a=new List<int>();a.Add(i);
            if(!incident.TryGetValue(edges[i].B,out var b))incident[edges[i].B]=b=new List<int>();b.Add(i);
        }
        var used=new bool[edges.Count];var loops=new List<List<Point3>>();
        for(int seed=0;seed<edges.Count;seed++)if(!used[seed])
        {
            var start=edges[seed].A;var at=start;var loop=new List<Point3>{ToPoint(start,s,h,z,scale)};int edge=seed,guard=edges.Count+1;
            while(guard-->0&&!used[edge])
            {
                used[edge]=true;var e=edges[edge];at=e.A.Equals(at)?e.B:e.A;loop.Add(ToPoint(at,s,h,z,scale));
                if(at.Equals(start))break;
                if(!incident.TryGetValue(at,out var candidates))break;
                edge=candidates.FirstOrDefault(id=>!used[id],-1);if(edge<0)break;
            }
            if(loop.Count>=4&&loop[^1].Equals(loop[0]))loops.Add(loop);
        }
        return loops.OrderByDescending(p=>Math.Abs(SignedArea(p))).ToList();
    }
    private static bool SegmentSafe(Point3 a,Point3 b,bool[,] safe,StockBounds s,double h)
    {
        int n=Math.Max(2,(int)Math.Ceiling(Math.Sqrt(Dist2(a,b))/Math.Max(h*.5,1e-6)));
        for(int q=0;q<=n;q++){double t=(double)q/n;int i=(int)Math.Round((a.X+(b.X-a.X)*t-s.MinX)/h),j=(int)Math.Round((a.Y+(b.Y-a.Y)*t-s.MinY)/h);if(i<0||j<0||i>=safe.GetLength(0)||j>=safe.GetLength(1)||!safe[i,j])return false;}
        return true;
    }
    private static Point3 ToPoint(GridPoint p,StockBounds s,double h,double z,int scale=1)=>new(s.MinX+p.X*h/scale,s.MinY+p.Y*h/scale,z);
    private static double SignedArea(List<Point3> p){double a=0;for(int i=0;i<p.Count-1;i++)a+=p[i].X*p[i+1].Y-p[i+1].X*p[i].Y;return a*.5;}
    private static double Dist2(Point3 a,Point3 b)=>(a.X-b.X)*(a.X-b.X)+(a.Y-b.Y)*(a.Y-b.Y);

    private static void RasterTriangle(Point3 a,Point3 b,Point3 c,StockBounds s,double h,int nx,int ny,double[,] map)
    {
        double den=(b.Y-c.Y)*(a.X-c.X)+(c.X-b.X)*(a.Y-c.Y);if(Math.Abs(den)<1e-12)return;
        int imin=Math.Max(0,(int)Math.Floor((Math.Min(a.X,Math.Min(b.X,c.X))-s.MinX)/h));int imax=Math.Min(nx-1,(int)Math.Ceiling((Math.Max(a.X,Math.Max(b.X,c.X))-s.MinX)/h));
        int jmin=Math.Max(0,(int)Math.Floor((Math.Min(a.Y,Math.Min(b.Y,c.Y))-s.MinY)/h));int jmax=Math.Min(ny-1,(int)Math.Ceiling((Math.Max(a.Y,Math.Max(b.Y,c.Y))-s.MinY)/h));
        for(int i=imin;i<=imax;i++)for(int j=jmin;j<=jmax;j++)
        {
            double x=s.MinX+i*h,y=s.MinY+j*h;
            double u=((b.Y-c.Y)*(x-c.X)+(c.X-b.X)*(y-c.Y))/den;
            double v=((c.Y-a.Y)*(x-c.X)+(a.X-c.X)*(y-c.Y))/den;double w=1-u-v;
            if(u>=-1e-9&&v>=-1e-9&&w>=-1e-9){double z=u*a.Z+v*b.Z+w*c.Z;if(z>map[i,j])map[i,j]=z;}
        }
    }
    private static QualityReport Measure(List<List<Point3>> paths){var lens=paths.SelectMany(p=>p.Zip(p.Skip(1),(a,b)=>Math.Sqrt((a.X-b.X)*(a.X-b.X)+(a.Y-b.Y)*(a.Y-b.Y)+(a.Z-b.Z)*(a.Z-b.Z)))).ToArray();if(lens.Length==0)return new(0,0,0,0,0,0,0);var mean=lens.Average();var sd=Math.Sqrt(lens.Average(x=>(x-mean)*(x-mean)));var zs=paths.SelectMany(x=>x).Select(x=>x.Z).ToArray();return new(paths.Count,paths.Sum(x=>x.Count),lens.Sum(),mean,mean==0?0:sd/mean,zs.Min(),zs.Max());}
}
