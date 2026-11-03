namespace CamGh.Core;

/// <summary>Lightweight value vector used by the multi-axis kernels (Point3 stays the JSON transport type).</summary>
public readonly record struct Vec3(double X, double Y, double Z)
{
    public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static Vec3 operator -(Vec3 a) => new(-a.X, -a.Y, -a.Z);
    public static Vec3 operator *(Vec3 a, double s) => new(a.X * s, a.Y * s, a.Z * s);
    public static Vec3 operator *(double s, Vec3 a) => new(a.X * s, a.Y * s, a.Z * s);
    public double Dot(Vec3 b) => X * b.X + Y * b.Y + Z * b.Z;
    public Vec3 Cross(Vec3 b) => new(Y * b.Z - Z * b.Y, Z * b.X - X * b.Z, X * b.Y - Y * b.X);
    public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
    public Vec3 Normalized() { double l = Length; return l < 1e-300 ? this : new(X / l, Y / l, Z / l); }
    public double this[int axis] => axis == 0 ? X : axis == 1 ? Y : Z;
    public static Vec3 Axis(int axis) => axis == 0 ? new(1, 0, 0) : axis == 1 ? new(0, 1, 0) : new(0, 0, 1);
    public Point3 ToPoint() => new(X, Y, Z);
    public static Vec3 From(Point3 p) => new(p.X, p.Y, p.Z);
}

/// <summary>
/// Right-handed orthonormal tool frame. W is the tool axis, pointing from the cutter tip towards the spindle,
/// so in local coordinates the tool approaches from +W and every 3-axis algorithm can run unchanged.
/// The origin is the world origin, which keeps W-levels comparable between operations.
/// </summary>
public sealed class Frame
{
    public Vec3 U { get; }
    public Vec3 V { get; }
    public Vec3 W { get; }
    private Frame(Vec3 u, Vec3 v, Vec3 w) { U = u; V = v; W = w; }

    public static Frame FromAxis(Vec3 axis)
    {
        var w = axis.Normalized();
        if (w.Length < 0.5) throw new ArgumentException("Tool axis must be non-zero.");
        // Keep U horizontal where possible so local layers read naturally; +Z reproduces the world frame.
        Vec3 u = Math.Abs(w.Z) > 0.999999 ? new Vec3(1, 0, 0) : new Vec3(0, 0, 1).Cross(w).Normalized();
        if (Math.Abs(w.Z) > 0.999999 && w.Z < 0) u = new Vec3(1, 0, 0);
        var v = w.Cross(u).Normalized();
        u = v.Cross(w).Normalized();
        return new Frame(u, v, w);
    }

    public Vec3 ToLocal(Vec3 p) => new(p.Dot(U), p.Dot(V), p.Dot(W));
    public Vec3 ToWorld(Vec3 l) => U * l.X + V * l.Y + W * l.Z;
    public Vec3 ToWorld(double x, double y, double z) => U * x + V * y + W * z;
    public Point3 ToWorld(Point3 l) => ToWorld(l.X, l.Y, l.Z).ToPoint();

    public MeshInput Transform(MeshInput mesh) => new(
        mesh.Vertices.Select(v => { var l = ToLocal(new Vec3(v[0], v[1], v[2])); return new[] { l.X, l.Y, l.Z }; }).ToArray(),
        mesh.Faces);

    /// <summary>Local axis-aligned bounds of a world box, expanded by a margin on every side.</summary>
    public StockBounds LocalBounds(StockBounds box, double margin)
    {
        double[] lo = { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity };
        double[] hi = { double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity };
        foreach (var c in Corners(box))
        {
            var l = ToLocal(c);
            for (int a = 0; a < 3; a++) { lo[a] = Math.Min(lo[a], l[a]); hi[a] = Math.Max(hi[a], l[a]); }
        }
        return new StockBounds(lo[0] - margin, lo[1] - margin, lo[2] - margin, hi[0] + margin, hi[1] + margin, hi[2] + margin);
    }

    public static IEnumerable<Vec3> Corners(StockBounds b)
    {
        foreach (var x in new[] { b.MinX, b.MaxX }) foreach (var y in new[] { b.MinY, b.MaxY }) foreach (var z in new[] { b.MinZ, b.MaxZ })
            yield return new Vec3(x, y, z);
    }
}

/// <summary>Indexed triangle mesh with unit face normals, areas and edge adjacency.</summary>
public sealed class TriMesh
{
    public Vec3[] Vertices { get; }
    public int[] Tris { get; }               // 3 indices per triangle
    public Vec3[] Normals { get; }
    public double[] Areas { get; }
    public int Count => Tris.Length / 3;
    /// <summary>Index of the input face each triangle came from (degenerate input faces are dropped).</summary>
    public int[] SourceFace { get; }
    private int[][]? _neighbours;
    private (int Face, double Length)[][]? _edges;

    private TriMesh(Vec3[] vertices, int[] tris, int[]? sourceFace = null)
    {
        Vertices = vertices; Tris = tris;
        SourceFace = sourceFace ?? Enumerable.Range(0, tris.Length / 3).ToArray();
        Normals = new Vec3[Count]; Areas = new double[Count];
        for (int f = 0; f < Count; f++)
        {
            var c = (B(f) - A(f)).Cross(C(f) - A(f));
            Areas[f] = c.Length * 0.5; Normals[f] = c.Normalized();
        }
    }

    public Vec3 A(int f) => Vertices[Tris[3 * f]];
    public Vec3 B(int f) => Vertices[Tris[3 * f + 1]];
    public Vec3 C(int f) => Vertices[Tris[3 * f + 2]];
    public Vec3 Centroid(int f) => (A(f) + B(f) + C(f)) * (1.0 / 3.0);

    /// <summary>Builds a mesh, drops degenerate faces, and orients a closed mesh outwards (positive volume).</summary>
    public static TriMesh From(MeshInput input)
    {
        if (input.Vertices.Length < 3 || input.Faces.Length == 0) throw new ArgumentException("A triangulated part mesh is required.");
        var raw = input.Vertices.Select(p => p.Length >= 3 ? new Vec3(p[0], p[1], p[2]) : throw new ArgumentException("Mesh vertices need X, Y and Z.")).ToArray();
        // Weld coincident vertices (Rhino meshes are often split at seams) so closedness and adjacency work.
        var remap = new int[raw.Length]; var welded = new List<Vec3>(raw.Length); var index = new Dictionary<(long, long, long), int>();
        for (int i = 0; i < raw.Length; i++)
        {
            var key = ((long)Math.Round(raw[i].X * 1e5), (long)Math.Round(raw[i].Y * 1e5), (long)Math.Round(raw[i].Z * 1e5));
            if (!index.TryGetValue(key, out int w)) { w = welded.Count; welded.Add(raw[i]); index[key] = w; }
            remap[i] = w;
        }
        var v = welded.ToArray();
        var t = new List<int>(input.Faces.Length * 3); var source = new List<int>(input.Faces.Length);
        for (int fi = 0; fi < input.Faces.Length; fi++)
        {
            var face = input.Faces[fi];
            if (face.Length < 3) continue;
            if (face.Take(3).Any(i => i < 0 || i >= raw.Length)) throw new ArgumentException("Mesh face index out of range.");
            int a = remap[face[0]], b = remap[face[1]], c = remap[face[2]];
            if (a == b || b == c || a == c) continue;
            if ((v[b] - v[a]).Cross(v[c] - v[a]).Length < 1e-14) continue;
            t.Add(a); t.Add(b); t.Add(c); source.Add(fi);
        }
        if (t.Count == 0) throw new ArgumentException("The part mesh has no valid triangles.");
        var mesh = new TriMesh(v, t.ToArray(), source.ToArray());
        if (mesh.IsClosed() && mesh.SignedVolume() < 0)
        {
            var flipped = t.ToArray();
            for (int i = 0; i < flipped.Length; i += 3) (flipped[i + 1], flipped[i + 2]) = (flipped[i + 2], flipped[i + 1]);
            mesh = new TriMesh(v, flipped, source.ToArray());
        }
        return mesh;
    }

    public TriMesh Transformed(Frame frame) => new(Vertices.Select(frame.ToLocal).ToArray(), Tris, SourceFace);

    /// <summary>Edge-adjacent faces with the length of the shared edge.</summary>
    public (int Face, double Length)[][] NeighbourEdges()
    {
        if (_edges is not null) return _edges;
        var byEdge = new Dictionary<(int, int), List<int>>();
        for (int f = 0; f < Count; f++)
            for (int e = 0; e < 3; e++)
            {
                int a = Tris[3 * f + e], b = Tris[3 * f + (e + 1) % 3];
                var k = a < b ? (a, b) : (b, a);
                if (!byEdge.TryGetValue(k, out var list)) byEdge[k] = list = new List<int>(2);
                list.Add(f);
            }
        var n = new List<(int, double)>[Count];
        for (int f = 0; f < Count; f++) n[f] = new List<(int, double)>(3);
        foreach (var (key, list) in byEdge)
        {
            double len = (Vertices[key.Item1] - Vertices[key.Item2]).Length;
            for (int i = 0; i < list.Count; i++) for (int j = 0; j < list.Count; j++) if (i != j) n[list[i]].Add((list[j], len));
        }
        return _edges = n.Select(l => l.ToArray()).ToArray();
    }

    public double SignedVolume()
    {
        double vol = 0;
        for (int f = 0; f < Count; f++) vol += A(f).Dot(B(f).Cross(C(f)));
        return vol / 6.0;
    }

    public bool IsClosed()
    {
        var count = new Dictionary<(int, int), int>();
        for (int f = 0; f < Count; f++)
            for (int e = 0; e < 3; e++)
            {
                int a = Tris[3 * f + e], b = Tris[3 * f + (e + 1) % 3];
                var k = a < b ? (a, b) : (b, a);
                count[k] = count.TryGetValue(k, out var c) ? c + 1 : 1;
            }
        return count.Values.All(c => c == 2);
    }

    /// <summary>Faces sharing an edge with each face.</summary>
    public int[][] Neighbours()
    {
        if (_neighbours is not null) return _neighbours;
        var byEdge = new Dictionary<(int, int), List<int>>();
        for (int f = 0; f < Count; f++)
            for (int e = 0; e < 3; e++)
            {
                int a = Tris[3 * f + e], b = Tris[3 * f + (e + 1) % 3];
                var k = a < b ? (a, b) : (b, a);
                if (!byEdge.TryGetValue(k, out var list)) byEdge[k] = list = new List<int>(2);
                list.Add(f);
            }
        var n = new List<int>[Count];
        for (int f = 0; f < Count; f++) n[f] = new List<int>(3);
        foreach (var list in byEdge.Values)
            for (int i = 0; i < list.Count; i++) for (int j = 0; j < list.Count; j++) if (i != j) n[list[i]].Add(list[j]);
        return _neighbours = n.Select(l => l.Distinct().ToArray()).ToArray();
    }

    public (Vec3 Min, Vec3 Max) Bounds()
    {
        double x0 = double.MaxValue, y0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue, z1 = double.MinValue;
        foreach (var p in Vertices) { x0 = Math.Min(x0, p.X); y0 = Math.Min(y0, p.Y); z0 = Math.Min(z0, p.Z); x1 = Math.Max(x1, p.X); y1 = Math.Max(y1, p.Y); z1 = Math.Max(z1, p.Z); }
        return (new Vec3(x0, y0, z0), new Vec3(x1, y1, z1));
    }

    public double MeanEdgeLength()
    {
        double sum = 0; int n = 0;
        for (int f = 0; f < Count; f += Math.Max(1, Count / 5000)) { sum += (B(f) - A(f)).Length + (C(f) - B(f)).Length + (A(f) - C(f)).Length; n += 3; }
        return n == 0 ? 1 : sum / n;
    }
}

/// <summary>Uniform XY bins over a mesh expressed in a local frame (tool axis = +Z).</summary>
public sealed class TriangleBins
{
    private readonly TriMesh _mesh;
    private readonly double _x0, _y0, _cell;
    private readonly int _nx, _ny;
    private readonly int[][] _bins;
    public TriMesh Mesh => _mesh;

    public TriangleBins(TriMesh localMesh, double cell)
    {
        _mesh = localMesh;
        var (min, max) = localMesh.Bounds();
        double area = Math.Max(1e-9, (max.X - min.X) * (max.Y - min.Y));
        // Keep the bin count bounded (≈ 4M) whatever the requested cell size.
        _cell = Math.Max(cell, Math.Sqrt(area / 4_000_000.0));
        _x0 = min.X - _cell; _y0 = min.Y - _cell;
        _nx = (int)Math.Ceiling((max.X - _x0) / _cell) + 2; _ny = (int)Math.Ceiling((max.Y - _y0) / _cell) + 2;
        var lists = new List<int>?[_nx * _ny];
        for (int f = 0; f < localMesh.Count; f++)
        {
            var a = localMesh.A(f); var b = localMesh.B(f); var c = localMesh.C(f);
            int i0 = CellX(Math.Min(a.X, Math.Min(b.X, c.X))), i1 = CellX(Math.Max(a.X, Math.Max(b.X, c.X)));
            int j0 = CellY(Math.Min(a.Y, Math.Min(b.Y, c.Y))), j1 = CellY(Math.Max(a.Y, Math.Max(b.Y, c.Y)));
            for (int i = i0; i <= i1; i++) for (int j = j0; j <= j1; j++) (lists[i * _ny + j] ??= new List<int>(4)).Add(f);
        }
        _bins = lists.Select(l => l?.ToArray() ?? Array.Empty<int>()).ToArray();
    }

    private int CellX(double x) => Math.Clamp((int)Math.Floor((x - _x0) / _cell), 0, _nx - 1);
    private int CellY(double y) => Math.Clamp((int)Math.Floor((y - _y0) / _cell), 0, _ny - 1);

    /// <summary>Visits every triangle whose bins overlap the rectangle; each triangle is visited once per call.</summary>
    public void Query(double xmin, double xmax, double ymin, double ymax, int[] stamp, ref int stampValue, Action<int> visit)
    {
        stampValue++;
        int i0 = CellX(xmin), i1 = CellX(xmax), j0 = CellY(ymin), j1 = CellY(ymax);
        for (int i = i0; i <= i1; i++)
            for (int j = j0; j <= j1; j++)
                foreach (int f in _bins[i * _ny + j])
                {
                    if (stamp[f] == stampValue) continue;
                    stamp[f] = stampValue; visit(f);
                }
    }

    public int[] NewStamp() => new int[_mesh.Count];

    /// <summary>True when a vertical ray from p towards +Z hits a triangle more than eps above p.</summary>
    public bool IsOccludedAbove(Vec3 p, double eps, int ignoreFace = -1)
    {
        foreach (int f in _bins[CellX(p.X) * _ny + CellY(p.Y)])
        {
            if (f == ignoreFace) continue;
            if (VerticalHit(f, p.X, p.Y, out double z) && z > p.Z + eps) return true;
        }
        return false;
    }

    public bool VerticalHit(int f, double x, double y, out double z)
    {
        var a = _mesh.A(f); var b = _mesh.B(f); var c = _mesh.C(f); z = 0;
        double den = (b.Y - c.Y) * (a.X - c.X) + (c.X - b.X) * (a.Y - c.Y);
        if (Math.Abs(den) < 1e-14) return false;
        double u = ((b.Y - c.Y) * (x - c.X) + (c.X - b.X) * (y - c.Y)) / den;
        double v = ((c.Y - a.Y) * (x - c.X) + (a.X - c.X) * (y - c.Y)) / den;
        double w = 1 - u - v;
        if (u < -1e-12 || v < -1e-12 || w < -1e-12) return false;
        z = u * a.Z + v * b.Z + w * c.Z; return true;
    }
}

/// <summary>Small triangle helpers.</summary>
public static class TriMath
{
    /// <summary>Barycentric coordinates (u,v,w for a,b,c) of the point of triangle abc closest to p (Ericson, Real-Time Collision Detection 5.1.5).</summary>
    public static (double U, double V, double W) ClosestBarycentric(Vec3 p, Vec3 a, Vec3 b, Vec3 c)
    {
        var ab = b - a; var ac = c - a; var ap = p - a;
        double d1 = ab.Dot(ap), d2 = ac.Dot(ap); if (d1 <= 0 && d2 <= 0) return (1, 0, 0);
        var bp = p - b; double d3 = ab.Dot(bp), d4 = ac.Dot(bp); if (d3 >= 0 && d4 <= d3) return (0, 1, 0);
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) { double v = d1 / (d1 - d3); return (1 - v, v, 0); }
        var cp = p - c; double d5 = ab.Dot(cp), d6 = ac.Dot(cp); if (d6 >= 0 && d5 <= d6) return (0, 0, 1);
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) { double w = d2 / (d2 - d6); return (1 - w, 0, w); }
        double va = d3 * d6 - d5 * d4;
        if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) { double w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); return (0, 1 - w, w); }
        double denom = 1 / (va + vb + vc), vv = vb * denom, ww = vc * denom;
        return (1 - vv - ww, vv, ww);
    }
}
