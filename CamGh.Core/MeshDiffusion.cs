namespace CamGh.Core;

/// <summary>
/// Implicit heat diffusion on a triangle mesh: solves (M + t·L) u = M·u0 for the lumped vertex-area matrix M and the
/// cotangent stiffness matrix L (positive semi-definite on any triangle mesh, so the system is always SPD).
/// Backward Euler is unconditionally stable, so the time step can be as large as needed. The same operator is what
/// the heat method for geodesic distance (Crane et al.) factorises, so this class is also the base for that.
/// Solved with Jacobi-preconditioned conjugate gradients; no external packages.
/// </summary>
public sealed class MeshDiffusion
{
    private readonly int _n;
    private readonly double[] _mass;
    private readonly int[] _rowStart, _col;
    private readonly double[] _stiff;          // stiffness values, same sparsity as _col
    private readonly double[] _diagStiff;
    public int VertexCount => _n;
    public double[] VertexArea => _mass;

    public MeshDiffusion(TriMesh mesh)
    {
        _n = mesh.Vertices.Length;
        _mass = new double[_n];
        var entries = new Dictionary<long, double>(mesh.Count * 4);
        void Add(int i, int j, double v)
        {
            long key = (long)i * _n + j;
            entries[key] = entries.TryGetValue(key, out var old) ? old + v : v;
        }
        for (int f = 0; f < mesh.Count; f++)
        {
            int a = mesh.Tris[3 * f], b = mesh.Tris[3 * f + 1], c = mesh.Tris[3 * f + 2];
            var pa = mesh.Vertices[a]; var pb = mesh.Vertices[b]; var pc = mesh.Vertices[c];
            double area = mesh.Areas[f]; if (area < 1e-14) continue;
            double third = area / 3.0; _mass[a] += third; _mass[b] += third; _mass[c] += third;
            // cot of the angle opposite each edge; clamped so near-degenerate faces cannot dominate.
            double cotA = Cot(pb - pa, pc - pa), cotB = Cot(pa - pb, pc - pb), cotC = Cot(pa - pc, pb - pc);
            Edge(a, b, cotC); Edge(b, c, cotA); Edge(c, a, cotB);
        }
        void Edge(int i, int j, double cot)
        {
            double w = 0.5 * cot;                       // L_ij = -w, L_ii += w, L_jj += w
            Add(i, j, -w); Add(j, i, -w); Add(i, i, w); Add(j, j, w);
        }
        for (int i = 0; i < _n; i++) if (_mass[i] <= 0) _mass[i] = 1e-12;
        // CSR
        var rows = new List<(int Col, double V)>[_n];
        for (int i = 0; i < _n; i++) rows[i] = new List<(int, double)>(8);
        foreach (var (key, v) in entries) rows[(int)(key / _n)].Add(((int)(key % _n), v));
        _rowStart = new int[_n + 1]; int nnz = 0;
        for (int i = 0; i < _n; i++) { _rowStart[i] = nnz; nnz += rows[i].Count; }
        _rowStart[_n] = nnz;
        _col = new int[nnz]; _stiff = new double[nnz]; _diagStiff = new double[_n];
        for (int i = 0; i < _n; i++)
        {
            int p = _rowStart[i];
            foreach (var (col, v) in rows[i].OrderBy(e => e.Col)) { _col[p] = col; _stiff[p] = v; if (col == i) _diagStiff[i] = v; p++; }
        }
    }

    private static double Cot(Vec3 u, Vec3 v)
    {
        double cross = u.Cross(v).Length;
        if (cross < 1e-12) return 0;
        return Math.Clamp(u.Dot(v) / cross, -1e3, 1e3);
    }

    /// <summary>Mean edge length estimate from the mean vertex area (equilateral triangles: area per vertex = √3/2 · h²).</summary>
    public double MeanSpacing() => Math.Sqrt(2.0 / Math.Sqrt(3.0) * _mass.Average());

    /// <summary>One backward-Euler diffusion step of duration t applied to u0 (warm start from u0). Returns the CG iteration count.</summary>
    public int Diffuse(double t, double[] u0, double[] result, double tolerance = 1e-6, int maxIterations = 400)
    {
        int n = _n;
        var b = new double[n]; var x = new double[n]; var r = new double[n]; var z = new double[n]; var p = new double[n]; var ap = new double[n];
        var inv = new double[n];
        for (int i = 0; i < n; i++) { b[i] = _mass[i] * u0[i]; x[i] = u0[i]; inv[i] = 1.0 / (_mass[i] + t * _diagStiff[i]); }
        Apply(t, x, ap);
        double bnorm = 0, rz = 0;
        for (int i = 0; i < n; i++) { r[i] = b[i] - ap[i]; bnorm += b[i] * b[i]; }
        bnorm = Math.Sqrt(Math.Max(bnorm, 1e-300));
        for (int i = 0; i < n; i++) { z[i] = r[i] * inv[i]; p[i] = z[i]; rz += r[i] * z[i]; }
        int it = 0;
        for (; it < maxIterations; it++)
        {
            double rn = 0; for (int i = 0; i < n; i++) rn += r[i] * r[i];
            if (Math.Sqrt(rn) <= tolerance * bnorm) break;
            Apply(t, p, ap);
            double pap = 0; for (int i = 0; i < n; i++) pap += p[i] * ap[i];
            if (pap <= 0) break;
            double alpha = rz / pap;
            for (int i = 0; i < n; i++) { x[i] += alpha * p[i]; r[i] -= alpha * ap[i]; }
            double rzNew = 0;
            for (int i = 0; i < n; i++) { z[i] = r[i] * inv[i]; rzNew += r[i] * z[i]; }
            double beta = rzNew / rz; rz = rzNew;
            for (int i = 0; i < n; i++) p[i] = z[i] + beta * p[i];
        }
        Array.Copy(x, result, n);
        return it;
    }

    private void Apply(double t, double[] x, double[] y)
    {
        Parallel.For(0, _n, i =>
        {
            double s = _mass[i] * x[i], l = 0;
            for (int k = _rowStart[i]; k < _rowStart[i + 1]; k++) l += _stiff[k] * x[_col[k]];
            y[i] = s + t * l;
        });
    }
}
