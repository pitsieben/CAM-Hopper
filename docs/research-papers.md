# Research papers and their relevance to CamGh

Nine papers are in the project library. This page summarises each one and says what it means for CamGh (tri-dexel stock, Z-level and surface-offset roughing, Horizontal/Multi-axis roughing, Parallel/Region finishing, `SurfaceLinker`).

Status: **Implemented** = an idea already in the code, **Planned** = good fit for the roadmap, **Background** = context or longer-term.

## Summary table

| # | Paper | Topic | Relevance | Status |
|---|-------|-------|-----------|--------|
| 1 | Makhanov (2022), *Vector fields for five-axis machining: a survey*, IJAMT 122 | Preferred-direction vector fields for 5-axis finishing | High (finishing strategy) | Background / Planned |
| 2 | Crane, Weischedel, Wardetzky (2017), *The Heat Method for Distance Computation*, CACM 60(11) | Fast geodesic distance by heat diffusion | High (geodesic/offset paths) | Planned |
| 3 | Li, Zhang, Li, Geng (2011), *Tool-path with smooth posture change… cutter accessibility map*, IJAMT 53 | A-map / S-map, posture smoothness | Medium (robot/6-axis) | Planned |
| 4 | Xu, Bradley, Zhang, Loh, Wong (2002), *Tool-path generation for 5-axis machining… accessibility analysis*, IJPR 40(14) | Per-CC-point feasible tool orientations from an STL mesh | High (STL input, collision) | Planned |
| 5 | Han, Zhang, Luo, Wu (2014), *Optimal plunge cutter selection and tool path… impeller channel*, IJAMT 71 | Largest-cutter selection and plunge roughing | Medium | Background |
| 6 | Chen, Li, Tang (2017), *Variable-depth multi-pass tool path on mesh surfaces*, IJAMT 95 | Non-planar interface surfaces, AMRR | High (roughing) | Planned |
| 7 | Lefebvre, Lauwers (2005), *3D morphing for intermediate roughing levels*, CAD&A 2 | Smooth intermediate stock shapes via morphing | High (roughing) | Planned |
| 8 | Matsukawa, Nakatsuji, Nishida (2025), *Automated tool-path generation… 5-axis indexing… STL*, IJAT 19(5) | STL-only CAM with simulation and GPU air-cut removal | Very high (same architecture) | Partly implemented |
| 9 | Dutta, Zhang, Fang, Yigit, Wang, *Vector field based volume peeling for multi-axis machining* | Irrotational vector field → curved roughing layers | High (multi-axis roughing) | Planned |

## 1. Makhanov: vector fields for five-axis machining (survey, 2022)

A survey of about 200 papers on generating tool paths that follow a field of preferred feed directions (VFPD). The direction is chosen to maximise machining strip width, material removal rate, or machine-dependent feed rate. The field is then turned into a path by offsetting an initial curve, tracing streamlines, clustering into regions with regular patterns, or solving a PDE whose iso-lines follow the field. Key points: zigzag and spiral are special cases of space-filling curves; the hard open problem is decomposing a messy field on a real STL part into regular regions; most published methods are tested only on simple parametric surfaces.

**For CamGh:** this is the theory behind Region Finishing (strip-width tensor, regions) and Parallel Finishing direction choice. It supports picking the pass direction per region instead of one global angle, and it warns that kinematic error and rotary-axis speed matter for the robot target. It also confirms that STL robustness is the differentiator.

## 2. Heat Method for distance (Crane et al., 2017)

Geodesic distance is computed in three steps: diffuse heat from the source, normalise the gradient, solve a Poisson equation. The two sparse linear systems can be pre-factored once and reused, so repeated queries are about an order of magnitude cheaper than fast marching. Works on triangle meshes, grids, and point clouds. The paper cites Zou et al. (iso-level tool paths) as a CAM application.

**For CamGh:** a drop-in way to get distance fields on the mesh for iso-scallop or geodesic-parallel finishing and for uncut-region elimination, and a replacement for slow wavefront propagation. Pre-factoring fits the engine model (build once, query per region).

## 3. Li et al. (2011): smooth posture change via accessibility maps

For each cutter-contact point the paper builds an accessibility map (interference-free tilt/rotation range) and a smoothness map (posture change rate per feed direction). It generates an initial path with the smoothest posture change and then adjacent paths with maximum step-over, correcting them to keep posture change small. Compared with iso-planar, it gave roughly half the path length and a much lower average posture change rate in the example.

**For CamGh:** relevant to the 6-axis robot output, where posture jumps cost time and surface quality. The S-map idea can drive tool-orientation smoothing after the geometric planner. Heavy to compute, so it should be sampled sparsely.

## 4. Xu et al. (2002): accessibility analysis from STL

Starts from a triangular mesh, makes cutter-contact points by plane slicing, then for each point builds three feasible domains in tilt/rotation space (machine axis limits, collision, gouging) and intersects them. Verified by real cutting; the traditional fixed lead angle gouged and collided where the method did not. Complexity is O(N²) in facets unless domains are narrowed progressively.

**For CamGh:** the closest published match to the STL-first approach. It describes exactly the per-point collision and gouge checks needed for tilted tools in `MultiAxisRoughing`/finishing, and the order trick (machine limits first, then collision, then gouge) is a cheap speed-up we can use.

## 5. Han et al. (2014): plunge cutter selection for impellers

Finds the largest plunge cutter that can reach every point of an impeller channel (valid angle range per point, intersected over the path), then derives row spacing and step distance from the cutter size. Real test: about 44% shorter path and 40% less time than the experience-chosen cutter.

**For CamGh:** niche but useful. The "largest tool that fits" idea could become a tool-selection helper for roughing, and plunge roughing is a candidate strategy for deep pockets on the robot.

## 6. Chen, Li, Tang (2017): variable-depth multi-pass paths

Instead of constant-Z layers, the cut volume is parametrised into a unit cube and linear interpolation gives smooth non-planar interface surfaces between raw and design surface. The layer spacing is chosen by maximising average material removal rate under cutting-force and kinematic limits. Result: about 18–20% less machining time than constant-Z and no islands or tool transits between them.

**For CamGh:** speaks directly to the staircase terraces we see with Z-level and Horizontal roughing. It would be the principled version of "finer Horizontal levels" on the to-do list, and the AMRR criterion could pick stepdown automatically.

## 7. Lefebvre, Lauwers (2005): 3D morphing for roughing levels

Argues that 2.5D roughing leaves "stairs" that cause large cutting-load swings in semi-finishing, and proposes morphing the stock shape into the part shape to produce smooth intermediate levels, which in turn motivates 5-axis roughing. Roughing is 70–90% of total machining time, so it is where savings matter.

**For CamGh:** the conceptual origin of the surface-offset and morphing ideas; it justifies the surface-offset strategy and the interest in near-net-shape roughing.

## 8. Matsukawa et al. (2025): STL-based tool paths for 5-axis indexing

Generates tool paths directly from an STL model, runs shape simulation at the same time as path generation to find where material actually remains and drop air-cut moves, supports overhangs via 3+2 indexing, and uses GPU parallelism to make the many iterative checks and air-cut detection fast. A machined case study confirmed the result.

**For CamGh:** architecturally almost the same goal as this project (STL in, simulation-aware planning, 3+2). Three takeaways: (a) our shared tri-dexel stock plus remaining-material checks is the right design; (b) air-cut elimination should be a first-class step, which ties to the open speed to-do; (c) GPU offload is the credible route to a big speedup beyond the stock-height-aware ray test.

## 9. Dutta et al.: vector-field volume peeling

Computes a vector field aligned with user anchor vectors, optimises it to be irrotational so that it is the gradient of a scalar field, and uses the iso-surfaces as curved roughing layers. Handles singularities, guarantees collision-free directions and no floating material, and was verified by physical machining. Compared with 2.5D roughing, it cuts shape error and cutting-depth variation during finishing.

**For CamGh:** the most relevant roughing paper for `MultiAxisRoughing`. It generalises Horizontal Roughing (a single vertical direction) to a field of directions, and gives the manufacturability criteria (collision-free direction at every point, no floating volume) that we should test in the planner.

## How they map to the roadmap

- **Roughing quality:** 6, 7, 9 → replace constant-Z terraces with curved layers; pick stepdown by AMRR.
- **Finishing direction and regions:** 1 → per-region preferred direction; 2 → distance fields for scallop-uniform and geodesic paths.
- **Tilted tools and robot:** 3, 4 → accessibility maps, posture smoothing.
- **Speed and architecture:** 8 → simulation-driven air-cut removal and GPU offload; 5 → tool selection.

*Summaries are based on the abstracts and main text of the supplied PDFs; figures in the table are the papers' own reported results, not CamGh measurements.*
