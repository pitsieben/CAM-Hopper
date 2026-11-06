# Algorithms and limits

Technical notes on how CamGh plans toolpaths. For the project overview see the [README](../README.md); for the build history see [IMPLEMENTATION_LOG.md](IMPLEMENTATION_LOG.md).

## Multi-axis (3+2) roughing and region finishing

These operations target closed, sculptural parts with undercuts and through-holes, milled from a block by a 6-axis robot. Every path is machined with one fixed tool axis (3+2 / indexed). The robot's sixth axis, rotation about the spindle, is left free for the robot software to use for reach and singularity avoidance.

**Conventions.** A *direction* or *tool axis* points from the cutter tip towards the spindle: `+Z` machines from above and `+X` machines the +X side. CAM Core's `Tool Planes` output has its origin at the cutter tip, Z pointing along the tool into the material (spindle → tip) and X along the feed direction. This is the usual target convention for robot plugins; check your plugin's TCP definition once.

**Shared 3D stock (tri-dexel).** CAM Core keeps one stock model for the whole job. It is made of three families of rays along X, Y and Z, and each ray stores the material intervals it crosses, so undercuts and holes are represented. Roughing cuts are subtracted using exact ray intersections with the swept cutter: a stadium prism for a flat end mill, and a capsule plus prism for a ball. Volumes match analytic values within the ray spacing. Every roughing operation, including the older Horizontal Roughing, updates this stock. `Stock Preview` shows what is left.

**Multi-Axis Roughing (R1).**
1. For each tool direction, the part and the remaining 3D stock are expressed in that tool frame.
2. The existing stock-aware Z-level engine runs unchanged in that frame. Its height field is seeded from the top of the remaining 3D stock, so a direction only cuts what earlier directions left.
3. The paths are subtracted from the stock before the next direction is planned.

Directions can be given, or chosen automatically by visibility. In the automatic mode, each round scores about 60 candidate axes by the remaining material above the part (plus allowance) within reach and above the fixture. The best axis is roughed for real, the stock is updated, and the scoring repeats, so later axes target the terraces and shadows that really remain. Selection stops when the next gain is below 0.5 % of the block, or at the maximum count. `Tool Reach` limits depth below the highest remaining material (default 4 × diameter). `Fixture Z` keeps every part of the tool above the clamping base. Raster patterns are used in rotated frames, because the mesh-section Offset contours assume an open height-field surface.

**Region Finishing (F1).**
1. **Patch segmentation.** This is k-means on face normals and position, built into the worker in place of a Grasshopper k-means plus Kangaroo pipeline.
   - The cost of putting a face in a patch is (1 − cos tilt) plus `Position Weight` × (distance to the patch centre / half the part diagonal)². With 0 it clusters on normals only; higher values give compact patches (default 0.5).
   - A face may only join an axis if its normal is within `Max Wall Angle` of the axis and the line from the face towards the spindle is not blocked by the part. Axes never point below the horizon.
   - Lloyd iterations move each axis to its patch's area-weighted mean normal (shortened if coverage would drop) and each centre to its mean position.
   - The number of patches is automatic: axes are added until the surface is covered, then redundant patches are merged. `Max Wall Angle` is the main control and `Max Orientations` caps the count.
2. **Border smoothing (curvature flow).** `Border Smoothing` is a radius in mm (default 16).
   - Each patch's indicator is diffused over the surface with the cotangent heat operator, every vertex takes the strongest patch it may belong to, and this repeats a few times (Merriman–Bence–Osher threshold dynamics). Borders move by their curvature, which is what relaxing border polylines with edge-length goals in Kangaroo does. Junctions, closed loops and vanishing patches need no special handling, and features smaller than the radius disappear.
   - The final field is kept per vertex and interpolated inside triangles, so borders run as smooth curves through faces instead of zig-zagging along mesh edges. That matters on coarse quad meshes from SubD.
   - The wall limit and line of sight are still enforced at every point.
   - Fragments below about 16 r² are merged first.
3. Given directions skip the clustering; faces are assigned and smoothed only.
4. Patches are grown by `Overlap Rings` so seams overlap.
5. Each patch gets parallel lanes along its long direction. Every sample is a drop-cutter (vertex, edge and facet contact tests for ball and flat cutters) against the whole part, so lanes never gouge neighbouring patches. A sample is kept when the exact contact point lies inside the patch's smooth border, or within the overlap margin set by `Overlap Rings`, so the lane ends follow the smooth borders too.
6. For ball cutters, the stock to leave is an exact normal offset, achieved by dropping a ball of radius r + allowance.

CAM Core's `Patches` output shows one coloured mesh per patch, with faces split where smooth borders cross them (unreachable faces last, in dark red). `Tilt Map` colours each face by the angle between its normal and its patch axis: green 0°, yellow half the wall limit, red at the wall limit, purple unreachable.

The operation note reports the share of surface area actually contacted and the share no chosen axis can reach.

**Verified on a test part** (upright wavy torus with a horizontal through-hole, 28,800 triangles, in a 170 × 70 × 240 mm block): roughing with a Ø12 flat cutter at 6 mm stepdown used 6 axes and removed 67 % of the block in about 9 s. No roughing point came closer than the 1 mm allowance, and no interior part material was removed from the stock. Finishing with a Ø6 ball at 0.9 mm lanes used 6 patches, contacted 96.8 % of the surface and left 0.6 % unreachable in about 4 s.
- Patch segmentation (75° wall): mean tilt 28.9° before smoothing, against 30.3° for the earlier greedy assignment.
- 12 mm border smoothing shortened patch borders from 2,811 to 2,259 mm, raising mean tilt to 30.8°.
- At a 45° wall limit: 9 patches, 22.4° mean tilt. An independent point–triangle check found every ball centre at least the tool radius from the mesh (no gouges). The test lives in `tests/Program.cs` (see Build).

**Limits.**
- Links between orientations are clearance moves only; the robot reorients during them, so check them in robot simulation.
- Holder and spindle collision is covered by the tool-reach limit, not by a full holder model.
- Engagement and load are not controlled.
- Flat-cutter finishing applies the allowance along the axis.
- Seams between finishing regions are visible on the part.

## Parallel finishing

`ParallelPlanner` projects a direction and its perpendicular onto XY, places uniformly spaced scanlines over the projected mesh bounds, intersects each scanline with all mesh triangles, interpolates the crossings linearly, then sorts and joins segments. Disconnected spans remain separate. Quad faces are triangulated by the Grasshopper adapter.

This is the simple fixed-pitch raster baseline. It does not keep constant surface distance or constant scallop height on a curved mesh; it is not PyCAM's drop-cutter, and it does not yet create safe links, lead-ins, or NC post-processing. The supplied shallow/deep NC comparison should be used as a regression reference as the operations mature.

## Horizontal roughing

The roughing command rasterizes the upper mesh envelope into an XY height field for in-process stock and conservative safety checks. Flat-tool Offset geometry is generated separately: each Z level intersects the mesh triangles, stitches the section contours, fits a periodic cubic spline only when its deviation remains within tolerance, and uses robust vector offsets at tool-radius, stock-allowance, and stepover distances. Self-intersections and split loops are cleaned by Clipper2 polygon offsetting; the stock field filters unsafe or inactive loops. Other patterns still emit raster-derived paths. Stepdown and stepover percentages are converted to distances using the tool diameter.

This is a useful prototype for a single-valued upper surface, including an open height-field mesh. It conservatively treats overlapping projected triangles using their highest Z. It does not model engagement/load, handle overhangs as true 3D geometry, account for flute/holder collision, or prove all inter-contour links safe. The grid still governs safety/stock checks and raster-based patterns, subject to the two-million-cell limit. Treat these as candidate paths for visual review, not verified machine code.

Per-operation results include total cut length, point count, average segment length, and segment-length coefficient of variation. These let us compare path discretization quality and are motivated by the supplied commercial CAM study. They are not a feed-rate, machining-time, or roughness prediction. Tool cutting speed uses `Vc = π × diameter × RPM / 1000` and assumes diameter is in millimetres.

## Research notes and next steps

1. **Ball-end cusp spacing (small analytic helper)**: `ScallopMath.PlanarBallEndStep` estimates planar stepover from tool radius and target cusp height: `s = 2 sqrt(2 R h - h^2)`. The geodesic paper generalizes this using signed normal curvature and the two effective cutter radii. That curvature-dependent equation is not implemented yet.
2. **Geodesic iso-distance finishing**: implement a triangle-mesh heat-method solver, geodesic distance contours, curvature-adaptive contour spacing, and uniform curve resampling. Add singularity/coverage checks and a cleanup pass for uncut zones as separate, inspectable stages.
3. **Machining-strip-width tensor**: implement per-contact tensor estimation, eigen-direction analysis, and region partitioning from Liu et al. This is the research route for adapting the preferred direction to curvature and cutter geometry rather than using a global raster direction.
4. **Horizontal roughing**: refine the implemented height-field Minkowski envelope, then add in-process stock updates (adaptive height field or voxel/SDF), engagement limits, and safe linking/retracts. Current paths are stock-bounds-clipped candidates, not full stock simulation.
5. **Machine-aware output**: add safe links, feed/plunge constraints, path simplification with bounded chord error, then controller-specific post processors. Benchmark using path length, segment statistics, and controlled cuts before estimating time or finish.

## Architecture

PRC documents a persistent compute server with lightweight Grasshopper clients: the server owns computation and exposes status/logging, while the Core component forwards changed work and presents results. CamGh now follows that process-lifecycle pattern locally: the engine is a separate worker, and its status window reports startup, health, failure details, and the idle “awaiting commands” state. The current transport remains versioned loopback HTTP/JSON rather than PRC's HTTPS gRPC; algorithms remain in the engine process.
