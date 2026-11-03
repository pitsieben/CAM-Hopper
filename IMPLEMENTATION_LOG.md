# CAM Gh implementation log

Updated: 2 November 2026 (steps 1–3 of the 6-axis plan added)
Project: Rhino 8 / Grasshopper CAM milling plugin (`CamGh`)

## Purpose and architecture

CamGh is a research prototype for generating 3-axis mesh toolpaths from Grasshopper. Grasshopper supplies operation commands and job inputs; a separate local .NET worker performs the computation over loopback HTTP at `127.0.0.1:47831`. The CAM Core component starts/checks the worker and reports its status. The worker is managed with Rhino's process lifecycle and is intended to stop when Rhino exits.

## Grasshopper components implemented

- **Tool** — milling tool diameter, flat or ball shape, flute count (default 2), spindle RPM, and calculated peripheral cutting speed (`Vc = π × diameter × RPM / 1000`).
- **Cut Parameters** — stepover and stepdown percentages, pattern selector (integer values 0–5), engage/retract choices, clearance mode/value, and stock allowance.
- **Stock** — axis-aligned stock bounds.
- **Machine Setup** — machine name, units, and safe-start Z.
- **Horizontal Roughing** — creates a typed roughing command from target mesh, tool, cut parameters, and optional grid resolution.
- **Parallel Finishing** — creates a typed finishing command from target mesh, tool, cut parameters, direction, and edge inset.
- **CAM Core** — accepts operation commands, stock, and optional machine setup; returns toolpath points, per-operation results, a job summary, and engine status.
- **CAM Analysis** — accepts CAM Core's per-operation results; reports paths, points, cut length, RPM, cutting speed, and mean Z for each path. It returns point and curve trees grouped by operation and path, including the **Contour Lines** curve output.

- **Multi-Axis Roughing** (new) — creates a 3+2 roughing command: part mesh, tool, cut parameters, optional tool directions, max orientations, tool reach, fixture Z, grid and stock resolution.
- **Region Finishing** (new) — creates a 3+2 finishing command: part mesh, tool, cut parameters, optional tool directions, max orientations, max wall angle, sample step, overlap rings.
- **CAM Core** additionally returns **Tool Axes** (one per motion branch), **Tool Planes** (robot targets per point) and **Stock Preview** (remaining 3D stock). New outputs are appended after the existing ones, so existing wiring is unchanged.

All components have distinct transparent 50×50 icons. Pattern slider mapping: `0 Offset`, `1 Offset Spiral`, `2 Linear`, `3 Spiral`, `4 Radial`, `5 High Speed`.

## Toolpath generation implemented

### Horizontal roughing

- Rasterizes a triangulated target into an upper-envelope XY height field for stock tracking and conservative cutter-safety checks. Projected overlapping triangles use their highest Z, so this assumes a single-valued upper surface and does not model overhangs as full 3D geometry.
- Generates flat-tool **Offset** passes from mesh/Z-level section contours, with tolerance-bounded periodic cubic fitting and Clipper2 polygon offsets to clean splits and self-intersections.
- Closes open mesh section chains against the cutter-safe interior boundary derived from the base stock bounds; if that route is unavailable, it falls back to a direct closure so the offset stage receives a closed loop.
- Uses stock and remaining-material checks to reject unsafe or inactive paths, updates the in-process height field after paths, and attempts safe linking/retract behavior.
- Stepdown and stepover percentages are converted to distances using tool diameter.

### Parallel finishing and other patterns

- Parallel finishing currently uses fixed-pitch XY scanlines intersected with mesh triangles. It does not yet maintain constant geodesic spacing or constant scallop height on curved surfaces.
- Offset Spiral, Spiral, Linear, Radial, and High Speed are prototype raster-based patterns. High Speed is not yet an adaptive-load HSM solver.
- The Lambregts-style segment-site Voronoi/monotone-area spiral approach is not implemented yet; current Offset is a contour-first vector-offset method.

### Links and analysis

- CAM Core combines operation paths and adds clearance-plane motions/retracts. Stay-down links are conservative; the current stock model cannot prove all links collision-free.
- CAM Analysis reports peripheral cutting speed and RPM. Feed rate, machining-time estimation, controller-specific post-processing, and NC output are not implemented.

## 6-axis plan, steps 1–3 (2 November 2026)

Source: `CamGh: Toolpath strategies for 6-axis robot milling` (the design doc). Steps 1–3 are implemented; steps 4–5 (heat-method helical finishing, grain- and curvature-aware field) are not.

### Step 1 — 3D stock model (tri-dexel)

- `TriDexelStock`: three ray families along X, Y and Z, each storing sorted material intervals, so undercuts and through-holes are represented. The default spacing is the longest stock side / 250 (≥ 0.25 mm), capped at about 3M rays.
- Removal is exact per ray against the swept cutter: a stadium prism for flat end mills, and a capsule plus prism for ball end mills. Plunges extend the cutter; ramps are chained static cutters.
- Queries: volume, point classification, the top of the remaining material along any tool axis (a conservative splat), and a preview as line segments.
- CAM Core owns one stock per job. Every roughing operation updates it, including the existing Horizontal Roughing. Finishing does not.
- Roughing subtraction models the cutter body as about one stepdown deep. This errs towards keeping material, so later operations may cut air but never assume air that is still wood.

### Step 2 — R1 multi-orientation 3+2 roughing

- `MultiAxisRoughPlanner` runs the existing Z-level engine in a rotated tool frame (`Frame`; the axis points from tip to spindle; the world origin is kept).
- `RoughingContext` (a new, optional hook on `HorizontalRoughPlanner`) seeds the height field from the 3D stock top, limits depth to the tool reach, adds a fixture-clearance test for the lowest point of the tool, lets the cutter travel around the stock, and forces raster patterns. Without a context the planner behaves exactly as before. The single behavioural change, which does not alter existing results, is that the stock update now only lowers the remaining height.
- Directions are either given or chosen automatically. The automatic mode is plan → cut → re-score: about 60 candidate axes, each scored by the remaining material above part plus allowance, within reach and above the fixture. It stops below 0.5 % of block volume or at the max count.
- Paths are simplified (0.005 mm), then subtracted from the stock.

### Step 3 — F1 region finishing

- `RegionFinishPlanner`: accessibility test (wall angle plus line of sight towards the spindle), greedy area-cover axis choice, most-head-on assignment with neighbour smoothing, and region growth by overlap rings.
- Lanes run along the region's principal direction. Every sample is a `DropCutter` (vertex, edge and facet tests for ball and flat cutters) against the whole part, so it is gouge-free with respect to all faces. A sample is kept only when the contact face is in the region. For ball cutters the stock to leave is an exact normal offset.
- The operation note reports the surface area contacted and the area no chosen axis can reach.

### Step 3b — patch segmentation for F1 (2 November 2026, evening)

This replaces the user's Grasshopper pipeline (k-means on face normals plus Kangaroo border smoothing) with a built-in `PatchSegmenter`:

- **Admissibility:** a face may join an axis only if it is within the wall angle of that axis and in line of sight of the spindle. Axes stay at or above the horizon.
- **Spherical k-means (Lloyd):** each axis moves to the area-weighted mean normal of its faces, with partial moves when a full move would lose coverage.
- **Automatic k:** axes are added from about 100 candidates until the surface is covered, then redundant patches are merged.
- **Border relaxation:** a radius-based mode filter, followed by an edge-length (Potts/ICM) clean-up and island merging.

Region Finishing now uses the segmenter. The command gains `BorderSmoothing` (mm, default 12). `OperationResult` gains `FaceLabels`, `FaceTilt` and `PatchAxes`, and CAM Core gains `Patches` and `Tilt Map` mesh outputs. `TriMesh` now records the source face for each triangle, plus shared-edge lengths.

Test-part results (75° wall):

| Method | Mean tilt | Border length |
|---|---|---|
| Greedy assignment (previous F1) | 30.3° | 2,694 mm |
| k-means, no smoothing | 28.9° | 2,811 mm |
| k-means, 12 mm smoothing | 30.8° | 2,259 mm |

At a 45° wall limit: 9 patches, 22.4° mean tilt, 3.9 % of the area unreachable (downward-facing surface). Tests check that every patch face respects the wall limit and line of sight, that no axis points below the horizon, and that smoothing shortens the borders.

### Step 3c — position-weighted clustering and curvature-flow border smoothing (3 November 2026)

This reproduces the rest of the user's Grasshopper workflow: clustering weighted by both normal and position, and Kangaroo relaxation of the patch borders.

- **Clustering:** k-means cost = (1 − cos tilt) + `PositionWeight` × (distance to patch centre / half diagonal)²; Lloyd updates both axes and centres. `RegionFinishingCommand.PositionWeight` defaults to 0.5.
- **`MeshDiffusion` (new):** cotangent stiffness plus lumped mass matrices, solving (M + tL)u = Mu₀ with Jacobi-preconditioned CG. No packages; it is also the operator the heat method needs in step 4.
- **Border smoothing:** MBO threshold dynamics (diffuse each patch indicator, take the arg-max at each vertex, repeat 6×; total time = radius² / 2). This is curvature flow, the equivalent of edge-length goals, with no border tracking. The admissibility mask is applied per vertex (majority of incident area) and exactly per face at evaluation.
- **Sub-face borders:** the final diffused field is kept per vertex. `PatchSegmentation.LabelAt` and `Margin` evaluate it at barycentric points. Lane samples use the exact contact point (closest point on the contact face to the ball centre or tip), and the overlap band is a field margin, so the band edge is smooth as well. `Refine` splits faces for previews; `SmoothBorderLength` measures the zero-crossing curves.
- **Outputs:** `PatchTriangles` (border-refined preview per patch) on the operation result; Grasshopper gains a `Position Weight` input, and `Patches` now uses the refined triangles. `BorderSmoothing` now defaults to 16 mm.

Results on the test torus:

| Setting | Patches | Mean tilt | Border length |
|---|---|---|---|
| Normals only, no smoothing | 6 | 28.9° | 2,811 mm |
| Position 0.5, smoothing 16 mm | 5 | 32.3° | 1,961 mm |

Tessellation check with axes held fixed, comparing two coarse meshes (72 × 36 and a jittered, diagonal-flipped 60 × 30) against a 240 × 60 reference:

- Evaluating borders inside faces matched the reference better than face-level labels in every setting: +0.5 to +1 percentage points.
- Overall agreement of 91–95 % is limited by where the clustering puts borders between near-equal normals, not by the tessellation.

All earlier checks still pass, including the gouge check on finishing.

### Job output

- `OperationResult.PathAxes`: one tool axis per path.
- `CamJobResult.MotionAxes`: one per motion. Motions are split wherever the axis changes. Vertical work keeps the absolute clearance plane; other axes clear the stock box by the clearance offset along their own axis.
- `StockPreview` and the stock volumes are reported in the summary.

### Verification

`tests/Program.cs` is a headless test. All checks passed:

- Stock removal matches the analytic slot volume (2,900 vs 2,893 mm³), and side cuts along a horizontal axis remove only the cutter volume.
- Test part: an upright wavy torus with a horizontal through-hole (28,800 triangles) in a 170 × 70 × 240 mm block.
- Roughing (Ø12 flat, 6 mm stepdown, 1 mm allowance) chose 6 axes and removed 67 % of the block in about 9 s. No CL point came inside part plus allowance. All 1,763 interior part samples are still material.
- Finishing (Ø6 ball, 0.9 mm lanes) used 6 regions and contacted 96.6 % of the area, with 0.7 % unreachable, in about 2 s. An independent point–triangle check confirmed every sampled ball centre is at least the tool radius from the mesh.
- Full job through JSON round trip: 12 motions, each starting outside the stock along its own axis.

## Known limits and safety status

- This is a research prototype, not machine-ready CAM. Always simulate and inspect generated paths before machining.
- Cutter engagement/load, holder collision, overhang-aware 3D stock removal, and complete collision proof are not implemented.
- The roughing safety field is grid-based and limited to two million cells by default; the grid is used for safety/stock checks even when vector Offset creates the cutting curves.
- Engage settings and controller feed rates are not applied to generated paths.
- 3+2 operations: moves between orientations are clearance moves only; the robot reorients there, so these moves must be checked in robot simulation. Holder and spindle collision is approximated by the tool-reach limit. Region finishing leaves seams between regions. Flat-cutter finishing applies the allowance along the axis.
- Path-level Z in CAM Analysis is the mean Z of each path. For horizontal roughing this indicates its layer; for parallel finishing it is a summary of that path's varying heights.

## Build and package status

- The Grasshopper project built successfully on 28 October 2026 with zero warnings and zero errors after the analysis component and stock-relative contour closure changes.
- The self-contained worker and `.gha` package were assembled under `publish/package` and copied to `C:\Users\pitsi\Documents\Code\CAM gha\publish\package`.
- Rhino runtime behavior and the newest contour coverage have not been visually validated in Rhino after that build. Restart Rhino and load/install the updated package to make the new component and code available.

- 2 November 2026: the restored 28 October source was compile-checked, with the steps 1–3 additions, against the Rhino 8 RhinoCommon/Grasshopper assemblies (zero warnings) using the C# compiler bundled with PowerShell 7, because no .NET SDK was available in that environment. It has **not** yet been built with `dotnet build` or loaded in Rhino. Rebuild, then republish `publish/package` (engine + .gha), before using the new components. The worker (`CamGh.Engine/Program.cs`) gained a larger request-body limit; that file was not compile-checked.

- 3 November 2026: CAM Core Settings window and engine dashboard.
  - Core: `TriDexelStock.ToMesh` (watertight surface nets from the exact ray crossings). `CamJobPlanner.Generate(job, snapshots)` records the stock before and after every operation. `MotionOperations` maps motions to operations. Finishing operations (parallel and region) are now subtracted from the stock, so the remaining volume and the per-operation removal include finishing. `JobScene.cs` holds the scene summary and binary blobs.
  - Engine: `EngineState.cs` (version 0.3.0, uptime, issues, activity log). Endpoints `/`, `/api/v1/status`, `/api/v1/scene*`. `wwwroot/index.html` is the dashboard: Overview and Simulation, WebGL2, SVG tool drawing.
  - Grasshopper: `CamGhCoreAttributes.cs` adds the Settings button. `CamGhEngineWindow.cs` is now an Eto `WebView` window on the dashboard, with a browser fallback. CAM Core autostarts the engine and shows its status as the component message. The old status pop-up on placement is gone.
  - Tests: a stock-surface watertight and volume check was added. All checks pass. The dashboard was checked with the real job data in headless Chromium. The Grasshopper and engine projects compile against the Rhino 8 and ASP.NET assemblies, but have not been run in Rhino yet.

- 3 November 2026: one-click packaging. A new publish profile, `CamGh.Engine/Properties/PublishProfiles/CamGhPackage.pubxml`, publishes the engine. A post-publish target, `CamGhPackage` in `CamGh.Engine.csproj`, then builds the Grasshopper project and fills `publish/package`. It has not been run yet, because no .NET SDK was reachable from the build environment. The first Publish in Visual Studio is its test.

## Main project files

- `CamGh.Core\HorizontalRoughing.cs` — roughing layers, mesh contours, closure, offsets, stock/collision checks, linking.
- `CamGh.Core\CamModels.cs` — tools, cut parameters, commands, operation/job results, job planning.
- `CamGh.Core\Toolpaths.cs` — parallel finishing baseline and quality metrics.
- `CamGh.Core\Geometry.cs` — `Vec3`, tool `Frame`, `TriMesh` (oriented, adjacency), `TriangleBins`.
- `CamGh.Core\TriDexelStock.cs` — 3D in-process stock.
- `CamGh.Core\DropCutter.cs` — ball/flat drop-cutter.
- `CamGh.Core\Orientations.cs` — candidate axes, visibility scoring, part envelopes.
- `CamGh.Core\MultiAxisRoughing.cs` — R1 planner.
- `CamGh.Core\RegionFinishing.cs` — F1 planner.
- `CamGh.Core\PatchSegmentation.cs` — k-means patch segmentation (normal + position), curvature-flow border smoothing, sub-face labels.
- `CamGh.Core\MeshDiffusion.cs` — cotangent heat diffusion (CG solver), shared with the planned heat method.
- `CamGh.Grasshopper\MultiAxisRoughComponent.cs`, `RegionFinishingComponent.cs` — new components.
- `tests\Program.cs`, `tests\CamGh.Tests.csproj` — headless regression test.
- `CamGh.Engine\Program.cs` — local worker API and dashboard endpoints.
- `CamGh.Engine\EngineState.cs` — status, activity log, last job scene.
- `CamGh.Engine\wwwroot\index.html` — Settings window dashboard (Overview, Simulation).
- `CamGh.Core\JobScene.cs` — dashboard scene data and binary formats.
- `CamGh.Grasshopper\CamGhCoreAttributes.cs`, `CamGhEngineWindow.cs` — Settings button and dashboard window.
- `CamGh.Grasshopper\CamGhCoreComponent.cs` — Grasshopper job submission and outputs.
- `CamGh.Grasshopper\OperationAnalysisComponent.cs` — operation report, speed, path level, point and curve outputs.
- `CamGh.Grasshopper\ComponentIcons.cs` — transparent component icons.
- `README.md` in the project root — build instructions, component connections, and algorithm notes.
