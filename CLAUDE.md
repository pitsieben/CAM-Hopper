# CamGh: working notes for Claude

Rhino 8 / Grasshopper CAM plug-in for subtractive milling (3-axis CNC and 6-axis robot). Planning runs in a separate local engine; Grasshopper is a thin client. Read `README.md` for the overview, `docs/algorithms.md` for how the planners work and `docs/IMPLEMENTATION_LOG.md` for history. Do not re-read them unless the task needs them.

## Layout
- `src/CamGh.Core`: algorithms, tri-dexel stock (`TriDexelStock.cs`), job planning (`CamModels.cs`), finishing (`RegionFinishing.cs`, `SurfaceLinker.cs`), roughing (`HorizontalRoughing.cs`, `MultiAxisRoughing.cs`). No Rhino dependency.
- `src/CamGh.Engine`: loopback HTTP worker (127.0.0.1:47831) and dashboard `wwwroot/index.html`.
- `src/CamGh.Grasshopper`: Rhino components, `ComponentIcons.cs` (embedded PNG icons, sources in `docs/images/icons`).
- `tests/Program.cs`: headless regression test. `examples/`: sample meshes and `.gh` files.

## Commands
- Test: `dotnet run -c Release --project tests` (prints PASS/FAIL per check).
- Package (Windows): `dotnet publish src/CamGh.Engine -p:PublishProfile=CamGhPackage`, then close Rhino and run `Install-CamGh.bat`.
- Timing breakdown: set `CAMGH_PROFILE=1` (see `CamJobPlanner.Mark`).
- In the cloud sandbox there is no dotnet SDK: compile-check with the Roslyn in PowerShell 7 (`build.sh`/`build.ps1` live outside the repo, in the sandbox).

## Conventions and gotchas
- Stock is shared: every operation subtracts from the same `TriDexelStock`. Rays are replaced, not edited in place; parallel updates use striped locks.
- `RoughingStrategy { ZLevel = 0, SurfaceOffset = 1 }`. Z-level levels are exactly one stepdown apart. Horizontal Roughing = multi-axis planner with one vertical direction.
- Parallel/Region finishing link passes with `SurfaceLinker` (stay-down); retract only when no safe link exists.
- Keep changes small and re-run the test before committing. Never claim a fix without measuring it.
- Commits end with the attribution lines from the session reminder.

## Performance baseline (sculpture.stl, 2 cores)
Region finishing ~44 s, Parallel finishing ~30 s, surface-offset roughing ~13 s, Z-level ~9 s, Horizontal 2.4-3.4 s. Dominant cost: `TriDexelStock.SubtractPlanar` (~700 ray tests per tool move).

## Open to-do
1. Speed: stock-height-aware ray test so only rays meeting the tool body are tested; cheaper path simplification before subtracting.
2. Rhino runtime validation of the latest build (icons, Settings window fallback).
3. Optional: finer Horizontal Roughing levels (lower stepdown) if terraces matter.

## Token-efficient workflow
Work as orchestrator: delegate to the agents in `.claude/agents/` and keep only their short summaries.
- `explorer` (haiku): find code and answer where/what questions; returns paths and a few lines.
- `profiler` (haiku): run timing scripts, return a small table.
- `tester` (haiku): run the regression test, return PASS/FAIL and failing numbers.
- `docs-release` (sonnet): README/docs edits, packaging, commit and push.
- `algo-engineer` (opus): hard algorithm changes from a precise brief; returns a diff and measured results.
Rules: agents exchange summaries or file paths, never full logs; give each agent only the files it needs; filter long output in the shell (`tail`, `grep`) before reading it.
