# Reference notes

## PRC architecture

PRC's documentation describes Grasshopper as a lightweight client and a separate server as the place for heavier simulation and code generation. CamGh follows that separation with a dedicated local worker process and a versioned request/response API. PRC uses gRPC; this prototype uses loopback HTTP/JSON to avoid third-party dependencies at the first milestone.

- [PRC documentation overview](https://docs.robotsinarchitecture.org/)
- [PRC Grasshopper integration](https://docs.robotsinarchitecture.org/getting-started/grasshopper/)
- [PRC installation and runtime requirements](https://docs.robotsinarchitecture.org/getting-started/installation/)

## PyCAM

PyCAM is a useful open-source reference for mesh/cutter contact calculations and operation separation. It is not assumed to match RhinoCAM internals. In particular, its drop-cutter strategy searches for safe cutter contact heights along path samples; CamGh's current scanline baseline only intersects triangle geometry.

- [PyCAM DropCutter](https://github.com/SebKuzminsky/pycam/blob/master/pycam/PathGenerators/DropCutter.py)
- [PyCAM common path generator geometry](https://github.com/SebKuzminsky/pycam/blob/master/pycam/PathGenerators/__init__.py)
- [PyCAM PushCutter](https://github.com/SebKuzminsky/pycam/blob/master/pycam/PathGenerators/PushCutter.py)
- [PyCAM ContourFollow](https://github.com/SebKuzminsky/pycam/blob/master/pycam/PathGenerators/ContourFollow.py)

## Supplied research papers

- `sources/Geodesic-based-tool-path-planning-for-complex-free-form-surfaces-with-uncut-region-elimination.pdf`: heat-method geodesic distances, distance contours, curvature-aware scallop spacing, coverage checks, and removal of uncut regions. The prototype only has a planar ball-end stepover helper; the mesh heat solver and curvature-aware path spacing remain future implementation.
- `sources/liu2018.pdf`: region-based path planning from a machining-strip-width tensor and its principal directions. This motivates the planned direction-field and surface partition modules; it is not implemented in the current scanline path.
- `sources/Study_of_tool_paths_calculated_by_differ.pdf`: reports significant differences among commercial CAM paths in code size, segment lengths, machining time, and surface roughness. This motivates reporting segment-length metrics. The prototype does not predict time or roughness.

The papers are synced project source files and remain unchanged.
