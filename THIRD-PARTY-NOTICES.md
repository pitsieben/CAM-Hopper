# Third-party components

## Clipper2

`CamGh.Core/lib/Clipper2Lib.dll` is a compiled copy of Clipper2 by Angus
Johnson, used for the polygon offsetting behind mesh-offset roughing contours.

- Project: https://github.com/AngusJohnson/Clipper2
- Licence: Boost Software License 1.0 (https://www.boost.org/LICENSE_1_0.txt)

It is checked into this repository so the solution builds without a package
restore for it.

## Rhino 8 and Grasshopper

`CamGh.Grasshopper` compiles against RhinoCommon, Grasshopper, GH_IO and Eto
from a local Rhino 8 installation. Those assemblies are **not** redistributed
here: the project references them in place with `<Private>false</Private>`, so
a Rhino 8 install is required to build. Rhino is a commercial product of
Robert McNeel & Associates.
