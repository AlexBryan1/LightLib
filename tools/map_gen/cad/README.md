# Drop CAD here

Put your VEX field CAD file in this folder, then run from the repo root:

```bash
python tools/map_gen/slice.py --face front --height 2.5
```

Supported formats: `.step` / `.stp` (with `cascadio` installed),
`.stl`, `.obj`, `.ply`, `.dxf`, `.glb`, `.gltf`.

If you have more than one file in here, pass `--cad <path>` to pick one.

VEX publishes field CAD on the game manual / VEX library / RobotEvents
page for each season. STL or OBJ exports out of SolidWorks / Fusion /
Onshape work without extra Python deps.

See [../README.md](../README.md) for the full slicer docs.
