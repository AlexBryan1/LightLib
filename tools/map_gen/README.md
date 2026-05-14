# Field CAD → MCL Map Slicer

Offline tool that turns a VEX field CAD into per-face `src/maps/<face>_map.cpp`
files that LightCast's MCL particle filter raycasts against.

The slicing happens once, on your laptop. The V5 brain only ever sees the
emitted 2D segment array, so the hot path stays cheap (200 particles × N
sensors × 10 Hz).

## Setup (one time)

```bash
pip install -r tools/map_gen/requirements.txt
```

For STEP files (the canonical VEX field CAD format), also:

```bash
pip install cascadio
```

(STL / OBJ / PLY work without it.)

> The defaults (`--unit m`, `--axis-up y`, `--axis-forward z`) are tuned for
> the canonical VEX field STEP loaded through cascadio. If you're slicing a
> different CAD (your own STL export, etc.), pass the appropriate flags —
> the slicer will print a warning if the bbox looks wrong.

## Usage

1. Drop a field CAD into [tools/map_gen/cad/](cad/) (one file).
2. Run the slicer:

   ```bash
   python tools/map_gen/slice.py --face front --height 2.5
   ```

3. Rebuild the cold maps package:

   ```bash
   pros make maps
   ```

The slicer **overwrites** `src/maps/front_map.cpp`. Repeat for each face
your robot has a distance sensor on. Faces without a sensor can keep their
default stub.

## All options

| Flag | Default | Meaning |
| --- | --- | --- |
| `--face` | required | `front` / `back` / `left` / `right`. Picks which `<face>_map.cpp` to write. |
| `--height` | required | Sensor mounting height above the **field tiles**, in inches. (The tile thickness is added internally — pass what you actually measured on the robot.) |
| `--tile-thickness` | `0.63` | VEX field tile thickness in inches, added to `--height` before slicing. Override only if you're not on standard VRC foam. |
| `--cad` | only file in `cad/` | Path to the CAD file, if you have multiple. |
| `--unit` | `m` | `mm` / `cm` / `m` / `in`. Default is meters because `cascadio` normalizes STEP files to meters. Override with `--unit mm` for STL/OBJ exports. |
| `--axis-up` | `y` | Which CAD axis points up. Default `y` matches the canonical VEX field STEP. |
| `--axis-forward` | `z` | Which CAD axis maps to field `+Y` (forward). Default `z` matches the canonical VEX field STEP. |
| `--origin` | `bbox-center` | `bbox-center` recenters the field to (0,0) and drops the floor to Z=0. `as-is` trusts the CAD origin. |
| `--exclude` | `[]` | Substrings of mesh names to drop. Use this for moving game elements: `--exclude ball mobile_goal cap`. |
| `--max-segments` | `1000` | Hard cap on emitted segments. Fail loud if exceeded. |
| `--out` | `src/maps/<face>_map.cpp` | Override output path. |
| `--dry-run` | off | Print the summary, don't write. |


## Coordinate frame contract

The emitted code assumes the field frame from
[include/Lightlib/drive/field_map.hpp](../../include/Lightlib/drive/field_map.hpp):

- Origin at field center
- `+X` right, `+Y` forward, `theta` CCW from `+Y` (radians)
- Units: inches
- Field is a 144" × 144" axis-aligned box; the perimeter is already handled
  by `light::field::raycast()`. The emitter only writes interior segments.

Moving game elements (balls, mobile goals) **must not** be in the slice —
LightCast's outlier rejection (>6" shorter readings ignored, see
[lightcast.cpp](../../src/LightLib/drive/lightcast.cpp)) handles those at
runtime. Use `--exclude` to drop them by mesh name.

## What gets emitted

```cpp
constexpr float SENSOR_HEIGHT_IN = 2.500f;

struct Seg { float x1, y1, x2, y2; };
constexpr Seg kSegments[] = {
    { ... },
    ...
};

float raycast_front(float x, float y, float angleRad, float max_range) {
  float t = raycast(x, y, angleRad, max_range);   // perimeter
  // walk kSegments, take nearest hit
  return std::min(t, max_range);
}
```

Same shape as the existing stub — just with a real segment array instead of
a pure delegation.

## Build-time impact

Zero on regular `pros make`. The maps live in `firmware/libmaps.a` (the
cold package) and only recompile when you run `pros make maps`. With
`--max-segments 256` (default) the emitted file compiles in milliseconds.
