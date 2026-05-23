"""
CAD -> MCL field map slicer.

Drop a STEP / STL / OBJ / PLY field CAD into tools/map_gen/cad/, then run:

    python tools/map_gen/slice.py --face front --height 2.5

Slices the geometry at the given sensor height (inches) and overwrites
src/maps/<face>_map.cpp with a constexpr segment array plus ray-vs-segment
math. The hot path stays 2D; this script runs offline on your laptop.

See tools/map_gen/README.md for the full option list.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from emit import Segment, write_face_map


# -----------------------------------------------------------------------------
# Constants matching include/Lightlib/drive/field_map.hpp
# -----------------------------------------------------------------------------

FIELD_HALF_IN = 72.0  # 144" / 2

UNIT_TO_INCHES = {
    "mm": 1.0 / 25.4,
    "cm": 1.0 / 2.54,
    "m": 39.3700787,
    "in": 1.0,
}

AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="slice.py",
        description=(
            "Slice a VEX field CAD at a sensor height and emit "
            "src/maps/<face>_map.cpp."
        ),
    )
    p.add_argument(
        "--face", required=True, choices=("front", "back", "left", "right"),
        help="Which distance sensor face to generate the map for.",
    )
    p.add_argument(
        "--height", required=True, type=float,
        help=(
            "Sensor mounting height above the field tiles, in inches. The "
            "slicer adds --tile-thickness internally so this is what you "
            "actually measured on the robot."
        ),
    )
    p.add_argument(
        "--tile-thickness", type=float, default=0.63,
        help=(
            "VEX field tile thickness in inches, added to --height before "
            "slicing. Default: 0.63 (standard VRC foam tile)."
        ),
    )
    p.add_argument(
        "--cad", type=Path, default=None,
        help=(
            "Path to a CAD file. Defaults to the only file in "
            "tools/map_gen/cad/ (errors if there is more than one)."
        ),
    )
    # Defaults match the canonical VEX field STEP (loaded via cascadio,
    # which normalizes to meters; the field uses Y-up, Z-forward).
    p.add_argument(
        "--unit", choices=tuple(UNIT_TO_INCHES.keys()), default="m",
        help="Units of the input CAD. Default: m (cascadio's STEP output).",
    )
    p.add_argument(
        "--axis-up", choices=("x", "y", "z"), default="y",
        help="Which CAD axis points up (gravity). Default: y.",
    )
    p.add_argument(
        "--axis-forward", choices=("x", "y", "z"), default="z",
        help="Which CAD axis points 'forward' in the field frame. Default: z.",
    )
    p.add_argument(
        "--origin", choices=("bbox-center", "as-is"), default="bbox-center",
        help=(
            "How to place the field origin. 'bbox-center' (default) recenters "
            "the XY bounding box on (0,0). 'as-is' trusts the CAD origin."
        ),
    )
    p.add_argument(
        "--exclude", nargs="*", default=[],
        help=(
            "Substrings of mesh names to drop (case-insensitive). Use this "
            "for moving game elements: --exclude ball goal mobile_goal"
        ),
    )
    p.add_argument(
        "--max-segments", type=int, default=5000,
        help=(
            "Hard cap on emitted segments. Fail if exceeded so we don't blow "
            "up libmaps.a. Default: 5000."
        ),
    )
    p.add_argument(
        "--out", type=Path, default=None,
        help="Output .cpp path. Default: src/maps/<face>_map.cpp",
    )
    p.add_argument(
        "--dry-run", action="store_true",
        help="Print the summary but don't write the .cpp file.",
    )
    return p.parse_args(argv)


# -----------------------------------------------------------------------------
# Steps
# -----------------------------------------------------------------------------

def resolve_cad_path(arg: Path | None) -> Path:
    if arg is not None:
        if not arg.exists():
            sys.exit(f"error: CAD file not found: {arg}")
        return arg

    cad_dir = Path(__file__).parent / "cad"
    candidates = [
        p for p in cad_dir.iterdir()
        if p.is_file() and p.suffix.lower() in {
            ".step", ".stp", ".stl", ".obj", ".ply", ".dxf", ".glb", ".gltf",
        }
    ]
    if not candidates:
        sys.exit(
            f"error: no CAD files found in {cad_dir}\n"
            f"       drop a .step/.stl/.obj/.ply file there or pass --cad."
        )
    if len(candidates) > 1:
        names = ", ".join(p.name for p in candidates)
        sys.exit(
            f"error: multiple CAD files in {cad_dir}: {names}\n"
            f"       pass --cad to pick one."
        )
    return candidates[0]


def load_mesh(path: Path):
    """Load CAD; returns a trimesh.Scene (uniform handling for single/multi)."""
    try:
        import trimesh
    except ImportError:
        sys.exit(
            "error: trimesh not installed. Run:\n"
            "       pip install -r tools/map_gen/requirements.txt"
        )

    try:
        loaded = trimesh.load(path, force="scene")
    except ValueError as e:
        if path.suffix.lower() in {".step", ".stp"}:
            sys.exit(
                f"error: STEP loading failed ({e}).\n"
                "       Install a STEP backend: pip install cascadio\n"
                "       Or export your CAD as STL/OBJ instead."
            )
        raise

    if loaded.is_empty:
        sys.exit(f"error: loaded CAD has no geometry: {path}")

    total_geoms = len(loaded.geometry)
    total_faces = sum(
        len(g.faces) for g in loaded.geometry.values()
        if hasattr(g, "faces")
    )
    print(f"  loaded {total_geoms} geometries, {total_faces} triangles total")
    if total_faces == 0:
        sys.exit(
            "error: CAD loaded but has 0 triangles. STEP files need to be\n"
            "       tessellated by cascadio; check `pip show cascadio`. If\n"
            "       cascadio is installed, try re-exporting as STL/OBJ from\n"
            "       your CAD program."
        )
    return loaded


def axis_remap_matrix(axis_up: str, axis_forward: str) -> np.ndarray:
    """3x3 rotation that maps CAD axes to field axes (+X right, +Y forward, +Z up)."""
    if axis_up == axis_forward:
        sys.exit("error: --axis-up and --axis-forward must be different axes.")
    cad_up = AXIS_INDEX[axis_up]
    cad_fwd = AXIS_INDEX[axis_forward]
    cad_right = ({0, 1, 2} - {cad_up, cad_fwd}).pop()

    M = np.zeros((3, 3))
    M[0, cad_right] = 1.0   # field X = CAD right
    M[1, cad_fwd] = 1.0     # field Y = CAD forward
    M[2, cad_up] = 1.0      # field Z = CAD up

    # Right-handed check: if det == -1 the user's axis pair flips chirality.
    # Negate the "right" axis to keep right-handedness.
    if np.linalg.det(M) < 0:
        M[0, cad_right] = -1.0
    return M


def reframe_meshes(
    scene,
    unit: str,
    axis_up: str,
    axis_forward: str,
    origin_mode: str,
    exclude: list[str],
):
    """
    Yield per-mesh (name, vertices_inches_field_frame, faces) tuples.

    Vertices are converted to inches and rotated so +Z is up, +Y is forward,
    +X is right. If origin_mode='bbox-center', XY are recentered to the field
    bbox center and Z is shifted so the lowest point is at Z=0 (the field
    surface).
    """
    scale = UNIT_TO_INCHES[unit]
    R = axis_remap_matrix(axis_up, axis_forward)

    excludes_lower = [e.lower() for e in exclude]

    raw_meshes = []
    for name, geom in scene.geometry.items():
        if hasattr(geom, "vertices") and hasattr(geom, "faces"):
            if any(e in name.lower() for e in excludes_lower):
                print(f"  skip (excluded): {name}")
                continue
            verts = np.asarray(geom.vertices, dtype=float) * scale
            verts = verts @ R.T
            raw_meshes.append((name, verts, np.asarray(geom.faces)))

    if not raw_meshes:
        sys.exit("error: nothing left to slice after exclusions.")

    if origin_mode == "bbox-center":
        all_verts = np.concatenate([v for _, v, _ in raw_meshes])
        bbox_min = all_verts.min(axis=0)
        bbox_max = all_verts.max(axis=0)
        center_xy = (bbox_min[:2] + bbox_max[:2]) * 0.5
        floor_z = bbox_min[2]
        offset = np.array([center_xy[0], center_xy[1], floor_z])
        raw_meshes = [(n, v - offset, f) for (n, v, f) in raw_meshes]

    # Post-reframe extents, printed so the user can sanity-check unit/axis flags.
    all_verts = np.concatenate([v for _, v, _ in raw_meshes])
    bb_min = all_verts.min(axis=0)
    bb_max = all_verts.max(axis=0)
    extent = float(np.max(bb_max - bb_min))
    print(
        f"  field-frame bbox (in): "
        f"x=[{bb_min[0]:.2f}, {bb_max[0]:.2f}] "
        f"y=[{bb_min[1]:.2f}, {bb_max[1]:.2f}] "
        f"z=[{bb_min[2]:.2f}, {bb_max[2]:.2f}]"
    )
    if extent < 10.0:
        print(
            "  warning: largest dimension is < 10 in. The unit is probably\n"
            "           wrong. STEP files loaded via cascadio come out in\n"
            "           METERS, not mm: re-run with --unit m."
        )
    elif extent > 400.0:
        print(
            f"  warning: largest dimension is {extent:.0f} in (field is 144).\n"
            "           Unit may be too coarse: try --unit cm or --unit in."
        )

    return raw_meshes


def slice_meshes(meshes, height_in: float) -> np.ndarray:
    """Run trimesh.intersections.mesh_plane on each mesh, concatenate segments."""
    import trimesh
    from trimesh.intersections import mesh_plane

    # Z-range warning: if the user's --height is outside the geometry, every
    # slice will be empty and the failure mode is silent without this.
    all_verts = np.concatenate([v for _, v, _ in meshes])
    z_min, z_max = float(all_verts[:, 2].min()), float(all_verts[:, 2].max())
    if not (z_min - 0.01 <= height_in <= z_max + 0.01):
        print(
            f"  warning: height {height_in:.2f} in is outside the geometry's "
            f"Z range [{z_min:.2f}, {z_max:.2f}]. Every slice will be empty.\n"
            f"           Likely causes: wrong --unit, wrong --axis-up, or the\n"
            f"           CAD floor is not at the bbox minimum (try --origin as-is)."
        )

    all_segs: list[np.ndarray] = []
    plane_origin = np.array([0.0, 0.0, height_in])
    plane_normal = np.array([0.0, 0.0, 1.0])

    for name, verts, faces in meshes:
        m = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        if len(m.faces) == 0:
            continue
        segs = mesh_plane(m, plane_normal=plane_normal, plane_origin=plane_origin)
        if len(segs) > 0:
            all_segs.append(segs)
            print(f"  {name}: {len(segs)} segments")

    if not all_segs:
        return np.zeros((0, 2, 3))
    return np.concatenate(all_segs, axis=0)


def to_2d_segments(segs_3d: np.ndarray) -> list[Segment]:
    """Drop Z (slicing plane is horizontal, Z values are all == height)."""
    out = []
    for (a, b) in segs_3d:
        out.append(Segment(float(a[0]), float(a[1]), float(b[0]), float(b[1])))
    return out


def clip_to_field(segs: list[Segment]) -> list[Segment]:
    """Drop segments fully outside the 144x144 box; the perimeter is already
    handled by raycast() in field_map.cpp so we don't need them."""
    eps = 0.01
    half = FIELD_HALF_IN + eps
    kept = []
    for s in segs:
        # both endpoints share an outside half-plane => fully outside, drop
        if s.x1 > half and s.x2 > half: continue
        if s.x1 < -half and s.x2 < -half: continue
        if s.y1 > half and s.y2 > half: continue
        if s.y1 < -half and s.y2 < -half: continue
        # zero-length sliver from a coplanar triangle, drop
        if abs(s.x1 - s.x2) < 1e-4 and abs(s.y1 - s.y2) < 1e-4: continue
        kept.append(s)
    return kept


def dedupe(segs: list[Segment], tol: float = 0.04) -> list[Segment]:
    """Drop near-duplicates. Tolerance is in inches (~1 mm)."""
    seen = set()
    out = []
    for s in segs:
        key_a = (round(s.x1 / tol), round(s.y1 / tol),
                 round(s.x2 / tol), round(s.y2 / tol))
        key_b = (round(s.x2 / tol), round(s.y2 / tol),
                 round(s.x1 / tol), round(s.y1 / tol))
        if key_a in seen or key_b in seen:
            continue
        seen.add(key_a)
        out.append(s)
    return out


# -----------------------------------------------------------------------------
# main
# -----------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    cad_path = resolve_cad_path(args.cad)
    repo_root = Path(__file__).resolve().parent.parent.parent
    out_path = (
        args.out if args.out is not None
        else repo_root / "src" / "maps" / f"{args.face}_map.cpp"
    )

    slice_z = args.height + args.tile_thickness

    print(f"CAD       : {cad_path}")
    print(f"face      : {args.face}")
    print(
        f"height    : {args.height} in above tiles "
        f"(+{args.tile_thickness} tile = slice at z={slice_z:.3f} in)"
    )
    print(f"unit      : {args.unit}")
    print(f"axes      : up={args.axis_up}, forward={args.axis_forward}")
    print(f"origin    : {args.origin}")
    if args.exclude:
        print(f"exclude   : {args.exclude}")

    print("loading...")
    scene = load_mesh(cad_path)

    print("reframing...")
    meshes = reframe_meshes(
        scene, args.unit, args.axis_up, args.axis_forward,
        args.origin, args.exclude,
    )

    print(f"slicing at z = {slice_z:.3f} in...")
    segs_3d = slice_meshes(meshes, slice_z)

    segs = to_2d_segments(segs_3d)
    print(f"  raw segments        : {len(segs)}")
    segs = clip_to_field(segs)
    print(f"  after perimeter cull: {len(segs)}")
    segs = dedupe(segs)
    print(f"  after dedupe        : {len(segs)}")

    if len(segs) > args.max_segments:
        sys.exit(
            f"error: {len(segs)} segments exceeds --max-segments "
            f"{args.max_segments}.\n"
            f"       Either raise the cap (impacts libmaps.a size) or add "
            f"more --exclude filters."
        )

    if segs:
        xs = [c for s in segs for c in (s.x1, s.x2)]
        ys = [c for s in segs for c in (s.y1, s.y2)]
        print(
            f"  bbox (in)           : "
            f"x=[{min(xs):.2f}, {max(xs):.2f}] "
            f"y=[{min(ys):.2f}, {max(ys):.2f}]"
        )

    if args.dry_run:
        print("dry-run: not writing.")
        return 0

    cad_rel = cad_path.relative_to(repo_root) if repo_root in cad_path.parents \
        else cad_path
    write_face_map(
        out_path=out_path,
        face=args.face,
        height_in=args.height,
        cad_path=str(cad_rel).replace("\\", "/"),
        segments=segs,
    )
    print(f"wrote {out_path}")
    print("rebuild with: pros make maps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
