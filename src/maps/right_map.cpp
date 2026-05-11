// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       RIGHT DISTANCE-SENSOR MAP                         │
// │                                                                         │
// │  Geometry the RIGHT distance sensor raycasts against during MCL.        │
// │  Default: the 12×12 ft VRC field perimeter (delegates to raycast).      │
// │                                                                         │
// │  Coordinate frame: origin at field center, +X right, +Y forward,        │
// │  theta CCW from +Y. Units are inches; angleRad is in radians.           │
// │                                                                         │
// │  To add interior walls, goal cutouts, or face-specific occlusion,       │
// │  replace the body with your own ray vs. segment math and return the     │
// │  nearest positive hit distance, clamped to max_range.                   │
// │                                                                         │
// │  Built into firmware/libmaps.a — only recompiled by `pros make maps`.   │
// └─────────────────────────────────────────────────────────────────────────┘

#include "LightLib/path/field_map.hpp"

namespace light::field {

float raycast_right(float x, float y, float angleRad, float max_range) {
  return raycast(x, y, angleRad, max_range);
}

}  // namespace light::field
