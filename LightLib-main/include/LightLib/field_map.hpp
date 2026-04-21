#pragma once

// Fixed-map ray caster used by LightCast's sensor model.
// The map is the VRC field perimeter (12 ft × 12 ft axis-aligned box) plus
// optional static segments. Moving game elements are intentionally NOT mapped:
// LightCast's outlier rejection handles a ball reading shorter than the wall.
//
// Coordinate frame: origin at field center, +x right, +y forward, theta CCW
// from +y (radians). Units are inches.

namespace light::field {

constexpr float FIELD_SIZE_IN = 144.0f;  // 12 ft × 12 ft
constexpr float FIELD_HALF    = FIELD_SIZE_IN / 2.0f;

// Cast a ray from (x, y) in direction angleRad (world frame).
// Returns distance to nearest obstacle along the ray, clamped to max_range.
// Never returns negative; a ray that starts outside the field returns 0.
float raycast(float x, float y, float angleRad, float max_range = FIELD_SIZE_IN);

}  // namespace light::field
