#pragma once

/**
 * \file field_map.hpp
 *
 * Fixed-map ray caster used by LightCast's sensor model.
 *
 * The map is the VRC field perimeter (12 ft × 12 ft axis-aligned box) plus
 * optional static segments. Moving game elements are intentionally NOT
 * mapped: LightCast's outlier rejection handles a ball reading shorter
 * than the wall.
 *
 * \par Coordinate frame
 * Origin at field center, `+x` right, `+y` forward, `theta` CCW from `+y`
 * (radians). Units are inches.
 */

namespace light::field {

constexpr float FIELD_SIZE_IN = 144.0f;        ///< Field side length, inches (12 ft).
constexpr float FIELD_HALF = FIELD_SIZE_IN / 2.0f; ///< Half-side, inches.

/**
 * Cast a ray from `(x, y)` in direction `angleRad` (world frame).
 *
 * \param x
 *        ray origin X, inches
 * \param y
 *        ray origin Y, inches
 * \param angleRad
 *        ray direction in world frame, radians
 * \param max_range
 *        clamp the returned distance to this many inches
 * \return  distance to nearest obstacle along the ray, never negative.
 *          A ray that starts outside the field returns 0.
 */
float raycast(float x, float y, float angleRad, float max_range = FIELD_SIZE_IN);

}  // namespace light::field
