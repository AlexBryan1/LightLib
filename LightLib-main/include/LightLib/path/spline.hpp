#pragma once

#include <array>
#include <optional>
#include <vector>

/**
 * \file spline.hpp
 *
 * Quintic Hermite spline over 2-D waypoints.
 *
 * Used by RAMSETE trajectory generation. Produces a C² continuous curve with
 * controllable heading at endpoints and optional heading at interior
 * waypoints.
 *
 * \par Convention (LightLib)
 * `+Y` forward, `+X` right. `theta = 0` faces `+Y`, CW-positive. Tangent
 * direction for a heading θ is `(sin θ, cos θ)`. Curvature sign: positive
 * curvature = path turning CW (robot heading increases).
 */

namespace light {

/** Single 2-D waypoint with optional heading and per-point speed cap. */
struct Waypoint {
  float x = 0.0f;                  ///< X position, inches.
  float y = 0.0f;                  ///< Y position, inches.
  std::optional<float> headingRad; ///< If absent, tangent taken from adjacent chord.
  std::optional<float> speed;      ///< Optional per-waypoint speed cap (not used yet).
};

/** Pose / velocity / acceleration sample produced by spline evaluation. */
struct SplineSample {
  float x = 0.0f, y = 0.0f;     ///< Position, inches.
  float dx = 0.0f, dy = 0.0f;   ///< First derivative w.r.t. parameter `u`.
  float ddx = 0.0f, ddy = 0.0f; ///< Second derivative w.r.t. parameter `u`.
};

/** One segment of a quintic Hermite spline. */
struct HermiteSegment {
  float p0x, p0y, v0x, v0y, a0x, a0y; ///< Start position, velocity, acceleration.
  float p1x, p1y, v1x, v1y, a1x, a1y; ///< End position, velocity, acceleration.

  /**
   * Evaluate the segment at parameter `u`.
   *
   * \param u
   *        normalized parameter, 0..1
   * \return SplineSample at that point
   */
  SplineSample eval(float u) const;
};

/**
 * Concatenated quintic Hermite spline with arc-length parameterization.
 *
 * Construct from a list of Waypoints; the resulting spline can then be
 * sampled by global arc length `s ∈ [0, totalArcLen()]`.
 */
class Spline {
 public:
  Spline() = default;

  /**
   * Build a spline from waypoints.
   *
   * \param wps
   *        ordered list of waypoints; must have at least 2 entries
   */
  explicit Spline(const std::vector<Waypoint>& wps);

  /** \return true if the spline has at least one segment. */
  bool valid() const { return !segs_.empty(); }

  /** \return Total arc length of the spline, inches. */
  float totalArcLen() const { return totalLen_; }

  /** \return Number of internal Hermite segments. */
  size_t segmentCount() const { return segs_.size(); }

  /**
   * Sample pose / derivatives at arc length `s`.
   *
   * \param s
   *        arc length from start; clamped to [0, totalArcLen()]
   */
  SplineSample sampleAt(float s) const;

  /**
   * Heading at arc length `s`, in radians.
   *
   * \param s
   *        arc length from start
   * \return  `atan2(dx, dy)` — LightLib convention (+Y forward)
   */
  float headingAt(float s) const;

  /**
   * Signed curvature at arc length `s`.
   *
   * \param s
   *        arc length from start
   * \return  curvature in 1/in; positive = CW (matches theta sign)
   */
  float curvatureAt(float s) const;

 private:
  std::vector<HermiteSegment> segs_;
  std::vector<std::array<float, 65>> segArcTables_;  // per-segment u→s LUT
  std::vector<float> segLens_;
  float totalLen_ = 0.0f;

  void sToSegU(float s, int& segIdx, float& u) const;
};

}  // namespace light
