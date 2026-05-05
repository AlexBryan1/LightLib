#pragma once

#include <vector>

#include "LightLib/path/spline.hpp"

/**
 * \file trajectory.hpp
 *
 * Time-parameterized trajectory over a Spline.
 *
 * A Trajectory is a dense table of states sampled every 10 ms (the RAMSETE
 * control period). It is produced once by generateTrajectory() from a list
 * of Waypoints and consumed by light::followTrajectory().
 *
 * Each TrajState carries pose, linear velocity v, angular velocity ω = v·κ,
 * and the linear acceleration `a` needed by the per-wheel feedforward.
 */

namespace light {

/** Kinematic limits used by the trajectory generator. */
struct TrajConstraints {
  float vMax = 48.0f;     ///< Max linear speed, in/s.
  float aMax = 60.0f;     ///< Max forward accel, in/s².
  float aDecMax = 60.0f;  ///< Max decel, in/s² (split so downshift-heavy).
  float aLatMax = 60.0f;  ///< Max lateral (centripetal) accel, in/s².
};

/** One sample of a time-parameterized trajectory. */
struct TrajState {
  float t = 0.0f;     ///< Time from start, seconds.
  float x = 0.0f;     ///< X position, inches.
  float y = 0.0f;     ///< Y position, inches.
  float theta = 0.0f; ///< Heading, radians (LightLib convention).
  float v = 0.0f;     ///< Linear speed, in/s (signed if reversed).
  float omega = 0.0f; ///< Angular velocity, rad/s (`v·κ`, sign-matched).
  float a = 0.0f;     ///< Linear accel, in/s² (signed).
  float kappa = 0.0f; ///< Curvature, 1/in (signed, CW-positive).
};

/** Time-parameterized trajectory: dense state table + total duration. */
class Trajectory {
 public:
  std::vector<TrajState> pts; ///< 10 ms-spaced trajectory states.
  float duration = 0.0f;      ///< Total trajectory length in seconds.

  /** \return true if the trajectory is empty (generation failed). */
  bool empty() const { return pts.empty(); }

  /**
   * Sample the trajectory at time `t`.
   *
   * \param t
   *        time in seconds; clamped to [0, duration]
   */
  TrajState sample(float t) const;
};

/**
 * Generate a time-parameterized trajectory from waypoints.
 *
 * \param wps
 *        ordered waypoint list
 * \param cons
 *        kinematic limits
 * \param reversed
 *        true if the robot should drive the path in reverse
 * \return  the generated Trajectory; empty on failure
 */
Trajectory generateTrajectory(const std::vector<Waypoint>& wps,
                              const TrajConstraints& cons,
                              bool reversed = false);

}  // namespace light
