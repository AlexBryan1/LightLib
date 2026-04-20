#pragma once
// ─── Time-parameterized trajectory over a Spline ─────────────────────────────
//
// A Trajectory is a dense table of states sampled every 10 ms (the RAMSETE
// control period). It is produced once by generateTrajectory() from a list of
// Waypoints and consumed by light::followTrajectory().
//
// Each TrajState carries pose, linear velocity v, angular velocity ω = v·κ,
// and the linear acceleration 'a' needed by the per-wheel feedforward.

#include <vector>
#include "LightLib/spline.hpp"

namespace light {

struct TrajConstraints {
    float vMax    = 48.0f;   // in/s, linear
    float aMax    = 60.0f;   // in/s², forward accel ceiling
    float aDecMax = 60.0f;   // in/s², decel ceiling (split so downshift-heavy)
    float aLatMax = 60.0f;   // in/s², lateral (centripetal) accel ceiling
};

struct TrajState {
    float t     = 0.0f;   // seconds from start
    float x     = 0.0f;
    float y     = 0.0f;
    float theta = 0.0f;   // radians, LightLib convention
    float v     = 0.0f;   // in/s (signed if reversed)
    float omega = 0.0f;   // rad/s (v·κ, sign-matched)
    float a     = 0.0f;   // in/s² (signed)
    float kappa = 0.0f;   // 1/in (signed, CW-positive)
};

class Trajectory {
public:
    std::vector<TrajState> pts;
    float duration = 0.0f;

    bool empty() const { return pts.empty(); }
    TrajState sample(float t) const;
};

Trajectory generateTrajectory(const std::vector<Waypoint>& wps,
                              const TrajConstraints& cons,
                              bool reversed = false);

} // namespace light
