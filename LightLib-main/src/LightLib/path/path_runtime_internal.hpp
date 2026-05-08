#pragma once

// ─── path_runtime_internal.hpp — non-public follower runtime ─────────────────
//
// Shared state and helpers used by both the RAMSETE and Pure Pursuit
// followers. Lives in src/ (not include/) so it is NOT part of LightLib's
// public API — only ramsete.cpp and pure_pursuit.cpp should include it.
//
// Symbols here are defined in ramsete.cpp; pure_pursuit.cpp only references
// them. Do NOT define any of these in this header — every entry is either an
// extern declaration or a forward declaration.

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

#include "LightLib/path/ramsete.hpp"
#include "LightLib/path/spline.hpp"
#include "LightLib/path/trajectory.hpp"
#include "pros/motor_group.hpp"

namespace light {

class Drive;

// Shared follower configuration registered by ramsete_configure().
extern RamseteConfig g_rc;
extern DriveFF g_ff;
extern TrajConstraints g_defaultCons;
extern bool g_configured;
extern float g_charVoltage_mV;

// Cached: motorRPM × this = wheel in/s. Refreshed in ramsete_configure().
extern float s_rpmToInPerSec;

// Held by either follower while it owns the motors so the two cannot overlap
// or recurse into themselves.
extern std::atomic<bool> g_genLocked;

// Per-wheel velocity-loop state passed across iterations of a follower's
// inner loop. EMA-filtered actual velocity for the kP term.
struct WheelState {
  float vFiltered = 0.0f;
  bool seeded = false;
};

// Resolved mid-path event. Both followers convert PathEvent (waypoint-indexed)
// to ResolvedEvent (time-indexed) up-front, then check it in the inner loop.
struct ResolvedEvent {
  float t;
  std::function<void()> action;
};

// EZ-Template motor-ownership RAII guard. On construction: park EZ in
// DISABLE so its drive task stops writing to the motors. On destruction:
// zero both sides and let EZ's next motion call re-arm cleanly.
struct EzPauseGuard {
  light::Drive* chassis;
  pros::MotorGroup* left;
  pros::MotorGroup* right;

  EzPauseGuard(light::Drive* c, pros::MotorGroup* l, pros::MotorGroup* r);
  ~EzPauseGuard();
};

// motorRPM (at the motor) → wheel linear speed in/s. Uses s_rpmToInPerSec.
float motorRpmToInPerSec(float rpm);

// Per-wheel feedforward + velocity-P → battery-clamped voltage [-12, 12].
// `batteryScale` re-scales kV/kA against the cell voltage at characterization
// time so a low cell still tracks the same target speed.
float wheelVoltageCmd(float vTarget, float aTarget,
                      float vActual, WheelState& ws,
                      float batteryScale);

// If either side would saturate, scale BOTH down together. Preserves the
// commanded steering ratio. Returns true if clipped — used to count
// saturation ticks for the post-run telemetry.
bool balanceSaturation(float& vL, float& vR, float limit = 12.0f);

// Locate the trajectory time closest in (x,y) to wps[idx]. O(N) scan; called
// once per event before the control loop starts.
float waypointTime(const std::vector<Waypoint>& wps,
                   int idx, const Trajectory& traj);

// Per-follower control-loop entry points. Both take the same shape so the
// dispatcher in ramsete.cpp can hand off to either based on Follower.
//
// followTrajectoryCore lives in ramsete.cpp; followPurePursuitCore lives in
// pure_pursuit.cpp. Each owns the motors (via EzPauseGuard) and the velocity
// profile from `traj` for the duration of the call.
bool followTrajectoryCore(const Trajectory& traj,
                          std::vector<ResolvedEvent> events,
                          int timeoutMs,
                          float poseErrBailIn);

bool followPurePursuitCore(const Trajectory& traj,
                           std::vector<ResolvedEvent> events,
                           int timeoutMs,
                           float poseErrBailIn);

}  // namespace light
