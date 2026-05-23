#pragma once

// Non-public follower runtime: shared by ramsete.cpp and pure_pursuit.cpp.
// All symbols are defined in ramsete.cpp; this header only declares.

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

#include "LightLib/drive/ramsete.hpp"
#include "LightLib/drive/spline.hpp"
#include "LightLib/drive/trajectory.hpp"
#include "pros/motor_group.hpp"

namespace light {

class Drive;

extern RamseteConfig g_rc;
extern DriveFF g_ff;
extern TrajConstraints g_defaultCons;
extern bool g_configured;
extern float g_charVoltage_mV;

// motorRPM × s_rpmToInPerSec = wheel in/s. Refreshed in ramsete_configure().
extern float s_rpmToInPerSec;

// Held by the active follower so the two can't overlap or self-recurse.
extern std::atomic<bool> g_genLocked;

// EMA-filtered actual velocity for the per-wheel kP term.
struct WheelState {
  float vFiltered = 0.0f;
  bool seeded = false;
};

// PathEvent (waypoint-indexed) is converted up-front to time-indexed.
struct ResolvedEvent {
  float t;
  std::function<void()> action;
};

// Parks EZ in DISABLE on construction; zeros motors on destruction.
struct EzPauseGuard {
  light::Drive* chassis;
  pros::MotorGroup* left;
  pros::MotorGroup* right;

  EzPauseGuard(light::Drive* c, pros::MotorGroup* l, pros::MotorGroup* r);
  ~EzPauseGuard();
};

float motorRpmToInPerSec(float rpm);

// Per-wheel FF + velocity-P → battery-clamped voltage in [−12, 12].
// batteryScale rescales kV/kA against the characterization-time cell voltage.
float wheelVoltageCmd(float vTarget, float aTarget,
                      float vActual, WheelState& ws,
                      float batteryScale);

// Proportional rescale on saturation preserves the steering ratio. Returns
// true if clipped (counted in post-run telemetry).
bool balanceSaturation(float& vL, float& vR, float limit = 12.0f);

// O(N) scan; runs once per event before the control loop.
float waypointTime(const std::vector<Waypoint>& wps,
                   int idx, const Trajectory& traj);

// followTrajectoryCore lives in ramsete.cpp; followPurePursuitCore in
// pure_pursuit.cpp. Each owns the motors (via EzPauseGuard) for the call.
bool followTrajectoryCore(const Trajectory& traj,
                          std::vector<ResolvedEvent> events,
                          int timeoutMs,
                          float poseErrBailIn);

bool followPurePursuitCore(const Trajectory& traj,
                           std::vector<ResolvedEvent> events,
                           int timeoutMs,
                           float poseErrBailIn);

}  // namespace light
