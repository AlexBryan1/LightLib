#pragma once

#include <functional>
#include <vector>

#include "LightLib/path/trajectory.hpp"

/**
 * \file ramsete.hpp
 * RAMSETE / Pure Pursuit trajectory follower public API. The follower owns
 * the motor groups exclusively for the duration of the motion and pauses
 * EZ-Template's drive PID task.
 */

namespace light {

/** Path-following controller selector. Public entry points default to PURE_PURSUIT. */
enum class Follower {
  RAMSETE,
  PURE_PURSUIT,
};

/**
 * Mid-path action fired once when the trajectory passes its resolved time.
 * `atWaypoint` is zero-based; for jerryio CSVs it indexes the heading-bearing
 * lines, falling back to raw line index when no heading columns exist.
 * Callback runs on the control thread — slow callbacks delay the next tick.
 */
struct PathEvent {
  int atWaypoint = 0;            ///< Index of the waypoint to fire on.
  std::function<void()> action;  ///< Callback invoked when the trigger is reached.
};

/** RAMSETE controller and chassis-geometry config. */
struct RamseteConfig {
  float b = 2.0f;          ///< Aggressiveness — larger = more correction.
  float zeta = 0.7f;       ///< Damping, [0, 1]; 0.7 is classic.
  float trackWidthIn = 12.0f; ///< Track width, inches.
  float wheelDiamIn = 3.25f;  ///< Wheel diameter, inches.
  float gearRatio = 0.75f; ///< wheel_rpm / motor_rpm (0.75 = 36:48 blue).
};

/** Per-wheel feedforward + simple velocity-loop gains. */
struct DriveFF {
  float kS = 0.0f;  ///< Volts to break static friction.
  float kV = 0.0f;  ///< Volts per (in/s) at steady state.
  float kA = 0.0f;  ///< Volts per (in/s²).
  float kP = 0.0f;  ///< Volts per (in/s) of velocity error.
};

/**
 * Configure the RAMSETE follower. Call once in `initialize()` before any
 * `followTrajectory` / `runJerryioPath` call.
 *
 * \param rc
 *        controller and chassis geometry
 * \param ff
 *        wheel feedforward gains
 * \param defaultCons
 *        default kinematic limits used when none are passed explicitly
 */
void ramsete_configure(RamseteConfig rc, DriveFF ff, TrajConstraints defaultCons);

/**
 * Follow a pre-generated Trajectory.
 *
 * \param traj
 *        the trajectory to follow
 * \param timeoutMs
 *        wall-time bail in ms (-1 = no timeout)
 * \param poseErrBailIn
 *        bail if the pose error grows above this many inches
 * \return  true on completion, false on bail/timeout
 */
bool followTrajectory(const Trajectory& traj,
                      int timeoutMs = -1,
                      float poseErrBailIn = 8.0f,
                      Follower follower = Follower::PURE_PURSUIT);

/**
 * Generate and follow a trajectory from waypoints.
 *
 * \param wps
 *        waypoint list
 * \param cons
 *        kinematic limits
 * \param reversed
 *        drive the path in reverse
 * \param timeoutMs
 *        wall-time bail in ms (-1 = no timeout)
 * \param poseErrBailIn
 *        bail if the pose error grows above this many inches
 * \param follower
 *        which controller to use (defaults to PURE_PURSUIT)
 */
bool followTrajectory(const std::vector<Waypoint>& wps,
                      const TrajConstraints& cons,
                      bool reversed = false,
                      int timeoutMs = -1,
                      float poseErrBailIn = 8.0f,
                      Follower follower = Follower::PURE_PURSUIT);

/**
 * Generate-and-follow variant with mid-path event triggers. Events whose
 * index is out of range are skipped (with a printf warning); the path
 * still runs.
 */
bool followTrajectory(const std::vector<Waypoint>& wps,
                      const TrajConstraints& cons,
                      std::vector<PathEvent> events,
                      bool reversed = false,
                      int timeoutMs = -1,
                      float poseErrBailIn = 8.0f,
                      Follower follower = Follower::PURE_PURSUIT);

/**
 * Generate and follow a trajectory from waypoints, using the default
 * `TrajConstraints` registered via `ramsete_configure()`.
 */
bool followTrajectory(const std::vector<Waypoint>& wps,
                      bool reversed = false,
                      int timeoutMs = -1,
                      float poseErrBailIn = 8.0f,
                      Follower follower = Follower::PURE_PURSUIT);

/**
 * Same as above, with mid-path events. Uses the default `TrajConstraints`
 * from `ramsete_configure()`.
 */
bool followTrajectory(const std::vector<Waypoint>& wps,
                      std::vector<PathEvent> events,
                      bool reversed = false,
                      int timeoutMs = -1,
                      float poseErrBailIn = 8.0f,
                      Follower follower = Follower::PURE_PURSUIT);

/**
 * Run a waypoint CSV (path.jerryio.com export or hand-written). Lines are
 * `"x,y"` or `"x,y,heading_rad"`; extra columns ignored. Inches/radians,
 * LightLib convention (+Y forward, CW+). Uses TrajConstraints from
 * ramsete_configure().
 */
bool runJerryioPath(const char* csv,
                    bool reversed = false,
                    int timeoutMs = -1,
                    float poseErrBailIn = 8.0f,
                    Follower follower = Follower::PURE_PURSUIT);

/** Variant with mid-path event triggers. See PathEvent for index semantics. */
bool runJerryioPath(const char* csv,
                    std::vector<PathEvent> events,
                    bool reversed = false,
                    int timeoutMs = -1,
                    float poseErrBailIn = 8.0f,
                    Follower follower = Follower::PURE_PURSUIT);

/** Read the CSV from the V5 SD card (e.g. `"/usd/paths/red_left.csv"`). */
bool runJerryioPathFromSD(const char* filePath,
                          bool reversed = false,
                          int timeoutMs = -1,
                          float poseErrBailIn = 8.0f,
                          Follower follower = Follower::PURE_PURSUIT);

/** SD-card variant with mid-path event triggers. */
bool runJerryioPathFromSD(const char* filePath,
                          std::vector<PathEvent> events,
                          bool reversed = false,
                          int timeoutMs = -1,
                          float poseErrBailIn = 8.0f,
                          Follower follower = Follower::PURE_PURSUIT);

/**
 * \name Characterization routines
 * Each is meant to be run as its own selectable auton.
 * @{
 */
/** Identify kV / kA / kS by ramping voltage and recording velocity. */
void characterize_kV_kA_kS(float maxVoltage = 10.0f, float rampVps = 0.25f);

/** Identify the effective track width by spinning in place. */
float characterize_track_width(int rotations = 10);

/** Identify the maximum sustainable lateral acceleration. */
float characterize_a_lat_max();

/**
 * Friction-FF auto-ID for non-trajectory PIDs. Fits
 * V = kS·sgn(v) + kV·v + kQ·v·|v| (motor-cmd units) and applies via
 * chassis.friction_constants_set(). Dumps to /usd/friction_id.csv if SD present.
 */
void characterize_friction(float maxVoltage = 10.0f, float stepV = 0.5f);
/** @} */

/**
 * \name Bracket-and-bisect PID auto-tune (drive / turn / swing)
 * Two-phase tuner that uses the chassis's own PID with kI=0.
 *   Phase 1 — find kP boundary: doubles kP until the robot overshoots, then
 *   bisects the [no-overshoot, overshoot] interval until within tolerance.
 *   Phase 2 — find kD: boosts kP past the boundary (×2 by default) so the
 *   robot intentionally overshoots, then doubles kD until the overshoot is
 *   damped and bisects to find the smallest sufficient kD.
 * Net result: aggressive kP with just-enough kD — faster response than pure-P
 * with no residual overshoot. Final values are applied live via
 * pid_*_constants_set; transcribe the printed FINAL line into
 * default_constants() to persist across reboots. Space: turn/swing ~3 ft²;
 * drive ≥6 ft straight.
 * @{
 */
void autotune_turn_pid (float turnAngleDeg  = 180.0f, float overshootThreshDeg = 3.0f,
                        float kpStart = 1.0f, int maxIters = 10);
void autotune_drive_pid(float driveDistIn   = 24.0f, float overshootThreshIn   = 0.5f,
                        float kpStart = 2.0f, int maxIters = 10);
void autotune_swing_pid(float swingAngleDeg = 90.0f, float overshootThreshDeg  = 3.0f,
                        float kpStart = 1.0f, int maxIters = 10);
/** @} */

/**
 * Relay-feedback (Åström–Hägglund) heading-correction PID auto-tune. Robot
 * creeps forward at forwardV while relay modulates L/R differential. Ku/Pu →
 * Z-N classic gains, applied live via pid_heading_constants_set. Needs ≥8 ft
 * lane. Heading PID is a corrective overlay during drives — it doesn't fit the
 * velocity-based tuner pattern that turn/swing/drive use.
 */
void autotune_heading_pid(float forwardV = 3.0f, float reliefV = 2.0f,
                          int cycles = 5, int timeoutMs = 15000,
                          int chunkCycles = 2, int coolMs = 5000);

/**
 * Stationary EKF process-noise calibration. Robot parked. Per-tick var(x,y,θ)
 * → Q values, pushed live via setConfig().
 */
void autotune_ekf_noise(int sampleMs = 10, int durationMs = 5000, int warmupMs = 500);

/**
 * Stationary MCL distance-sensor noise calibration. Robot parked with each
 * sensor pointed at a wall in range. Mean per-sensor stddev → sensorSigmaIn;
 * outlierGapIn = ~3·sigma. Pushed live via setConfig().
 */
void autotune_mcl_noise(int sampleMs = 20, int durationMs = 4000, int warmupMs = 300);

}  // namespace light
