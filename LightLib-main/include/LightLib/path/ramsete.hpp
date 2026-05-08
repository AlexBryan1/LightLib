#pragma once

#include <functional>
#include <vector>

#include "LightLib/path/trajectory.hpp"

/**
 * \file ramsete.hpp
 *
 * RAMSETE trajectory follower public API.
 *
 * Time-parameterized path following on the VEX tankdrive. The follower owns
 * the motor groups exclusively for the duration of the motion and pauses
 * the EZ-Template drive PID task while it runs.
 *
 * \par Typical usage
 * \code
 *   // In initialize():
 *   light::ramsete_configure(
 *       { 2.0f, 0.7f, 11.5f, 3.25f, 0.75f },     // RamseteConfig
 *       { 0.60f, 0.18f, 0.03f, 0.02f },          // DriveFF (kS, kV, kA, kP)
 *       { 48.0f, 60.0f, 60.0f, 60.0f });         // TrajConstraints
 *
 *   // In an auton:
 *   std::vector<light::Waypoint> path = { {0,0}, {24,24,M_PI/2}, {48,0} };
 *   light::followTrajectory(path, { 48, 60, 60, 40 });
 * \endcode
 */

namespace light {

/**
 * Selects which path-following controller runs the trajectory.
 *
 * Both followers consume the same Waypoint / Spline / Trajectory pipeline —
 * they only differ in the inner control law. RAMSETE is time-parameterized
 * with pose-error correction; PURE_PURSUIT is a lookahead-carrot follower
 * that reuses the same velocity profile.
 *
 * Defaults to PURE_PURSUIT on every public path entry point, so calls like
 * `followTrajectory(path)` route through Pure Pursuit. Pass
 * `Follower::RAMSETE` as the trailing argument to opt back in.
 */
enum class Follower {
  RAMSETE,
  PURE_PURSUIT,
};

/**
 * Mid-path action triggered at a specific input waypoint.
 *
 * `atWaypoint` is zero-based. Semantics depend on the entry point:
 *   - `followTrajectory(wps, cons, events, …)` — index into the `wps` vector.
 *   - `runJerryioPath` / `runJerryioPathFromSD` — index into the CSV's
 *     *heading-bearing* lines (the points you clicked in Jerryio). For CSVs
 *     with no heading columns at all, falls back to raw line index.
 *
 * `action` is invoked from the 10 ms control tick on which the trajectory's
 * sampled time first passes the event's resolved time. Each event fires at
 * most once. Callbacks may call `light::setPose()` to re-zero odom mid-path
 * — the next tick will read the new pose and the RAMSETE error term will
 * correct against the unchanged trajectory.
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
 * Run a waypoint path exported from path.jerryio.com (or any CSV-like text).
 *
 * Each non-empty, non-`#`-commented line has 2 or 3 floats separated by
 * commas, tabs, or whitespace: `"x, y"` or `"x, y, heading_rad"`. Units are
 * inches and radians (LightLib convention: +Y forward, theta=0 faces +Y,
 * CW+). Uses the TrajConstraints from `ramsete_configure()`. Extra columns
 * past 3 are ignored, so exports with speed/lookahead fields still work.
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
 * Friction-FF auto-ID for the non-trajectory PID controllers.
 *
 * Sweeps voltage open-loop, fits `V = kS·sgn(v) + kV·v + kQ·v·|v|` in motor-
 * cmd units, and applies the result via `chassis.friction_constants_set()`.
 * Raw samples are written to `/usd/friction_id.csv` when an SD card is
 * present.
 */
void characterize_friction(float maxVoltage = 10.0f, float stepV = 0.5f);
/** @} */

/**
 * \name Relay-feedback (Åström–Hägglund) PID auto-tune
 *
 * Each routine runs a bang-bang oscillation around the current pose,
 * measures the steady-state amplitude `a` and period `Tu`, computes
 * \code
 *   Ku = 4·h/(π·a),  Pu = Tu,
 *   kP = 0.6·Ku,  kI = 2·kP/Pu,  kD = kP·Pu/8       (Z-N classic PID)
 * \endcode
 * then printfs the result AND calls the matching EZ `pid_*_constants_set`
 * so the tune is live on the chassis immediately. You still need to
 * transcribe the printout into `default_constants()` for it to survive a
 * program restart.
 *
 * \par Common parameters
 *   - `reliefV` — relay voltage amplitude (volts, ±). Must exceed kS — if
 *     the robot never moves, the tune times out and aborts without
 *     overwriting the previous PID values.
 *   - `cycles` — total number of full oscillation cycles to average.
 *   - `timeoutMs` — active-time bail (cooldowns don't count against this).
 *   - `chunkCycles` — cycles per chunk before a motor cooldown kicks in.
 *     `0` = no cooldowns (legacy continuous behavior).
 *   - `coolMs` — motor-off pause between chunks. Wall time
 *     ≈ `timeoutMs + (cycles/chunkCycles - 1)·coolMs`.
 *
 * \par Space requirements
 *   - turn / swing — about 2 ft² (in-place oscillation).
 *   - drive — ≥ 8 ft straight clear ahead.
 *   - heading — ≥ 8 ft straight lane; the robot drives forward at
 *     `forwardV` while the relay corrects heading around 0.
 * @{
 */
void autotune_turn_pid(float reliefV = 4.0f, int cycles = 6, int timeoutMs = 15000,
                       int chunkCycles = 2, int coolMs = 5000);
void autotune_drive_pid(float reliefV = 6.0f, int cycles = 5, int timeoutMs = 15000,
                        int chunkCycles = 2, int coolMs = 5000);
void autotune_swing_pid(float reliefV = 4.0f, int cycles = 6, int timeoutMs = 15000,
                        int chunkCycles = 2, int coolMs = 5000);
void autotune_heading_pid(float forwardV = 3.0f, float reliefV = 2.0f,
                          int cycles = 5, int timeoutMs = 15000,
                          int chunkCycles = 2, int coolMs = 5000);
/** @} */

/**
 * Stationary EKF process-noise calibration. Robot must be parked on the
 * field throughout. Per-tick variance of `(x, y, theta)` over the sample
 * window is integrated to Q values and pushed into the live EKF via
 * `setConfig()`.
 *
 * \param sampleMs
 *        sample period in ms (default 10 — matches odom tick)
 * \param durationMs
 *        total wall time to sample (ms)
 * \param warmupMs
 *        discard initial transient before sampling (ms)
 */
void autotune_ekf_noise(int sampleMs = 10, int durationMs = 5000, int warmupMs = 500);

/**
 * Stationary MCL distance-sensor noise calibration. Robot must be parked
 * with every configured distance sensor pointed at a wall in range.
 * Per-sensor std-dev of raw range readings (in inches) is averaged to set
 * `sensorSigmaIn`, and `outlierGapIn` is set to ~3·sigma. Pushed into live
 * MCL via `setConfig()`.
 */
void autotune_mcl_noise(int sampleMs = 20, int durationMs = 4000, int warmupMs = 300);

}  // namespace light
