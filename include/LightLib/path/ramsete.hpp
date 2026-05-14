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

} // namespace light