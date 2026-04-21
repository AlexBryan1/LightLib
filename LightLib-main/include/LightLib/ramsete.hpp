#pragma once
// ─── RAMSETE trajectory follower ─────────────────────────────────────────────
//
// Public API for time-parameterized path following on the VEX tankdrive.
// The follower owns the motor groups exclusively for the duration of the
// motion and pauses the EZ-Template drive PID task while it runs.
//
// Typical usage:
//
//   // In initialize():
//   light::ramsete_configure(
//       { /*b=*/2.0f, /*zeta=*/0.7f,
//         /*trackWidthIn=*/11.5f, /*wheelDiamIn=*/3.25f, /*gearRatio=*/0.75f },
//       { /*kS=*/0.60f, /*kV=*/0.18f, /*kA=*/0.03f, /*kP=*/0.02f },
//       { /*vMax=*/48.0f, /*aMax=*/60.0f, /*aDec=*/60.0f, /*aLat=*/60.0f });
//
//   // In an auton:
//   std::vector<light::Waypoint> path = { {0,0}, {24,24,M_PI/2}, {48,0} };
//   light::followTrajectory(path, { 48, 60, 60, 40 });

#include <functional>
#include <vector>
#include "LightLib/trajectory.hpp"

namespace light {

// Mid-path action triggered at a specific input waypoint.
//
// `atWaypoint` is zero-based. Semantics depend on the entry point:
//   * followTrajectory(wps, cons, events, ...):  index into the `wps` vector.
//   * runJerryioPath / runJerryioPathFromSD:    index into the CSV's
//     *heading-bearing* lines (i.e., the points you clicked in Jerryio).
//     For CSVs with no heading columns at all, falls back to raw line index.
//
// `action` is invoked from the 10 ms control tick on which the trajectory's
// sampled time first passes the event's resolved time. Each event fires at
// most once. Callbacks may call light::setPose() to re-zero odom mid-path —
// the next tick will read the new pose and the RAMSETE error term will
// correct against the unchanged trajectory.
struct PathEvent {
    int atWaypoint = 0;
    std::function<void()> action;
};

struct RamseteConfig {
    float b    = 2.0f;   // aggressiveness  — larger = more correction
    float zeta = 0.7f;   // damping         — [0, 1], 0.7 is classic
    float trackWidthIn = 12.0f;
    float wheelDiamIn  = 3.25f;
    float gearRatio    = 0.75f;  // wheel_rpm / motor_rpm (0.75 = 36:48 blue)
};

struct DriveFF {
    float kS = 0.0f;   // V to break static friction
    float kV = 0.0f;   // V per (in/s) steady-state
    float kA = 0.0f;   // V per (in/s²)
    float kP = 0.0f;   // V per (in/s) velocity-error
};

void ramsete_configure(RamseteConfig rc, DriveFF ff, TrajConstraints defaultCons);

bool followTrajectory(const Trajectory& traj,
                      int timeoutMs = -1,
                      float poseErrBailIn = 8.0f);

bool followTrajectory(const std::vector<Waypoint>& wps,
                      const TrajConstraints& cons,
                      bool reversed = false,
                      int timeoutMs = -1,
                      float poseErrBailIn = 8.0f);

// Variant with mid-path event triggers. Events whose index is out of range
// are skipped (with a printf warning); the path still runs.
bool followTrajectory(const std::vector<Waypoint>& wps,
                      const TrajConstraints& cons,
                      std::vector<PathEvent> events,
                      bool reversed = false,
                      int timeoutMs = -1,
                      float poseErrBailIn = 8.0f);

// Run a waypoint path exported from path.jerryio.com (or any CSV-like text).
// Each non-empty, non-'#'-commented line has 2 or 3 floats separated by
// commas, tabs, or whitespace: "x, y" or "x, y, heading_rad".
// Units: inches, radians. LightLib convention: +Y forward, theta=0 faces +Y, CW+.
// Uses the TrajConstraints from ramsete_configure(). Extra columns past 3
// are ignored so exports with speed/lookahead fields still work.
bool runJerryioPath(const char* csv,
                    bool reversed = false,
                    int timeoutMs = -1,
                    float poseErrBailIn = 8.0f);

// Variant with mid-path event triggers. See PathEvent for index semantics.
bool runJerryioPath(const char* csv,
                    std::vector<PathEvent> events,
                    bool reversed = false,
                    int timeoutMs = -1,
                    float poseErrBailIn = 8.0f);

// Same, but read the CSV from the V5 SD card (e.g. "/usd/paths/red_left.csv").
bool runJerryioPathFromSD(const char* filePath,
                          bool reversed = false,
                          int timeoutMs = -1,
                          float poseErrBailIn = 8.0f);

bool runJerryioPathFromSD(const char* filePath,
                          std::vector<PathEvent> events,
                          bool reversed = false,
                          int timeoutMs = -1,
                          float poseErrBailIn = 8.0f);

// Characterization — run each as its own selectable auton.
void  characterize_kV_kA_kS(float maxVoltage = 10.0f, float rampVps = 0.25f);
float characterize_track_width(int rotations = 10);
float characterize_a_lat_max();

// ── Relay-feedback (Åström-Hägglund) PID auto-tune ──────────────────────────
//
// Each routine runs a bang-bang oscillation around the current pose, measures
// the steady-state amplitude `a` and period `Tu`, computes
//     Ku = 4·h/(π·a),  Pu = Tu,
//     kP = 0.6·Ku, kI = 2·kP/Pu, kD = kP·Pu/8       (Z-N classic PID)
// then printf's the result AND calls the matching EZ `pid_*_constants_set`
// so the tune is live on the chassis immediately. You still need to
// transcribe the printout into default_constants() for it to survive a
// program restart.
//
// Params common to all four:
//   reliefV       relay voltage amplitude (volts, ±). Must exceed kS — if the
//                 robot never moves, the tune will time out and abort without
//                 overwriting the previous PID values.
//   cycles        total number of full oscillation cycles to average.
//   timeoutMs     active-time bail (cooldowns don't count against this).
//   chunkCycles   cycles per chunk before a motor cooldown kicks in. 0 = no
//                 cooldowns (legacy continuous behavior).
//   coolMs        motor-off pause between chunks. Keeps drive motors cool
//                 during long tunes. Wall time ≈ timeoutMs + (cycles/chunkCycles - 1)·coolMs.
//
// Space requirements:
//   turn/swing  — about 2 ft² (in-place oscillation).
//   drive       — ≥8 ft straight clear ahead.
//   heading     — ≥8 ft straight lane; the robot drives forward at forwardV
//                 while the relay corrects heading around 0.
void autotune_turn_pid   (float reliefV = 4.0f, int cycles = 6, int timeoutMs = 15000,
                          int chunkCycles = 2, int coolMs = 5000);
void autotune_drive_pid  (float reliefV = 6.0f, int cycles = 5, int timeoutMs = 15000,
                          int chunkCycles = 2, int coolMs = 5000);
void autotune_swing_pid  (float reliefV = 4.0f, int cycles = 6, int timeoutMs = 15000,
                          int chunkCycles = 2, int coolMs = 5000);
void autotune_heading_pid(float forwardV = 3.0f, float reliefV = 2.0f,
                          int cycles = 5, int timeoutMs = 15000,
                          int chunkCycles = 2, int coolMs = 5000);

} // namespace light
