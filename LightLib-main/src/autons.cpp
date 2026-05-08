#include "autons.hpp"

#include <cmath>

#include "LightLib/util/auton_timer.hpp"
#include "LightLib/drive/custom_move.hpp"
#include "LightLib/drive/drive_utils.hpp"
#include "LightLib/main.h"
#include "LightLib/drive/odometry.hpp"
#include "LightLib/path/ramsete.hpp"
#include "LightLib/path/pure_pursuit.hpp"
#include "LightLib/util/util.hpp"
#include "LightLib/path/paths.hpp"
#include "pros/motors.h"
#include "subsystems.hpp"

// Auton routines + setup. See ex_* functions below for copy-paste examples
// of every motion type. Full docs: docs/tutorials/.
//
// Motion cheat-sheet (speeds 0–127, distances in inches, angles in degrees):
//   chassis.pid_drive_set(in [, speed])           straight; negative = reverse
//   chassis.pid_turn_set(deg [, speed])           pivot to absolute heading
//   chassis.pid_swing_set(SIDE, deg [, speed])    one-side swing; SIDE = light::LEFT_SWING / RIGHT_SWING
//   chassis.pid_odom_set({x,y,heading}, speed)    drive to a field point
//   light::moveToPoint(x, y, timeout, speed, reversed)   simple odom move
//   light::followTrajectory(waypoints [, cons])  smooth path; defaults to
//                                                Pure Pursuit. Pass
//                                                light::Follower::RAMSETE
//                                                as the trailing arg to use
//                                                the time-parameterized
//                                                RAMSETE follower instead.
//                                                Same toggle works on
//                                                runJerryioPath / runPath.
//
// When the speed arg is omitted, the chassis uses light::DRIVE_SPEED /
// TURN_SPEED / SWING_SPEED (defined just below — edit those to change defaults).
//
//   chassis.pid_wait()              block until current motion finishes
//   chassis.pid_wait_until(x)       block until robot passes x in/deg
//   chassis.pid_wait_quick_chain()  exit early, start next motion immediately

// Default speeds for chassis.pid_drive_set / pid_turn_set / pid_swing_set when
// the speed argument is omitted. Declared in LightLib/drive/drive.hpp.
namespace light {
int DRIVE_SPEED = 127;
int TURN_SPEED  = 127;
int SWING_SPEED = 127;
}

void ramsete_char_kVkAkS() { light::characterize_kV_kA_kS(); }
void ramsete_char_trackwidth() { light::characterize_track_width(); }
void ramsete_char_alat() { light::characterize_a_lat_max(); }

// Set every piston to its safe starting state. Called once before each match.
// .set(true) = extended, .set(false) = retracted.
void default_positions() {
  Wings.set(true);
  Loader.set(true);
  MidGoal.set(false);
  Hood.set(true);
}

// Tune PID gains, exit conditions, slew, and odom settings here.
// Called once from initialize(). PID format is (kP, kI, kD); see
// docs/tutorials/03_pid_tuning.md for what each gain does.
void default_constants() {
  // ── PID gains ──
  chassis.pid_drive_constants_set(3.0, 0.0, 0.0);            // straight-line drive
  chassis.pid_heading_constants_set(3.0, 0.0, 0.0);          // heading-hold during drive
  chassis.pid_turn_constants_set(3.0, 0.0, 0.0);             // in-place pivot
  chassis.pid_swing_constants_set(3.0, 0.0, 0.0);            // single-side swing
  chassis.pid_odom_angular_constants_set(3.0, 0.0, 0.0);     // odom heading
  chassis.pid_odom_boomerang_constants_set(3.0, 0.0, 0.0);   // boomerang angular

  // ── Exit conditions: (small_t, small_err, big_t, big_err, vel_t, mech_t) ──
  // Exits when error stays under small_err for small_t, OR under big_err for big_t.
  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 300_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);

  // ── Chain thresholds: how close before pid_wait_quick_chain() releases ──
  chassis.pid_turn_chain_constant_set(10_deg);
  chassis.pid_swing_chain_constant_set(15_deg);
  chassis.pid_drive_chain_constant_set(5_in);

  // ── Slew: (ramp_distance, start_speed) — prevents wheel spin off the line ──
  chassis.slew_turn_constants_set(10_deg, 55);
  chassis.slew_drive_constants_set(3_in, 50);
  chassis.slew_swing_constants_set(3_in, 80);

  // ── Odom settings ──
  chassis.odom_turn_bias_set(1.0);              // 1.0 = prioritize turns (use w/ tracking wheels)
  chassis.odom_look_ahead_set(15_in);           // pure-pursuit / boomerang lookahead
  chassis.odom_boomerang_distance_set(16_in);   // max carrot distance
  chassis.odom_boomerang_dlead_set(0.625);      // 0 gentle … 1 sharp
  chassis.pid_angle_behavior_set(light::shortest);

  // ── RAMSETE: paste output of the Char: * autons here once tuned ──
  light::ramsete_configure(
      // geometry
      {/*b=*/2.0f, /*zeta=*/0.7f,
       /*trackWidthIn=*/11.5f, /*wheelDiamIn=*/3.25f, /*gearRatio=*/0.75f},
      // feedforward + velocity P
      {/*kS=*/0.60f, /*kV=*/0.18f, /*kA=*/0.03f, /*kP=*/0.02f},
      // default trajectory constraints
      {/*vMax=*/48.0f, /*aMax=*/60.0f, /*aDecMax=*/60.0f, /*aLatMax=*/40.0f});

  // ── Pure Pursuit: carrot lookahead (only PP-specific knob; chassis +
  //    feedforward come from ramsete_configure() above). Larger lookahead =
  //    smoother but cuts corners; smaller = tighter but oscillates.
  light::pure_pursuit_configure({/*lookaheadIn=*/12.0f});
}



// ── Example autons: one per motion type. Copy/edit when writing real autons.
//    Register any you want on the picker in auton_config.cpp. ──

// pid_drive_set: straight forward, then back. Negative inches = reverse.
// Speed arg is optional — defaults to light::DRIVE_SPEED.
void ex_drive() {
  chassis.pid_drive_set(24_in);
  chassis.pid_wait();
  chassis.pid_drive_set(-24_in);
  chassis.pid_wait();
}

// pid_turn_set: pivot in place to absolute headings.
void ex_turn() {
  chassis.pid_turn_set(90_deg);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg);
  chassis.pid_wait();
}

// pid_swing_set: turn using only one side of the drivetrain (sweeping arc).
void ex_swing() {
  chassis.pid_swing_set(light::LEFT_SWING, 45_deg);
  chassis.pid_wait();
  chassis.pid_swing_set(light::RIGHT_SWING, -45_deg);
  chassis.pid_wait();
}

// pid_wait_quick_chain: flow drive→turn→drive without fully stopping.
// Each motion exits early once within its chain threshold (set in default_constants).
void ex_chain() {
  chassis.pid_drive_set(24_in);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(24_in);
  chassis.pid_wait();
}

// pid_odom_set: drive to a single field coordinate (x, y in inches, heading in deg).
void ex_odom_ptp() {
  chassis.pid_odom_set({{24_in, 24_in, 90_deg}, light::fwd, DRIVE_SPEED});
  chassis.pid_wait();
}

// pid_odom_set with a vector of waypoints — chains odom moves into a path.
void ex_odom_path() {
  chassis.pid_odom_set({
      {{24_in, 0_in,  0_deg},  fwd, DRIVE_SPEED},
      {{24_in, 24_in, 90_deg}, light::fwd, DRIVE_SPEED},
      {{0_in,  24_in, 180_deg}, light::fwd, DRIVE_SPEED},
  });
  chassis.pid_wait();
}

// light::moveToPoint: simple odom drive-to-point with a hard timeout.
void ex_move_to_point() {
  light::moveToPoint(/*x=*/36, /*y=*/12, /*timeout_ms=*/2500,
                     /*speed=*/100, /*reversed=*/false);
}

// followTrajectory: smooth RAMSETE path through 3+ waypoints (x, y, heading_deg).
// Headings are in degrees; omit (use {} or trailing comma) to let the spline
// pick a smooth tangent. Uses the default TrajConstraints from ramsete_configure()
// in default_constants(); pass an explicit `light::TrajConstraints{...}` as a 2nd
// arg to override per-call.
void ramsete_demo_s_curve() {
  // Smooth S-curve: start at origin facing +Y, kiss (24, 24) heading east (90°),
  // end at (48, 0) facing +Y again. Good first real-geometry test.
  std::vector<light::Waypoint> path = {
      {0,  0,  0},
      {24, 24, 90},
      {48, 0,  0}};
  light::followTrajectory(path);
}

// Run a Jerryio-authored path by name. Path bodies live in
// include/LightLib/paths/<name>.hpp and are registered in src/LightLib/paths.cpp.
// The densified format is auto-detected and translated so point 0 sits at
// the robot's current pose — so this starts wherever the robot happens to be.
void run_jerryio_path_1() {
  light::runPath("test_path");
}

