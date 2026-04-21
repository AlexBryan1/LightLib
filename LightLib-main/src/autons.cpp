#include "autons.hpp"
#include <cmath>
#include "EZ-Template/util.hpp"
#include "LightLib/auton_timer.hpp"
#include "LightLib/cstm_move.hpp"
#include "LightLib/main.h"
#include "pros/motors.h"
#include "subsystems.hpp"
#include "LightLib/drive_utils.hpp"
#include "LightLib/odom.hpp"
#include "LightLib/ramsete.hpp"
#include "Lightlib/paths.hpp"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                          AUTONOMOUS CODE                                │
// │                                                                         │
// │  This file has three jobs:                                              │
// │    1. default_constants()  — tune your PID / slew / exit conditions     │
// │    2. default_positions()  — set your pistons before every match        │
// │    3. Auton routines       — write the actual autonomous paths          │
// │                                                                         │
// │  QUICK REFERENCE — common EZ-Template motion calls:                     │
// │                                                                         │
// │  chassis.pid_drive_set(inches, speed)                                   │
// │      Drive straight.  Negative inches = backwards.                      │
// │                                                                         │
// │  chassis.pid_turn_set(degrees, speed)                                   │
// │      Turn to an absolute heading (0–360, clockwise-positive).           │
// │                                                                         │
// │  chassis.pid_swing_set(SIDE, degrees, speed, slop, DIR)                 │
// │      One-side swing turn.  SIDE = ez::LEFT_SWING / ez::RIGHT_SWING.     │
// │      DIR  = ez::cw / ez::ccw (which way to swing).                      │
// │      slop = how many degrees before exit the other side can move.       │
// │                                                                         │
// │  chassis.pid_wait()          — block until the current motion finishes  │
// │  chassis.pid_wait_until(x)   — block until the robot reaches x          │
// │                                (inches for drive, degrees for turn)     │
// │  chassis.pid_wait_quick_chain() — exit early and start the next motion  │
// │                                                                         │
// │  light::moveToPoint(x, y, timeout, speed, reversed)                     │
// │      Drive to a field coordinate using odometry.                        │
// └─────────────────────────────────────────────────────────────────────────┘

// ── Speed presets ─────────────────────────────────────────────────────────────
// Used by the test functions at the bottom of this file.
// Your actual auton routines pass speed directly into each motion call.
const int DRIVE_SPEED = 127;  // max drive speed (0–127)
const int TURN_SPEED  = 127;  // max turn speed  (0–127)
const int SWING_SPEED = 127;  // max swing speed (0–127)


// ┌─────────────────────────────────────────────────────────────────────────┐
// │  default_positions()                                                    │
// │  Called once from initialize() before every match.                      │
// │  Set every piston to its safe starting state here.                      │
// │                                                                         │
// │  .set(true)  = piston extended                                          │
// │  .set(false) = piston retracted                                         │
// └─────────────────────────────────────────────────────────────────────────┘
void default_positions() {
    Wings.set(true);
    Loader.set(true);
    MidGoal.set(false);
    Hood.set(true);
}


// ┌─────────────────────────────────────────────────────────────────────────┐
// │  default_constants()                                                    │
// │  Called once from initialize().  Tune your PID gains, exit conditions,  │
// │  and slew rates here.  These apply to every auton.                      │
// │                                                                         │
// │  PID format:  (kP, kI, kD)                                              │
// │    kP — proportional: how hard the robot corrects for error             │
// │    kI — integral: corrects for small steady-state error (keep near 0)   │
// │    kD — derivative: dampens overshooting (increase if oscillating)      │
// └─────────────────────────────────────────────────────────────────────────┘
void default_constants() {

    // ── PID gains ─────────────────────────────────────────────────────────
    // Drive: used for all straight-line motions (odom and non-odom)
    chassis.pid_drive_constants_set(10.0, 0.1, 30.0);

    // Heading: keeps the robot pointed straight during a drive motion
    chassis.pid_heading_constants_set(17.0, 0.0, 70.0);

    // Turn: used for pid_turn_set() in-place pivots
    chassis.pid_turn_constants_set(5.0, 0.0, 45.0);

    // Swing: used for pid_swing_set() single-side turns
    chassis.pid_swing_constants_set(4.0, 0.0, 45.0);

    // Odom angular: heading correction during odom drive-to-point motions
    chassis.pid_odom_angular_constants_set(1.0, 0.0, 10.0);

    // Odom boomerang: angular control for boomerang (curved) odom paths
    chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);

    // ── Exit conditions ───────────────────────────────────────────────────
    // Format: (small_timeout, small_error, big_timeout, big_error, velocity_timeout, mech_timeout)
    //   The motion exits when error stays within small_error for small_timeout,
    //   OR within big_error for big_timeout.  Tune these to balance speed vs. accuracy.
    chassis.pid_turn_exit_condition_set (100_ms, 3_deg, 300_ms,  7_deg, 500_ms, 500_ms);
    chassis.pid_swing_exit_condition_set ( 90_ms, 3_deg, 250_ms,  7_deg, 500_ms, 500_ms);
    chassis.pid_drive_exit_condition_set ( 50_ms, 1_in,  250_ms,  3_in,  500_ms, 500_ms);
    chassis.pid_odom_turn_exit_condition_set ( 90_ms, 3_deg, 250_ms,  7_deg, 500_ms, 750_ms);
    chassis.pid_odom_drive_exit_condition_set( 90_ms, 1_in,  250_ms,  3_in,  500_ms, 750_ms);

    // ── Chain constants ───────────────────────────────────────────────────
    // How close to the target the robot needs to be before pid_wait_quick_chain()
    // exits and starts the next motion.  Larger = exit sooner (faster, less accurate).
    chassis.pid_turn_chain_constant_set (10_deg);
    chassis.pid_swing_chain_constant_set(15_deg);
    chassis.pid_drive_chain_constant_set( 5_in);

    // ── Slew constants ────────────────────────────────────────────────────
    // Slew ramps the motor output up from a starting speed over a distance,
    // so the robot doesn't spin its wheels on the first inch.
    // Format: (distance_to_ramp_over, starting_speed)
    chassis.slew_turn_constants_set (10_deg, 55);
    chassis.slew_drive_constants_set ( 3_in,  30);
    chassis.slew_swing_constants_set ( 3_in,  80);

    // ── Odom settings ─────────────────────────────────────────────────────
    // Turn bias: how much turning is prioritized over driving in odom motions.
    // 1.0 = full priority on turns (recommended if you have tracking wheels).
    chassis.odom_turn_bias_set(1.0);

    // Look-ahead: how far ahead on the path the robot targets (boomerang/pure-pursuit).
    chassis.odom_look_ahead_set(15_in);

    // Boomerang distance: max distance the "carrot" point can be from the target.
    chassis.odom_boomerang_distance_set(16_in);

    // Boomerang dlead: aggressiveness of the approach at the end of boomerang motions.
    // 0 = very gentle, 1 = very sharp.
    chassis.odom_boomerang_dlead_set(0.625);

    // Angle behavior: shortest path by default (robot won't spin 270° when 90° works).
    chassis.pid_angle_behavior_set(ez::shortest);

    // ── RAMSETE / trajectory follower constants ──────────────────────────
    // These come from the characterize_* routines — run them once per robot
    // build and paste the printed numbers here. The defaults below are
    // placeholders; they will drive but not cleanly until characterized.
    light::ramsete_configure(
        // geometry
        { /*b=*/2.0f, /*zeta=*/0.7f,
          /*trackWidthIn=*/11.5f, /*wheelDiamIn=*/3.25f, /*gearRatio=*/0.75f },
        // feedforward + velocity P
        { /*kS=*/0.60f, /*kV=*/0.18f, /*kA=*/0.03f, /*kP=*/0.02f },
        // default trajectory constraints
        { /*vMax=*/48.0f, /*aMax=*/60.0f, /*aDecMax=*/60.0f, /*aLatMax=*/40.0f });
}


// ┌─────────────────────────────────────────────────────────────────────────┐
// │  AUTONOMOUS ROUTINES                                                    │
// │  Write one function per auton below.  Register each one in              │
// │  auton_config.cpp with light::auton_selector.add() so it appears        │
// │  on the brain screen selector.                                          │
// └─────────────────────────────────────────────────────────────────────────┘


// Turret tracking — points turret at a fixed field coordinate using odometry.
// Set basket_x / basket_y to the target goal position and turret_p to taste.
// Not used in competition; run in a separate task if you need it.
void track_basket() {
    double basket_x  = 0.0;  // field X of the goal you're scoring in (inches)
    double basket_y  = 0.0;  // field Y of the goal you're scoring in (inches)
    double turret_p  = 0.0;  // proportional gain for the turret P-loop

    while (true) {
        double current_x     = chassis.odom_x_get();
        double current_y     = chassis.odom_y_get();
        double current_angle = chassis.odom_theta_get();

        double dx       = basket_x - current_x;
        double dy       = basket_y - current_y;
        double target   = std::atan2(dy, dx) * (180.0 / M_PI);
        double error    = target - current_angle;
        double output   = error * turret_p;
        (void)output;   // apply output to turret motor here

        pros::delay(10);
    }
}
// ── RAMSETE demo + characterization autons ─────────────────────────────────
// Register these in auton_config.cpp via light::auton_selector.add() if you
// want them on the brain selector. They share the drivetrain with EZ-Template
// via an internal pause/resume guard.

void ramsete_demo_s_curve() {
    // Smooth S-curve: start at origin facing +Y, kiss (24, 24) heading east,
    // end at (48, 0) facing +Y again. Good first real-geometry test.
    std::vector<light::Waypoint> path = {
        { 0.0f,  0.0f, 0.0f },
        { 24.0f, 24.0f, (float)M_PI_2 },
        { 48.0f,  0.0f, 0.0f }
    };
    light::TrajConstraints cons{ /*vMax=*/40.0f, /*aMax=*/50.0f,
                                 /*aDecMax=*/50.0f, /*aLatMax=*/40.0f };
    light::followTrajectory(path, cons);
}

void ramsete_char_kVkAkS()     { light::characterize_kV_kA_kS(); }
void ramsete_char_trackwidth() { light::characterize_track_width(); }
void ramsete_char_alat()       { light::characterize_a_lat_max(); }

// Run a Jerryio-authored path by name. Path bodies live in
// include/LightLib/paths/<name>.hpp and are registered in src/LightLib/paths.cpp.
// The densified format is auto-detected and translated so point 0 sits at
// the robot's current pose — so this starts wherever the robot happens to be.
void run_jerryio_path_1() {
    light::runPath("test_path");
}

void shake() {
    chassis.pid_turn_set(10_deg, 127);
    pros::delay(100);
    chassis.pid_turn_set(-10_deg, 127);
    pros::delay(100);
    chassis.pid_turn_set(10_deg, 127);
    pros::delay(100);
    chassis.pid_turn_set(-10_deg, 127);
    pros::delay(100);
    chassis.pid_turn_set(10_deg, 127);
    pros::delay(100);
    chassis.pid_turn_set(-10_deg, 127);
    pros::delay(100);
    chassis.pid_turn_set(10_deg, 127);
    pros::delay(100);
    chassis.pid_turn_set(-10_deg, 127);
    pros::delay(100);
}
