#include "autons.hpp"
#include <cmath>
#include "EZ-Template/util.hpp"
#include "LightLib/cstm_move.hpp"
#include "LightLib/main.h"
#include "pros/motors.h"
#include "subsystems.hpp"
#include "LightLib/drive_utils.hpp"
#include "LightLib/odom.hpp"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                          AUTONOMOUS CODE                                 │
// │                                                                         │
// │  This file has three jobs:                                              │
// │    1. default_constants()  — tune your PID / slew / exit conditions     │
// │    2. default_positions()  — set your pistons before every match        │
// │    3. Auton routines       — write the actual autonomous paths          │
// │                                                                         │
// │  QUICK REFERENCE — common EZ-Template motion calls:                    │
// │                                                                         │
// │  chassis.pid_drive_set(inches, speed)                                   │
// │      Drive straight.  Negative inches = backwards.                     │
// │                                                                         │
// │  chassis.pid_turn_set(degrees, speed)                                   │
// │      Turn to an absolute heading (0–360, clockwise-positive).          │
// │                                                                         │
// │  chassis.pid_swing_set(SIDE, degrees, speed, slop, DIR)                 │
// │      One-side swing turn.  SIDE = ez::LEFT_SWING / ez::RIGHT_SWING.    │
// │      DIR  = ez::cw / ez::ccw (which way to swing).                     │
// │      slop = how many degrees before exit the other side can move.      │
// │                                                                         │
// │  chassis.pid_wait()          — block until the current motion finishes  │
// │  chassis.pid_wait_until(x)   — block until the robot reaches x         │
// │                                (inches for drive, degrees for turn)     │
// │  chassis.pid_wait_quick_chain() — exit early and start the next motion  │
// │                                                                         │
// │  light::moveToPoint(x, y, timeout, speed, reversed)                    │
// │      Drive to a field coordinate using odometry.                       │
// └─────────────────────────────────────────────────────────────────────────┘

// ── Speed presets ─────────────────────────────────────────────────────────────
// Used by the test functions at the bottom of this file.
// Your actual auton routines pass speed directly into each motion call.
const int DRIVE_SPEED = 127;  // max drive speed (0–127)
const int TURN_SPEED  = 127;  // max turn speed  (0–127)
const int SWING_SPEED = 127;  // max swing speed (0–127)


// ┌─────────────────────────────────────────────────────────────────────────┐
// │  default_positions()                                                     │
// │  Called once from initialize() before every match.                     │
// │  Set every piston to its safe starting state here.                     │
// │                                                                         │
// │  .set(true)  = piston extended                                          │
// │  .set(false) = piston retracted                                         │
// └─────────────────────────────────────────────────────────────────────────┘
void default_positions() {
    Wings.set(true);
    Loader.set(true);
    MidGoal.set(false);
    Hood.set(true);
    IntakeLift.set(true);
}


// ┌─────────────────────────────────────────────────────────────────────────┐
// │  default_constants()                                                     │
// │  Called once from initialize().  Tune your PID gains, exit conditions, │
// │  and slew rates here.  These apply to every auton.                     │
// │                                                                         │
// │  PID format:  (kP, kI, kD)                                             │
// │    kP — proportional: how hard the robot corrects for error             │
// │    kI — integral: corrects for small steady-state error (keep near 0)  │
// │    kD — derivative: dampens overshooting (increase if oscillating)     │
// └─────────────────────────────────────────────────────────────────────────┘
void default_constants() {

    // ── PID gains ─────────────────────────────────────────────────────────
    // Drive: used for all straight-line motions (odom and non-odom)
    chassis.pid_drive_constants_set(10.0, 0.1, 30.0);

    // Heading: keeps the robot pointed straight during a drive motion
    chassis.pid_heading_constants_set(17.0, 0.0, 50.0);

    // Turn: used for pid_turn_set() in-place pivots
    chassis.pid_turn_constants_set(5.0, 0.0, 35.0);

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
}


// ┌─────────────────────────────────────────────────────────────────────────┐
// │  AUTONOMOUS ROUTINES                                                     │
// │  Write one function per auton below.  Register each one in             │
// │  auton_config.cpp with light::auton_selector.add() so it appears       │
// │  on the brain screen selector.                                          │
// └─────────────────────────────────────────────────────────────────────────┘

void skills() {
    // ── 60-second skills routine ──────────────────────────────────────────
}

// ── Rush left side ────────────────────────────────────────────────────────────
void rush_left() {
    Score.move(-127);
    Wings.set(false);
    chassis.pid_swing_set(ez::RIGHT_SWING, 210_deg, 127, 50, ez::ccw);
    chassis.pid_wait_until(220);
    Loader.set(false);
    chassis.pid_drive_set(35_in, 127);
    chassis.pid_wait_until(25_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 40, ez::ccw);
    pros::delay(300);
    chassis.pid_drive_set(50_in, 80);
    pros::delay(1000);
    chassis.pid_drive_set(-30_in, 127);
    chassis.pid_wait_until(-15_in);
    Hood.set(false);
    pros::delay(1650);
    chassis.pid_swing_set(ez::RIGHT_SWING, 100_deg, 127, 5, ez::ccw);
    chassis.pid_wait_until(110_deg);
    Hood.set(true);
    chassis.pid_swing_set(ez::LEFT_SWING, 185_deg, 127, 4, ez::cw);
    chassis.pid_wait_until(177_deg);
    Wings.set(true);
    chassis.pid_drive_set(-25_in, 127);
    chassis.pid_wait();
}

// ── Rush right side ───────────────────────────────────────────────────────────
void rush_right() {
    // Add routine here
}

// ── Rush to mid goal — left start ─────────────────────────────────────────────
void rush_mid_left() {
    // Add routine here
}

// ── Rush to mid goal — right start ───────────────────────────────────────────
void rush_mid_right() {
    Score.move(-127);
    chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 50, ez::ccw);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    chassis.pid_swing_set(ez::LEFT_SWING, 300_deg, 127, 0, ez::cw);
    chassis.pid_wait_quick_chain();
    Loader.set(true);
    chassis.pid_drive_set(20_in, 127);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    pros::delay(200);
}

// ── Seven-ball — left start ───────────────────────────────────────────────────
void sevenball_left() {
    // Add routine here
}

// ── Seven-ball — right start ──────────────────────────────────────────────────
void sevenball_right() {
    // Add routine here
}

// ── Split — left start ────────────────────────────────────────────────────────
void split_left() {
    Score.move(-127);
    chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 45, ez::ccw);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    chassis.pid_swing_set(ez::LEFT_SWING, 300_deg, 127, 0, ez::cw);
    chassis.pid_wait_quick_chain();
    Loader.set(true);
    chassis.pid_drive_set(20_in, 127);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    pros::delay(200);
    chassis.pid_swing_set(ez::RIGHT_SWING, 80_deg, 127, 30, ez::cw);
    chassis.pid_wait_until(45_deg);
    chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 0, ez::cw);
    chassis.pid_wait_until(160_deg);
    chassis.pid_drive_set(-30_in, 127);
    Hood.set(false);
    pros::delay(700);
    chassis.pid_turn_set(175_deg, 127);
    pros::delay(800);
    Hood.set(true);
    chassis.pid_drive_set(30_in, 127);
    chassis.pid_wait_until(15_in);
    chassis.pid_speed_max_set(40);
    pros::delay(800);
    chassis.pid_speed_max_set(127);
    chassis.pid_swing_set(ez::RIGHT_SWING, 217_deg, 127, 20, ez::cw);
    chassis.pid_wait_until(212_deg);
    chassis.pid_drive_set(-45_in, 127);
    Score.move(127);
    pros::delay(100);
    Score.move(0);
    chassis.pid_wait_until(-25_in);
    chassis.pid_speed_max_set(80);
    MidGoal.set(true);
    Score.move(-127);
    pros::delay(1000);
    chassis.pid_speed_max_set(127);
    Score.move(0);
    chassis.pid_drive_set(20_in, 127);
    chassis.pid_wait_until(15_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 150_deg, 127, 5, ez::ccw);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-25_in, 127);
    chassis.pid_wait_until(-10_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 0, ez::cw);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 25000_ms, 3_in, 50000_ms, 50000_ms);
    MidGoal.set(false);
    Loader.set(true);
    chassis.pid_drive_set(-3_in, 127);
    chassis.pid_wait();
}

// ── Split — right start ───────────────────────────────────────────────────────
void split_right() {
    Score.move(-127);
    chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 50, ez::ccw);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::LEFT_SWING, 340_deg, 90, 40, ez::cw);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    pros::delay(200);
    chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 90, 60, ez::cw);
    chassis.pid_wait_quick_chain();
    chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 127, 10, ez::cw);
    chassis.pid_wait_quick_chain();
}
void sawp(){
  
}

// ┌─────────────────────────────────────────────────────────────────────────┐
// │  UTILITY FUNCTIONS                                                       │
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

// Pulse the scorer motor briefly to clear a jam.
void antijam() {
    Score.move(127);
    pros::delay(150);
    Score.move(-127);
}


// ┌─────────────────────────────────────────────────────────────────────────┐
// │  TEST / TUNING FUNCTIONS                                                │
// │  Register these in auton_config.cpp during practice then remove them    │
// │  before competition.                                                    │
// └─────────────────────────────────────────────────────────────────────────┘

// Basic connectivity / control check used during build/practice.
void con_test() {
    Hood.set(false);
    MidGoal.set(true);
    Score.move(-127);
    chassis.pid_drive_set(-10, 127);
    pros::delay(4000);
    light::moveToPoint(15.0, 10.0, 5000, 127, 0);
    chassis.pid_turn_set(0, 127);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-20, 127);
    chassis.pid_wait();
}
