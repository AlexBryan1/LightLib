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
    chassis.pid_heading_constants_set(17.0, 0.0, 70.0);

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
    Score.move(-127);
    chassis.pid_drive_set(90_in, 75);
    chassis.pid_wait();
    pros::delay(300);
    chassis.pid_swing_set(ez::RIGHT_SWING, 110_deg, 127, 0, ez::cw);
    chassis.pid_wait_until(85_deg);
    chassis.pid_drive_set(-10_in, 127);
    pros::delay(500);
    chassis.pid_drive_set(20_in, 127);
    chassis.pid_wait();
}

void rush_left() {
    Score.move(-127);
    chassis.pid_swing_set(ez::RIGHT_SWING, 230_deg, 127, 45, ez::ccw);
    chassis.pid_wait_until(315_deg);
    Loader.set(false);
    chassis.pid_wait_until(240_deg);
    chassis.pid_drive_set(50_in, 127, false);
    chassis.pid_wait_until(18_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 175_deg, 127, 0, ez::ccw);
    chassis.pid_wait_until(185_deg);
    chassis.pid_drive_set(-50_in, 127, false);
    pros::delay(100);
    Hood.set(false);
    pros::delay(1300);
    chassis.pid_swing_set(ez::RIGHT_SWING, 100_deg, 127, 5, ez::ccw);
    chassis.pid_wait_until(130_deg);
    chassis.pid_swing_set(ez::LEFT_SWING, 183_deg, 127, 5, ez::cw);
    chassis.pid_wait_until(173_deg);
    chassis.pid_drive_set(-30_in, 127, false);
    chassis.pid_wait_until(-20_in);
}

void rush_right() {
    Score.move(-127);
    chassis.pid_swing_set(ez::LEFT_SWING, -230_deg, 127, 45, ez::cw);
    chassis.pid_wait_until(-315_deg);
    Loader.set(false);
    chassis.pid_wait_until(-240_deg);
    chassis.pid_drive_set(50_in, 127, false);
    chassis.pid_wait_until(20_in);
    chassis.pid_swing_set(ez::LEFT_SWING, 175_deg, 127, 0, ez::cw);
    chassis.pid_wait_until(-185_deg);
    Hood.set(false);
    chassis.pid_drive_set(-50_in, 127, false);
    pros::delay(1000);
    chassis.pid_swing_set(ez::RIGHT_SWING, 100_deg, 127, 5, ez::ccw);
    chassis.pid_wait_until(130_deg);
    chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 127, 5, ez::cw);
    chassis.pid_wait_until(170_deg);
    chassis.pid_drive_set(-30_in, 127, false);
    chassis.pid_wait_until(-20_in);
}

void rush_mid_left() {
    Score.move(-127);
    chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 45, ez::ccw);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    chassis.pid_swing_set(ez::LEFT_SWING, 310_deg, 127, 0, ez::cw);
    chassis.pid_wait_quick_chain();
    Loader.set(true);
    chassis.pid_drive_set(21_in, 127);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    pros::delay(200);
    chassis.pid_drive_set(-10_in, 127);
    chassis.pid_wait_until(-3_in);
    chassis.pid_swing_set(ez::LEFT_SWING, 225_deg, 127, 32, ez::ccw);
    chassis.pid_wait_until(260_deg);
    chassis.pid_drive_set(-15_in, 90, false);
    chassis.pid_wait_until(-2_in);
    MidGoal.set(true);
    pros::delay(1200);
    MidGoal.set(false);
    chassis.pid_drive_set(10_in, 127);
    chassis.pid_wait_until(5_in);
    chassis.pid_drive_set(-5_in, 60);
    pros::delay(1000);
}

void sevenball_left() {
    Score.move(-127);
    Wings.set(false);
    chassis.pid_swing_set(ez::RIGHT_SWING, 200_deg, 127, 50, ez::ccw);
    chassis.pid_wait_until(320);
    Loader.set(false);
    chassis.pid_wait_until(225);
    chassis.pid_drive_set(50_in, 127);
    chassis.pid_wait_until(28_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 70, ez::ccw);
    chassis.pid_wait_until(190_deg);
    chassis.pid_drive_set(50_in, 70);
    pros::delay(600);
    chassis.pid_turn_set(180_deg, 127);
    pros::delay(300);
    chassis.pid_drive_set(-30_in, 127);
    chassis.pid_wait_until(-15_in);
    Hood.set(false);
    pros::delay(1700);
    Hood.set(true);
    Wings.set(true);
    Loader.set(true);
}

void sevenball_right() {
    Score.move(-127);
    Wings.set(false);
    chassis.pid_swing_set(ez::LEFT_SWING, -200_deg, 127, 50, ez::cw);
    chassis.pid_wait_until(-320);
    Loader.set(false);
    chassis.pid_wait_until(-225);
    chassis.pid_drive_set(50_in, 127);
    chassis.pid_wait_until(30_in);
    chassis.pid_swing_set(ez::LEFT_SWING, -180_deg, 127, 70, ez::cw);
    chassis.pid_wait_until(-190_deg);
    chassis.pid_drive_set(50_in, 70);
    pros::delay(400);
    chassis.pid_turn_set(-180_deg, 127);
    pros::delay(400);
    chassis.pid_drive_set(-30_in, 127);
    chassis.pid_wait_until(-15_in);
    Hood.set(false);
    pros::delay(1700);
    Hood.set(true);
    Wings.set(true);
    Loader.set(true);
}

void split_left() {
    Score.move(-127);
    chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 45, ez::ccw);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    chassis.pid_swing_set(ez::LEFT_SWING, 310_deg, 127, 2, ez::cw);
    chassis.pid_wait_quick_chain();
    Loader.set(true);
    chassis.pid_drive_set(21_in, 127);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    pros::delay(200);
    chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 127, 15, ez::cw);
    chassis.pid_wait_until(330_deg);
    Loader.set(true);
    chassis.pid_wait_until(350_deg);
    chassis.pid_drive_set(7_in, 127);
    chassis.pid_wait_until(3_in);
    chassis.pid_drive_constants_set(10.0, 0.3, 20.0);
    chassis.pid_drive_set(-28_in, 127);
    chassis.pid_wait_until(-23_in);
    chassis.pid_drive_constants_set(10.0, 0.3, 30.0);
    chassis.pid_swing_set(ez::RIGHT_SWING, 178_deg, 127, -30, ez::cw);
    chassis.pid_wait_until(173_deg);
    Loader.set(false);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(30_in, 50);
    pros::delay(1300);
    chassis.pid_turn_set(175_deg, 127);
    pros::delay(300);
    chassis.pid_swing_set(ez::RIGHT_SWING, 217_deg, 127, 30, ez::cw);
    chassis.pid_wait_until(213_deg);
    chassis.pid_drive_set(-45_in, 127);
    Top.move(127);
    pros::delay(100);
    Top.move(0);
    chassis.pid_wait_until(-25_in);
    chassis.pid_speed_max_set(80);
    Wings.set(false);
    MidGoal.set(true);
    pros::delay(450); 
    Score.move(-127);
    pros::delay(350);
    MidGoal.set(false);
    Loader.set(true);
    chassis.pid_speed_max_set(127);
    chassis.pid_drive_set(20_in, 127);
    chassis.pid_wait_until(15_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 150_deg, 127, 5, ez::ccw);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-20_in, 127);
    chassis.pid_wait_until(-14_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 0, ez::cw);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 25000_ms, 3_in, 50000_ms, 50000_ms);
    MidGoal.set(false);
    Loader.set(true);
    Wings.set(true);
    chassis.pid_drive_set(25_in, 127);
    chassis.pid_wait_until(20_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 140_deg, 127, -20, ez::ccw);
    chassis.pid_wait_until(150_deg);
    chassis.pid_drive_set(-10_in, 127);
    chassis.pid_wait_until(-5_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, -25, ez::cw);
    chassis.pid_wait_until(160_deg);
    Hood.set(false);
    pros::delay(2500);
    Hood.set(true);
}

// Score.move(-127);
//     chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 45, ez::ccw);
//     chassis.pid_wait_quick_chain();
//     Loader.set(false);
//     chassis.pid_swing_set(ez::LEFT_SWING, 310_deg, 127, 2, ez::cw);
//     chassis.pid_wait_quick_chain();
//     Loader.set(true);
//     chassis.pid_drive_set(21_in, 127);
//     chassis.pid_wait_quick_chain();
//     Loader.set(false);
//     pros::delay(200);
//     chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 127, 15, ez::cw);
//     chassis.pid_wait_quick_chain();
//     chassis.pid_drive_set(-18_in, 127);
//     chassis.pid_wait_until(-8_in);
//     chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, -25, ez::cw);
//     chassis.pid_wait_until(155_deg);
//     chassis.pid_drive_set(-30_in, 127);
//     pros::delay(300);
//     Hood.set(false);
//     pros::delay(200);
//     chassis.pid_turn_set(175_deg, 127);
//     pros::delay(1300);
//     Hood.set(true);
//     chassis.pid_drive_set(40_in, 127);
//     chassis.pid_wait_until(15_in);
//     chassis.pid_speed_max_set(45);
//     pros::delay(1000);
//     chassis.pid_speed_max_set(127);
//     chassis.pid_swing_set(ez::RIGHT_SWING, 218_deg, 127, 25, ez::cw);
//     chassis.pid_wait_until(213_deg);
//     chassis.pid_drive_set(-45_in, 127);
//     Top.move(127);
//     pros::delay(100);
//     Top.move(0);
//     chassis.pid_wait_until(-25_in);
//     chassis.pid_speed_max_set(80);
//     MidGoal.set(true);
//     Score.move(-127);
//     pros::delay(1000);
//     chassis.pid_speed_max_set(127);
//     Score.move(0);
//     chassis.pid_drive_set(20_in, 127);
//     chassis.pid_wait_until(15_in);
//     chassis.pid_swing_set(ez::RIGHT_SWING, 150_deg, 127, 5, ez::ccw);
//     chassis.pid_wait_quick_chain();
//     chassis.pid_drive_set(-25_in, 127);
//     chassis.pid_wait_until(-10_in);
//     chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 0, ez::cw);
//     chassis.pid_wait_quick_chain();
//     chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 25000_ms, 3_in, 50000_ms, 50000_ms);
//     MidGoal.set(false);
//     Loader.set(true);
//     chassis.pid_drive_set(-3_in, 127);
//     chassis.pid_wait();

void split_right() {
    Score.move(-127);
    chassis.pid_swing_set(ez::LEFT_SWING, -285_deg, 127, 45, ez::cw);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    chassis.pid_swing_set(ez::RIGHT_SWING, -310_deg, 127, 0, ez::ccw);
    chassis.pid_wait_quick_chain();
    Loader.set(true);
    chassis.pid_drive_set(22_in, 127);
    chassis.pid_wait_quick_chain();
    Loader.set(false);
    pros::delay(200);
    chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, 127, 30, ez::ccw);
    chassis.pid_wait_until(-45_deg);
    chassis.pid_swing_set(ez::LEFT_SWING, -180_deg, 127, 0, ez::ccw);
    chassis.pid_wait_until(-160_deg);
    chassis.pid_drive_set(-30_in, 127);
    Hood.set(false);
    pros::delay(700);
    chassis.pid_turn_set(180_deg, 127);
    pros::delay(800);
    Hood.set(true);
    chassis.pid_drive_set(40_in, 127);
    chassis.pid_wait_until(15_in);
    chassis.pid_speed_max_set(45);
    pros::delay(800);
    chassis.pid_speed_max_set(127);
    chassis.pid_turn_set(-43_deg, 127);
    chassis.pid_wait_until(-38_deg);
    chassis.pid_drive_set(55_in, 127);
    Score.move(127);
    pros::delay(100);
    Score.move(0);
    Loader.set(true);
    chassis.pid_wait_until(25_in);
    chassis.pid_speed_max_set(80);
    Score.move(127);
    pros::delay(1000);
    chassis.pid_speed_max_set(127);
    Score.move(0);
    chassis.pid_drive_set(-20_in, 127);
    chassis.pid_wait_until(-15_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, -350_deg, 127, 5, ez::cw);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(25_in, 127);
    chassis.pid_wait_until(17_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 127, 0, ez::ccw);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 25000_ms, 3_in, 50000_ms, 50000_ms);
    chassis.pid_drive_set(8_in, 127);
    chassis.pid_wait();
}

void sawp(){
    Score.move(-127);
    chassis.pid_drive_set(2_in, 127);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-29.5_in, 127);
    chassis.pid_wait_until(-10_in);
    Loader.set(false);
    chassis.pid_wait_until(-23_in);
    chassis.pid_turn_set(270_deg, 127);
    chassis.pid_wait_until(290_deg);
    chassis.pid_drive_set(20_in, 70);
    pros::delay(1200);
    chassis.pid_drive_set(-30_in, 127);
    chassis.pid_wait_until(-18_in);
    chassis.pid_speed_max_set(60);
    Hood.set(false);
    pros::delay(1300);
    chassis.pid_speed_max_set(127);
    Loader.set(true);
    Hood.set(true);
    chassis.pid_swing_set(ez::LEFT_SWING, 50_deg, 127, 0, ez::cw);
    chassis.pid_wait_until(40_deg);
    chassis.pid_drive_set(11_in, 127);
    chassis.pid_wait_until(6_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 127, 60, ez::ccw);
    chassis.pid_wait_until(45_deg);
    Loader.set(false);
    chassis.pid_wait_until(20_deg);
    Loader.set(true);
    chassis.pid_drive_set(45_in, 127, false);
    chassis.pid_wait_until(27_in);
    Loader.set(false);
    chassis.pid_swing_set(ez::RIGHT_SWING, 330_deg, 127, 50, ez::ccw);
    chassis.pid_wait_until(345);
    chassis.pid_drive_set(30_in, 127, false);
    chassis.pid_wait_until(20_in);
    chassis.pid_swing_set(ez::RIGHT_SWING, 270_deg, 127, 0, ez::ccw);
    chassis.pid_wait_until(285_deg);
    chassis.pid_drive_set(-25_in, 127);
    pros::delay(300);
    Hood.set(false);
    pros::delay(500);
    chassis.pid_turn_set(268_deg, 127);
    pros::delay(1000);
    Hood.set(true);
    chassis.pid_drive_set(40_in, 127);
    chassis.pid_wait_until(15_in);
    chassis.pid_speed_max_set(60);
    pros::delay(1000);
    chassis.pid_speed_max_set(127);
    chassis.pid_swing_set(ez::RIGHT_SWING, 310_deg, 127, 30, ez::cw);
    chassis.pid_wait_until(305_deg);
    chassis.pid_drive_set(-45_in, 127);
    Top.move(127);
    pros::delay(100);
    Top.move(0);
    chassis.pid_wait_until(-25_in);
    chassis.pid_speed_max_set(80);
    MidGoal.set(true);
    Score.move(-127);
    pros::delay(1700);
    chassis.pid_speed_max_set(127);
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
