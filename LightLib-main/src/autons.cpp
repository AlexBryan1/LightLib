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
// │  AUTONOMOUS ROUTINES                                                     │
// │  Write one function per auton below.  Register each one in             │
// │  auton_config.cpp with light::auton_selector.add() so it appears       │
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

// Jerryio-authored path, exported from path.jerryio.com. The densified
// format is auto-detected and translated so point 0 sits at the robot's
// current pose — so this starts wherever the robot happens to be.
void run_jerryio_path_1() {
    static const char* kPath = R"JERRYIO(#PATH-POINTS-START Path
-133.062,8.316,120,90
-131.068,8.468,120
-129.074,8.625,120
-127.08,8.787,120
-125.087,8.954,120
-123.095,9.128,120
-121.103,9.309,120
-119.112,9.497,120
-117.121,9.692,120
-115.132,9.895,120
-113.143,10.107,120
-111.155,10.329,120
-109.169,10.561,120
-107.184,10.804,120
-105.2,11.06,120
-103.218,11.329,120
-101.238,11.611,120
-99.261,11.91,120
-97.286,12.226,120
-95.314,12.559,120
-93.346,12.914,120
-91.382,13.292,120
-89.422,13.693,120
-87.469,14.123,120
-85.523,14.583,120
-83.585,15.077,120
-81.657,15.608,120
-79.741,16.181,120
-77.84,16.802,120
-75.957,17.477,120
-74.097,18.211,120
-72.264,19.011,120
-70.465,19.885,120
-68.71,20.844,120
-67.008,21.893,120
-65.372,23.043,120
-63.817,24.3,120
-62.358,25.667,120
-61.01,27.144,120
-59.789,28.727,120
-58.703,30.406,120
-57.756,32.167,120
-56.948,33.996,120
-56.272,35.878,120
-55.718,37.799,120
-55.271,39.748,120
-54.92,41.717,120
-54.654,43.699,120
-54.459,45.689,120
-54.326,47.685,120
-54.244,49.683,120
-54.203,51.683,120,0
-54.162,53.682,120
-54.179,55.682,120
-54.277,57.679,120
-54.487,59.668,120
-54.833,61.637,120
-55.338,63.572,120
-56.011,65.454,120
-56.845,67.272,120
-57.821,69.017,120
-58.913,70.692,120
-60.1,72.301,120
-61.36,73.855,120
-62.676,75.36,120
-64.038,76.825,120
-65.434,78.257,120
-66.858,79.661,120
-68.304,81.043,120
-69.768,82.405,120
-71.247,83.752,120
-72.738,85.085,120
-74.238,86.407,120
-75.747,87.72,120
-77.262,89.026,120,315
-78.824,90.274,120
-80.397,91.509,120
-81.962,92.754,120
-83.521,94.007,120
-85.077,95.263,120
-86.633,96.52,120
-88.191,97.775,120
-89.755,99.02,120
-91.33,100.253,120
-92.92,101.467,120
-94.529,102.655,120
-96.16,103.811,120
-97.818,104.93,120
-99.504,106.006,120
-101.221,107.032,120
-102.969,108.004,120
-104.747,108.919,120
-106.554,109.776,120
-108.388,110.572,120
-110.247,111.31,120
-112.128,111.99,120
-114.028,112.615,120
-115.944,113.186,120
-117.875,113.707,120
-119.818,114.183,120
-121.771,114.614,120
-123.732,115.006,120
-125.7,115.361,120
-127.674,115.682,120
-129.653,115.971,120
-131.636,116.23,120
-133.623,116.463,120
-135.612,116.672,120
-137.603,116.856,120
-139.596,117.021,120
-141.591,117.164,120
-143.587,117.292,120
-145.584,117.4,120
-147.582,117.494,120
-149.58,117.573,120
-151.579,117.638,120
-153.579,117.691,120
-155.578,117.732,120
-157.578,117.762,120
-159.578,117.78,120
-161.578,117.789,120
-163.578,117.789,120
-165.578,117.78,120
-167.578,117.763,120
-169.578,117.738,120
-172.179,117.695,120,270
-172.179,117.695,0,270
)JERRYIO";
    light::runJerryioPath(kPath);
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
