// ┌─────────────────────────────────────────────────────────────────────────────┐
// │  ez_extra.cpp — Utilities that extend EZ-Template                          │
// │                                                                            │
// │  This file contains helper functions that sit on top of EZ-Template and    ���
// │  PROS.  Nothing here is called automatically — you wire these into         │
// │  initialize(), opcontrol(), or autonomous() yourself.                      │
// │                                                                            │
// │  CONTENTS:                                                                 │
// │                                                                            │
// │  1. screen_print_tracker()  — prints a tracking wheel's value on the brain │
// │  2. ez_screen_task()        — background task that displays odom data on   │
// │                               the brain screen when not in competition     │
// │  3. ez_template_extras()    — enables the EZ-Template PID tuner via the    ��
// │                               controller's X button (practice mode only)   │
// │  4. checkMotorTemp()        — rumbles the controller if motors are hot     │
// │  5. turret_reset()          — re-homes a turret motor to its zero position │
// │  6. light::moveToPoint()    — drives to an (x, y) field coordinate using  │
// │                               odometry and a dual-PID (linear + angular)   │
// │                               control loop                                 │
// │                                                                            │
// │  The most important function here is moveToPoint() — it's a standalone    │
// │  odom drive-to-point that doesn't depend on EZ-Template's built-in odom.  │
// │  Use it when you want direct control over the PID tuning.                  │
// └─────────────────────────────────────────────────────────────────────────────┘

#include "LightLib/main.h"
#include "pros/motors.h"
#include "LightLib/odom.hpp"
#include "subsystems.hpp"
#include "LightLib/odom.hpp"
#include "pros/rtos.hpp"
#include "LightLib/ez_extra.hpp"
#include <cmath>
#include <algorithm>
#include <atomic>

// If any motor exceeds this temperature (Celsius), the controller will rumble.
#define MOTOR_TEMP_THRESHOLD 55.0

// Runtime pointers to the user's drivetrain.  Set by light::ez_extra_init()
// from initialize() — LightLib lives in the cold package and cannot reference
// hot-defined externs directly, so callers register their objects instead.
static ez::Drive*        g_chassis      = nullptr;
static pros::MotorGroup* g_leftMotors   = nullptr;
static pros::MotorGroup* g_rightMotors  = nullptr;

void light::ez_extra_init(ez::Drive* chassis,
                          pros::MotorGroup* leftMotors,
                          pros::MotorGroup* rightMotors) {
    g_chassis     = chassis;
    g_leftMotors  = leftMotors;
    g_rightMotors = rightMotors;
}

// ─── Brain screen helpers ────────────────────────────────────────────────────
// These display tracking wheel data and odom readings on the V5 Brain screen.
// Useful for debugging sensor calibration and odom accuracy in the pits.

// Print a single tracking wheel's current value and its distance-to-center offset
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}
// Background task that shows live odom data on the brain screen.
// Run this as a PROS task from initialize():
//   pros::Task screen_task(ez_screen_task);
//
// When NOT connected to a competition switch (i.e., practice/pit mode):
//   - Shows X, Y, Angle and all tracking wheel readings on a blank page
// When connected to competition:
//   - Removes all blank pages so the auton selector / match info is visible
void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (g_chassis && g_chassis->odom_enabled() && !g_chassis->pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          // Show current robot position from odometry
          ez::screen_print("x: " + util::to_string_with_precision(g_chassis->odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(g_chassis->odom_y_get()) +
                               "\na: " + util::to_string_with_precision(g_chassis->odom_theta_get()),
                           1);  // line 1 — don't overwrite the page title on line 0

          // Show each tracking wheel's raw value and its distance-to-center
          screen_print_tracker(g_chassis->odom_tracker_left, "l", 4);
          screen_print_tracker(g_chassis->odom_tracker_right, "r", 5);
          screen_print_tracker(g_chassis->odom_tracker_back, "b", 6);
          screen_print_tracker(g_chassis->odom_tracker_front, "f", 7);
        }
      }
    } else {
      // In competition mode, clear debug pages so the driver sees match info
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
// ─── EZ-Template PID tuner toggle ────────────────────────────────────────────
// Call this every loop iteration inside opcontrol().
// In practice mode (no competition switch):
//   - Press X on the controller to toggle the PID tuner on/off
//   - While the tuner is on, use A/Y to adjust values, arrow keys to navigate
//   - Press B + DOWN to re-apply the current brake mode (useful after tuning)
//   - Once you find good values, copy them into default_constants() in autons.cpp
// In competition mode:
//   - The PID tuner is automatically disabled so it can't interfere with a match
void ez_template_extras() {
  if (!g_chassis) return;
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      g_chassis->pid_tuner_toggle();

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = g_chassis->drive_brake_get();
      g_chassis->drive_brake_set(preference);
    }

    g_chassis->pid_tuner_iterate();
  } else {
    if (g_chassis->pid_tuner_enabled())
      g_chassis->pid_tuner_disable();
  }
}



// ─── Motor temperature warning ───────────────────────────────────────────────
// Call periodically in opcontrol() to warn the driver when intake/scoring motors
// are overheating.  The controller vibrates with ". . ." and pauses 2 seconds
// so the driver knows to ease off before a motor cuts out.
void checkMotorTemp(pros::Controller& controller, pros::Motor& Top, pros::Motor& Bottom) {
    if (Top.get_temperature() >= MOTOR_TEMP_THRESHOLD ||
        Bottom.get_temperature() >= MOTOR_TEMP_THRESHOLD) {
        controller.rumble(". . .");
        pros::delay(2000);
    }
}

// ─── Turret re-homing ────────────────────────────────────────────────────────
// Drives the turret motor back to its zero position, nudges it slightly past
// zero to remove backlash, then resets the encoder.  Call this after a shot
// so the turret is ready for the next track_basket() aim cycle.
void turret_reset(){
    turret.move_absolute(0, 127);  // drive to encoder position 0
    turret.move(-20);              // nudge past zero to seat against hard stop
    pros::delay(300);
    turret.move(0);                // stop
    turret.tare_position();        // reset encoder to 0 at the hard stop
}


// ─── moveToPoint — Odometry drive-to-coordinate ──────────────────────────────
// Drives the robot to a field-relative (x, y) position using LightLib's odom
// and a dual-PID control loop (one PID for distance, one for heading).
//
// This is independent of EZ-Template's built-in odom motions, giving you full
// control over tuning.  Use it when EZ-Template's pid_odom_set isn't behaving
// how you want.
//
// HOW IT WORKS (each 10ms loop iteration):
//   1. Get the robot's current position from odom (x, y, theta)
//   2. Compute straight-line distance and angle to the target
//   3. The LINEAR PID controls forward/backward speed based on distance
//   4. The ANGULAR PID steers the robot to face the target
//   5. A cosine scale reduces forward speed when the robot is pointed far off
//      target — this prevents it from driving sideways at full power
//   6. Linear + angular outputs are mixed into left/right motor speeds
//   7. If either side exceeds maxSpeed, both are scaled proportionally
//   8. Exit when within 0.2 inches of target, or timeout expires
//
// Parameters:
//   targetX, targetY — field coordinates in inches
//   timeout          — max duration in ms before forced stop
//   maxSpeed         — motor speed cap, 0–127 (default 127)
//   reversed         — if true, drive to the point backwards

// Motor groups are passed in via light::ez_extra_init() — see top of file.

void light::moveToPoint(float targetX, float targetY, int timeout, float maxSpeed, bool reversed) {

    if (!g_leftMotors || !g_rightMotors) return;  // not registered — nothing to drive

    // Two separate PID controllers — one for distance, one for heading.
    // Tune these for your robot.  Good starting points:
    //   linearPID:  ~125% of your EZ-Template drive PID (since this runs open-loop on motors)
    //   angularPID: ~100% of your EZ-Template swing PID
    LightPID linearPID (9.0f, 0.0f, 125.0f);
    LightPID angularPID( 6.0f, 0.0f, 50.0f);

    const float LINEAR_EXIT = 0.2f;  // inches — "close enough" to stop

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < (uint32_t)timeout) {
        Pose pose = light::getPose();  // current position (inches, degrees)

        // Vector from robot to target
        float dx = targetX - pose.x;
        float dy = targetY - pose.y;
        float distance = sqrtf(dx * dx + dy * dy);

        if (distance < LINEAR_EXIT) break;  // close enough — done

        // Desired heading to face the target (degrees, 0 = +Y axis)
        float angleToTarget = atan2f(dx, dy) * 180.0f / M_PI;
        if (reversed) angleToTarget += 180.0f;

        // Wrap heading error to [-180, 180] so the robot takes the shortest turn
        float angularError = angleToTarget - pose.theta;
        while (angularError >  180.0f) angularError -= 360.0f;
        while (angularError < -180.0f) angularError += 360.0f;

        // Cosine scaling — if we're pointed 90° away from the target, don't
        // drive forward at all; only turn.  cos(0°) = 1 (full speed ahead),
        // cos(90°) = 0 (turn in place).
        float linearScale = cosf(angularError * M_PI / 180.0f);
        float linearError = distance * linearScale;

        // PID outputs
        float linearPower  = linearPID.update(linearError);
        float angularPower = angularPID.update(angularError);

        if (reversed) linearPower = -linearPower;

        linearPower = std::clamp(linearPower, -maxSpeed, maxSpeed);

        // Mix linear + angular into left/right differential drive
        float leftPower  = linearPower + angularPower;
        float rightPower = linearPower - angularPower;

        // If either side exceeds maxSpeed, scale both down proportionally
        // so we don't lose the steering ratio
        float maxOut = std::max(std::abs(leftPower), std::abs(rightPower));
        if (maxOut > maxSpeed) {
            leftPower  = leftPower  / maxOut * maxSpeed;
            rightPower = rightPower / maxOut * maxSpeed;
        }

        g_leftMotors->move(leftPower);
        g_rightMotors->move(rightPower);

        pros::delay(10);  // 10ms control loop — yields to PROS scheduler
    }

    // Always stop the motors when we're done
    g_leftMotors->move(0);
    g_rightMotors->move(0);
}