#pragma once

#include "LightLib/lib.hpp"
#include "LightLib/drive/odometry.hpp"
#include "pros/motor_group.hpp"

/**
 * \file extras.hpp
 *
 * Miscellaneous helpers wired into the LightLib base init: brain-screen
 * status print, motor-temperature controller buzz, a moveToPoint helper, and
 * a small standalone PID struct used by characterization paths.
 */

/**
 * Render one tracking-wheel's live position to a brain-screen line.
 *
 * \param tracker
 *        tracking_wheel to print
 * \param name
 *        label printed before the value
 * \param line
 *        brain-screen line index (0..7)
 */
void screen_print_tracker(light::tracking_wheel* tracker, std::string name, int line);

/** Background task that paints status text to the brain screen. */
void screen_task();

/** Boots all the LightLib helper tasks (screen, temp watch, etc.). */
void lib_extras();

/**
 * Buzz the controller when a key drive motor exceeds the configured heat
 * threshold (`HEAT_BUZZ_TEMP` in main.cpp).
 *
 * \param controller
 *        controller to buzz
 * \param Top
 *        first motor to monitor
 * \param Bottom
 *        second motor to monitor
 */
void checkMotorTemp(pros::Controller& controller, pros::Motor& Top, pros::Motor& Bottom);

/** Set initial piston / mechanism states. User-defined in autons.cpp. */
void default_positions();

/** Tracks a basket / scoring target (project-specific). */
void track_basket();

namespace light {
/**
 * Register the user's drivetrain objects with LightLib. Must be called once
 * in `initialize()` before screen_task / lib_extras / moveToPoint are used.
 *
 * Takes pointers rather than externing globals so LightLib can live in the
 * PROS cold package without linker errors.
 *
 * \param chassis
 *        the user's Drive instance
 * \param leftMotors
 *        left-side motor group
 * \param rightMotors
 *        right-side motor group
 */
void extras_init(light::Drive* chassis,
                 pros::MotorGroup* leftMotors,
                 pros::MotorGroup* rightMotors);

/**
 * Look up the registered drivetrain motor groups.
 *
 * Used by RAMSETE / characterization so they command the same motor groups
 * the user registered via extras_init().
 *
 * \param leftOut
 *        out: pointer to the registered left motor group, or nullptr
 * \param rightOut
 *        out: pointer to the registered right motor group, or nullptr
 */
void getDriveMotorGroups(pros::MotorGroup** leftOut,
                         pros::MotorGroup** rightOut);

/** \return Pointer to the registered chassis, or nullptr if not registered. */
light::Drive* getChassis();
}  // namespace light

/**
 * Lightweight PID controller used by characterization helpers.
 *
 * Distinct from light::PID — no exit conditions, no telemetry, just kP/kI/kD
 * with an integral cap.
 */
struct LightPID {
  float kP, kI, kD;             ///< Gains.
  float prevError = 0;          ///< Previous-cycle error.
  float integral = 0;           ///< Accumulated integral term.
  float integralCap = 1000.0f;  ///< Anti-windup clamp (set 0 to disable).

  /** Construct with gains. */
  LightPID(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}

  /**
   * Advance one step.
   *
   * \param error
   *        current error
   * \return  control output
   */
  float update(float error) {
    integral += error;
    if (integralCap > 0.0f) {
      if (integral > integralCap) integral = integralCap;
      if (integral < -integralCap) integral = -integralCap;
    }
    float derivative = error - prevError;
    prevError = error;
    return kP * error + kI * integral + kD * derivative;
  }

  /** Zero out the integral and previous-error history. */
  void reset() {
    prevError = 0;
    integral = 0;
  }
};

/**
 * Move the chassis to a field-relative `(x, y)` point.
 *
 * \param x
 *        target X in inches
 * \param y
 *        target Y in inches
 * \param timeout
 *        give up after this many milliseconds
 * \param maxSpeed
 *        speed cap, 0..127
 * \param reversed
 *        drive to the point backwards
 */
void moveToPoint(float x, float y, int timeout,
                 float maxSpeed = 127.0f, bool reversed = false);
