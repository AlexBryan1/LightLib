#pragma once
#include <climits>
#include <cmath>
#include <vector>

#include "LightLib/api.h"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

/**
 * \file holo_drive.hpp
 *
 * Holonomic and H-drive chassis classes (alternatives to the tank `Drive`).
 *
 * Both classes implement opcontrol mixing, IMU-stabilized straight motions,
 * and per-axis PID for autonomous moves.
 */

namespace light {

/**
 * Normalize an angle in degrees to the half-open interval (-180, 180] so
 * PID always takes the shortest path.
 *
 * \param a
 *        input angle in degrees
 */
double wrap180(double a);

/**
 * Internal PID used by HoloDrive / HDrive.
 *
 * Distinct from light::PID — no exit-condition machinery, supports both
 * error-derivative and measurement-derivative variants.
 */
struct HoloPID {
  double kP = 0;            ///< Proportional gain.
  double kI = 0;            ///< Integral gain.
  double kD = 0;            ///< Derivative gain.
  double integral = 0;      ///< Accumulated integral term.
  /**
   * Same gating semantics as `LightPID::integralCap`: > 0 enables the clamp.
   * Default chosen to match LightPID; set to ≤ 0 to disable.
   */
  double integralCap = 1000.0;
  double prev_error = 0;    ///< Previous-cycle error.
  double prev_measured = 0; ///< Previous-cycle measurement (for derivative-on-measurement).
  bool seeded_err = false;  ///< Whether `prev_error` is valid.
  bool seeded_meas = false; ///< Whether `prev_measured` is valid.

  /**
   * Compute output using derivative on error.
   *
   * \param error
   *        current error
   */
  double compute(double error) {
    integral += error;
    if (integralCap > 0.0) {
      if (integral > integralCap) integral = integralCap;
      if (integral < -integralCap) integral = -integralCap;
    }
    double deriv = seeded_err ? (error - prev_error) : 0.0;
    prev_error = error;
    seeded_err = true;
    return kP * error + kI * integral + kD * deriv;
  }

  /**
   * Compute output using derivative on measurement (suppresses derivative
   * kick on target changes).
   *
   * \param error
   *        current error
   * \param measured
   *        current sensor measurement
   */
  double compute(double error, double measured) {
    integral += error;
    if (integralCap > 0.0) {
      if (integral > integralCap) integral = integralCap;
      if (integral < -integralCap) integral = -integralCap;
    }
    double dmeas = seeded_meas ? (measured - prev_measured) : 0.0;
    prev_measured = measured;
    seeded_meas = true;
    return kP * error + kI * integral - kD * dmeas;
  }

  /** Zero out integral and previous-state history. */
  void reset() {
    integral = 0;
    prev_error = 0;
    prev_measured = 0;
    seeded_err = false;
    seeded_meas = false;
  }

  /** Set all three gains in one call. */
  void set(double p, double i = 0, double d = 0) {
    kP = p;
    kI = i;
    kD = d;
  }
};

/**
 * 4-motor holonomic drive — supports X-Drive and Mecanum configurations.
 *
 * Both wheel styles use the same control math; pick the type that matches
 * your hardware.
 *
 * \par Motor layout (viewed from above, robot nose pointing up)
 * \code
 *   FL  FR
 *   BL  BR
 * \endcode
 * Use a negative port number to reverse a motor.
 *
 * \par Wheel mixing ("X" pattern, standard for VEX)
 * \code
 *   FL = throttle − strafe + turn
 *   FR = throttle + strafe − turn
 *   BL = throttle + strafe + turn
 *   BR = throttle − strafe − turn
 * \endcode
 * Positive turn = clockwise (right). Positive strafe = rightward.
 *
 * \warning Before running any autonomous method you MUST call `set_*_pid()`
 *          for each axis. Call calibrate() once inside `initialize()` if no
 *          other code calibrates the IMU.
 */
class HoloDrive {
 public:
  /** Wheel layout option. */
  enum class Type { XDRIVE,   ///< Omni wheels mounted at 45°.
                    MECANUM };///< Standard mecanum X-pattern.

  /**
   * Construct a 4-motor holonomic drive.
   *
   * \param fl_port
   *        front-left motor port (negative = reversed)
   * \param fr_port
   *        front-right motor port (negative = reversed)
   * \param bl_port
   *        back-left motor port (negative = reversed)
   * \param br_port
   *        back-right motor port (negative = reversed)
   * \param imu_port
   *        inertial sensor port
   * \param wheel_diameter
   *        wheel diameter in inches
   * \param gear_ratio
   *        `output_rpm / motor_rpm` (1.0 = direct drive)
   * \param type
   *        wheel layout
   */
  HoloDrive(int fl_port, int fr_port, int bl_port, int br_port,
            int imu_port,
            double wheel_diameter,
            double gear_ratio = 1.0,
            Type type = Type::MECANUM);

  /**
   * \name Opcontrol
   * @{
   */
  /**
   * Drive from joystick inputs. Pass raw joystick values −127..127.
   *
   * \param throttle
   *        forward / back
   * \param strafe
   *        left / right
   * \param turn
   *        CW / CCW
   */
  void opcontrol(int throttle, int strafe, int turn);
  /** @} */

  /**
   * \name Autonomous moves
   * Straight drive / strafe hold heading via IMU throughout.
   * @{
   */
  /** Drive forward `inches` (negative = reverse). */
  void drive(double inches, int max_speed = 100, int timeout_ms = 3000);
  /** Strafe right `inches` (negative = left). */
  void strafe(double inches, int max_speed = 100, int timeout_ms = 3000);
  /** Turn to absolute heading 0–360°. */
  void turn_to(double heading_deg, int max_speed = 80, int timeout_ms = 2000);
  /** Turn `degrees` CW (positive) or CCW (negative). */
  void turn_relative(double degrees, int max_speed = 80, int timeout_ms = 2000);
  /** @} */

  /**
   * \name PID constants
   * @{
   */
  /** Set the linear-drive PID gains. */
  void set_drive_pid(double kP, double kI = 0, double kD = 0);
  /** Set the strafe PID gains. */
  void set_strafe_pid(double kP, double kI = 0, double kD = 0);
  /** Set the turn-in-place PID gains. */
  void set_turn_pid(double kP, double kI = 0, double kD = 0);
  /** Set the heading-correction PID used during straight drive / strafe. */
  void set_heading_pid(double kP, double kI = 0, double kD = 0);
  /** @} */

  /**
   * \name Misc
   * @{
   */
  /** Calibrate the IMU. \param wait if true, blocks until settled. */
  void calibrate(bool wait = true);
  /** Tare motor encoders. */
  void reset_sensors();
  /** Set brake mode for all four motors. */
  void set_brake_mode(pros::motor_brake_mode_e_t mode);
  /** \return Current heading 0–360°, CW-positive. */
  double get_heading() const;
  /** @} */

 private:
  pros::Motor fl_, fr_, bl_, br_;
  pros::Imu imu_;
  Type type_;
  double wheel_circumference_;
  double gear_ratio_;

  HoloPID drive_pid_, strafe_pid_, turn_pid_, heading_pid_;

  // Last command actually written to each motor — set_holonomic() skips the
  // smartport write when the new value matches. INT_MIN sentinel means
  // "never written" so the first call always pushes a value.
  int last_fl_ = INT_MIN, last_fr_ = INT_MIN;
  int last_bl_ = INT_MIN, last_br_ = INT_MIN;

  // Unified motor-mixing call — also handles scaling to ±127.
  void set_holonomic(double throttle, double strafe, double turn);
  double inches_to_deg(double inches) const;
  double drive_pos() const;   // avg forward  encoder (all 4 motors)
  double strafe_pos() const;  // avg rightward encoder (−FL+FR+BL−BR)/4
};

/**
 * Tank drive with a single center-mounted strafe wheel (H-drive layout).
 *
 * Left and right groups may contain any number of motors.
 *
 * \warning Before running any autonomous method you MUST call `set_*_pid()`
 *          for each axis. Call calibrate() once inside `initialize()` if no
 *          other code calibrates the IMU.
 */
class HDrive {
 public:
  /**
   * Construct an H-drive.
   *
   * \param left_ports
   *        left side motor ports (negative = reversed)
   * \param right_ports
   *        right side motor ports (negative = reversed)
   * \param center_port
   *        center strafe motor port
   * \param imu_port
   *        inertial sensor port
   * \param wheel_diameter
   *        wheel diameter in inches
   * \param gear_ratio
   *        `output_rpm / motor_rpm` (1.0 = direct drive)
   */
  HDrive(std::vector<int> left_ports,
         std::vector<int> right_ports,
         int center_port,
         int imu_port,
         double wheel_diameter,
         double gear_ratio = 1.0);

  /**
   * \name Opcontrol
   * @{
   */
  /**
   * Drive from joystick inputs.
   *
   * \param throttle
   *        forward / back
   * \param strafe
   *        center wheel left / right
   * \param turn
   *        CW / CCW
   */
  void opcontrol(int throttle, int strafe, int turn);
  /** @} */

  /**
   * \name Autonomous moves
   * @{
   */
  /** Drive forward `inches` (negative = reverse). */
  void drive(double inches, int max_speed = 100, int timeout_ms = 3000);
  /** Strafe right `inches` (negative = left). */
  void strafe(double inches, int max_speed = 100, int timeout_ms = 3000);
  /** Turn to absolute heading 0–360°. */
  void turn_to(double heading_deg, int max_speed = 80, int timeout_ms = 2000);
  /** Turn `degrees` CW (positive) or CCW (negative). */
  void turn_relative(double degrees, int max_speed = 80, int timeout_ms = 2000);
  /** @} */

  /**
   * \name PID constants
   * @{
   */
  void set_drive_pid(double kP, double kI = 0, double kD = 0);
  void set_strafe_pid(double kP, double kI = 0, double kD = 0);
  void set_turn_pid(double kP, double kI = 0, double kD = 0);
  void set_heading_pid(double kP, double kI = 0, double kD = 0);
  /** @} */

  /**
   * \name Misc
   * @{
   */
  void calibrate(bool wait = true);
  void reset_sensors();
  void set_brake_mode(pros::motor_brake_mode_e_t mode);
  double get_heading() const;
  /** @} */

 private:
  pros::MotorGroup left_, right_;
  pros::Motor center_;
  pros::Imu imu_;
  double wheel_circumference_;
  double gear_ratio_;

  HoloPID drive_pid_, strafe_pid_, turn_pid_, heading_pid_;

  void set_tank(double left_v, double right_v);
  double inches_to_deg(double inches) const;
  double drive_pos() const;  // average of all left + right motor encoders
};

}  // namespace light
