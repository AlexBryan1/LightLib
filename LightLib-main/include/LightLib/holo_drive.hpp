#pragma once
#include "LightLib/api.h"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include <vector>
#include <cmath>

namespace light {

// ── Internal PID ──────────────────────────────────────────────────────────────
struct HoloPID {
    double kP = 0, kI = 0, kD = 0;
    double integral = 0, prev_error = 0;
    double compute(double error) {
        integral  += error;
        double deriv = error - prev_error;
        prev_error = error;
        return kP * error + kI * integral + kD * deriv;
    }
    void reset() { integral = 0; prev_error = 0; }
    void set(double p, double i = 0, double d = 0) { kP = p; kI = i; kD = d; }
};

// ── HoloDrive ─────────────────────────────────────────────────────────────────
// 4-motor holonomic drive — supports X-Drive and Mecanum configurations.
// Both use the same control math; pick the type that matches your wheel style.
//
// Motor layout (viewed from above, robot nose pointing up):
//     FL  FR
//     BL  BR
// Use a negative port number to reverse a motor.
//
// Wheel mixing ("X" pattern — standard for VEX):
//     FL = throttle − strafe + turn
//     FR = throttle + strafe − turn
//     BL = throttle + strafe + turn
//     BR = throttle − strafe − turn
// positive turn = clockwise (right), positive strafe = rightward
//
// Before running any autonomous method you MUST call set_*_pid() for each axis.
// Call calibrate() once inside initialize() if no other code calibrates the IMU.
class HoloDrive {
public:
    enum class Type { XDRIVE, MECANUM };

    // wheel_diameter: inches.
    // gear_ratio: output_rpm / motor_rpm  (1.0 = direct drive).
    HoloDrive(int fl_port, int fr_port, int bl_port, int br_port,
              int    imu_port,
              double wheel_diameter,
              double gear_ratio = 1.0,
              Type   type       = Type::MECANUM);

    // ── Opcontrol ─────────────────────────────────────────────────────────────
    // Pass raw joystick values (−127..127).
    // throttle = forward/back   strafe = left/right   turn = CW/CCW
    void opcontrol(int throttle, int strafe, int turn);

    // ── Autonomous ────────────────────────────────────────────────────────────
    // Straight drive / strafe hold heading via IMU throughout.
    void drive        (double inches,      int max_speed = 100, int timeout_ms = 3000);
    void strafe       (double inches,      int max_speed = 100, int timeout_ms = 3000);
    // turn_to: absolute heading 0–360°.   turn_relative: degrees CW (positive) or CCW (negative).
    void turn_to      (double heading_deg, int max_speed = 80,  int timeout_ms = 2000);
    void turn_relative(double degrees,     int max_speed = 80,  int timeout_ms = 2000);

    // ── PID constants ─────────────────────────────────────────────────────────
    void set_drive_pid  (double kP, double kI = 0, double kD = 0);
    void set_strafe_pid (double kP, double kI = 0, double kD = 0);
    void set_turn_pid   (double kP, double kI = 0, double kD = 0);
    // heading_pid: used during drive/strafe to maintain straight-line heading.
    void set_heading_pid(double kP, double kI = 0, double kD = 0);

    // ── Misc ──────────────────────────────────────────────────────────────────
    void   calibrate    (bool wait = true);   // reset + wait for IMU to settle
    void   reset_sensors();                   // tare motor encoders
    void   set_brake_mode(pros::motor_brake_mode_e_t mode);
    double get_heading() const;               // 0–360°, CW-positive

private:
    pros::Motor fl_, fr_, bl_, br_;
    pros::Imu   imu_;
    Type        type_;
    double      wheel_circumference_;
    double      gear_ratio_;

    HoloPID drive_pid_, strafe_pid_, turn_pid_, heading_pid_;

    // Unified motor-mixing call — also handles scaling to ±127.
    void   set_holonomic(double throttle, double strafe, double turn);
    double inches_to_deg(double inches) const;
    double drive_pos()  const;   // avg forward  encoder (all 4 motors)
    double strafe_pos() const;   // avg rightward encoder (−FL+FR+BL−BR)/4
    static double wrap180(double a);
};

// ── HDrive ────────────────────────────────────────────────────────────────────
// Tank drive with a single center-mounted strafe wheel (H-drive configuration).
// Left and right groups may contain any number of motors.
//
// Before running any autonomous method you MUST call set_*_pid() for each axis.
// Call calibrate() once inside initialize() if no other code calibrates the IMU.
class HDrive {
public:
    // left_ports / right_ports: use negative values for reversed motors.
    HDrive(std::vector<int> left_ports,
           std::vector<int> right_ports,
           int    center_port,
           int    imu_port,
           double wheel_diameter,
           double gear_ratio = 1.0);

    // ── Opcontrol ─────────────────────────────────────────────────────────────
    // throttle = forward/back   strafe = center wheel (left/right)   turn = CW/CCW
    void opcontrol(int throttle, int strafe, int turn);

    // ── Autonomous ────────────────────────────────────────────────────────────
    void drive        (double inches,      int max_speed = 100, int timeout_ms = 3000);
    void strafe       (double inches,      int max_speed = 100, int timeout_ms = 3000);
    void turn_to      (double heading_deg, int max_speed = 80,  int timeout_ms = 2000);
    void turn_relative(double degrees,     int max_speed = 80,  int timeout_ms = 2000);

    // ── PID constants ─────────────────────────────────────────────────────────
    void set_drive_pid  (double kP, double kI = 0, double kD = 0);
    void set_strafe_pid (double kP, double kI = 0, double kD = 0);
    void set_turn_pid   (double kP, double kI = 0, double kD = 0);
    void set_heading_pid(double kP, double kI = 0, double kD = 0);

    // ── Misc ──────────────────────────────────────────────────────────────────
    void   calibrate    (bool wait = true);
    void   reset_sensors();
    void   set_brake_mode(pros::motor_brake_mode_e_t mode);
    double get_heading() const;

private:
    pros::MotorGroup left_, right_;
    pros::Motor      center_;
    pros::Imu        imu_;
    double           wheel_circumference_;
    double           gear_ratio_;

    HoloPID drive_pid_, strafe_pid_, turn_pid_, heading_pid_;

    void   set_tank(double left_v, double right_v);
    double inches_to_deg(double inches) const;
    double drive_pos() const;   // average of all left + right motor encoders
    static double wrap180(double a);
};

} // namespace light
