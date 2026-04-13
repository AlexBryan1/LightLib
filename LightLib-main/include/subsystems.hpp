#pragma once

#include "EZ-Template/api.hpp"
#include "LightLib/api.h"
#include "pros/motors.hpp"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                         SUBSYSTEMS                                      │
// │  Declare every motor, sensor, and piston your robot uses here.          │
// │  Anything declared inline is available in every .cpp file automatically │
// │  once main.h is included — no extern declarations needed elsewhere.     │
// └─────────────────────────────────────────────────────────────────────────┘

// The drive chassis is declared in main.cpp and exposed here for auton code.
extern Drive chassis;

// ── Motors ────────────────────────────────────────────────────────────────────
// inline pros::Motor name(PORT);
// Prefix the port with a minus sign to reverse the motor, e.g. pros::Motor arm(-3);
// For multiple motors that share a purpose, group them:
//   inline pros::MotorGroup intake({1, -2});

inline pros::Motor Top(7);
inline pros::Motor Bottom(17);
inline pros::MotorGroup Score({17, 7});

// inline pros::Motor turret(0);         // set port to 0 if not used
inline pros::Motor turret(0);
inline pros::Optical optical(15);

// ── Pistons (ADI / 3-wire) ────────────────────────────────────────────────────
// inline ez::Piston name('PORT');
// Port is a letter A–H matching the 3-wire port on the brain or expander.
// ez::Piston tracks toggle state and supports .set(bool) and .button_toggle().

inline ez::Piston Wings('A');
inline ez::Piston IntakeLift('B');
inline ez::Piston Loader('C');
inline ez::Piston MidGoal('D');
inline ez::Piston Hood('E');

// ── Distance sensors ──────────────────────────────────────────────────────────
// Defined conditionally in main.cpp based on the DIST_*_PORT macros.
// If a port macro is 0 the pointer is nullptr — always null-check before use.
extern pros::Distance* left_front_sensor;
extern pros::Distance* left_back_sensor;
extern pros::Distance* leftDist;    // alias for left_back_sensor (used by WallRide)
extern pros::Distance* frontDist;

// ── Alliance color ────────────────────────────────────────────────────────────
// Set allianceColor at the start of autonomous to control color-sort behavior.
enum Colors { BLUE = 0, NEUTRAL = 1, RED = 2 };
extern Colors allianceColor;

// ── Holonomic drives (optional) ───────────────────────────────────────────────
// Uncomment and configure ONE of the blocks below if you are using a holonomic
// drive instead of (or alongside) the EZ-Template tank chassis.
//
// ── Mecanum or X-Drive ──────────────────────────────────────────────────────
// light::HoloDrive::Type::MECANUM  →  standard mecanum "X" wheel pattern
// light::HoloDrive::Type::XDRIVE   →  omni wheels mounted at 45°
//
//   inline light::HoloDrive holoDrive(
//       FL_PORT, FR_PORT, BL_PORT, BR_PORT,   // use negative to reverse
//       IMU_PORT,
//       WHEEL_DIAMETER,                        // inches
//       GEAR_RATIO,                            // output_rpm / motor_rpm (1.0 = direct)
//       light::HoloDrive::Type::MECANUM);
//
// Then in initialize():
//   holoDrive.calibrate();
//   holoDrive.set_drive_pid  (kP, kI, kD);
//   holoDrive.set_strafe_pid (kP, kI, kD);
//   holoDrive.set_turn_pid   (kP, kI, kD);
//   holoDrive.set_heading_pid(kP, kI, kD);   // straight-line correction
//
// Opcontrol (inside the while loop):
//   holoDrive.opcontrol(master.get_analog(ANALOG_LEFT_Y),
//                       master.get_analog(ANALOG_LEFT_X),
//                       master.get_analog(ANALOG_RIGHT_X));
//
// Autonomous:
//   holoDrive.drive(24);          // 24 inches forward
//   holoDrive.strafe(12);         // 12 inches right (negative = left)
//   holoDrive.turn_to(90);        // face 90° (absolute, 0–360, CW-positive)
//   holoDrive.turn_relative(45);  // rotate 45° clockwise
//
// ── H-Drive ─────────────────────────────────────────────────────────────────
// Tank sides for fwd/back/turn + a single center wheel for strafing.
//
//   inline light::HDrive hDrive(
//       {LEFT_PORTS},                          // e.g. {-1, -2}
//       {RIGHT_PORTS},                         // e.g. {3, 4}
//       CENTER_PORT,
//       IMU_PORT,
//       WHEEL_DIAMETER,
//       GEAR_RATIO);
//
// Setup and usage are identical to HoloDrive above.
