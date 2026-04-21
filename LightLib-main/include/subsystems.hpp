#pragma once

#include "EZ-Template/api.hpp"
#include "LightLib/api.h"
#include "pros/motors.hpp"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                         SUBSYSTEMS                                      │
// │                                                                         │
// │  Declare every motor, sensor, and piston your robot uses here.          │
// │  Everything declared inline is available in every .cpp file that        │
// │  includes this header — no extern declarations needed elsewhere.        │
// │                                                                         │
// │  The drive chassis is declared in main.cpp (built from the #defines     │
// │  at the top of that file) and exposed here as extern Drive chassis.     │
// └─────────────────────────────────────────────────────────────────────────┘

// The drive chassis — configured in main.cpp, used everywhere else.
extern Drive chassis;


// ── Motors ────────────────────────────────────────────────────────────────────
// Syntax:   inline pros::Motor  name(PORT);
//           inline pros::Motor  name(-PORT);   // negative port = reversed
//
// For multiple motors that share a purpose, use a MotorGroup:
//           inline pros::MotorGroup name({PORT_A, -PORT_B});
//
// Useful methods:
//   name.move(speed)          — run at -127 … 127
//   name.move_voltage(mV)     — run at -12000 … 12000 mV
//   name.get_temperature()    — degrees C (>55 = hot, >70 = thermal limit)
//   name.get_actual_velocity()

inline pros::Motor       Top(7);
inline pros::Motor       Bottom(17);
inline pros::MotorGroup  Score({17, 7});   // Top + Bottom together

inline pros::Motor       turret(0);        // set port to 0 if not installed
inline pros::Optical     optical(15);


// ── Pistons (ADI / 3-wire) ────────────────────────────────────────────────────
// Syntax:  inline ez::Piston name('PORT');
// PORT is the letter A–H matching the 3-wire port on the brain or expander.
//
// ez::Piston tracks toggle state and supports:
//   name.set(true / false)      — extend / retract
//   name.button_toggle(btn)     — toggle state when btn is newly pressed

inline ez::Piston Wings     ('A');   // intake wings / expander
inline ez::Piston Loader    ('C');   // ball loader / indexer
inline ez::Piston MidGoal   ('D');   // mid-goal clamp or mechanism
inline ez::Piston Hood      ('E');   // scoring hood


// ── Distance sensors ──────────────────────────────────────────────────────────
// Defined conditionally in main.cpp based on DIST_*_PORT.
// If a port macro is 0 the pointer is nullptr — always null-check before use:
//   if (leftDist && leftDist->get() < 100) { ... }

extern pros::Distance* left_front_sensor;  // left-front distance sensor
extern pros::Distance* left_back_sensor;   // left-back  distance sensor
extern pros::Distance* leftDist;           // alias for left_back_sensor (WallRide)
extern pros::Distance* frontDist;          // forward-facing distance sensor


// ── Alliance color ────────────────────────────────────────────────────────────
// Set allianceColor at the start of autonomous (in user_autonomous() or your
// auton routine) to control color-sort behavior.
//   allianceColor = RED;    // sort out blue rings
//   allianceColor = BLUE;   // sort out red rings
//   allianceColor = NEUTRAL; // sort nothing
enum Colors { BLUE = 0, NEUTRAL = 1, RED = 2 };
extern Colors allianceColor;


// ── Holonomic / non-tank drives (optional) ────────────────────────────────────
// Uncomment ONE block below if you are using a holonomic drive instead of (or
// alongside) the EZ-Template tank chassis.
//
// ── Mecanum or X-Drive ──────────────────────────────────────────────────────
// light::HoloDrive::Type::MECANUM  — standard mecanum "X" wheel layout
// light::HoloDrive::Type::XDRIVE   — omni wheels mounted at 45°
//
//   inline light::HoloDrive holoDrive(
//       FL_PORT, FR_PORT, BL_PORT, BR_PORT,   // negative = reversed
//       IMU_PORT,
//       WHEEL_DIAMETER,                        // inches
//       GEAR_RATIO,                            // output / motor RPM (1.0 = direct)
//       light::HoloDrive::Type::MECANUM);
//
// Setup in initialize():
//   holoDrive.calibrate();
//   holoDrive.set_drive_pid  (kP, kI, kD);
//   holoDrive.set_strafe_pid (kP, kI, kD);
//   holoDrive.set_turn_pid   (kP, kI, kD);
//   holoDrive.set_heading_pid(kP, kI, kD);
//
// Opcontrol (inside the while loop):
//   holoDrive.opcontrol(master.get_analog(ANALOG_LEFT_Y),
//                       master.get_analog(ANALOG_LEFT_X),
//                       master.get_analog(ANALOG_RIGHT_X));
//
// Autonomous:
//   holoDrive.drive(24);           // 24 inches forward
//   holoDrive.strafe(12);          // 12 inches right (negative = left)
//   holoDrive.turn_to(90);         // face 90° absolute (0–360, CW-positive)
//   holoDrive.turn_relative(45);   // rotate 45° clockwise from current heading
//
// ── H-Drive ─────────────────────────────────────────────────────────────────
// Tank sides for forward / back / turn + a single center wheel for strafing.
//
//   inline light::HDrive hDrive(
//       {LEFT_PORTS},              // e.g. {-1, -2}
//       {RIGHT_PORTS},             // e.g. {3, 4}
//       CENTER_PORT,
//       IMU_PORT,
//       WHEEL_DIAMETER,
//       GEAR_RATIO);
//
// Setup and usage are identical to HoloDrive above.
