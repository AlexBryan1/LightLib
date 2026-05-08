#pragma once

#include "LightLib/lib.hpp"
#include "LightLib/util/rotational_snap.hpp"
#include "pros/motors.hpp"

/**
 * \file subsystems.hpp
 *
 * Robot subsystem declarations: motors, sensors, and pistons.
 *
 * Everything declared `inline` here is available in every .cpp file that
 * includes this header — no extern declarations needed elsewhere.
 *
 * The drive chassis is declared in main.cpp (built from the `#define`s at the
 * top of that file) and re-exposed here as `extern Drive chassis`.
 */

/** The drive chassis, configured in main.cpp and used everywhere else. */
extern Drive chassis;

/**
 * \name Motors
 * Syntax:
 * \code
 *   inline pros::Motor name(PORT);          // forward
 *   inline pros::Motor name(-PORT);         // negative port = reversed
 *   inline pros::MotorGroup name({A, -B});  // group multiple motors
 * \endcode
 *
 * Useful methods:
 *   - `name.move(speed)`              run at -127 … 127
 *   - `name.move_voltage(mV)`         run at -12000 … 12000 mV
 *   - `name.get_temperature()`        degrees C (>55 = hot, >70 = thermal limit)
 *   - `name.get_actual_velocity()`    measured RPM
 * @{
 */
inline pros::Motor Top(20);
inline pros::Motor Bottom(-10);
inline pros::MotorGroup Score({10, 20});  ///< Top + Bottom together.

inline pros::Motor turret(0);     ///< Set port to 0 if not installed.
inline pros::Optical optical(15); ///< Optical sensor for color sorting.
/** @} */

/**
 * \name Lift with rotational snapping
 * A motor + rotation sensor pair that snaps to the nearest preset angle when
 * the operator stops driving it. While `DIGITAL_UP` / `DIGITAL_DOWN` is held —
 * or the right joystick is deflected — the motor moves manually. As soon as
 * the operator releases, the lift latches the closest snap angle and runs a P
 * controller to it, then holds (motor brake mode is set to HOLD in main.cpp).
 *
 * To use: replace port 0 with your real motor / rotation-sensor ports, then
 * edit the snap angles (degrees) to match your mechanism's rest stops.
 * @{
 */
inline pros::Motor LiftMotor(0);  ///< 0 = disabled until you set the port.
inline pros::Rotation LiftRot(0); ///< Rotation sensor on the lift axis.

/** Lift assembly: snaps to {0, 45, 90, 135, 180}° at rest. */
inline light::RotationalSnap Lift(
    LiftMotor, LiftRot,
    /* snap_angles_deg = */ {0.0, 45.0, 90.0, 135.0, 180.0},
    /* kP             = */ 1.5,
    /* tolerance_deg  = */ 1.0,
    /* max_snap_speed = */ 80);
/** @} */

/**
 * \name Pistons (ADI / 3-wire)
 * Syntax: `inline light::Piston name('PORT');`
 * `PORT` is the letter A–H matching the 3-wire port on the brain or expander.
 *
 * `light::Piston` tracks toggle state and supports:
 *   - `name.set(true / false)`   extend / retract
 *   - `name.button_toggle(btn)`  toggle state when btn is newly pressed
 * @{
 */
inline light::Piston Wings('A');
inline light::Piston Loader('C');
inline light::Piston MidGoal('H');
inline light::Piston Hood('E');     
/** @} */

/**
 * \name Alliance color
 * Set `allianceColor` at the start of autonomous (in `user_autonomous()` or
 * your auton routine) to control color-sort behavior:
 *   - `allianceColor = RED;`     sort out blue rings
 *   - `allianceColor = BLUE;`    sort out red rings
 *   - `allianceColor = NEUTRAL;` sort nothing
 * @{
 */
/** Alliance color enum used by the color-sort logic. */
enum Colors { BLUE = 0,
              NEUTRAL = 1,
              RED = 2 };
extern Colors allianceColor; ///< Currently active alliance color.
/** @} */

/**
 * \name Holonomic / non-tank drives (optional)
 * Uncomment ONE block below if you are using a holonomic drive instead of (or
 * alongside) the EZ-Template tank chassis.
 *
 * \par Mecanum or X-Drive
 * `light::HoloDrive::Type::MECANUM` — standard mecanum "X" wheel layout.
 * `light::HoloDrive::Type::XDRIVE`  — omni wheels mounted at 45°.
 *
 * \code
 *   inline light::HoloDrive holoDrive(
 *       FL_PORT, FR_PORT, BL_PORT, BR_PORT,   // negative = reversed
 *       IMU_PORT,
 *       WHEEL_DIAMETER,                        // inches
 *       GEAR_RATIO,                            // output / motor RPM (1.0 = direct)
 *       light::HoloDrive::Type::MECANUM);
 * \endcode
 *
 * Setup in `initialize()`:
 * \code
 *   holoDrive.calibrate();
 *   holoDrive.set_drive_pid  (kP, kI, kD);
 *   holoDrive.set_strafe_pid (kP, kI, kD);
 *   holoDrive.set_turn_pid   (kP, kI, kD);
 *   holoDrive.set_heading_pid(kP, kI, kD);
 * \endcode
 *
 * Opcontrol (inside the while loop):
 * \code
 *   holoDrive.opcontrol(master.get_analog(ANALOG_LEFT_Y),
 *                       master.get_analog(ANALOG_LEFT_X),
 *                       master.get_analog(ANALOG_RIGHT_X));
 * \endcode
 *
 * Autonomous:
 * \code
 *   holoDrive.drive(24);          // 24 inches forward
 *   holoDrive.strafe(12);         // 12 inches right (negative = left)
 *   holoDrive.turn_to(90);        // face 90° absolute (0–360, CW-positive)
 *   holoDrive.turn_relative(45);  // rotate 45° clockwise from current heading
 * \endcode
 *
 * \par H-Drive
 * Tank sides for forward / back / turn + a single center wheel for strafing.
 *
 * \code
 *   inline light::HDrive hDrive(
 *       {LEFT_PORTS},              // e.g. {-1, -2}
 *       {RIGHT_PORTS},             // e.g. {3, 4}
 *       CENTER_PORT,
 *       IMU_PORT,
 *       WHEEL_DIAMETER,
 *       GEAR_RATIO);
 * \endcode
 *
 * Setup and usage are identical to HoloDrive above.
 * @{
 */
/** @} */
