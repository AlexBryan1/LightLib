#pragma once

#include "LightLib/lib.hpp"
#include "LightLib/main.h"

/**
 * \file custom_move.hpp
 *
 * Custom movement primitives using LightLib's `Drive` and dual distance
 * sensors.
 *
 * \par Dependencies
 *   - LightLib (provides `light::Drive`)
 *   - PROS 4 (provides `pros::Distance`, `pros::millis`, etc.)
 *   - `LightLib/main.h` (PROS umbrella header)
 *
 * \par Setup
 * Two steps required before use:
 *   1. Call custom_move_init() once inside `initialize()` in main.cpp,
 *      passing your existing `light::Drive` chassis object:
 *      \code
 *        custom_move_init(chassis);
 *      \endcode
 *   2. Declare two `pros::Distance` sensors — one on the front of the robot
 *      and one on the left side — and call WallRide():
 *      \code
 *        pros::Distance frontDist(PORT_A);
 *        pros::Distance leftDist(PORT_B);
 *        WallRide(&frontDist, &leftDist, 6.0, 4.0);
 *      \endcode
 *      Drives forward until front sensor reads ≤ 6 inches, maintaining 4
 *      inches from the left wall throughout.
 *
 * \par Motor control scale
 * All speed values use LightLib's `-127`..`127` scale (not raw millivolts).
 *
 * \note The `light::Drive` reference is stored as a pointer at runtime inside
 * custom_move_init(), avoiding the PROS cold/hot package linker constraint.
 */

/**
 * Register the chassis used by the custom movement primitives.
 *
 * Must be called once in `initialize()` before WallRide().
 *
 * \param chassis
 *        the user's Drive instance
 */
void custom_move_init(light::Drive& chassis);

/**
 * Drive forward while wall-tracking on the left.
 *
 * Combines two behaviors:
 *   1. Stops when the front distance sensor reads ≤ `stopDistIn`.
 *   2. Maintains `targetDistIn` from a left-side wall via a PD steering loop.
 *
 * Pass `nullptr` for a sensor whose port is 0 (not installed). Returns
 * immediately if `frontSensor` is `nullptr`. If `leftSensor` is `nullptr`,
 * the robot drives straight with no wall correction.
 *
 * \param frontSensor
 *        front-facing distance sensor (stop condition)
 * \param leftSensor
 *        left-facing distance sensor (wall-tracking)
 * \param stopDistIn
 *        front threshold in inches — robot stops when front reads ≤ this
 * \param targetDistIn
 *        desired lateral distance from the left wall, inches
 * \param baseSpeed
 *        forward drive speed, 0–127
 * \param timeout
 *        max run duration in milliseconds before forced exit
 */
void WallRide(pros::Distance* frontSensor,
              pros::Distance* leftSensor,
              double stopDistIn,
              double targetDistIn,
              int baseSpeed = 80,
              int timeout = 5000);
