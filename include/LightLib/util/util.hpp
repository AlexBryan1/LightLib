/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

/**
 * \file util.hpp
 *
 * Shared types, enums, math helpers, and unit conversions used across
 * LightLib. Pulled into nearly every other header.
 */

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "LightLib/api.h"
#include "light/units.hpp"

using namespace okapi::literals;

/** Global V5 master controller, used by opcontrol throughout LightLib. */
extern pros::Controller master;

namespace light {

/**
 * Prints our branding all over your pros terminal.
 */
void ez_template_print();

/**
 * Prints to the brain screen in one string.
 *
 * Splits input between lines with '\n' or when text longer then 32 characters.
 *
 * \param text
 *        input string
 * \param line
 *        starting line to print on, defaults to 0
 */
void screen_print(std::string text, int line = 0);

/**
 * \name Public types and enums
 * @{
 */

/** Arcade joystick layout. */
enum e_type { SINGLE = 0, ///< Single-stick arcade (one stick = throttle + turn).
              SPLIT = 1 };///< Split-stick arcade (left = throttle, right = turn).

/** Direction for swing-turn motions. */
enum e_swing { LEFT_SWING = 0,  ///< Left side stationary, right side drives.
               RIGHT_SWING = 1 };///< Right side stationary, left side drives.

/** Result codes returned by PID::exit_condition. */
enum exit_output { RUNNING = 1,            ///< Still inside the motion.
                   SMALL_EXIT = 2,         ///< Small-error timer expired.
                   BIG_EXIT = 3,           ///< Big-error timer expired.
                   VELOCITY_EXIT = 4,      ///< Stopped moving long enough.
                   mA_EXIT = 5,            ///< Motor current limit hit.
                   ERROR_NO_CONSTANTS = 6 };///< No PID constants set.

/** Active chassis motion mode. */
enum e_mode { DISABLE = 0,       ///< No motion active.
              SWING = 1,         ///< Swing turn.
              TURN = 2,          ///< Turn-in-place to absolute heading.
              TURN_TO_POINT = 3, ///< Turn to face a field point.
              DRIVE = 4,         ///< Straight-line drive.
              POINT_TO_POINT = 5,///< Drive to a field point.
              PURE_PURSUIT = 6 };///< Pure-pursuit path follower.

/** Drive direction with multiple spelling aliases. */
enum drive_directions { FWD = 0,
                        FORWARD = FWD,
                        fwd = FWD,
                        forward = FWD,
                        REV = 1,
                        REVERSE = REV,
                        rev = REV,
                        reverse = REV };

/** How to choose a turn direction when reaching an absolute heading. */
enum e_angle_behavior { raw = 0,                ///< Use signed delta as-is (no wrap).
                        left_turn = 1,          ///< Always turn counterclockwise.
                        LEFT_TURN = 1,
                        counterclockwise = 1,
                        ccw = 1,
                        right_turn = 2,         ///< Always turn clockwise.
                        RIGHT_TURN = 2,
                        clockwise = 2,
                        cw = 2,
                        shortest = 3,           ///< Pick the smaller of CW/CCW.
                        longest = 4 };          ///< Pick the larger of CW/CCW.

/** Sentinel value indicating "no angle specified". */
const double ANGLE_NOT_SET = 0.0000000000000000000001;
/** Sentinel value indicating "no angle specified" (united). */
const okapi::QAngle p_ANGLE_NOT_SET = 0.0000000000000000000001_deg;

/** 2D pose: position (in) and heading (deg). */
typedef struct pose {
  double x;                    ///< X position, inches.
  double y;                    ///< Y position, inches.
  double theta = ANGLE_NOT_SET;///< Heading, degrees.
} pose;

/** 2D pose with okapi units. */
typedef struct united_pose {
  okapi::QLength x;                      ///< X position with units.
  okapi::QLength y;                      ///< Y position with units.
  okapi::QAngle theta = p_ANGLE_NOT_SET; ///< Heading with units.
} united_pose;

/** One waypoint of an odom-mode movement. */
typedef struct odom {
  pose target;                                  ///< Target pose.
  drive_directions drive_direction;             ///< Drive forward or reverse.
  int max_xy_speed;                             ///< Speed cap, 0..127.
  e_angle_behavior turn_behavior = shortest;    ///< How to choose turn direction.
} odom;

/** One waypoint of an odom-mode movement, with okapi units. */
typedef struct united_odom {
  united_pose target;                           ///< Target pose with units.
  drive_directions drive_direction;             ///< Drive forward or reverse.
  int max_xy_speed;                             ///< Speed cap, 0..127.
  e_angle_behavior turn_behavior = shortest;    ///< How to choose turn direction.
} united_odom;

/** @} */

/**
 * Returns a human-readable name for an exit_output code.
 *
 * \param input
 *        the exit_output value
 */
std::string exit_to_string(exit_output input);

/**
 * Single source of truth for unit conversions. Multiplying is faster and
 * clearer than repeating the magic literals (25.4, 0.0254, M_PI/180) inline.
 */
namespace units {
inline constexpr double MM_PER_IN = 25.4;          ///< Millimeters per inch.
inline constexpr double IN_PER_MM = 1.0 / 25.4;    ///< Inches per millimeter.
inline constexpr double M_PER_IN = 0.0254;         ///< Meters per inch.
inline constexpr double IN_PER_M = 1.0 / 0.0254;   ///< Inches per meter.
inline constexpr double DEG_PER_RAD = 180.0 / M_PI;///< Degrees per radian.
inline constexpr double RAD_PER_DEG = M_PI / 180.0;///< Radians per degree.
}  // namespace units

/** Misc math, geometry, and unit-conversion helpers. */
namespace util {
extern bool AUTON_RAN; ///< Set true once any autonomous routine has run this match.

/**
 * Returns the amount of places after a decimal, maxing out at 6.
 *
 * \param input
 *        your input value with decimals
 * \param min
 *        minimum number of decimal places to print, this is defaulted to 0
 */
int places_after_decimal(double input, int min = 0);

/**
 * Returns a string with a specific number of decimal points.
 *
 * \param input
 *        your input value
 * \param n
 *        the amount of decimals you want to display
 */
std::string to_string_with_precision(double input, int n = 2);

/**
 * Returns 1 if input is positive and -1 if input is negative.
 *
 * \param input
 *        your input value
 */
int sgn(double input);

/**
 * Returns true if the input is < 0.
 *
 * \param input
 *        your input value
 */
bool reversed_active(double input);

/**
 * Returns input restricted to min-max threshold.
 *
 * \param input
 *        your input value
 * \param max
 *        the maximum input can be
 * \param min
 *        the minimum input can be
 */
double clamp(double input, double max, double min);

/**
 * Returns input restricted to min-max threshold.
 *
 * The minimum used is negative max.
 *
 * \param input
 *        your input value
 * \param max
 *        the absolute value maximum input can be
 */
double clamp(double input, double max);

/** True if an SD card is currently inserted. Captured at program start. */
const bool SD_CARD_ACTIVE = pros::usd::is_installed();

/** Default loop period for LightLib background tasks, in milliseconds. */
const int DELAY_TIME = 10;

/**
 * Converts radians to degrees.
 *
 * \param input
 *        your input radian
 */
double to_deg(double input);

/**
 * Converts degrees to radians.
 *
 * \param input
 *        your input degree
 */
double to_rad(double input);

/**
 * Returns the angle between two points.
 *
 * \param itarget
 *        target position
 * \param icurrent
 *        current position
 */
double absolute_angle_to_point(pose itarget, pose icurrent);

/**
 * Returns the distance between two points.
 *
 * \param itarget
 *        target position
 * \param icurrent
 *        current position
 */
double distance_to_point(pose itarget, pose icurrent);

/**
 * Constrains an angle between 180 and -180.
 *
 * \param theta
 *        input angle in degrees
 */
double wrap_angle(double theta);

/**
 * Wraps an angle in radians to the half-open interval (-pi, pi].
 *
 * Uses `atan2(sin, cos)` rather than a while-loop normalization: branch-free
 * and stable across very large inputs.
 *
 * \param a
 *        input angle in radians
 */
inline float wrap_rad(float a) {
  return std::atan2(std::sin(a), std::cos(a));
}

/**
 * Returns a new pose that is projected off of the current pose.
 *
 * \param added
 *        how far to project a new point
 * \param icurrent
 *        point to project off of
 */
pose vector_off_point(double added, pose icurrent);

/**
 * Returns the shortest angle for the robot to turn to in order to get to target.
 *
 * \param target
 *        target value in degrees
 * \param current
 *        current value in degrees
 * \param print = false
 *        will print the new value if this is true, defaults to false
 */
double turn_shortest(double target, double current, bool print = false);

/**
 * Returns the farthest away angle for the robot to turn to in order to get to target.
 *
 * \param target
 *        target value in degrees
 * \param current
 *        current value in degrees
 * \param print = false
 *        will print the new value if this is true, defaults to false
 */
double turn_longest(double target, double current, bool print = false);

/**
 * Converts pose with okapi units to a pose without okapi units.
 *
 * \param input
 *        a pose with units
 */
pose united_pose_to_pose(united_pose input);

/**
 * Converts vector of poses with okapi units to a vector of poses without okapi units.
 *
 * \param inputs
 *        poses with units
 */
std::vector<odom> united_odoms_to_odoms(std::vector<united_odom> inputs);

/**
 * Converts odom movement with united pose to an odom movement without united pose.
 *
 * \param input
 *        odom movement with units
 */
odom united_odom_to_odom(united_odom input);

}  // namespace util
}  // namespace light
