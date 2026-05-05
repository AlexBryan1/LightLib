#pragma once

#include "LightLib/lib.hpp"

/**
 * \file drive.hpp (top-level)
 *
 * Legacy global-namespace drive helpers — short-name wrappers around the
 * `light::Drive` API plus a Stanley/path-injection helper that predates
 * the RAMSETE follower.
 *
 * Most new code should use the `light::Drive` methods directly. These
 * wrappers remain for older auton routines and for the path-injection /
 * Stanley path-follower entry points.
 */

const double WHEEL_DIAMETER = 3.3369184232; ///< Default wheel diameter, inches.
const double ROBOT_WIDTH = 11.4375;         ///< Default robot track width, inches.
const int KEY = 267267;                     ///< Misc. magic constant retained for legacy code.

/** Which side a distance sensor faces for the legacy distance helpers. */
enum DistanceSide {
  BACK = 0,
  RIGHT = 1,
  LEFT = 2
};

/** Auton mode selector for the legacy path runner. */
enum AutonMode {
  STANLEY = 0, ///< Stanley path-follower.
  ODOM = 1,    ///< Odom-based motion primitives.
  PLAIN = 2,   ///< Direct PID drive sequencing.
  BRAIN = 3,   ///< Driven by the brain-screen selector.
};

/** Wait-mode selector for `pidWait()`. */
enum Wait { WAIT = 0,
            QUICK = 1,
            CHAIN = 2 };

/**
 * One waypoint in the legacy injected-path representation.
 *
 * Holds a target pose, optional left/right wheel velocities for forward
 * simulation, and the angle-behavior used at the point.
 */
class Coordinate {
 public:
  double x = 0;     ///< Field-frame X, inches.
  double y = 0;     ///< Field-frame Y, inches.
  double t = 0;     ///< Heading at the waypoint, degrees.
  double right = 127; ///< Right wheel command at the waypoint.
  double left = 0;    ///< Left wheel command at the waypoint.
  light::e_angle_behavior behavior = light::cw; ///< Turn behavior at the waypoint.
};

extern double angle_offset;            ///< Heading offset applied by the legacy helpers.
extern AutonMode autonMode;            ///< Active auton mode for the legacy runner.
extern std::vector<Coordinate> autonPath; ///< Path being executed by the legacy runner.

/**
 * \name Internal math (legacy path helpers)
 * @{
 */
double getDistance(Coordinate point1, Coordinate point2);
double getTheta(Coordinate point1, Coordinate point2, light::drive_directions direction);
double getVelocity(double voltage);
double getTimeToPoint(double distance, double velocity);
Coordinate getPoint(Coordinate startPoint, double distance);
Coordinate getPoint(Coordinate startPoint, double v_left, double v_right, double time);
std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, e_angle_behavior behavior, double left, double right, double theta,
                                    double lookAhead);
std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead);
/** @} */

/**
 * \name Position wrappers
 * @{
 */
void setPosition(double x, double y);
void setPosition(double x, double y, double t);
double getDistanceActual(DistanceSide sensor, bool use_theta, double failsafe);
/** @} */

/**
 * \name Wait wrappers
 * @{
 */
void pidWait(Wait type);
void pidWaitUntil(okapi::QLength distance);
void pidWaitUntil(okapi::QAngle distance);
void pidWaitUntil(Coordinate coordinate);
void delayMillis(int millis);
void delayMillis(int millis, bool ignore);
/** @} */

/**
 * \name Drive-set wrappers
 * @{
 */
void driveSet(double distance, int speed, bool slew);
void driveSet(double distance, int speed);
void driveSmartSet(double distance, int speed);
/** @} */

/**
 * \name Turn-set wrappers
 * @{
 */
void turnSet(double theta, int speed, e_angle_behavior behavior);
void turnSet(double theta, int speed);
void turnSet(Coordinate point, drive_directions direction, int speed, e_angle_behavior behavior);
void turnSet(Coordinate point, drive_directions direction, int speed);
void turnSetRelative(double theta, int speed, e_angle_behavior behavior);
void turnSetRelative(double theta, int speed);
/** @} */

/**
 * \name Swing-set wrappers
 * @{
 */
void swingSet(light::e_swing side, double theta, double main, double opp, light::e_angle_behavior behavior);
void swingSet(light::e_swing side, double theta, double main, light::e_angle_behavior behavior);
void swingSet(light::e_swing side, double theta, double main, double opp);
void swingSet(light::e_swing side, double theta, double main);
/** @} */

/**
 * \name Path debugging
 * @{
 */
/** Print the current `autonPath` to the PROS terminal. */
void getPath();
/** Print the post-injection `autonPath` to the PROS terminal. */
void getPathInjected();
/** @} */