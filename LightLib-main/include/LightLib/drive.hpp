#pragma once

#include "EZ-Template/api.hpp"
#include "EZ-Template/util.hpp"

const double WHEEL_DIAMETER = 3.3369184232;
const double ROBOT_WIDTH = 11.4375;
const int KEY = 267267;

enum Distance {
	BACK = 0,
	RIGHT = 1,
	LEFT = 2
};

enum AutonMode {
    STANLEY = 0,
	ODOM = 1,
	PLAIN = 2,
	BRAIN = 3,
};

enum Wait { WAIT = 0, QUICK = 1, CHAIN = 2 };

class Coordinate {
   public:
	double x = 0;
	double y = 0;
	double t = 0;
	double right = 127;
	double left = 0;
	ez::e_angle_behavior behavior = ez::cw;
};

extern double angle_offset;
extern AutonMode autonMode;
extern vector<Coordinate> autonPath;

// Internal math
double getDistance(Coordinate point1, Coordinate point2);
double getTheta(Coordinate point1, Coordinate point2, ez::drive_directions direction);
double getVelocity(double voltage);
double getTimeToPoint(double distance, double velocity);
Coordinate getPoint(Coordinate startPoint, double distance);
Coordinate getPoint(Coordinate startPoint, double v_left, double v_right, double time);
std::vector<Coordinate> injectPoint(Coordinate startPoint, Coordinate endPoint, e_angle_behavior behavior, double left, double right, double theta,
									double lookAhead);
std::vector<Coordinate> injectPath(std::vector<Coordinate> coordList, double lookAhead);

// Set position wrappers
void setPosition(double x, double y);
void setPosition(double x, double y, double t);
double getDistanceActual(Distance sensor, bool use_theta, double failsafe);

// Wait wrappers
void pidWait(Wait type);
void pidWaitUntil(okapi::QLength distance);
void pidWaitUntil(okapi::QAngle distance);
void pidWaitUntil(Coordinate coordinate);
void delayMillis(int millis);
void delayMillis(int millis, bool ignore);

// Move to point wrappers

// Drive set wrappers
void driveSet(double distance, int speed, bool slew);
void driveSet(double distance, int speed);
void driveSmartSet(double distance, int speed);

// Turn set wrappers
void turnSet(double theta, int speed, e_angle_behavior behavior);
void turnSet(double theta, int speed);
void turnSet(Coordinate point, drive_directions direction, int speed, e_angle_behavior behavior);
void turnSet(Coordinate point, drive_directions direction, int speed);
void turnSetRelative(double theta, int speed, e_angle_behavior behavior);
void turnSetRelative(double theta, int speed);

// Swing set wrappers
void swingSet(ez::e_swing side, double theta, double main, double opp, ez::e_angle_behavior behavior);
void swingSet(ez::e_swing side, double theta, double main, ez::e_angle_behavior behavior);
void swingSet(ez::e_swing side, double theta, double main, double opp);
void swingSet(ez::e_swing side, double theta, double main);

// Print path
void getPath();
void getPathInjected();