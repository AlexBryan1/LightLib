#pragma once

#include "LightLib/drive/odometry.hpp"

/**
 * \file sensor_aux.hpp
 *
 * Override-on-confidence accuracy boosters for partial sensor configs.
 *
 * Four detectors run from the 100 Hz odom tick after light::update().
 * Each gates itself on a confidence check and only fires when the gate
 * trips. Trigger conditions:
 *
 *  - IMU bias (ZUPT): robot stationary for AUX_ZUPT_DWELL_MS — sample
 *    gyro rate, expose as a bias offset that odometry.cpp subtracts.
 *  - Single-wheel rotation correction: exactly one tracking wheel on
 *    an axis — odometry.cpp subtracts the rotational arc component
 *    using the wheel's offset and the IMU heading delta.
 *  - Wheel-slip detection: wheel-derived dtheta disagrees with IMU
 *    dtheta beyond AUX_SLIP_DEG_PER_100MS — odometry.cpp zeros the
 *    wheel-derived translation for the duration of the slip.
 *  - Single-axis wall-snap: a distance sensor reads consistent with a
 *    known wall — sensor_aux directly overrides the perpendicular
 *    pose axis via light::setPose().
 */

namespace light::sensor_aux {

/** Capture the sensor bundle and detect single-wheel modes. */
void init(const OdomSensors& sensors);

/**
 * Run all four detectors. Call from the odom task immediately after
 * light::update().
 */
void tick();

/** \return Cumulative IMU gyro bias estimate, rad/s. Zero before the
 *          first ZUPT trigger; updated each time the robot is detected
 *          stationary. odometry.cpp subtracts `bias * dt` from
 *          `deltaImu` each tick. */
float imuBiasRadPerSec();

/** \return true when exactly one vertical tracking wheel is configured. */
bool singleVertMode();

/** \return Stored offset of the single vertical wheel (0 if not in
 *          single-vert mode). */
float singleVertOffset();

/** \return true when exactly one horizontal tracking wheel is configured. */
bool singleHorizMode();

/** \return Stored offset of the single horizontal wheel (0 if not in
 *          single-horiz mode). */
float singleHorizOffset();

/** \return true when wheel-IMU heading disagreement exceeds the slip
 *          threshold. odometry.cpp zeros translation predict when set. */
bool slipping();

/** \return Number of times the wall-snap fired this auton (diagnostic). */
int wallSnapsThisAuton();

/** Reset diagnostic counters at auton start. */
void resetCounters();

}  // namespace light::sensor_aux
