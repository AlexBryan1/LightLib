#pragma once

#include "LightLib/drive/mcl_config.hpp"
#include "LightLib/drive/odometry.hpp"

/**
 * \file ekf.hpp
 *
 * 6-state Extended Kalman Filter driving the fused pose output.
 *
 * \par State
 * `[x, y, theta, vx, vy, omega]`  (inches, radians, per-second).
 *
 * \par Frame
 * Same as odometry.hpp `Pose` — origin at field center, angles CCW from
 * `+y`.
 *
 * The arc integration in `odometry.cpp::update()` produces local-frame
 * deltas `(dLocalX, dLocalY, dTheta)` each tick. predict() integrates
 * those into the state mean and propagates covariance.
 *
 * Measurements are applied independently via update*() calls; each has
 * its own R (measurement variance). The Kalman gain automatically weights
 * noisy measurements less. There is no manual if/else on which sensor
 * "wins".
 *
 * When LightCast converges and the EKF is uncertain, `odometry.cpp`
 * calls reset() to snap the state to the LightCast best estimate.
 */

namespace light::ekf {

/**
 * Initialize the filter.
 *
 * \param initial
 *        starting pose (mean)
 * \param cfg
 *        process-noise / convergence thresholds
 */
void init(const Pose& initial, MCLConfig cfg = {});

/**
 * Snap the state mean to `pose` and reset covariance to a small value.
 *
 * Used when LightCast strongly disagrees with the EKF.
 */
void reset(const Pose& pose);

/**
 * Propagate mean + covariance one tick forward using local-frame arc
 * deltas.
 *
 * \param dLocalX
 *        forward delta this tick, inches
 * \param dLocalY
 *        lateral delta this tick, inches
 * \param dTheta
 *        heading delta this tick, radians
 * \param dt
 *        seconds since last predict
 */
void predict(float dLocalX, float dLocalY, float dTheta, float dt);

/**
 * \name Measurement updates
 * Each update is independent. `variance` is in (state-units)².
 * @{
 */
void updateHeadingIMU(float thetaRad, float variance);
void updateGPS(float x, float y, float variance);
void updateWheelPose(const Pose& measured, float variance);
/** @} */

/** \return Current state mean as a Pose. */
Pose mean();

/** \return Current velocity as a Pose-shaped triple `(vx, vy, omega)`. */
Pose velocity();

/** \return `trace(P[0:2, 0:2])` — position uncertainty in in². */
float covTrace();

/** \return true if the position covariance trace exceeds `threshold_in_sq`. */
bool diverged(float threshold_in_sq);

/**
 * \name Live tuning
 * Used by the on-brain tuner to push noise constants without
 * reinitializing the filter.
 * @{
 */
MCLConfig config();
void setConfig(const MCLConfig& cfg);
/** @} */

}  // namespace light::ekf
