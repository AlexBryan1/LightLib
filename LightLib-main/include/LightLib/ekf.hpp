#pragma once

// 6-state Extended Kalman Filter driving the fused pose output.
//
// State: [x, y, theta, vx, vy, omega]  (inches, radians, per-second)
// Frame: same as odom.hpp Pose — origin at field center, angles CCW from +y.
//
// The arc integration in odometry.cpp::update() produces local-frame deltas
// (dLocalX, dLocalY, dTheta) each tick. predict() integrates those into the
// state mean and propagates covariance.
//
// Measurements are applied independently via update*() calls; each has its
// own R (measurement variance). The Kalman gain automatically weights noisy
// measurements less. No manual if/else on which sensor "wins".
//
// When LightCast converges and EKF is uncertain, odometry.cpp calls reset()
// to snap the state to the LightCast best estimate.

#include "LightLib/odom.hpp"
#include "LightLib/mcl_config.hpp"

namespace light::ekf {

void init(const Pose& initial, MCLConfig cfg = {});
void reset(const Pose& pose);  // snap mean, reset covariance to small

// Propagate mean + covariance one tick forward using local-frame arc deltas.
void predict(float dLocalX, float dLocalY, float dTheta, float dt);

// Independent measurement updates. Variance is in (state-units)^2.
void updateHeadingIMU(float thetaRad, float variance);
void updateGPS(float x, float y, float variance);
void updateWheelPose(const Pose& measured, float variance);

Pose  mean();
Pose  velocity();
float covTrace();  // trace(P[0:2, 0:2]) — position uncertainty in in^2
bool  diverged(float threshold_in_sq);

// Live tuning — used by the on-brain tuner to push noise constants without
// reinitializing the filter.
MCLConfig config();
void      setConfig(const MCLConfig& cfg);

}  // namespace light::ekf
