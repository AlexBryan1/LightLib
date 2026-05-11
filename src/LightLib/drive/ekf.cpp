#include "LightLib/drive/ekf.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "LightLib/util/util.hpp"
#include "pros/rtos.hpp"

// 6-state EKF. Mean uses the same arc integration as the non-fused odom;
// covariance uses a diagonal additive process-noise model (not textbook
// Jacobian) — cheap, stable, drives the divergence detector. Specialized
// update paths avoid pulling in a general 6×6 linalg routine.

namespace light::ekf {
namespace {

float x_[6] = {0};
float P_[6][6] = {{0}};
MCLConfig cfg_;
pros::Mutex mtx_;

using ::light::util::wrap_rad;

void zeroP() {
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) P_[i][j] = 0.0f;
}

}  // namespace

void init(const Pose& p, MCLConfig cfg) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  cfg_ = cfg;
  x_[0] = p.x;
  x_[1] = p.y;
  x_[2] = p.theta;
  x_[3] = x_[4] = x_[5] = 0.0f;
  zeroP();
  P_[0][0] = P_[1][1] = 0.25f;  // ±0.5 in
  P_[2][2] = 0.001f;            // ~1.8°
  P_[3][3] = P_[4][4] = 0.25f;
  P_[5][5] = 0.001f;
}

void reset(const Pose& p) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  x_[0] = p.x;
  x_[1] = p.y;
  x_[2] = p.theta;
  // Velocities preserved for smoothness across the snap; position/heading
  // cross-covariance wiped to reflect the fresh belief.
  for (int k = 0; k < 3; ++k) {
    for (int j = 0; j < 6; ++j) {
      P_[k][j] = 0.0f;
      P_[j][k] = 0.0f;
    }
  }
  P_[0][0] = P_[1][1] = 0.25f;
  P_[2][2] = 0.001f;
}

void predict(float dLocalX, float dLocalY, float dTheta, float dt) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  float thetaMid = x_[2] + dTheta * 0.5f;
  float s = std::sin(thetaMid);
  float c = std::cos(thetaMid);

  float dWorldX = dLocalY * s - dLocalX * c;
  float dWorldY = dLocalY * c + dLocalX * s;

  x_[0] += dWorldX;
  x_[1] += dWorldY;
  x_[2] = wrap_rad(x_[2] + dTheta);

  if (dt > 1e-6f) {
    x_[3] = dWorldX / dt;
    x_[4] = dWorldY / dt;
    x_[5] = dTheta / dt;
  }

  P_[0][0] += cfg_.ekfQPos * dt;
  P_[1][1] += cfg_.ekfQPos * dt;
  P_[2][2] += cfg_.ekfQTheta * dt;
  P_[3][3] += cfg_.ekfQVel * dt;
  P_[4][4] += cfg_.ekfQVel * dt;
  P_[5][5] += cfg_.ekfQTheta * dt;
}

static void symmetrize() {
  for (int i = 0; i < 6; ++i) {
    for (int j = i + 1; j < 6; ++j) {
      float avg = 0.5f * (P_[i][j] + P_[j][i]);
      P_[i][j] = avg;
      P_[j][i] = avg;
    }
  }
}

static void scalarUpdate(int idx, float innovation, float variance) {
  float S = P_[idx][idx] + variance;
  if (S < 1e-9f) return;

  float K[6];
  for (int i = 0; i < 6; ++i) K[i] = P_[i][idx] / S;

  for (int i = 0; i < 6; ++i) x_[i] += K[i] * innovation;

  // Joseph form: P_new[i][m] = T[i][m] − K[m]·T[i][idx] + var·K[i]·K[m],
  // T[i][k] = P[i][k] − K[i]·P[idx][k]. Stays conditioned under non-optimal K.
  // In-place: we only need the original idx column of T, so compute that 6-vec
  // first, then walk the matrix using P[i][m]'s current value.
  // value (= T[i][m]) plus the Joseph correction terms.

  // P ← T = (I − K H) P. Save row idx since P[idx][k] is read across all i.
  float P_idx_row[6];
  for (int k = 0; k < 6; ++k) P_idx_row[k] = P_[idx][k];
  for (int i = 0; i < 6; ++i)
    for (int k = 0; k < 6; ++k)
      P_[i][k] -= K[i] * P_idx_row[k];

  // Capture T[:, idx] before the next loop modifies it.
  float T_col_idx[6];
  for (int i = 0; i < 6; ++i) T_col_idx[i] = P_[i][idx];

  for (int i = 0; i < 6; ++i)
    for (int m = 0; m < 6; ++m)
      P_[i][m] += variance * K[i] * K[m] - K[m] * T_col_idx[i];

  symmetrize();
}

static void updateHeadingIMU_locked(float thetaRad, float variance) {
  float y = wrap_rad(thetaRad - x_[2]);
  scalarUpdate(2, y, variance);
  x_[2] = wrap_rad(x_[2]);
}

static void updateGPS_locked(float gx, float gy, float variance) {
  // H selects state[0], state[1]; S is 2×2.
  float innov[2] = {gx - x_[0], gy - x_[1]};
  float S00 = P_[0][0] + variance;
  float S01 = P_[0][1];
  float S10 = P_[1][0];
  float S11 = P_[1][1] + variance;
  float det = S00 * S11 - S01 * S10;
  if (std::fabs(det) < 1e-6f) return;
  float invS00 = S11 / det;
  float invS01 = -S01 / det;
  float invS10 = -S10 / det;
  float invS11 = S00 / det;

  // K = P · Hᵀ · S⁻¹; Hᵀ picks columns 0,1 of P.
  float K[6][2];
  for (int i = 0; i < 6; ++i) {
    float c0 = P_[i][0];
    float c1 = P_[i][1];
    K[i][0] = c0 * invS00 + c1 * invS10;
    K[i][1] = c0 * invS01 + c1 * invS11;
  }

  for (int i = 0; i < 6; ++i)
    x_[i] += K[i][0] * innov[0] + K[i][1] * innov[1];
  x_[2] = wrap_rad(x_[2]);

  float T[6][6];
  for (int i = 0; i < 6; ++i)
    for (int k = 0; k < 6; ++k)
      T[i][k] = P_[i][k] - K[i][0] * P_[0][k] - K[i][1] * P_[1][k];

  for (int i = 0; i < 6; ++i) {
    for (int m = 0; m < 6; ++m) {
      float v = T[i][m] - K[m][0] * T[i][0] - K[m][1] * T[i][1] + variance * (K[i][0] * K[m][0] + K[i][1] * K[m][1]);
      P_[i][m] = v;
    }
  }
  symmetrize();
}

void updateHeadingIMU(float thetaRad, float variance) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  updateHeadingIMU_locked(thetaRad, variance);
}

void updateGPS(float gx, float gy, float variance) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  updateGPS_locked(gx, gy, variance);
}

void updateWheelPose(const Pose& m, float variance) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  updateGPS_locked(m.x, m.y, variance);
  updateHeadingIMU_locked(m.theta, variance);
}

Pose mean() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return {x_[0], x_[1], x_[2]};
}
Pose velocity() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return {x_[3], x_[4], x_[5]};
}
float covTrace() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return P_[0][0] + P_[1][1];
}
bool diverged(float t) { return covTrace() > t; }

MCLConfig config() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return cfg_;
}
void setConfig(const MCLConfig& cfg) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  cfg_ = cfg;
}

}  // namespace light::ekf
