#include "LightLib/ekf.hpp"
#include <cmath>
#include <algorithm>

// Pragmatic 6-state EKF for V5. Mean is advanced by the arc integration the
// odom task already does (same math the non-fused pose used). Covariance uses
// a diagonal additive process-noise model — not textbook Jacobian propagation,
// but cheap, stable, and good enough to drive the divergence detector + to
// weight measurements via Kalman gain. Three specialized update paths avoid
// dragging in a general-purpose 6x6 linear algebra routine.

namespace light::ekf {
namespace {

float x_[6]      = {0};
float P_[6][6]   = {{0}};

// Process noise per unit dt. Larger = less confidence in motion model →
// measurements pull the state more. Tune after first track.
constexpr float Q_POS   = 0.02f;    // in^2 per second
constexpr float Q_THETA = 0.0005f;  // rad^2 per second
constexpr float Q_VEL   = 1.0f;     // velocity less trusted than position

float wrapRad(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

void zeroP() {
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) P_[i][j] = 0.0f;
}

}  // namespace

void init(const Pose& p) {
    x_[0] = p.x;
    x_[1] = p.y;
    x_[2] = p.theta;
    x_[3] = x_[4] = x_[5] = 0.0f;
    zeroP();
    P_[0][0] = P_[1][1] = 0.25f;   // ±0.5 in initial position uncertainty
    P_[2][2] = 0.001f;             // ~1.8° initial heading uncertainty
    P_[3][3] = P_[4][4] = 0.25f;
    P_[5][5] = 0.001f;
}

void reset(const Pose& p) {
    x_[0] = p.x;
    x_[1] = p.y;
    x_[2] = p.theta;
    // Don't touch velocities — smoothness across the snap.
    // Wipe position/heading cross-covariance to reflect the fresh belief.
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
    float thetaMid = x_[2] + dTheta * 0.5f;
    float s = std::sin(thetaMid);
    float c = std::cos(thetaMid);

    float dWorldX = dLocalY * s - dLocalX * c;
    float dWorldY = dLocalY * c + dLocalX * s;

    x_[0] += dWorldX;
    x_[1] += dWorldY;
    x_[2] = wrapRad(x_[2] + dTheta);

    if (dt > 1e-6f) {
        x_[3] = dWorldX / dt;
        x_[4] = dWorldY / dt;
        x_[5] = dTheta  / dt;
    }

    P_[0][0] += Q_POS   * dt;
    P_[1][1] += Q_POS   * dt;
    P_[2][2] += Q_THETA * dt;
    P_[3][3] += Q_VEL   * dt;
    P_[4][4] += Q_VEL   * dt;
    P_[5][5] += Q_THETA * dt;
}

// Scalar Kalman update on state[idx]. Shared helper for 1D measurements.
static void scalarUpdate(int idx, float innovation, float variance) {
    float S = P_[idx][idx] + variance;
    if (S < 1e-9f) return;

    float K[6];
    for (int i = 0; i < 6; ++i) K[i] = P_[i][idx] / S;

    for (int i = 0; i < 6; ++i) x_[i] += K[i] * innovation;

    // P -= K * row[idx]   (H picks row idx)
    float row[6];
    for (int j = 0; j < 6; ++j) row[j] = P_[idx][j];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            P_[i][j] -= K[i] * row[j];
}

void updateHeadingIMU(float thetaRad, float variance) {
    float y = wrapRad(thetaRad - x_[2]);
    scalarUpdate(2, y, variance);
    x_[2] = wrapRad(x_[2]);
}

void updateGPS(float gx, float gy, float variance) {
    // H selects state[0], state[1]. S is 2x2 = [[P00+R, P01],[P10, P11+R]].
    float innov[2] = { gx - x_[0], gy - x_[1] };
    float S00 = P_[0][0] + variance;
    float S01 = P_[0][1];
    float S10 = P_[1][0];
    float S11 = P_[1][1] + variance;
    float det = S00 * S11 - S01 * S10;
    if (std::fabs(det) < 1e-9f) return;
    float invS00 =  S11 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;
    float invS11 =  S00 / det;

    // K = P * H^T * S^-1; H^T picks columns 0,1 of P.
    float K[6][2];
    for (int i = 0; i < 6; ++i) {
        float c0 = P_[i][0];
        float c1 = P_[i][1];
        K[i][0] = c0 * invS00 + c1 * invS10;
        K[i][1] = c0 * invS01 + c1 * invS11;
    }

    for (int i = 0; i < 6; ++i)
        x_[i] += K[i][0] * innov[0] + K[i][1] * innov[1];
    x_[2] = wrapRad(x_[2]);

    float row0[6], row1[6];
    for (int j = 0; j < 6; ++j) { row0[j] = P_[0][j]; row1[j] = P_[1][j]; }
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            P_[i][j] -= K[i][0] * row0[j] + K[i][1] * row1[j];
}

void updateWheelPose(const Pose& m, float variance) {
    updateGPS(m.x, m.y, variance);
    updateHeadingIMU(m.theta, variance);
}

Pose  mean()      { return {x_[0], x_[1], x_[2]}; }
Pose  velocity()  { return {x_[3], x_[4], x_[5]}; }
float covTrace()  { return P_[0][0] + P_[1][1]; }
bool  diverged(float t) { return covTrace() > t; }

}  // namespace light::ekf
