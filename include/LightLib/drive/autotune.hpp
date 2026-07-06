#pragma once

#include <cstddef>

namespace light {

/**
 * \name Tuner result line for the controller UI
 * Each tuner/characterizer posts one ≤18-char line (success or FAILED) when it
 * exits; auton_toggle() shows it on the controller until UP dismisses it.
 * @{
 */

/** Post a one-line (≤18 char) tuner result for the controller UI. printf-style. */
void autotune_result_set(const char* fmt, ...);

/** Consume the pending result into out; returns false if none pending. */
bool autotune_result_take(char* out, size_t n);

/** @} */


/**
 * \name Stationary noise calibration
 * @{
 */

/**
 * Stationary EKF process-noise calibration. Robot parked. Per-tick var(x,y,θ)
 * → Q values, pushed live via setConfig().
 */
void autotune_ekf_noise(int sampleMs = 10, int durationMs = 5000, int warmupMs = 500);

/**
 * Stationary MCL distance-sensor noise calibration. Robot parked with each
 * sensor pointed at a wall in range. Mean per-sensor stddev → sensorSigmaIn;
 * outlierGapIn = ~3·sigma. Pushed live via setConfig().
 */
void autotune_mcl_noise(int sampleMs = 20, int durationMs = 4000, int warmupMs = 300);

/** @} */


/**
 * \name Bracket-and-bisect PID auto-tune (drive / turn / swing / heading)
 * Three-phase tuner with kI=0. Phase 1 doubles kP until the robot oscillates
 * (≥3 zero-crossings of pos − target), then bisects between the last stable
 * and oscillating values. Phase 2 boosts that kP, doubles kD until the
 * oscillation damps, then bisects to find the smallest sufficient kD.
 * Phase 3 then pushes kP higher in +15% steps — bumping kD when oscillation
 * returns — until settle time stops improving (best-of-2 confirmed), and
 * writes the fastest-settling stable (kP, kD) pair. Final values are applied
 * live via pid_*_constants_set; transcribe the printed FINAL line into
 * default_constants() to persist across reboots. Heading scores on the
 * |L−R| motor voltage differential (the heading PID's own controller effort)
 * instead of IMU error, so its oscillation read isn't masked by drive PID
 * dynamics. Space: turn/swing ~3 ft²; drive ≥6 ft straight; heading ≥10 ft
 * straight.
 * @{
 */
void autotune_turn_pid   (float turnAngleDeg  = 180.0f, float overshootThreshDeg = 3.0f,    int maxIters = 20);
void autotune_drive_pid  (float driveDistIn   = 24.0f,  float overshootThreshIn  = 0.5f,    int maxIters = 20);
void autotune_swing_pid  (float swingAngleDeg = 90.0f,  float overshootThreshDeg = 3.0f,    int maxIters = 20);
void autotune_heading_pid(float driveDistIn   = 48.0f,  float peakDiffMv         = 3000.0f, int maxIters = 20);
/** @} */

}  // namespace light
