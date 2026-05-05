#pragma once

#include "LightLib/path/field_map.hpp"

/**
 * \file mcl_config.hpp
 *
 * Runtime tuning for LightCast (particle filter) and the EKF.
 *
 * Build one in `main.cpp` using the `MCL_*` / `EKF_*` macros and pass it to
 * `light::init()`. All fields have sensible defaults so zero-config builds
 * work.
 */

/**
 * Tuning knobs for the LightCast particle filter and the EKF.
 *
 * All fields are public and have defaults — override only what you need.
 */
struct MCLConfig {
  /**
   * \name Particle filter
   * @{
   */
  int numParticles = 200;                          ///< Particle count — reduce if CPU load is high.
  float sensorSigmaIn = 2.5f;                      ///< Distance-sensor noise std dev, inches.
  float outlierGapIn = 6.0f;                       ///< Readings this much shorter than expected → neutral.
  float maxRangeIn = light::field::FIELD_SIZE_IN;  ///< Max ray-cast distance, inches.
  float initPosSigmaIn = 1.0f;                     ///< Initial particle x/y spread around starting pose, inches.
  float initHeadingSigmaRad = 0.05f;               ///< Initial particle heading spread, radians (~3°).
  /** @} */

  /**
   * \name EKF process noise
   * Larger values = trust sensors more over the motion model.
   * @{
   */
  float ekfQPos = 0.02f;      ///< Position process noise, in² per second.
  float ekfQTheta = 0.0005f;  ///< Heading process noise, rad² per second.
  float ekfQVel = 1.0f;       ///< Velocity process noise.
  /** @} */

  /**
   * \name EKF / MCL fusion snap thresholds
   * @{
   */
  float snapDiverge = 9.0f;   ///< EKF cov trace (in²) above which MCL may snap.
  float snapConverge = 3.0f;  ///< MCL std dev (in) below which it's trusted.
  /** @} */
};
