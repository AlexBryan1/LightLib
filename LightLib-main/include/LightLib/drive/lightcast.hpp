#pragma once

#include "LightLib/drive/mcl_config.hpp"
#include "LightLib/drive/odometry.hpp"

/**
 * \file lightcast.hpp
 *
 * LightCast — ray-cast Monte-Carlo localization.
 *
 * A particle filter driven by distance-sensor ray-casts against the field
 * map. Runs at 10 Hz alongside the 100 Hz EKF. Purpose: give the hybrid
 * fusion a second opinion that the EKF can snap to when the EKF's own
 * covariance blows up (e.g. after a hard wall ram).
 *
 * \par Update cadence
 *   - predict() runs inline from the 100 Hz odom tick to stay synchronized
 *     with wheel arc deltas. Cheap (no ray-casts).
 *   - update() runs from the LightCast task (`lightcast.cpp`) at 10 Hz —
 *     reads distance sensors, computes per-particle likelihoods via
 *     `field::raycast()`, and resamples.
 */

namespace light::lightcast {

/** Robot-frame face that a distance sensor is mounted on. */
enum class Face { FRONT,
                  BACK,
                  LEFT,
                  RIGHT };

/**
 * Convenience builder — derives angle from `face`, offsets from
 * `(along, depth)`.
 *
 * \param s
 *        the PROS distance sensor (pointer, not owned)
 * \param face
 *        which face the sensor is mounted on
 * \param along
 *        position along the face, robot-frame signed (LEFT/RIGHT +/-)
 * \param depth
 *        distance from robot center perpendicular to the face (always
 *        positive)
 *
 * Raw `DistanceSensorSpec` (in odometry.hpp) stays exposed for diagonal
 * mounts.
 */
DistanceSensorSpec fromFace(pros::Distance* s, Face face, float along, float depth);

/**
 * Initialize the particle filter.
 *
 * \param initial
 *        starting pose
 * \param sensors
 *        list of distance-sensor specs
 * \param cfg
 *        tuning constants
 */
void init(const Pose& initial, const std::vector<DistanceSensorSpec>& sensors, MCLConfig cfg = {});

/** Propagate particles by a local-frame arc delta. Cheap, no ray-casts. */
void predict(float dLocalX, float dLocalY, float dTheta);

/** Measurement update: read sensors, weight particles, resample. */
void update();

/** Spawn the 10 Hz LightCast update task. Call after init(). */
void startTask();

/** \return The current best-estimate pose (highest-weight particle / mean). */
Pose best();

/** \return The standard deviation of the particle cloud, inches. */
float convergence();

/** \return true if convergence() < `threshold_in`. */
bool converged(float threshold_in = 3.0f);

/** \return Number of distance sensors registered. */
int sensorCount();

/** \return The registered sensor specs. */
std::vector<DistanceSensorSpec> sensors();

/**
 * Number of ticks where the filter saw all-degenerate weights and reset to
 * uniform (motion-model only). Increments without printing — caller can
 * poll on its own schedule for diagnostics.
 */
uint32_t degenerateTickCount();

/**
 * \name Live tuning
 * Mirrors the EKF API so the autotune routine and on-brain tuner can push
 * noise constants without reinitializing the filter.
 * @{
 */
MCLConfig config();
void setConfig(const MCLConfig& cfg);
/** @} */

}  // namespace light::lightcast
