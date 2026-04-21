#pragma once

// LightCast — ray-cast Monte-Carlo localization. A particle filter driven by
// distance-sensor ray-casts against the field map. Runs at 5 Hz alongside the
// 100 Hz EKF. Purpose: give the hybrid fusion a second opinion that the EKF
// can snap to when the EKF's own covariance blows up (e.g. after a hard wall
// ram).
//
// predict() runs inline from the 100 Hz odom tick to stay synchronized with
// wheel arc deltas — cheap (no ray-casts).
// update() runs from the LightCast task (lightcast_task.cpp) at 5 Hz — reads
// distance sensors, computes per-particle likelihoods via field::raycast(),
// resamples.

#include "LightLib/odom.hpp"

namespace light::lightcast {

enum class Face { FRONT, BACK, LEFT, RIGHT };

// Convenience builder — derives angle from face, offsets from (along, depth).
//   along = position along the face, robot-frame signed (LEFT/RIGHT +/-)
//   depth = distance from robot center perpendicular to the face (always +)
// Raw DistanceSensorSpec (in odom.hpp) stays exposed for diagonal mounts.
DistanceSensorSpec fromFace(pros::Distance* s, Face face, float along, float depth);

void  init(const Pose& initial, const std::vector<DistanceSensorSpec>& sensors);
void  predict(float dLocalX, float dLocalY, float dTheta);
void  update();
void  startTask();  // spawns the 5 Hz LightCast update task; call after init()
Pose  best();
float convergence();
bool  converged(float threshold_in = 3.0f);
int   sensorCount();

}  // namespace light::lightcast
