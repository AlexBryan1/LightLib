#pragma once

// MCLConfig — runtime tuning for LightCast (particle filter) and the EKF.
// Build one in main.cpp using the MCL_* / EKF_* macros and pass it to
// light::init().  All fields have sensible defaults so zero-config builds work.
struct MCLConfig {
    // Particle filter
    int   numParticles  = 200;    // particle count — reduce if CPU load is high
    float sensorSigmaIn = 2.5f;  // distance sensor noise std dev (inches)
    float outlierGapIn  = 6.0f;  // readings this much shorter than expected → neutral
    float maxRangeIn    = 144.0f; // max ray-cast distance (12 ft VRC field)

    // EKF process noise (larger = trust sensors more over motion model)
    float ekfQPos       = 0.02f;    // in² per second
    float ekfQTheta     = 0.0005f;  // rad² per second
    float ekfQVel       = 1.0f;     // velocity process noise

    // EKF/MCL fusion snap thresholds
    float snapDiverge   = 9.0f;  // EKF cov trace (in²) above which MCL can snap
    float snapConverge  = 3.0f;  // MCL std dev (in) below which it's trusted
};
