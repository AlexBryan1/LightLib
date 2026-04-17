// ┌─────────────────────────────────────────────────────────────────────────────┐
// │  cstm_move.cpp — Custom movement primitives                                │
// │                                                                            │
// │  This file provides "WallRide", a movement function that drives the robot  │
// │  forward while hugging a wall at a set distance.  It uses two distance     │
// │  sensors:                                                                  │
// │                                                                            │
// │      ┌──────────┐                                                         │
// │      │  ROBOT   │→ front sensor (stop condition)                          │
// │      │          │                                                         │
// │   ←──│          │   left sensor (wall tracking)                           │
// │      └──────────┘                                                         │
// │   ════════════════  wall                                                  │
// │                                                                            │
// │  HOW IT WORKS:                                                            │
// │    1. The robot drives forward at a constant base speed.                   │
// │    2. Every 10ms, the left sensor measures distance to the wall.           │
// │    3. A PD loop computes a steering correction:                            │
// │       - Too far from wall → steer left (toward wall)                      │
// │       - Too close to wall → steer right (away from wall)                  │
// │    4. The correction is applied as a speed differential between the        │
// │       left and right drive sides.                                          │
// │    5. When the front sensor reads ≤ stopDistIn, the robot stops.          │
// │                                                                            │
// │  SETUP:                                                                    │
// │    1. Call cstm_move_init(chassis) once in initialize() in main.cpp.      │
// │    2. Call WallRide(&frontSensor, &leftSensor, stopDist, targetDist)      │
// │       in your auton routine.                                               │
// │                                                                            │
// │  SPEED SCALE:                                                              │
// │    All speeds use EZ-Template's -127 to 127 range (NOT raw millivolts).   │
// │    A baseSpeed of 80 ≈ 63% power, a safe starting point for wall rides.   │
// └─────────────────────────────────────────────────────────────────��───────────┘

#include "LightLib/cstm_move.hpp"
#include <cmath>
#include <algorithm>

// ─── Runtime state ───────────────────────────────────────────────────────────
// Pointer to the EZ-Template chassis object.  Set once by cstm_move_init() so
// we don't need a global extern — avoids PROS cold/hot package linker issues.
static ez::Drive* g_chassis = nullptr;

// ─── Wall-Tracking PD Gains ───────────────────────────────────────────────────
// Units: WALL_KP is in (speed units / mm of lateral distance error).
//        EZ-Template speed scale is -127 to 127.
//        At KP=0.5, a 10mm lateral error produces a 5-unit steering correction.
//
// Tuning procedure:
//   1. Set WALL_KD = 0. Run the robot alongside a left wall and raise WALL_KP
//      until the robot corrects to targetDistIn without side-to-side oscillation.
//   2. If the robot oscillates left/right, raise WALL_KD in small steps to dampen.
//   3. If correction feels too sluggish, raise WALL_KP further, then re-tune WALL_KD.
static constexpr double WALL_KP = 0.5; // TODO: tune for your robot
static constexpr double WALL_KD = 0.1; // TODO: tune for your robot

// pros::Distance::get() returns PROS_ERR (~UINT32_MAX) when no object is detected.
// Readings at or above this threshold (mm) are treated as invalid → zero correction.
static constexpr double SENSOR_MAX_VALID_MM = 2000.0;

// ─── Initialization ──────────────────────────────────────────────────────────
// Must be called once before any WallRide() call.  Typically done in initialize().
void cstm_move_init(ez::Drive& chassis) {
    g_chassis = &chassis;
}

// ─── WallRide ────────────────────────────────────────────────────────────────
// Drives forward while maintaining a set distance from a left-side wall.
// Stops when the front sensor detects a wall within stopDistIn inches,
// or when the timeout expires.
//
// Parameters:
//   frontSensor  — distance sensor on the front (determines when to stop)
//   leftSensor   — distance sensor on the left side (feeds the PD wall-tracking loop)
//                   pass nullptr if not installed; robot drives straight instead
//   stopDistIn   — how close (inches) the front wall must be before stopping
//   targetDistIn — desired distance (inches) to maintain from the left wall
//   baseSpeed    — forward speed, 0–127 (default 80)
//   timeout      — max duration in ms before forced stop (default 5000)
void WallRide(pros::Distance* frontSensor,
              pros::Distance* leftSensor,
              double stopDistIn,
              double targetDistIn,
              int  baseSpeed,
              int  timeout) {

    // Safety guards — do nothing if chassis not initialised or front sensor absent.
    if (g_chassis == nullptr) return;
    if (frontSensor == nullptr) return;  // can't determine stop condition without front sensor

    // Convert inch thresholds to millimeters once up front.
    // pros::Distance::get() returns millimeters; all sensor comparisons use mm.
    const double stopDistMM   = stopDistIn   * 25.4;
    const double targetDistMM = targetDistIn * 25.4;

    double prevWallError     = 0.0;
    const uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < static_cast<uint32_t>(timeout)) {

        // ── Front sensor stop check ───────────────────────────────────────────
        // Read the front sensor every cycle. If it returns a valid reading that
        // is at or below the stop threshold, a wall is close enough ahead — exit.
        double frontMM = static_cast<double>(frontSensor->get());

        if (frontMM < SENSOR_MAX_VALID_MM && frontMM <= stopDistMM) break;

        // ── Left wall distance error ──────────────────────────────────────────
        // If leftSensor is nullptr (port was 0), skip wall correction entirely.
        double wallError = 0.0;

        if (leftSensor != nullptr) {
            double leftMM = static_cast<double>(leftSensor->get());
            if (leftMM < SENSOR_MAX_VALID_MM) {
                // positive wallError → robot too far from left wall  → steer left
                // negative wallError → robot too close to left wall  → steer right
                wallError = targetDistMM - leftMM;
            }
        }

        double derivative = wallError - prevWallError;
        prevWallError = wallError;

        // Compute the steering correction in EZ-Template speed units (-127 to 127).
        int correction = static_cast<int>(WALL_KP * wallError + WALL_KD * derivative);
        correction = std::clamp(correction, -baseSpeed, baseSpeed);

        // ── Differential speed application ───────────────────────────────────
        // Sensor is on the LEFT side. Positive wallError means robot is too far
        // from the left wall and must steer left (i.e., reduce left speed,
        // increase right speed so the robot arcs leftward).
        //
        // Steering convention for a left-side sensor:
        //   wallError > 0 (too far left)  → reduce left, increase right → arcs left
        //   wallError < 0 (too close left) → increase left, reduce right → arcs right
        //
        // NOTE: if your robot steers in the wrong direction, swap the + and - signs
        //       on leftSpeed and rightSpeed below.
        int leftSpeed  = baseSpeed - correction;
        int rightSpeed = baseSpeed + correction;

        // Clamp to valid EZ-Template speed range. Floor at 0 to prevent reversing.
        leftSpeed  = std::clamp(leftSpeed,  0, 127);
        rightSpeed = std::clamp(rightSpeed, 0, 127);

        g_chassis->drive_set(leftSpeed, rightSpeed);

        pros::delay(10); // 10 ms loop — required to yield to the PROS scheduler
    }

    // Stop the drive at the end of the move regardless of exit condition.
    g_chassis->drive_set(0, 0);
}
// ─── Turret tracking (example / reference) ───────────────────────────────────
// If your robot has a turret, you can add the following to opcontrol().
// Define the turret motor to the correct port, and change DIGITAL_R1 to
// whichever button you want to use for shooting.
//
//   void opcontrol() {
//       pros::Task turret_task(track_basket);   // start aiming task
//       bool pressed = false;
//       while (true) {
//           if (master.get_digital(DIGITAL_R1)) {
//               turret.move_relative(err_basket, p_turret);
//               pressed = true;
//           } else if (pressed) {
//               pros::Task turr_reset(turret_reset);
//               pressed = false;
//           }
//       }
//   }