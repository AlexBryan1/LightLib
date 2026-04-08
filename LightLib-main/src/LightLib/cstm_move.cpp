// cstm_move.cpp
// Custom movement primitives using EZ-Template motor control and dual distance sensors.
// PROS Makefile compiles all src/*.cpp files automatically — no Makefile edits needed.
//
// USAGE:
//   1. Call cstm_move_init(chassis) once in initialize() in main.cpp.
//   2. Call driveWallTrack(frontSensor, leftSensor, stopDistIn, targetDistIn) in auton.
//
// MOTOR CONTROL:
//   Uses ez::Drive::drive_set(left, right) — values are -127 to 127, NOT millivolts.
//   A baseSpeed of 80 ≈ 63% power, which is a safe starting point for wall tracking.
//
// STOP CONDITION:
//   The front distance sensor determines when to stop. When the front sensor reads
//   ≤ stopDistIn inches, the robot has reached a wall ahead and the function exits.
//   This replaces the encoder-based distance tracking used in the LemLib version.
//
// WALL TRACKING:
//   The left distance sensor feeds a PD loop that applies a differential speed
//   correction to the left and right drive sides each 10ms control cycle.

#include "cstm_move.hpp"
#include <cmath>
#include <algorithm>

// ─── Runtime state (populated by cstm_move_init) ─────────────────────────────
static ez::Drive* g_chassis = nullptr;
// ─────────────────────────────────────────────────────────────────────────────

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
// ─────────────────────────────────────────────────────────────────────────────

void cstm_move_init(ez::Drive& chassis) {
    g_chassis = &chassis;
}

void WallRide(pros::Distance& frontSensor,
                    pros::Distance& leftSensor,
                    double stopDistIn,
                    double targetDistIn,
                    int  baseSpeed,
                    int  timeout) {

    // Safety guard — do nothing if cstm_move_init() was never called.
    if (g_chassis == nullptr) return;

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
        double frontMM = static_cast<double>(frontSensor.get());

        if (frontMM < SENSOR_MAX_VALID_MM && frontMM <= stopDistMM) break;

        // ── Left wall distance error ──────────────────────────────────────────
        // Read the left sensor. Zero the correction if the reading is invalid
        // (out of range or sensor error) to prevent runaway steering.
        double leftMM    = static_cast<double>(leftSensor.get());
        double wallError = 0.0;

        if (leftMM < SENSOR_MAX_VALID_MM) {
            // positive wallError → robot too far from left wall  → steer left
            // negative wallError → robot too close to left wall  → steer right
            wallError = targetDistMM - leftMM;
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
///
// For turret functions addd the following to opcontrol, and define the turret motor to the right port
// P.S. change R1 to the button you want to press when shooting the turret, and add the shooting function under the first if statement
//
// void opcontrol(){
//    pros::Task turret_task(track_basket);
//    while (true){
//     if (master.get_digital(DIGITAL_R1)){
//         turret.move_relative(err_basket, p_turret);
//         bool pressed = true;
//     }
//     else if (pressed){
//         if (!master.get_digital(DIGITAL_R1)){
//             pros::Task turr_reset(turret_reset);
//         }
//     }
//    }
// }
//
//
//
//
//
//