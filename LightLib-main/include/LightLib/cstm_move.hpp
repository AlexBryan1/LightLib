#pragma once

// cstm_move.hpp
// Custom movement primitives using EZ-Template motor control and dual distance sensors.
//
// DEPENDENCIES:
//   - EZ-Template (provides ez::Drive)
//   - PROS 4 (provides pros::Distance, pros::millis, etc.)
//   - main.h (PROS umbrella header)
//
// SETUP — two steps required before use:
//
//   Step 1: Call cstm_move_init() once inside initialize() in main.cpp,
//           passing your existing ez::Drive chassis object:
//
//             cstm_move_init(chassis);
//
//   Step 2: Declare two pros::Distance sensors — one on the front of the
//           robot and one on the left side — and call driveWallTrack():
//
//             pros::Distance frontDist(PORT_A);
//             pros::Distance leftDist(PORT_B);
//             driveWallTrack(frontDist, leftDist, 6.0, 4.0);
//               → drives forward until front sensor reads ≤ 6 inches,
//                 maintaining 4 inches from the left wall throughout.
//
// MOTOR CONTROL SCALE:
//   All speed values use EZ-Template's -127 to 127 scale (not raw millivolts).
//   ez::Drive::drive_set(left, right) is called each 10ms control cycle.
//
// NO EXTERN DECLARATIONS:
//   The ez::Drive reference is stored as a pointer at runtime inside
//   cstm_move_init(), avoiding the PROS cold/hot package linker constraint.

#include "LightLib/main.h"  // PROS umbrella — provides pros::Distance, pros::millis, etc.
#include "EZ-Template/api.hpp" // provides ez::Drive

// driveWallTrack — drives the robot forward while:
//   (1) stopping when the front distance sensor reads ≤ stopDistIn from a wall ahead
//   (2) maintaining targetDistIn from a left-side wall via a PD steering loop
//
// cstm_move_init() must be called before this function is used.
//
// Parameters:
//   frontSensor  : pros::Distance sensor mounted on the front of the robot.
//                  Stop condition: reading drops to or below stopDistIn.
//   leftSensor   : pros::Distance sensor mounted on the left side of the robot.
//                  Used for wall-tracking PD correction to maintain targetDistIn.
//   stopDistIn   : front distance threshold in inches — robot stops when front
//                  sensor reads ≤ this value (i.e., a wall is this close ahead).
//   targetDistIn : desired lateral distance to maintain from the left wall, in inches.
//   baseSpeed    : forward drive speed, 0–127 EZ-Template scale  (default: 80)
//   timeout      : max run duration in milliseconds before forced exit (default: 5000)
void cstm_move_init(ez::Drive& chassis);

// Pass nullptr for a sensor whose port is 0 (not installed).
// WallRide returns immediately if frontSensor is nullptr.
// If leftSensor is nullptr, the robot drives straight (no wall correction).
void WallRide(pros::Distance* frontSensor,
              pros::Distance* leftSensor,
              double stopDistIn,
              double targetDistIn,
              int  baseSpeed = 80,
              int  timeout   = 5000);
