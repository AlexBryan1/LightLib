// WallRide: drive forward while a side distance sensor PD-tracks a wall;
// front sensor triggers stop. Speeds are EZ-Template's −127..127 scale.

#include "LightLib/drive/custom_move.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

// Registered via custom_move_init() — LightLib lives in the cold package and
// can't reference hot externs directly.
static light::Drive* g_chassis = nullptr;

// PD gain units: speed-units per mm of lateral error (EZ-Template −127..127).
static constexpr double WALL_KP = 0.5;  // TODO: tune for your robot
static constexpr double WALL_KD = 0.1;  // TODO: tune for your robot

static constexpr double WALL_KP_DEFAULT = 0.5;
static constexpr double WALL_KD_DEFAULT = 0.1;

// pros::Distance::get() returns PROS_ERR (~UINT32_MAX) when nothing detected.
static constexpr double SENSOR_MAX_VALID_MM = 2000.0;

void custom_move_init(light::Drive& chassis) {
  g_chassis = &chassis;
}

void WallRide(pros::Distance* frontSensor,
              pros::Distance* leftSensor,
              double stopDistIn,
              double targetDistIn,
              int baseSpeed,
              int timeout) {
  if (g_chassis == nullptr) return;
  if (frontSensor == nullptr) return;

  static bool warned_defaults = false;
  if (!warned_defaults &&
      WALL_KP == WALL_KP_DEFAULT && WALL_KD == WALL_KD_DEFAULT) {
    warned_defaults = true;
    printf("[WallRide] using default WALL_KP=%.3f WALL_KD=%.3f — tune in custom_move.cpp\n",
           WALL_KP, WALL_KD);
  }

  const double stopDistMM = stopDistIn * 25.4;
  const double targetDistMM = targetDistIn * 25.4;

  double prevWallError = 0.0;
  const uint32_t startTime = pros::millis();

  while (pros::millis() - startTime < static_cast<uint32_t>(timeout)) {
    double frontMM = static_cast<double>(frontSensor->get());

    if (frontMM < SENSOR_MAX_VALID_MM && frontMM <= stopDistMM) break;

    double wallError = 0.0;

    if (leftSensor != nullptr) {
      double leftMM = static_cast<double>(leftSensor->get());
      if (leftMM < SENSOR_MAX_VALID_MM) {
        wallError = targetDistMM - leftMM;
      }
    }

    double derivative = wallError - prevWallError;
    prevWallError = wallError;

    const int corrLimit = std::max(1, (baseSpeed * 6) / 10);
    int correction = static_cast<int>(WALL_KP * wallError + WALL_KD * derivative);
    correction = std::clamp(correction, -corrLimit, corrLimit);

    // Left-side sensor: wallError > 0 (too far) → arc left = slow left, fast right.
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed = std::clamp(leftSpeed, 0, 127);
    rightSpeed = std::clamp(rightSpeed, 0, 127);

    g_chassis->drive_set(leftSpeed, rightSpeed);

    pros::delay(10);
  }

  g_chassis->drive_set(0, 0);
}