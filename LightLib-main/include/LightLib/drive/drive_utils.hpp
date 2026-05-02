#pragma once
#include <algorithm>
#include <cmath>

#include "LightLib/drive/drive.hpp"
#include "LightLib/field_map.hpp"
#include "LightLib/odometry.hpp"
#include "LightLib/util.hpp"
#include "okapi/api/units/QLength.hpp"
#include "pros/distance.hpp"
#include "subsystems.hpp"

// ── drive_until_distance ──────────────────────────────────────────────────────

inline void drive_until_distance(okapi::QLength target,
                                 pros::Distance& sensor,
                                 int speed = 127,
                                 int timeout_ms = 10000) {
  int target_mm = (int)(target.convert(okapi::inch) * light::units::MM_PER_IN);
  // Drive forward or backward depending on speed sign.
  // 5000 is an arbitrarily large distance — the sensor triggers the real stop.
  int drive_dir = (speed >= 0) ? 5000 : -5000;
  chassis.pid_drive_set(drive_dir, std::abs(speed));

  int elapsed = 0;
  while (elapsed < timeout_ms) {
    int reading = sensor.get();
    if (reading != PROS_ERR && reading <= target_mm) {
      chassis.pid_drive_set(0_in, 0);
      return;
    }
    pros::delay(10);
    elapsed += 10;
  }
  chassis.pid_wait_quick_chain();
}

// ── drive_distance_reset ──────────────────────────────────────────────────────

inline void distance_reset(light::Drive& chassis,
                           pros::Distance& sensor,
                           double offset_fwd,
                           double offset_side,
                           bool fix_x,
                           bool faces_positive = false,
                           double field_size_in = light::field::FIELD_SIZE_IN) {
  int raw = sensor.get();
  if (raw == PROS_ERR) return;

  double heading_deg = chassis.drive_imu_get();
  double heading = heading_deg * light::units::RAD_PER_DEG;
  double dist_in = raw * light::units::IN_PER_MM;
  Pose cur = light::getPose();

  double projected_offset = fix_x
                                ? (offset_fwd * std::sin(heading) + offset_side * std::cos(heading))
                                : (offset_fwd * std::cos(heading) - offset_side * std::sin(heading));

  double center_to_wall = dist_in + projected_offset;
  double coord = faces_positive ? (field_size_in - center_to_wall)
                                : center_to_wall;

  if (fix_x)
    light::setPose(Pose(coord, cur.y, heading_deg));
  else
    light::setPose(Pose(cur.x, coord, heading_deg));
}

// ── drive_distance_reset ──────────────────────────────────────────────────────

inline void drive_distance_reset(light::Drive& chassis,
                                 double known_coord,
                                 bool fix_x) {
  double heading_deg = chassis.drive_imu_get();
  Pose cur = light::getPose();

  if (fix_x)
    light::setPose(Pose(known_coord, cur.y, heading_deg));
  else
    light::setPose(Pose(cur.x, known_coord, heading_deg));
}

// ── angle_reset ───────────────────────────────────────────────────────────────

inline void angle_reset(light::Drive& chassis,
                        pros::Distance& front_sensor,
                        pros::Distance& rear_sensor,
                        double separation) {
  int front_raw = front_sensor.get();
  int rear_raw = rear_sensor.get();
  if (front_raw == PROS_ERR || rear_raw == PROS_ERR) return;

  double angle_error_deg = std::atan2((rear_raw - front_raw) * light::units::IN_PER_MM, separation) * light::units::DEG_PER_RAD;
  chassis.drive_imu_reset(chassis.drive_imu_get() - angle_error_deg);
}