#pragma once
#include <algorithm>
#include <cmath>

#include "LightLib/drive/drive.hpp"
#include "LightLib/path/field_map.hpp"
#include "LightLib/drive/odometry.hpp"
#include "LightLib/util/util.hpp"
#include "okapi/api/units/QLength.hpp"
#include "pros/distance.hpp"
#include "subsystems.hpp"

/**
 * \file drive_utils.hpp
 *
 * Header-only drive helpers built on top of `light::Drive`:
 *   - drive_until_distance(): drive until a distance sensor reads under a
 *     threshold.
 *   - distance_reset(): re-zero one odom axis using a wall-facing sensor.
 *   - drive_distance_reset(): re-zero one odom axis from a known coordinate.
 *   - angle_reset(): correct heading from front+rear distance sensors on a
 *     parallel wall.
 */

/**
 * Drive forward (or reverse) until a distance sensor reads at or below
 * `target`.
 *
 * The PID is given an arbitrarily large 5000 in target — the sensor
 * triggers the real stop. Falls back to `pid_wait_quick_chain()` on timeout.
 *
 * \param target
 *        distance threshold (with units)
 * \param sensor
 *        distance sensor used for the stop trigger
 * \param speed
 *        signed drive speed, ±127. Negative = reverse.
 * \param timeout_ms
 *        bail after this many ms if the threshold is never crossed
 */
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

/**
 * Re-zero one odom axis (`x` or `y`) using a wall-facing distance sensor.
 *
 * Reads the sensor, projects its measurement through the current heading
 * to the axis of interest, and updates the odom pose with `light::setPose`.
 * Bails silently if the sensor returns `PROS_ERR`.
 *
 * \param chassis
 *        the drivetrain (used for IMU heading)
 * \param sensor
 *        wall-facing distance sensor
 * \param offset_fwd
 *        sensor offset in inches along the robot's forward axis
 * \param offset_side
 *        sensor offset in inches along the robot's lateral axis
 * \param fix_x
 *        true = correct X, false = correct Y
 * \param faces_positive
 *        true if the sensor faces the positive end of the field axis
 * \param field_size_in
 *        field size used to flip the coordinate when `faces_positive`
 */
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

/**
 * Re-zero one odom axis from a known field coordinate (e.g. when robot is
 * pinned against a wall whose coordinate is known a priori).
 *
 * \param chassis
 *        the drivetrain (used for IMU heading)
 * \param known_coord
 *        the known field-frame coordinate, inches
 * \param fix_x
 *        true = set X, false = set Y
 */
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

/**
 * Correct heading from front- and rear-mounted distance sensors looking at
 * the same parallel wall.
 *
 * The angular offset is `atan2(rear − front, separation)` and is applied to
 * the IMU's stored offset.
 *
 * \param chassis
 *        the drivetrain (IMU is reset on it)
 * \param front_sensor
 *        forward-mounted distance sensor
 * \param rear_sensor
 *        rear-mounted distance sensor on the same side
 * \param separation
 *        distance between the two sensors along the robot, inches
 */
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