#pragma once
#include "EZ-Template/drive/drive.hpp"
#include "pros/distance.hpp"
#include "LightLib/odom.hpp"
#include <cmath>
#include <algorithm>

// ── drive_until_distance ──────────────────────────────────────────────────────

inline void drive_until_distance(ez::Drive&      chassis,
                                 pros::Distance& sensor,
                                 double          target_in,
                                 int             speed      = 127,
                                 int             timeout_ms = 10000)
{
    int target_mm = (int)(target_in * 25.4);
    chassis.pid_drive_set(5000, speed);

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

inline void distance_reset(ez::Drive&      chassis,
                            pros::Distance& sensor,
                            double          offset_fwd,
                            double          offset_side,
                            bool            fix_x,
                            bool            faces_positive = false,
                            double          field_size_in  = 144.0)
{
    int raw = sensor.get();
    if (raw == PROS_ERR) return;

    double heading_deg = chassis.drive_imu_get();
    double heading     = heading_deg * M_PI / 180.0;
    double dist_in     = raw / 25.4;
    Pose   cur         = light::getPose();

    double projected_offset = fix_x
        ? (offset_fwd * std::sin(heading) + offset_side * std::cos(heading))
        : (offset_fwd * std::cos(heading) - offset_side * std::sin(heading));

    double center_to_wall = dist_in + projected_offset;
    double coord = faces_positive ? (field_size_in - center_to_wall)
                                  :  center_to_wall;

    if (fix_x)
        light::setPose(Pose(coord,  cur.y, heading_deg));
    else
        light::setPose(Pose(cur.x,  coord, heading_deg));
}

// ── drive_distance_reset ──────────────────────────────────────────────────────

inline void drive_distance_reset(ez::Drive& chassis,
                                  double     known_coord,
                                  bool       fix_x)
{
    double heading_deg = chassis.drive_imu_get();
    Pose   cur         = light::getPose();

    if (fix_x)
        light::setPose(Pose(known_coord, cur.y,   heading_deg));
    else
        light::setPose(Pose(cur.x,       known_coord, heading_deg));
}

// ── angle_reset ───────────────────────────────────────────────────────────────

inline void angle_reset(ez::Drive&      chassis,
                         pros::Distance& front_sensor,
                         pros::Distance& rear_sensor,
                         double          separation)
{
    int front_raw = front_sensor.get();
    int rear_raw  = rear_sensor.get();
    if (front_raw == PROS_ERR || rear_raw == PROS_ERR) return;

    double angle_error_deg = std::atan2((rear_raw - front_raw) / 25.4, separation) * 180.0 / M_PI;
    chassis.drive_imu_reset(chassis.drive_imu_get() - angle_error_deg);
}