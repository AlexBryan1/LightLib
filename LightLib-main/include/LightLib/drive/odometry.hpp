#pragma once
#include <cmath>
#include <vector>

#include "LightLib/drive/mcl_config.hpp"
#include "light/distance.hpp"
#include "light/gps.hpp"
#include "light/imu.hpp"
#include "light/motor_group.hpp"
#include "light/rotation.hpp"

/**
 * \file odometry.hpp
 * Pose, sensor specs, and the global odometry / pose-estimation API.
 * Fuses wheel-tracking deltas + IMU(s) + optional GPS + optional LightCast.
 */

/** Field-frame 2D pose. Theta is in degrees by default; use the `radians`
 *  flag on `getPose()` / `setPose()` to switch. */
struct Pose {
  float x;     ///< X position, inches.
  float y;     ///< Y position, inches.
  float theta; ///< Heading.
  Pose(float x = 0, float y = 0, float theta = 0) : x(x), y(y), theta(theta) {}
  Pose operator*(float scalar) const { return {x * scalar, y * scalar, theta * scalar}; }
  Pose operator+(const Pose& o) const { return {x + o.x, y + o.y, theta + o.theta}; }
  Pose operator-(const Pose& o) const { return {x - o.x, y - o.y, theta - o.theta}; }
};

/**
 * Single tracking wheel — works with either a Rotation sensor (preferred)
 * or a powered MotorGroup.
 */
class TrackingWheel {
 public:
  /**
   * Construct from an unpowered Rotation sensor.
   *
   * \param sensor
   *        rotation sensor (pointer, not owned)
   * \param wheelDiam
   *        wheel diameter, inches
   * \param offset
   *        signed perpendicular distance from robot center, inches
   * \param tpr
   *        ticks per revolution (default 36000 for Rotation sensor)
   */
  TrackingWheel(pros::Rotation* sensor, float wheelDiam, float offset, float tpr = 36000.0f)
      : rotSensor(sensor), motorGroup(nullptr), wheelCircumference(wheelDiam * M_PI), offset(offset), tpr(tpr), powered(false) {}

  /**
   * Construct from a powered MotorGroup (blue-cart default).
   *
   * \param motors
   *        motor group (pointer, not owned)
   * \param wheelDiam
   *        wheel diameter, inches
   * \param offset
   *        signed perpendicular distance from robot center, inches
   * \param tpr
   *        ticks per revolution (default 360 for blue cart)
   */
  TrackingWheel(pros::MotorGroup* motors, float wheelDiam, float offset, float tpr = 360.0f)
      : rotSensor(nullptr), motorGroup(motors), wheelCircumference(wheelDiam * M_PI), offset(offset), tpr(tpr), powered(true) {}

  /** \return Distance traveled since last reset, inches. */
  float getDistanceTraveled() const {
    if (rotSensor) return (rotSensor->get_position() / tpr) * wheelCircumference;
    if (motorGroup) return (motorGroup->get_position() / tpr) * wheelCircumference;
    return 0.0f;
  }

  /** \return Signed perpendicular offset from robot center, inches. */
  float getOffset() const { return offset; }

  /** \return true if backed by a powered motor group. */
  bool isPowered() const { return powered; }

  /** Zero the encoder. */
  void reset() {
    if (rotSensor) rotSensor->reset_position();
    if (motorGroup) motorGroup->tare_position();
  }

 private:
  pros::Rotation* rotSensor;
  pros::MotorGroup* motorGroup;
  float wheelCircumference;
  float offset;
  float tpr;
  bool powered;
};

/**
 * Distance sensor mount for LightCast. offsetX/Y in inches, robot frame;
 * angleRad = ray direction (0 = +Y, CCW+). In `light` (not `light::lightcast`)
 * to keep odometry.hpp independent of lightcast.hpp.
 */
struct DistanceSensorSpec {
  pros::Distance* sensor; ///< Backing PROS distance sensor (not owned).
  float offsetX;          ///< Mount X in robot frame, inches.
  float offsetY;          ///< Mount Y in robot frame, inches.
  float angleRad;         ///< Ray direction in robot frame, radians.
  /// nullptr falls back to perimeter raycast; lightcast::fromFace binds the
  /// matching face function from src/maps/.
  float (*raycastFn)(float, float, float, float) = nullptr;
};

/** Bundle of all sensors used by the LightLib pose estimator. */
struct OdomSensors {
  TrackingWheel* vertical1;   ///< Primary vertical (forward-axis) tracking wheel.
  TrackingWheel* vertical2;   ///< Secondary vertical tracking wheel (or nullptr).
  TrackingWheel* horizontal1; ///< Primary horizontal (lateral) tracking wheel.
  TrackingWheel* horizontal2; ///< Secondary horizontal tracking wheel (or nullptr).
  pros::Imu* imu;             ///< Primary IMU.
  pros::Imu* imu2;            ///< Optional second IMU; nullptr if not used.

  /// nullptr → EKF skips the GPS update step.
  pros::Gps* gps;
  float gpsOffsetX;  ///< meters, robot frame
  float gpsOffsetY;  ///< meters, robot frame

  /// Empty → LightCast inactive; EKF-only mode.
  std::vector<DistanceSensorSpec> distanceSensors;

  /** Wheel + IMU only constructor. */
  OdomSensors(TrackingWheel* v1, TrackingWheel* v2,
              TrackingWheel* h1, TrackingWheel* h2,
              pros::Imu* imu, pros::Imu* imu2 = nullptr)
      : vertical1(v1), vertical2(v2), horizontal1(h1), horizontal2(h2), imu(imu), imu2(imu2), gps(nullptr), gpsOffsetX(0.0f), gpsOffsetY(0.0f) {}

  /** Full constructor with GPS and distance sensors. */
  OdomSensors(TrackingWheel* v1, TrackingWheel* v2,
              TrackingWheel* h1, TrackingWheel* h2,
              pros::Imu* imu, pros::Imu* imu2,
              pros::Gps* gps, float gpsOffsetX, float gpsOffsetY,
              std::vector<DistanceSensorSpec> distanceSensors)
      : vertical1(v1), vertical2(v2), horizontal1(h1), horizontal2(h2), imu(imu), imu2(imu2), gps(gps), gpsOffsetX(gpsOffsetX), gpsOffsetY(gpsOffsetY), distanceSensors(std::move(distanceSensors)) {}
};

/** Initialize once via init(); poll getPose/setPose from any task. 10 ms tick. */
namespace light {

/** Reset all internal pose state to (0, 0, 0). */
void reset();

/**
 * Initialize the estimator.
 *
 * \param sensors
 *        sensor bundle (copied)
 * \param cfg
 *        MCL configuration; defaults are usually fine
 */
void init(OdomSensors sensors, MCLConfig cfg = {});

/** Stop the background odometry task. */
void stop();

/** Drive to a field-relative point (legacy). Inches; speed cap 0..127. */
void moveToPoint(float targetX, float targetY, int timeout, float maxSpeed, bool reversed);

/** \return Current fused pose. \param radians true for radians, false for degrees. */
Pose getPose(bool radians = false);

/** Overwrite the current pose. */
void setPose(Pose pose, bool radians = false);

/** \return Field-frame velocities (in/s, deg/s by default). */
Pose getSpeed(bool radians = false);

/** \return Robot-frame velocities (forward / strafe / yaw). */
Pose getLocalSpeed(bool radians = false);

/** Linear extrapolation of pose `time` seconds ahead at current velocity. */
Pose estimatePose(float time, bool radians = false);

/** Force one update (normally called by the background task). */
void update();

/** Radian-only accessor; trajectory code MUST use this to avoid deg/rad bugs. */
inline Pose getPoseRad() { return getPose(true); }
}  // namespace light
