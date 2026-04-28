#pragma once
#include "LightLib/mcl_config.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/motor_group.hpp"
#include "pros/gps.hpp"
#include "pros/distance.hpp"
#include <cmath>
#include <vector>

// ─── Pose ────────────────────────────────────────────────────────────────────
struct Pose {
    float x, y, theta;
    Pose(float x = 0, float y = 0, float theta = 0) : x(x), y(y), theta(theta) {}
    Pose operator*(float scalar) const { return {x * scalar, y * scalar, theta * scalar}; }
    Pose operator+(const Pose& o) const { return {x + o.x, y + o.y, theta + o.theta}; }
    Pose operator-(const Pose& o) const { return {x - o.x, y - o.y, theta - o.theta}; }
};

// ─── TrackingWheel ───────────────────────────────────────────────────────────
class TrackingWheel {
public:
    // Rotation sensor (unpowered — preferred)
    TrackingWheel(pros::Rotation* sensor, float wheelDiam, float offset, float tpr = 36000.0f)
        : rotSensor(sensor), motorGroup(nullptr),
          wheelCircumference(wheelDiam * M_PI), offset(offset), tpr(tpr), powered(false) {}

    // Motor group (powered — blue cart default)
    TrackingWheel(pros::MotorGroup* motors, float wheelDiam, float offset, float tpr = 360.0f)
        : rotSensor(nullptr), motorGroup(motors),
          wheelCircumference(wheelDiam * M_PI), offset(offset), tpr(tpr), powered(true) {}

    float getDistanceTraveled() const {
        if (rotSensor)  return (rotSensor->get_position()  / tpr) * wheelCircumference;
        if (motorGroup) return (motorGroup->get_position() / tpr) * wheelCircumference;
        return 0.0f;
    }

    float getOffset() const { return offset; }
    bool  isPowered() const { return powered; }

    void reset() {
        if (rotSensor)  rotSensor->reset_position();
        if (motorGroup) motorGroup->tare_position();
    }

private:
    pros::Rotation*   rotSensor;
    pros::MotorGroup* motorGroup;
    float wheelCircumference;
    float offset;
    float tpr;
    bool  powered;
};

// ─── DistanceSensorSpec ──────────────────────────────────────────────────────
// Per-sensor mount configuration for LightCast's ray-cast sensor model.
// offsetX/Y is the sensor body's position in robot frame (inches, center at 0).
// angleRad is the ray direction in robot frame: 0 = forward (+Y), CCW positive.
// Defined in `light` (not `light::lightcast`) to keep odom.hpp independent
// of lightcast.hpp.
struct DistanceSensorSpec {
    pros::Distance* sensor;
    float offsetX;
    float offsetY;
    float angleRad;
};

// ─── OdomSensors ─────────────────────────────────────────────────────────────
struct OdomSensors {
    TrackingWheel* vertical1;
    TrackingWheel* vertical2;
    TrackingWheel* horizontal1;
    TrackingWheel* horizontal2;
    pros::Imu*     imu;
    pros::Imu*     imu2;         // optional second IMU — nullptr if not used

    // Optional absolute-position sensors. If gps == nullptr, EKF skips the GPS
    // update step entirely and runs on wheel+IMU alone — no regression vs prior.
    pros::Gps*     gps;
    float          gpsOffsetX;   // meters, mount position in robot frame
    float          gpsOffsetY;

    // Optional distance sensors for LightCast. Empty vector = LightCast
    // inactive, system runs in EKF-only mode (still works — just no hybrid
    // snap on divergence).
    std::vector<DistanceSensorSpec> distanceSensors;

    OdomSensors(TrackingWheel* v1, TrackingWheel* v2,
                TrackingWheel* h1, TrackingWheel* h2,
                pros::Imu* imu, pros::Imu* imu2 = nullptr)
        : vertical1(v1), vertical2(v2),
          horizontal1(h1), horizontal2(h2),
          imu(imu), imu2(imu2),
          gps(nullptr), gpsOffsetX(0.0f), gpsOffsetY(0.0f) {}

    OdomSensors(TrackingWheel* v1, TrackingWheel* v2,
                TrackingWheel* h1, TrackingWheel* h2,
                pros::Imu* imu, pros::Imu* imu2,
                pros::Gps* gps, float gpsOffsetX, float gpsOffsetY,
                std::vector<DistanceSensorSpec> distanceSensors)
        : vertical1(v1), vertical2(v2),
          horizontal1(h1), horizontal2(h2),
          imu(imu), imu2(imu2),
          gps(gps), gpsOffsetX(gpsOffsetX), gpsOffsetY(gpsOffsetY),
          distanceSensors(std::move(distanceSensors)) {}
};

// ─── CactusOdom ──────────────────────────────────────────────────────────────
namespace light {
    void reset();
    void init(OdomSensors sensors, MCLConfig cfg = {});
    void stop();
    void moveToPoint(float targetX, float targetY, int timeout, float maxSpeed, bool reversed);
    Pose getPose(bool radians = false);
    void setPose(Pose pose, bool radians = false);

    Pose getSpeed(bool radians = false);
    Pose getLocalSpeed(bool radians = false);
    Pose estimatePose(float time, bool radians = false);

    void update();

    // Radian-only pose. RAMSETE and trajectory code MUST use this, never
    // getPose(false). Double-conversion between degrees and radians is the
    // top bug source in ports of this algorithm — centralizing the one
    // radian entry point avoids it.
    inline Pose getPoseRad() { return getPose(true); }
}