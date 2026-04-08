#pragma once
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/motor_group.hpp"
#include <cmath>

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

// ─── OdomSensors ─────────────────────────────────────────────────────────────
struct OdomSensors {
    TrackingWheel* vertical1;
    TrackingWheel* vertical2;
    TrackingWheel* horizontal1;
    TrackingWheel* horizontal2;
    pros::Imu*     imu;
    pros::Imu*     imu2; // optional second IMU — pass nullptr if not used

    OdomSensors(TrackingWheel* v1, TrackingWheel* v2,
                TrackingWheel* h1, TrackingWheel* h2,
                pros::Imu* imu, pros::Imu* imu2 = nullptr)
        : vertical1(v1), vertical2(v2),
          horizontal1(h1), horizontal2(h2),
          imu(imu), imu2(imu2) {}
};

// ─── CactusOdom ──────────────────────────────────────────────────────────────
namespace light {
    void reset();
    void init(OdomSensors sensors);
    void stop();
    void moveToPoint(float targetX, float targetY, int timeout, float maxSpeed, bool reversed);
    Pose getPose(bool radians = false);
    void setPose(Pose pose, bool radians = false);

    Pose getSpeed(bool radians = false);
    Pose getLocalSpeed(bool radians = false);
    Pose estimatePose(float time, bool radians = false);

    void update();
}