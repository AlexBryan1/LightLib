#pragma once

#include "EZ-Template/api.hpp"
#include "LightLib/api.h"
#include "LightLib/odom.hpp"
#include "pros/motor_group.hpp"


void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line);
void ez_screen_task();
void ez_template_extras();
void checkMotorTemp(pros::Controller& controller, pros::Motor& Top, pros::Motor& Bottom);
void default_positions();
void track_basket();

// Register the user's drivetrain objects with LightLib.  Must be called once
// in initialize() before ez_screen_task / ez_template_extras / moveToPoint are
// used.  Takes pointers rather than externing globals so LightLib can live in
// the PROS cold package without linker errors.
namespace light {
    void ez_extra_init(ez::Drive* chassis,
                       pros::MotorGroup* leftMotors,
                       pros::MotorGroup* rightMotors);

    // Accessors used by RAMSETE/characterization so they command the same
    // motor groups the user registered via ez_extra_init. Returns nullptr
    // through the out-params if ez_extra_init hasn't been called.
    void getDriveMotorGroups(pros::MotorGroup** leftOut,
                             pros::MotorGroup** rightOut);
    ez::Drive* getChassis();
}


// Simple PID controller struct
struct LightPID {
    float kP, kI, kD;
    float prevError = 0;
    float integral  = 0;

    LightPID(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}

    float update(float error) {
        integral += error;
        float derivative = error - prevError;
        prevError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    void reset() { prevError = 0; integral = 0; }
};

// Move to a field-relative (x, y) point
// timeout: milliseconds before giving up
// maxSpeed: 0-127
// reversed: drive to the point backwards
void moveToPoint(float x, float y, int timeout,
                 float maxSpeed = 127.0f, bool reversed = false);