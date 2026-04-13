#pragma once

#include "EZ-Template/api.hpp"
#include "LightLib/api.h"
#include "LightLib/odom.hpp"


void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line);
void ez_screen_task();
void ez_template_extras();
static void auton_task_fn(void*);
void auton_toggle();
void checkMotorTemp(pros::Controller& controller, pros::Motor& Top, pros::Motor& Bottom);
void default_positions();
void track_basket();


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