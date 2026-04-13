#include "LightLib/main.h"
#include "pros/motors.h"
#include "LightLib/odom.hpp"
#include "subsystems.hpp"
#include "LightLib/odom.hpp"
#include "pros/rtos.hpp"
#include "LightLib/ez_extra.hpp"
#include <cmath>
#include <algorithm>
#include <atomic>
#define MOTOR_TEMP_THRESHOLD 55.0 // Celsius — adjust as needed

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }

}



void checkMotorTemp(pros::Controller& controller, pros::Motor& Top, pros::Motor& Bottom) {
    if (Top.get_temperature() >= MOTOR_TEMP_THRESHOLD || 
        Bottom.get_temperature() >= MOTOR_TEMP_THRESHOLD) {
        controller.rumble(". . .");
        pros::delay(2000);
    }
}

void turret_reset(){
    turret.move_absolute(0, 127);
    turret.move(-20);
    pros::delay(300);
    turret.move(0);
    turret.tare_position();
}


// Motors — must match your robot_config.cpp declarations
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

void light::moveToPoint(float targetX, float targetY, int timeout, float maxSpeed, bool reversed) {

    // TODO: tune these PID constants for your robot
    LightPID linearPID (9.0f, 0.0f, 125.0f);  // distance to target (125% drive pid is good start)
    LightPID angularPID( 6.0f, 0.0f, 50.0f);  // heading error ( swing pid is good start)

    // Exit conditions
    const float LINEAR_EXIT  = 0.2f;  // inches — close enough to target

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < (uint32_t)timeout) {
        Pose pose = light::getPose(); // degrees, inches

        float dx = targetX - pose.x;
        float dy = targetY - pose.y;
        float distance = sqrtf(dx * dx + dy * dy);

        // Done?
        if (distance < LINEAR_EXIT) break;

        // Angle to target in degrees
        float angleToTarget = atan2f(dx, dy) * 180.0f / M_PI;
        if (reversed) angleToTarget += 180.0f;

        // Shortest angular error (-180 to 180)
        float angularError = angleToTarget - pose.theta;
        while (angularError >  180.0f) angularError -= 360.0f;
        while (angularError < -180.0f) angularError += 360.0f;

        // Slow linear movement when facing far off target
        // (avoids driving sideways at full speed)
        float linearScale = cosf(angularError * M_PI / 180.0f);
        float linearError = distance * linearScale;

        float linearPower  = linearPID.update(linearError);
        float angularPower = angularPID.update(angularError);

        if (reversed) linearPower = -linearPower;

        // Clamp linear, then mix
        linearPower = std::clamp(linearPower, -maxSpeed, maxSpeed);

        float leftPower  = linearPower + angularPower;
        float rightPower = linearPower - angularPower;

        // Scale down if either side exceeds maxSpeed
        float maxOut = std::max(std::abs(leftPower), std::abs(rightPower));
        if (maxOut > maxSpeed) {
            leftPower  = leftPower  / maxOut * maxSpeed;
            rightPower = rightPower / maxOut * maxSpeed;
        }

        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        pros::delay(10);
    }

    leftMotors.move(0);
    rightMotors.move(0);
}