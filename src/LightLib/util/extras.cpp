#include "LightLib/util/extras.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

#include "LightLib/main.h"
#include "LightLib/drive/odometry.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

#define MOTOR_TEMP_THRESHOLD 55.0  // Celsius

// Registered via extras_init(): LightLib (cold package) can't reference
// hot-defined externs directly.
static light::Drive* g_chassis = nullptr;
static pros::MotorGroup* g_leftMotors = nullptr;
static pros::MotorGroup* g_rightMotors = nullptr;

void light::extras_init(light::Drive* chassis,
                        pros::MotorGroup* leftMotors,
                        pros::MotorGroup* rightMotors) {
  g_chassis = chassis;
  g_leftMotors = leftMotors;
  g_rightMotors = rightMotors;
}

void light::getDriveMotorGroups(pros::MotorGroup** leftOut,
                                pros::MotorGroup** rightOut) {
  if (leftOut) *leftOut = g_leftMotors;
  if (rightOut) *rightOut = g_rightMotors;
}

light::Drive* light::getChassis() { return g_chassis; }

void screen_print_tracker(light::tracking_wheel* tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  light::screen_print(tracker_value + tracker_width, line);
}

void screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (g_chassis && g_chassis->odom_enabled() && !g_chassis->pid_tuner_enabled()) {
        // Stack-buffer snprintf saves ~6 heap allocs per tick vs chained std::string +.
        char odom_buf[64];
        snprintf(odom_buf, sizeof(odom_buf), "x: %.2f\ny: %.2f\na: %.2f",
                 g_chassis->odom_x_get(),
                 g_chassis->odom_y_get(),
                 g_chassis->odom_theta_get());
        light::screen_print(std::string(odom_buf), 1);

        screen_print_tracker(g_chassis->odom_tracker_left, "l", 4);
        screen_print_tracker(g_chassis->odom_tracker_right, "r", 5);
        screen_print_tracker(g_chassis->odom_tracker_back, "b", 6);
        screen_print_tracker(g_chassis->odom_tracker_front, "f", 7);
      }
    }

    pros::delay(light::util::DELAY_TIME);
  }
}

void lib_extras() {
  if (!g_chassis) return;
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      g_chassis->pid_tuner_toggle();

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = g_chassis->drive_brake_get();
      g_chassis->drive_brake_set(preference);
    }

    g_chassis->pid_tuner_iterate();
  } else {
    if (g_chassis->pid_tuner_enabled())
      g_chassis->pid_tuner_disable();
  }
}

void checkMotorTemp(pros::Controller& controller, pros::Motor& Top, pros::Motor& Bottom) {
  if (Top.get_temperature() >= MOTOR_TEMP_THRESHOLD ||
      Bottom.get_temperature() >= MOTOR_TEMP_THRESHOLD) {
    controller.rumble(". . .");
    pros::delay(2000);
  }
}

void turret_reset() {
  turret.move_absolute(0, 127);
  turret.move(-20);  // seat against hard stop to remove backlash
  pros::delay(300);
  turret.move(0);
  turret.tare_position();
}

void light::moveToPoint(float targetX, float targetY, int timeout, float maxSpeed, bool reversed) {
  if (!g_leftMotors || !g_rightMotors) return;

  LightPID linearPID(9.0f, 0.0f, 125.0f);
  LightPID angularPID(6.0f, 0.0f, 50.0f);

  const float LINEAR_EXIT = 0.2f;  // inches

  uint32_t startTime = pros::millis();

  while (pros::millis() - startTime < (uint32_t)timeout) {
    Pose pose = light::getPose();

    float dx = targetX - pose.x;
    float dy = targetY - pose.y;
    float distance = sqrtf(dx * dx + dy * dy);

    if (distance < LINEAR_EXIT) break;

    float angleToTarget = atan2f(dx, dy) * 180.0f / M_PI;
    if (reversed) angleToTarget += 180.0f;

    float angularError = angleToTarget - pose.theta;
    while (angularError > 180.0f) angularError -= 360.0f;
    while (angularError < -180.0f) angularError += 360.0f;

    // Cosine scaling on linear: at 90° heading error drive becomes turn-in-place.
    float linearScale = cosf(angularError * M_PI / 180.0f);
    float linearError = distance * linearScale;

    float linearPower = linearPID.update(linearError);
    float angularPower = angularPID.update(angularError);

    if (reversed) linearPower = -linearPower;

    linearPower = std::clamp(linearPower, -maxSpeed, maxSpeed);

    float leftPower = linearPower + angularPower;
    float rightPower = linearPower - angularPower;

    // Proportional rescale preserves the steering ratio if a side saturates.
    float maxOut = std::max(std::abs(leftPower), std::abs(rightPower));
    if (maxOut > maxSpeed) {
      leftPower = leftPower / maxOut * maxSpeed;
      rightPower = rightPower / maxOut * maxSpeed;
    }

    g_leftMotors->move(leftPower);
    g_rightMotors->move(rightPower);

    pros::delay(10);
  }

  g_leftMotors->move(0);
  g_rightMotors->move(0);
}