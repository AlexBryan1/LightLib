// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       DRIVETRAIN CONFIGURATION                          │
// │  Set your motor ports, wheel size, and sensor ports below.              │
// │  Use a negative port number to reverse that motor.                      │
// └─────────────────────────────────────────────────────────────────────────┘

#include <cmath>

#include "subsystems.hpp"
#define LEFT_PORTS {-15, 14, -13, 12, -3}  // left drive motors
#define RIGHT_PORTS {16, -17, 18, -19, 8}     // right drive motors

#define IMU_PORT 4   // primary inertial sensor
#define IMU2_PORT 7  // second IMU — set to 0 if you only have one

#define WHEEL_DIAMETER 3.25  // inches  (4" wheels are actually ~4.125")
#define WHEEL_RPM 450       // motor cartridge RPM × (motor sprocket / wheel sprocket)
#define TRACK_HALF_W 8.0f   // half of robot track width in inches (used for odometry)

// ── Tracking wheels — 0, 1, or 2 vertical and 0, 1, or 2 horizontal ──────────
//  PORT:    VEX rotation-sensor port (0 = not installed → falls back to drive
//           motors for vertical, no strafe tracking for horizontal)
//  OFFSET:  signed perpendicular distance from robot center, inches

#define VERT_WHEEL_DIA   2.75
#define VERT_1_PORT      6
#define VERT_1_OFFSET    0.0
#define VERT_2_PORT      0
#define VERT_2_OFFSET    0.0

#define HORIZ_WHEEL_DIA  2.75
#define HORIZ_1_PORT     0
#define HORIZ_1_OFFSET   0.0
#define HORIZ_2_PORT     0
#define HORIZ_2_OFFSET   0.0

// ── GPS sensor (set port to 0 if not installed) ──────────────────────────────
//  OFFSET_X / OFFSET_Y: GPS mount position relative to robot center, inches
#define GPS_PORT      0
#define GPS_OFFSET_X  0.0
#define GPS_OFFSET_Y  0.0

#define JOYSTICK_CURVE 0.0f   // expo curve strength (0 = linear, higher = more curve)
#define JOYSTICK_DEADZONE 10  // joystick values ±this are treated as 0 (0–127)

// ── Thermal buzz ──────────────────────────────────────────────────────────────
#define HEAT_BUZZ_ENABLED false // set to false to disable the temperature warning
#define HEAT_BUZZ_TEMP 55       // °C threshold that triggers the controller buzz

// ── Drive style ───────────────────────────────────────────────────────────────
//  1  Arcade        left stick = throttle,  right stick = turn
//  2  Tank          left stick = left side, right stick = right side
//  3  Single stick  left stick = throttle + turn (one hand)
//  4  X/Mecanum     left stick = move,      right stick = turn  (needs holoDrive in subsystems.hpp)
//  5  H-Drive       tank sides + strafe,    right stick = turn  (needs hDrive   in subsystems.hpp)
#define DRIVE_TYPE 1

// ── MCL distance sensors — one per side (set port to 0 if not installed) ─────
// just type where the sensor sits on the robot. X/Y in inches (robot frame),
// ANGLE is which way it points in degrees. 0 = forward, 90 = left, 180 = back,
// -90 = right. diagnoal mounts are fine, MCL uses the generic perimiter raycast.

#define MCL_FRONT_PORT  0
#define MCL_FRONT_X     0.0f
#define MCL_FRONT_Y     6.0f
#define MCL_FRONT_ANGLE 0.0f      // forward

#define MCL_BACK_PORT  0
#define MCL_BACK_X     0.0f
#define MCL_BACK_Y    -6.0f
#define MCL_BACK_ANGLE 180.0f

#define MCL_LEFT_PORT 0
#define MCL_LEFT_X  -6.0f
#define MCL_LEFT_Y   0.0f
#define MCL_LEFT_ANGLE  90.0f

#define MCL_RIGHT_PORT   0
#define MCL_RIGHT_X      6.0f
#define MCL_RIGHT_Y      0.0f
#define MCL_RIGHT_ANGLE -90.0f

// MCL/EKF tuning lives in default_constants() in autons.cpp.

#include "LightLib/robot_impl.inl"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       INITIALIZATION HOOK                               │
// │  Called at the end of initialize() — add any extra setup here.          │
// │  The chassis, IMU, and auton selector are already ready by this point.  │
// └─────────────────────────────────────────────────────────────────────────┘
void user_initialize() {
  // Hold both axes so the arm doesn't drop under gravity, then zero sensors.
  CascadeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  ArmMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  LiftArm.zero_sensors();
}

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       AUTONOMOUS HOOK                                   │
// │  Called at the start of autonomous(), before the selected routine runs. │
// │  Use this to set allianceColor or do any pre-auton setup.               │
// └─────────────────────────────────────────────────────────────────────────┘
void user_autonomous() {
}

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       OPERATOR CONTROL                                  │
// │  Map your buttons and subsystems here.                                  │
// │  Motors and pistons are declared in subsystems.hpp.                     │
// └─────────────────────────────────────────────────────────────────────────┘
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  // Hard-stop detection for the Score park-to-zero: if move_absolute keeps
  // commanding Score but it isn't moving, it's seated against a hard stop —
  // brake instead of stalling at full torque.
  constexpr double kScoreStallVel   = 5.0;  // RPM; below this = "not moving" (tune)
  constexpr int    kScoreStallTicks = 8;    // ~80ms of no motion => hard stop (tune)
  int scoreStallCount = 0;

  while (true) {
    auton_toggle();

    if (!auton_running) {
      _run_drive();  // style set by DRIVE_TYPE at the top of this file

      // ── Held buttons ──────────────────────────────────────────────
      if (master.get_digital(DIGITAL_R2))
        Bottom.move(-127);
      else if (master.get_digital(DIGITAL_R1))
        Bottom.move(127);
      else if (master.get_digital(DIGITAL_L2)) {
        Bottom.move(127);
        Score.move(-127);
        Hood.set(true);
        scoreStallCount = 0;  // operator drove Score — re-arm hard-stop detection
      }
      else if (master.get_digital(DIGITAL_L1))
        Bottom.move(-127);
      else {
        Bottom.move(0),
        Hood.set(false);
        if (scoreStallCount >= kScoreStallTicks) {
          Score.brake();  // seated against hard stop — stop pushing
        } else {
          Score.move_absolute(0, 127);
          if (std::fabs(Score.get_actual_velocity()) < kScoreStallVel)
            scoreStallCount++;
          else
            scoreStallCount = 0;  // still moving toward 0
        }
      }

      // ── Cascade lift + 2-bar arm ──────────────────────────────────
      // Presets (A/B = stow/score), joystick = arm jog, UP/DOWN = lift jog.
      constexpr float kPresetStow  = 4.0f;   // inches above floor — tune
      constexpr float kPresetScore = 30.0f;  // inches above floor — tune
      if (master.get_digital_new_press(DIGITAL_B))
        LiftArm.set_target_height(kPresetScore);
      else if (master.get_digital_new_press(DIGITAL_A))
        LiftArm.set_target_height(kPresetStow);

      int armJog_mV = master.get_analog(ANALOG_RIGHT_Y) * 94;  // 127 -> ~12000mV
      int cascadeJog_mV = 0;
      if (master.get_digital(DIGITAL_UP) && !ctrl_menu_open.load() && !ctrl_results_open.load()) cascadeJog_mV = 12000;
      else if (master.get_digital(DIGITAL_DOWN)) cascadeJog_mV = -12000;

      LiftArm.manual(cascadeJog_mV, armJog_mV);
      LiftArm.update();
    }

    // ── Toggle buttons ────────────────────────────────────────────────
    Lever.button_toggle(master.get_digital(DIGITAL_DOWN));
    Wings.button_toggle(master.get_digital(DIGITAL_B));
    Loader.button_toggle(master.get_digital(DIGITAL_Y));
    pros::delay(10);
  }
}
