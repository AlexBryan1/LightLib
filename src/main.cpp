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

  // ── TEMP: IMU yaw-scale measurement — REMOVE once the scaler is set ──────────
  // With the robot disabled, hand-spin it exactly 10 full turns (3600°) and read
  // `raw` (either the controller line or the `pros terminal` log). Then set
  //   scaler = 3600 / raw   in default_constants() (autons.cpp), and delete this.
  // `raw` is the port-4 IMU un-scaled; `scaled` reflects the current scaler.
  static pros::Task imu_scale_readout([] {
    while (true) {
      printf("[IMU-SCALE] raw=%.1f  scaled=%.1f  pose.theta=%.1f\n",
             imu.get_rotation(), chassis.drive_imu_get(), light::getPose().theta);
      master.print(2, 0, "raw %.0f th %.0f   ",
                   imu.get_rotation(), light::getPose().theta);
      pros::delay(300);
    }
  });
}

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       AUTONOMOUS HOOK                                   │
// │  Called at the start of autonomous(), before the selected routine runs. │
// │  Use this to set allianceColor or do any pre-auton setup.               │
// └─────────────────────────────────────────────────────────────────────────┘
void user_autonomous() {
}

// ── Char: IMU scale ──────────────────────────────────────────────────────────
// Spins in place 10 full turns (ground truth = drive encoders), then reports
// each IMU's raw rotation and the suggested drive_imu_scaler on the controller
// (lines 0/1 — line 2 is owned by the [IMU-SCALE] readout task above).
// Defined here (not autons.cpp) because `imu`, `imu2`, and IMU2_PORT only
// exist in this translation unit.
void char_imu_scale() {
  constexpr double kTargetDeg = 3600.0;  // 10 full turns
  constexpr int kSpinPower = 45;         // slow — minimize wheel slip
  constexpr uint32_t kSpinTimeoutMs = 60000;
  constexpr uint32_t kReportMs = 60000;

  double trackW = chassis.drive_width_get();
  if (trackW < 0.1) trackW = 11.5;  // same fallback as autotune_heading.cpp

  chassis.drive_imu_reset(0);
  chassis.drive_sensor_reset();
  pros::delay(100);

  double imu1_0 = imu.get_rotation();
#if IMU2_PORT != 0
  double imu2_0 = imu2.get_rotation();
#endif

  // Encoder-derived rotation: theta_rad = (L - R) / trackWidth (CW positive,
  // matching the IMU convention — same arc relation as odometry.cpp).
  auto encDeg = [&]() -> double {
    return (chassis.drive_sensor_left() - chassis.drive_sensor_right()) /
           trackW * (180.0 / M_PI);
  };

  chassis.drive_set(kSpinPower, -kSpinPower);  // spin clockwise
  uint32_t t0 = pros::millis();
  while (encDeg() < kTargetDeg && pros::millis() - t0 < kSpinTimeoutMs) {
    pros::delay(10);
  }
  chassis.drive_set(0, 0);
  pros::delay(700);  // settle; let the IMU catch up

  double enc = encDeg();
  double d1 = imu.get_rotation() - imu1_0;
#if IMU2_PORT != 0
  double d2 = imu2.get_rotation() - imu2_0;
  double fused = (d1 + d2) / 2.0;
#else
  double d2 = 0.0;
  double fused = d1;
#endif
  // The IMUs are trusted as ground truth (they track real yaw ≈1:1). `factor`
  // = how much the drive encoders under/over-read vs truth this run. The number
  // to COPY into chassis.drive_ratio_set() is the current ratio times factor,
  // so it's the correct absolute value whatever ratio is already applied (after
  // a good fix a re-run reports the same ratio and factor≈1.0).
  double factor = std::fabs(fused) > 1.0 ? enc / fused : 0.0;
  double suggestedRatio = chassis.drive_ratio_get() * factor;

  printf("[IMU-CHAR] enc=%.1f deg  imu1=%.1f  imu2=%.1f  fused=%.1f  "
         "factor=%.4f  set drive_ratio=%.4f (current=%.4f)\n",
         enc, d1, d2, fused, factor, suggestedRatio, chassis.drive_ratio_get());

  // Hold the results on the controller so they're readable without a screen.
  master.rumble("-");
  uint32_t tr = pros::millis();
  while (pros::millis() - tr < kReportMs) {
    master.print(0, 0, "1:%5.0f 2:%5.0f   ", d1, d2);
    pros::delay(150);
    master.print(1, 0, "E:%4.0f ratio:%.3f", enc, suggestedRatio);
    pros::delay(150);
  }
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
