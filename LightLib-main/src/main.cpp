#include "main.h"
#include <functional>
#include "autons.hpp"
#include "pros/motors.h"
#include "odom.hpp"
#include "custom_selector.hpp"
#include "cstm_move.hpp"
#include "ez_extra.hpp"
#include <atomic>

// ┌─────────────────────────────────────────────────────────────┐
// │                   ROBOT CONFIGURATION                       │
// │   Change your ports and settings here — nothing else needs  │
// │   to be touched to get the drivetrain working.              │
// └─────────────────────────────────────────────────────────────┘

// Drive motor ports — use a negative number to reverse that motor
#define LEFT_PORTS  {-10, -9, -8}
#define RIGHT_PORTS { 20, 19, 18}

#define IMU_PORT   21   // primary IMU
#define IMU2_PORT   0   // second IMU (set to 0 if you only have one)

#define WHEEL_DIAMETER 3.25   // inches  (4" wheels with screw holes are actually 4.125)
#define WHEEL_RPM      450    // motor cartridge RPM × (motor sprocket / wheel sprocket)
#define TRACK_HALF_W   3.0f   // half of your robot's track width, in inches (used for odometry)

#define JOYSTICK_CURVE 0.2f   // expo curve strength for driving (0 = linear)

// Distance sensor ports
#define DIST_LEFT_FRONT_PORT  11
#define DIST_LEFT_BACK_PORT   16
#define DIST_FRONT_PORT        5

// ─────────────────────────────────────────────────────────────────────────────
//  Everything below builds itself from the values above — no need to edit it
// ─────────────────────────────────────────────────────────────────────────────

ez::Drive chassis(LEFT_PORTS, RIGHT_PORTS, IMU_PORT, WHEEL_DIAMETER, WHEEL_RPM);

pros::MotorGroup leftMotors (LEFT_PORTS);
pros::MotorGroup rightMotors(RIGHT_PORTS);
pros::Imu imu(IMU_PORT);
pros::Imu imu2(IMU2_PORT);

TrackingWheel leftTracker (&leftMotors,  WHEEL_DIAMETER, -TRACK_HALF_W);
TrackingWheel rightTracker(&rightMotors, WHEEL_DIAMETER,  TRACK_HALF_W);

OdomSensors sensors(&leftTracker, &rightTracker, nullptr, nullptr, &imu, &imu2);

// Distance sensors — port 0 means "not installed"; pointer is nullptr in that case
#if DIST_LEFT_FRONT_PORT != 0
static pros::Distance _left_front_obj(DIST_LEFT_FRONT_PORT);
pros::Distance* left_front_sensor = &_left_front_obj;
#else
pros::Distance* left_front_sensor = nullptr;
#endif

#if DIST_LEFT_BACK_PORT != 0
static pros::Distance _left_back_obj(DIST_LEFT_BACK_PORT);
pros::Distance* left_back_sensor = &_left_back_obj;
pros::Distance* leftDist         = &_left_back_obj;  // alias used by WallRide
#else
pros::Distance* left_back_sensor = nullptr;
pros::Distance* leftDist         = nullptr;
#endif

#if DIST_FRONT_PORT != 0
static pros::Distance _front_obj(DIST_FRONT_PORT);
pros::Distance* frontDist = &_front_obj;
#else
pros::Distance* frontDist = nullptr;
#endif

static void temp_display_task(void*) {
    while (true) {
        double max_temp = 0.0;
        for (double t : leftMotors.get_temperature_all())  max_temp = std::max(max_temp, t);
        for (double t : rightMotors.get_temperature_all()) max_temp = std::max(max_temp, t);
        max_temp = std::max(max_temp, Top.get_temperature());
        max_temp = std::max(max_temp, Bottom.get_temperature());
        master.print(2, 8, "H:%3.0fC", max_temp);
        pros::delay(500);
    }
}

void initialize() {
  pros::delay(300);
    // Let EZ handle IMU calibration — don't call imu.reset() manually
  chassis.initialize(); // calibrates IMU internally
  pros::delay(300);
  chassis.drive_imu_reset();
  cstm_move_init(chassis);
    // Now init lightlib"s odom AFTER IMU is fully calibrated
  OdomSensors sensors(&leftTracker, &rightTracker, nullptr, nullptr, &imu, &imu2);
  light::init(sensors);

  default_constants();
  default_positions();

  
  
// Initialize chassis and auton selector
  light::pid_tuner.set_drive(&chassis); // your ez::Drive object
  light::pid_tuner.start_task();        // starts 25Hz graph sampler
  register_autons();
  light::auton_selector.init();

  static pros::Task temp_task(temp_display_task, nullptr, 3, TASK_STACK_DEPTH_DEFAULT, "Temp Display");

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  pros::delay(100);
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);

  pros::delay(10);        // let sensors settle at 0
  light::reset();     // sync prev* variables to the now-zeroed sensors
  light::setPose(Pose(0, 0, 0));

  uint32_t auton_start = pros::millis();
  light::auton_selector.run();
  uint32_t elapsed_ms = pros::millis() - auton_start;

  uint32_t secs = elapsed_ms / 1000;
  uint32_t ms   = elapsed_ms % 1000;
  master.print(0, 0, "Done: %lu.%03lus  ", secs, ms);

}


// ezScreenTask disabled — LightLib handles the display
// pros::Task ezScreenTask(ez_screen_task);

static std::atomic<bool> auton_running{false};
static pros::Task* auton_task = nullptr;


static void auton_task_fn(void*) {
  autonomous();
  auton_running.store(false);
}
void auton_toggle() {
  // Clean up task wrapper if auton finished naturally
  if (auton_task != nullptr && !auton_running.load()) {
    delete auton_task;
    auton_task = nullptr;
  }

  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    if (auton_task == nullptr) {
      // Start auton
      auton_running.store(true);
      auton_task = new pros::Task(auton_task_fn, nullptr, TASK_PRIORITY_DEFAULT,
                                  TASK_STACK_DEPTH_DEFAULT, "Auton Task");
    } else {
      // Kill auton
      auton_running.store(false);
      auton_task->remove();
      delete auton_task;
      auton_task = nullptr;
    }
  }
}

static float curve_lut[256]; // lookup table, index 0 = -127, index 255 = 127
void build_curve_lut() {
    for (int i = -127; i <= 127; i++) {
        float x = i / 127.0f;
        float curved = (std::exp(JOYSTICK_CURVE * std::abs(x)) - 1.0f) / (std::exp(JOYSTICK_CURVE) - 1.0f);
        curve_lut[i + 127] = std::copysign(curved * 127.0f, (float)i);
    }
    curve_lut[127] = 0.0f; // exact zero for zero input
}

inline float apply_curve(int raw) {
    return curve_lut[raw + 127];
}

void opcontrol() {
    build_curve_lut(); // compute once at start
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);

    bool wings_state      = false, wings_prev      = false;
    bool intakeLift_state = false, intakeLift_prev = false;
    bool doublePark_state = false, doublePark_prev = false;
    bool loader_state     = false, loader_prev     = false;

    while (true) {
        auton_toggle();

        if (!auton_running) {
            // Compute curves once, reuse for both motors
            float c_throttle = apply_curve(master.get_analog(ANALOG_LEFT_Y));
            float c_turn     = master.get_analog(ANALOG_RIGHT_X);

            static int last_left = 0, last_right = 0;

            int new_left  = std::clamp((int)(c_throttle + c_turn), -127, 127);
            int new_right = std::clamp((int)(c_throttle - c_turn), -127, 127);

            if (new_left  != last_left)  { chassis.drive_set(new_left, new_right);   last_left  = new_left;  }
            if (new_right != last_right) { chassis.drive_set(new_left, new_right); last_right = new_right; }


            if (master.get_digital(DIGITAL_R2))
                MidGoal.set(false), Score.move(-127), Hood.set(false);
            else if (master.get_digital(DIGITAL_R1))
                Score.move(127);
            else if (master.get_digital(DIGITAL_L2))
                MidGoal.set(true), Score.move(-127);
            else if (master.get_digital(DIGITAL_L1))
                Score.move(-127), Hood.set(true), MidGoal.set(false);
            else
                Score.move(0);
        }

        bool wings_btn = master.get_digital(DIGITAL_B);
        if (wings_btn && !wings_prev) { wings_state = !wings_state; Wings.set(wings_state); }
        wings_prev = wings_btn;

        bool intakeLift_btn = master.get_digital(DIGITAL_DOWN);
        if (intakeLift_btn && !intakeLift_prev) { intakeLift_state = !intakeLift_state; IntakeLift.set(intakeLift_state); }
        intakeLift_prev = intakeLift_btn;

        bool loader_btn = master.get_digital(DIGITAL_Y);
        if (loader_btn && !loader_prev) { loader_state = !loader_state; Loader.set(loader_state); }
        loader_prev = loader_btn;

        pros::delay(10);
    }
}
// void opcontrol() {
//   chassis.drive_brake_set(MOTOR_BRAKE_COAST);
//   while (true) {
//     auton_toggle();
//     if(!auton_running) {
//       chassis.opcontrol_arcade_standard(ez::SPLIT);
//       if (master.get_digital(DIGITAL_R2))
//         .set(true),
//         Score.move(-127),
//         Hood.set(false);
//       else if (master.get_digital(DIGITAL_R1))
//         Score.move(127);
//       else if (master.get_digital(DIGITAL_L2))
//         MidGoal.set(false),
//         Score.move(-127);
//       else if (master.get_digital(DIGITAL_L1))
//         Score.move(-127),
//         Hood.set(true),
//         .set(true);
//       else
//         Score.move(0);
//     }
//     Wings.button_toggle(master.get_digital(DIGITAL_B));
//     IntakeLift.button_toggle(master.get_digital(DIGITAL_DOWN));
//     DoublePark.button_toggle(master.get_digital(DIGITAL_RIGHT));
//     Loader.button_toggle(master.get_digital(DIGITAL_Y));

//     pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
//   }
// }
