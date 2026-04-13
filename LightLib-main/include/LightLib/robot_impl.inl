// ─────────────────────────────────────────────────────────────────────────────
//  robot_impl.inl — LightLib boilerplate
//
//  This file is #included by main.cpp AFTER the #define configuration block.
//  It expands in main.cpp's translation unit so it can see LEFT_PORTS, IMU_PORT,
//  WHEEL_DIAMETER, etc.  Do NOT include it anywhere else.
// ─────────────────────────────────────────────────────────────────────────────

// EZ-Template's tracking_wheel.hpp and drive.hpp use WHEEL_DIAMETER as a
// member variable name.  Save the user's value and remove the macro before
// pulling in any headers, then use the saved constant everywhere below.
static constexpr double _ROBOT_WHEEL_DIA = WHEEL_DIAMETER;
#undef WHEEL_DIAMETER

#include "LightLib/main.h"
#include <functional>
#include <atomic>
#include "autons.hpp"
#include "pros/motors.h"
#include "LightLib/odom.hpp"
#include "LightLib/custom_selector.hpp"
#include "LightLib/cstm_move.hpp"
#include "LightLib/ez_extra.hpp"

// ── Global objects built from the #defines in main.cpp ───────────────────────

ez::Drive chassis(LEFT_PORTS, RIGHT_PORTS, IMU_PORT, _ROBOT_WHEEL_DIA, WHEEL_RPM);

pros::MotorGroup leftMotors (LEFT_PORTS);
pros::MotorGroup rightMotors(RIGHT_PORTS);
pros::Imu imu(IMU_PORT);
pros::Imu imu2(IMU2_PORT);

TrackingWheel leftTracker (&leftMotors,  _ROBOT_WHEEL_DIA, -TRACK_HALF_W);
TrackingWheel rightTracker(&rightMotors, _ROBOT_WHEEL_DIA,  TRACK_HALF_W);

OdomSensors sensors(&leftTracker, &rightTracker, nullptr, nullptr, &imu, &imu2);

// ── Distance sensors ──────────────────────────────────────────────────────────
// Port 0 means "not installed" — the pointer is nullptr in that case.

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

// ── Alliance color ────────────────────────────────────────────────────────────
Colors allianceColor = NEUTRAL;

// ── Temperature display ───────────────────────────────────────────────────────
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

// ── Auton-during-driver task ──────────────────────────────────────────────────
static std::atomic<bool> auton_running{false};
static pros::Task* auton_task = nullptr;

static void auton_task_fn(void*) {
    autonomous();
    auton_running.store(false);
}

// Call this once per opcontrol loop tick.
// UP button starts/stops the selected auton for practice testing.
static void auton_toggle() {
    if (auton_task != nullptr && !auton_running.load()) {
        delete auton_task;
        auton_task = nullptr;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (auton_task == nullptr) {
            auton_running.store(true);
            auton_task = new pros::Task(auton_task_fn, nullptr, TASK_PRIORITY_DEFAULT,
                                        TASK_STACK_DEPTH_DEFAULT, "Auton Task");
        } else {
            auton_running.store(false);
            auton_task->remove();
            delete auton_task;
            auton_task = nullptr;
        }
    }
}

// ── Joystick expo curve ───────────────────────────────────────────────────────
static float curve_lut[256]; // index 0 = joystick -127, index 255 = joystick +127

static void build_curve_lut() {
    for (int i = -127; i <= 127; i++) {
        float x      = i / 127.0f;
        float curved = (std::exp(JOYSTICK_CURVE * std::abs(x)) - 1.0f) /
                       (std::exp(JOYSTICK_CURVE) - 1.0f);
        curve_lut[i + 127] = std::copysign(curved * 127.0f, (float)i);
    }
    curve_lut[127] = 0.0f; // exact zero for zero input
}

static inline float apply_curve(int raw) {
    if (raw > -JOYSTICK_DEADZONE && raw < JOYSTICK_DEADZONE) return 0.0f;
    return curve_lut[raw + 127];
}

// ── Drive dispatch — selected by DRIVE_TYPE in main.cpp ──────────────────────
static void _run_drive() {
#if DRIVE_TYPE == 1  // Arcade
    float t = apply_curve(master.get_analog(ANALOG_LEFT_Y));
    float r = master.get_analog(ANALOG_RIGHT_X);
    chassis.drive_set(std::clamp((int)(t + r), -127, 127),
                      std::clamp((int)(t - r), -127, 127));

#elif DRIVE_TYPE == 2  // Tank
    chassis.drive_set(apply_curve(master.get_analog(ANALOG_LEFT_Y)),
                      apply_curve(master.get_analog(ANALOG_RIGHT_Y)));

#elif DRIVE_TYPE == 3  // Single stick
    float t = apply_curve(master.get_analog(ANALOG_LEFT_Y));
    float r = master.get_analog(ANALOG_LEFT_X);
    chassis.drive_set(std::clamp((int)(t + r), -127, 127),
                      std::clamp((int)(t - r), -127, 127));

#elif DRIVE_TYPE == 4  // Holo / Mecanum / X-Drive  (holoDrive in subsystems.hpp)
    holoDrive.opcontrol(master.get_analog(ANALOG_LEFT_Y),
                        master.get_analog(ANALOG_LEFT_X),
                        master.get_analog(ANALOG_RIGHT_X));

#elif DRIVE_TYPE == 5  // H-Drive  (hDrive in subsystems.hpp)
    hDrive.opcontrol(master.get_analog(ANALOG_LEFT_Y),
                     master.get_analog(ANALOG_RIGHT_X),
                     master.get_analog(ANALOG_LEFT_X));
#endif
}


// ── User hooks (defined by the user in main.cpp) ──────────────────────────────
// Forward-declare so initialize() and autonomous() can call them before the
// definitions appear later in the file.
void user_initialize();
void user_autonomous();

// ── PROS lifecycle hooks ──────────────────────────────────────────────────────

void initialize() {
    pros::delay(300);
    chassis.initialize();          // calibrates IMU internally
    pros::delay(300);
    chassis.drive_imu_reset();
    cstm_move_init(chassis);

    OdomSensors odom_sensors(&leftTracker, &rightTracker, nullptr, nullptr, &imu, &imu2);
    light::init(odom_sensors);

    default_constants();           // PID / exit-condition / slew tuning (autons.cpp)
    default_positions();           // starting piston / mechanism positions (autons.cpp)

    light::pid_tuner.set_drive(&chassis);
    light::pid_tuner.start_task();
    register_autons();             // auton_config.cpp
    light::auton_selector.init();

    build_curve_lut();

    static pros::Task temp_task(temp_display_task, nullptr, 3,
                                TASK_STACK_DEPTH_DEFAULT, "Temp Display");

    user_initialize();               // user hook — defined in main.cpp

    master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
    pros::delay(100);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);

    pros::delay(10);
    light::reset();
    light::setPose(Pose(0, 0, 0));

    user_autonomous();               // user hook — defined in main.cpp

    uint32_t auton_start = pros::millis();
    light::auton_selector.run();
    uint32_t elapsed_ms = pros::millis() - auton_start;

    uint32_t secs = elapsed_ms / 1000;
    uint32_t ms   = elapsed_ms % 1000;
    master.print(0, 0, "Done: %lu.%03lus  ", secs, ms);
}
