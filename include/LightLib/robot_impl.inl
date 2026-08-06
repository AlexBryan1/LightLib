/**
 * \file robot_impl.inl
 *
 * LightLib boilerplate, expanded inline into main.cpp's translation unit.
 *
 * This file is `#include`d by main.cpp AFTER the `#define` configuration
 * block, so that it can see `LEFT_PORTS`, `IMU_PORT`, `WHEEL_DIAMETER`,
 * etc. Do NOT include it anywhere else.
 */

// EZ-Template's tracking_wheel.hpp and drive.hpp use WHEEL_DIAMETER as a
// member variable name.  Save the user's value and remove the macro before
// pulling in any headers, then use the saved constant everywhere below.
static constexpr double _ROBOT_WHEEL_DIA = WHEEL_DIAMETER;
#undef WHEEL_DIAMETER

// ── Optional-sensor default macros ────────────────────────────────────────────
// Any sensor the user doesn't `#define` in main.cpp silently stays disabled
// (port 0 / offset 0). Ensures zero-regression builds.
#ifndef VERT_WHEEL_DIA
#define VERT_WHEEL_DIA      2.75
#endif
#ifndef VERT_1_PORT
#define VERT_1_PORT         0
#endif
#ifndef VERT_1_OFFSET
#define VERT_1_OFFSET       0.0
#endif
#ifndef VERT_2_PORT
#define VERT_2_PORT         0
#endif
#ifndef VERT_2_OFFSET
#define VERT_2_OFFSET       0.0
#endif
#ifndef HORIZ_WHEEL_DIA
#define HORIZ_WHEEL_DIA     2.75
#endif
#ifndef HORIZ_1_PORT
#define HORIZ_1_PORT        0
#endif
#ifndef HORIZ_1_OFFSET
#define HORIZ_1_OFFSET      0.0
#endif
#ifndef HORIZ_2_PORT
#define HORIZ_2_PORT        0
#endif
#ifndef HORIZ_2_OFFSET
#define HORIZ_2_OFFSET      0.0
#endif
#ifndef GPS_PORT
#define GPS_PORT            0
#endif
// GPS_OFFSET_X / GPS_OFFSET_Y are the GPS sensor's mount position relative to
// the robot's center, in INCHES.
#ifndef GPS_OFFSET_X
#define GPS_OFFSET_X        0.0
#endif
#ifndef GPS_OFFSET_Y
#define GPS_OFFSET_Y        0.0
#endif

// MCL distance sensor fallbacks. user sets these in main.cpp, we just
// provide defaults so the build doesn't break if a macro is missing.
// X/Y are inches in robot frame, ANGLE is degrees (0 = forward).
#ifndef MCL_FRONT_PORT
#define MCL_FRONT_PORT   0
#endif
#ifndef MCL_FRONT_X
#define MCL_FRONT_X     0.0f
#endif
#ifndef MCL_FRONT_Y
#define MCL_FRONT_Y     6.0f
#endif
#ifndef MCL_FRONT_ANGLE
#define MCL_FRONT_ANGLE 0.0f
#endif
#ifndef MCL_BACK_PORT
#define MCL_BACK_PORT 0
#endif
#ifndef MCL_BACK_X
#define MCL_BACK_X     0.0f
#endif
#ifndef MCL_BACK_Y
#define MCL_BACK_Y    -6.0f
#endif
#ifndef MCL_BACK_ANGLE
#define MCL_BACK_ANGLE 180.0f
#endif
#ifndef MCL_LEFT_PORT
#define MCL_LEFT_PORT    0
#endif
#ifndef MCL_LEFT_X
#define MCL_LEFT_X   -6.0f
#endif
#ifndef MCL_LEFT_Y
#define MCL_LEFT_Y    0.0f
#endif
#ifndef MCL_LEFT_ANGLE
#define MCL_LEFT_ANGLE 90.0f
#endif
#ifndef MCL_RIGHT_PORT
#define MCL_RIGHT_PORT   0
#endif
#ifndef MCL_RIGHT_X
#define MCL_RIGHT_X       6.0f
#endif
#ifndef MCL_RIGHT_Y
#define MCL_RIGHT_Y       0.0f
#endif
#ifndef MCL_RIGHT_ANGLE
#define MCL_RIGHT_ANGLE  -90.0f
#endif

// MCL/EKF tuning lives in default_constants() in autons.cpp; the runtime
// defaults in MCLConfig (mcl_config.hpp) cover any field you don't override.

#include "LightLib/main.h"
#include <cmath>
#include <functional>
#include <atomic>
#include <vector>
#include "autons.hpp"
#include "pros/motors.h"
#include "LightLib/drive/autotune.hpp"
#include "LightLib/drive/odometry.hpp"
#include "LightLib/drive/lightcast.hpp"
#include "LightLib/drive/sensor_aux.hpp"
#include "LightLib/ui/custom_selector.hpp"
#include "LightLib/drive/custom_move.hpp"
#include "LightLib/util/extras.hpp"
#include "ui_config.hpp"

// ── Global objects built from the #defines in main.cpp ───────────────────────

light::Drive chassis(LEFT_PORTS, RIGHT_PORTS, IMU_PORT, _ROBOT_WHEEL_DIA, WHEEL_RPM);

pros::MotorGroup leftMotors (LEFT_PORTS);
pros::MotorGroup rightMotors(RIGHT_PORTS);
pros::Imu imu(IMU_PORT);
#if IMU2_PORT != 0
pros::Imu imu2(IMU2_PORT);
static pros::Imu* imu2_ptr = &imu2;
#else
static pros::Imu* imu2_ptr = nullptr;
#endif

TrackingWheel leftTracker (&leftMotors,  _ROBOT_WHEEL_DIA, -TRACK_HALF_W);
TrackingWheel rightTracker(&rightMotors, _ROBOT_WHEEL_DIA,  TRACK_HALF_W);

// ── Optional unpowered rotation-sensor trackers ──────────────────────────────
// Port 0 = not installed → nullptr → OdomSensors falls back to powered encoders.
#if VERT_1_PORT != 0
static pros::Rotation _vert1Rot(VERT_1_PORT);
static TrackingWheel  _vert1Tracker(&_vert1Rot, VERT_WHEEL_DIA, VERT_1_OFFSET);
static TrackingWheel* vert1Ptr = &_vert1Tracker;
#else
static TrackingWheel* vert1Ptr = nullptr;
#endif
#if VERT_2_PORT != 0
static pros::Rotation _vert2Rot(VERT_2_PORT);
static TrackingWheel  _vert2Tracker(&_vert2Rot, VERT_WHEEL_DIA, VERT_2_OFFSET);
static TrackingWheel* vert2Ptr = &_vert2Tracker;
#else
static TrackingWheel* vert2Ptr = nullptr;
#endif
#if HORIZ_1_PORT != 0
static pros::Rotation _horiz1Rot(HORIZ_1_PORT);
static TrackingWheel  _horiz1Tracker(&_horiz1Rot, HORIZ_WHEEL_DIA, HORIZ_1_OFFSET);
static TrackingWheel* horiz1Ptr = &_horiz1Tracker;
#else
static TrackingWheel* horiz1Ptr = nullptr;
#endif
#if HORIZ_2_PORT != 0
static pros::Rotation _horiz2Rot(HORIZ_2_PORT);
static TrackingWheel  _horiz2Tracker(&_horiz2Rot, HORIZ_WHEEL_DIA, HORIZ_2_OFFSET);
static TrackingWheel* horiz2Ptr = &_horiz2Tracker;
#else
static TrackingWheel* horiz2Ptr = nullptr;
#endif

// ── Optional GPS sensor ──────────────────────────────────────────────────────
static constexpr double _GPS_OFFSET_X_M = (double)GPS_OFFSET_X * light::units::M_PER_IN;
static constexpr double _GPS_OFFSET_Y_M = (double)GPS_OFFSET_Y * light::units::M_PER_IN;
#if GPS_PORT != 0
static pros::Gps _gpsObj(GPS_PORT, _GPS_OFFSET_X_M, _GPS_OFFSET_Y_M);
static pros::Gps* gpsPtr = &_gpsObj;
#else
static pros::Gps* gpsPtr = nullptr;
#endif

// ── Optional distance sensors for LightCast — one per face ───────────────────
#if MCL_FRONT_PORT != 0
static pros::Distance _dist_front_obj(MCL_FRONT_PORT);
#endif
#if MCL_BACK_PORT != 0
static pros::Distance _dist_back_obj(MCL_BACK_PORT);
#endif
#if MCL_LEFT_PORT != 0
static pros::Distance _dist_left_obj(MCL_LEFT_PORT);
#endif
#if MCL_RIGHT_PORT != 0
static pros::Distance _dist_right_obj(MCL_RIGHT_PORT);
#endif

// build the spec list - one entry per enabled port
static std::vector<DistanceSensorSpec> _build_distance_specs() {
    std::vector<DistanceSensorSpec> v;
    constexpr float kDegToRad = (float)(M_PI / 180.0);

    #if MCL_FRONT_PORT != 0
    v.push_back({&_dist_front_obj, MCL_FRONT_X, MCL_FRONT_Y, MCL_FRONT_ANGLE * kDegToRad, nullptr});
    #endif

    #if MCL_BACK_PORT != 0
    v.push_back(DistanceSensorSpec{
        &_dist_back_obj,
        MCL_BACK_X, MCL_BACK_Y,
        MCL_BACK_ANGLE * kDegToRad,
        nullptr});
    #endif

    #if MCL_LEFT_PORT != 0
    DistanceSensorSpec leftSpec{
        &_dist_left_obj,
        MCL_LEFT_X, MCL_LEFT_Y,
        MCL_LEFT_ANGLE * kDegToRad,
        nullptr};
    v.push_back(leftSpec);
    #endif

    #if MCL_RIGHT_PORT != 0
    v.push_back(DistanceSensorSpec{&_dist_right_obj,
                                   MCL_RIGHT_X, MCL_RIGHT_Y,
                                   MCL_RIGHT_ANGLE * kDegToRad,
                                   nullptr});
    #endif
    return v;
}

// Auto-select displacement wheels: prefer unpowered rotation trackers when the
// user provided them, fall back to powered motor encoders when not.
static TrackingWheel* _v1_selected = vert1Ptr ? vert1Ptr : &leftTracker;
static TrackingWheel* _v2_selected = vert2Ptr ? vert2Ptr : &rightTracker;

OdomSensors sensors(_v1_selected, _v2_selected, horiz1Ptr, horiz2Ptr,
                    &imu, imu2_ptr,
                    gpsPtr, (float)_GPS_OFFSET_X_M, (float)_GPS_OFFSET_Y_M,
                    _build_distance_specs());

// ── Alliance color ────────────────────────────────────────────────────────────
Colors allianceColor = NEUTRAL;

// ── Controller screen display ────────────────────────────────────────────────
// Three configurable slots (LEFT/MID/RIGHT) chosen in ui_config.hpp print to
// one controller line. auton_time_str is set by autonomous() and auton_toggle()
// so the AutonTimer slot shows the last/current auton's elapsed time.
static char auton_time_str[16] = "";

// While true, the controller auton menu owns UI_CTRL_LINE and
// temp_display_task must not print over it.
static std::atomic<bool> ctrl_menu_open{false};

// While true, a finished tuner auton's result line owns UI_CTRL_LINE (and
// gates main.cpp's DIGITAL_UP cascade jog). Dismissed by pressing UP.
static std::atomic<bool> ctrl_results_open{false};
static char ctrl_results_line[19] = "";
static bool ctrl_results_reprinted = false;
static bool ctrl_results_dismissing = false;
static uint32_t ctrl_results_shown_ms = 0;

static void fmt_ctrl_slot(CtrlSlot s, char* out, size_t n, double max_temp) {
    switch (s) {
        case CtrlSlot::MaxMotorTempC:
            snprintf(out, n, "%.0fC", max_temp); break;
        case CtrlSlot::AutonTimer:
            snprintf(out, n, "%s", auton_time_str); break;
        case CtrlSlot::BatteryPct:
            snprintf(out, n, "%.0f%%", pros::battery::get_capacity()); break;
        case CtrlSlot::OdomX:
            snprintf(out, n, "X%.1f", chassis.odom_x_get()); break;
        case CtrlSlot::OdomY:
            snprintf(out, n, "Y%.1f", chassis.odom_y_get()); break;
        case CtrlSlot::OdomTheta:
            snprintf(out, n, "T%.0f", chassis.odom_theta_get()); break;
        case CtrlSlot::None:
        default:
            out[0] = '\0'; break;
    }
}

static void temp_display_task(void*) {
    uint32_t last_buzz_ms = 0;
    while (true) {
        // max_temp is always computed — needed by the heat-buzz guard even
        // when MaxMotorTempC isn't shown in any slot.
        double max_temp = 0.0;
        for (double t : leftMotors.get_temperature_all())  max_temp = std::max(max_temp, t);
        for (double t : rightMotors.get_temperature_all()) max_temp = std::max(max_temp, t);
        for (double t : Score.get_temperature_all())       max_temp = std::max(max_temp, t);

        char a[8] = "", b[8] = "", c[8] = "";
        fmt_ctrl_slot(UI_CTRL_SLOT_LEFT,  a, sizeof(a), max_temp);
        fmt_ctrl_slot(UI_CTRL_SLOT_MID,   b, sizeof(b), max_temp);
        fmt_ctrl_slot(UI_CTRL_SLOT_RIGHT, c, sizeof(c), max_temp);
        if (!ctrl_menu_open.load() && !ctrl_results_open.load())
            master.print(UI_CTRL_LINE, 0, "%-6s %-5s %-5s", a, b, c);

        // Buzz the controller at 2-second intervals when any motor exceeds HEAT_BUZZ_TEMP
        if (HEAT_BUZZ_ENABLED && max_temp >= HEAT_BUZZ_TEMP) {
            uint32_t now = pros::millis();
            if (now - last_buzz_ms >= 2000) {
                master.rumble("_");
                last_buzz_ms = now;
            }
        }

        pros::delay(UI_CTRL_REFRESH_MS);
    }
}

// ── Auton-during-driver task ──────────────────────────────────────────────────
static std::atomic<bool> auton_running{false};
static pros::Task* auton_task = nullptr;
static uint32_t practice_auton_start_ms = 0;
static uint32_t menu_last_input_ms = 0;   // base for the 5 s idle timeout
static bool     menu_reprinted     = false;

static void auton_task_fn(void*) {
    autonomous();
    auton_running.store(false);
}

static void menu_print_name() {
    // %-18.18s pads/truncates to the temp line's exact 18-char width
    // ("%-6s %-5s %-5s") so stale status text is fully overwritten.
    master.print(UI_CTRL_LINE, 0, "%-18.18s",
                 light::auton_selector.name(light::auton_selector.selected()).c_str());
}

static void auton_start_practice_run() {
    practice_auton_start_ms = pros::millis();
    auton_running.store(true);
    auton_task = new pros::Task(auton_task_fn, nullptr, TASK_PRIORITY_DEFAULT,
                                TASK_STACK_DEPTH_DEFAULT, "Auton Task");
}

// Call this once per opcontrol loop tick.
// Idle: UP opens the auton menu on the controller LCD.
// Menu open: RIGHT cycles autons (brain selector stays in sync), UP runs the
// selected one, 5 s without a press closes the menu. Closing the menu lets
// temp_display_task repaint the temp/timer line within one 500 ms tick.
// Running: UP stops the practice run (unchanged).
// Tuner finished: its result line holds UI_CTRL_LINE until UP dismisses it.
static void auton_toggle() {
    if (auton_task != nullptr && !auton_running.load()) {
        delete auton_task;
        auton_task = nullptr;
    }

    if (auton_task != nullptr) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            uint32_t elapsed_ms = pros::millis() - practice_auton_start_ms;
            uint32_t secs = elapsed_ms / 1000;
            uint32_t ms   = elapsed_ms % 1000;
            snprintf(auton_time_str, sizeof(auton_time_str), "S:%lu.%03lus", secs, ms);
            auton_running.store(false);
            auton_task->remove();
            delete auton_task;
            auton_task = nullptr;
        }
        return;
    }

    // Results screen: UP dismisses. Must sit before the menu/idle branches so
    // the dismiss press is consumed here and can't re-open the auton menu.
    if (ctrl_results_open.load()) {
        if (ctrl_results_dismissing) {
            // Hold the flag until UP is released so main.cpp's DIGITAL_UP
            // cascade jog (gated on ctrl_results_open) doesn't fire mid-press.
            if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
                ctrl_results_open.store(false);  // temp task repaints ≤500 ms later
                ctrl_results_dismissing = false;
            }
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            ctrl_results_dismissing = true;
        } else if (!ctrl_results_reprinted &&
                   pros::millis() - ctrl_results_shown_ms >= 600) {
            // One-shot, same as menu_reprinted: temp_display_task may have
            // passed its flag check just as the result was posted.
            master.print(UI_CTRL_LINE, 0, "%-18.18s", ctrl_results_line);
            ctrl_results_reprinted = true;
        }
        return;
    }

    // A tuner auton just finished (practice task above or a competition
    // autonomous() run) — show its result line until UP dismisses it.
    if (light::autotune_result_take(ctrl_results_line, sizeof(ctrl_results_line))) {
        ctrl_results_open.store(true);
        ctrl_results_shown_ms = pros::millis();
        ctrl_results_reprinted = false;
        ctrl_results_dismissing = false;
        master.print(UI_CTRL_LINE, 0, "%-18.18s", ctrl_results_line);
        master.rumble(".");  // heads-up; the 600 ms reprint covers a dropped print
        return;
    }

    if (ctrl_menu_open.load()) {
        uint32_t now = pros::millis();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            light::auton_selector.select_next();
            menu_last_input_ms = now;
            menu_print_name();
            menu_reprinted = true;
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            ctrl_menu_open.store(false);
            auton_start_practice_run();
        } else if (now - menu_last_input_ms >= 5000) {
            ctrl_menu_open.store(false);  // idle timeout — close without running
        } else if (!menu_reprinted && now - menu_last_input_ms >= 600) {
            // One-shot: temp_display_task may have passed its flag check just
            // as the menu opened and overwritten the first name print.
            menu_print_name();
            menu_reprinted = true;
        }
        return;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (light::auton_selector.count() == 0) return;
        ctrl_menu_open.store(true);
        menu_last_input_ms = pros::millis();
        menu_reprinted = false;
        menu_print_name();
    }
}

// ── Joystick expo curve ───────────────────────────────────────────────────────
static float curve_lut[256]; // index 0 = joystick -127, index 255 = joystick +127

static void build_curve_lut() {
    for (int i = -127; i <= 127; i++) {
        // If curve is basically 0, just make it linear
        if (std::abs(JOYSTICK_CURVE) < 0.001f) {
            curve_lut[i + 127] = (float)i;
        } 
        else {
            float x = i / 127.0f;
            float curved = (std::exp(JOYSTICK_CURVE * std::abs(x)) - 1.0f) / (std::exp(JOYSTICK_CURVE) - 1.0f);
            curve_lut[i + 127] = std::copysign(curved * 127.0f, (float)i);
        }
    }
    curve_lut[127] = 0.0f; 
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
    chassis.drive_imu2_set(imu2_ptr);  // average both IMUs in the PID heading path
    chassis.initialize();          // calibrates both IMUs internally
    {
        const uint32_t imu_calib_deadline = pros::millis() + 3000;
        auto still_calibrating = [&]() -> bool {
            bool a = imu.is_calibrating();
            bool b = imu2_ptr ? imu2_ptr->is_calibrating() : false;
            return a || b;
        };
        while (still_calibrating() && pros::millis() < imu_calib_deadline) {
            pros::delay(20);
        }
        if (still_calibrating()) {
            printf("[INIT] IMU calibration timeout — odometry may be biased\n");
        }
    }
    chassis.drive_imu_reset();
    custom_move_init(chassis);
    light::extras_init(&chassis, &leftMotors, &rightMotors);

    // Use the global `sensors` built at TU scope — it carries the full config
    // (optional rotation trackers, GPS, LightCast distance specs).
    light::init(sensors);

    // Wire the override-on-confidence detectors (IMU bias / single-wheel /
    // slip / wall-snap). Must run after light::init() so the odom task
    // exists and can call sensor_aux::tick() each frame.
    light::sensor_aux::init(sensors);

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
    light::sensor_aux::resetCounters();

    user_autonomous();               // user hook — defined in main.cpp

    light::auton_start_ms = pros::millis();
    light::auton_selector.run();
    uint32_t elapsed_ms = pros::millis() - light::auton_start_ms;

    uint32_t secs = elapsed_ms / 1000;
    uint32_t ms   = elapsed_ms % 1000;
    snprintf(auton_time_str, sizeof(auton_time_str), "D:%lu.%03lus", secs, ms);
}
