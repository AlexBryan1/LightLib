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

// ── Optional-sensor default macros ────────────────────────────────────────────
// Any sensor the user doesn't `#define` in main.cpp silently stays disabled
// (port 0 / offset 0). Ensures zero-regression builds.
#ifndef VERT_WHEEL_DIA
#define VERT_WHEEL_DIA      2.75
#endif
#ifndef VERT_LEFT_PORT
#define VERT_LEFT_PORT      0
#endif
#ifndef VERT_LEFT_OFFSET
#define VERT_LEFT_OFFSET    0.0
#endif
#ifndef VERT_RIGHT_PORT
#define VERT_RIGHT_PORT     0
#endif
#ifndef VERT_RIGHT_OFFSET
#define VERT_RIGHT_OFFSET   0.0
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
#ifndef GPS_OFFSET_X
#define GPS_OFFSET_X        0.0
#endif
#ifndef GPS_OFFSET_Y
#define GPS_OFFSET_Y        0.0
#endif

// ── MCL distance sensors — one per face ──────────────────────────────────────
// Port: VEX port number (0 = not installed).
// Along: signed position along the face in robot frame (inches). 0 = centered.
// Depth: perpendicular distance from robot center to the sensor (inches).
#ifndef MCL_FRONT_PORT
#define MCL_FRONT_PORT   0
#endif
#ifndef MCL_FRONT_ALONG
#define MCL_FRONT_ALONG  0.0f
#endif
#ifndef MCL_FRONT_DEPTH
#define MCL_FRONT_DEPTH  6.0f
#endif
#ifndef MCL_BACK_PORT
#define MCL_BACK_PORT    0
#endif
#ifndef MCL_BACK_ALONG
#define MCL_BACK_ALONG   0.0f
#endif
#ifndef MCL_BACK_DEPTH
#define MCL_BACK_DEPTH   6.0f
#endif
#ifndef MCL_LEFT_PORT
#define MCL_LEFT_PORT    0
#endif
#ifndef MCL_LEFT_ALONG
#define MCL_LEFT_ALONG   0.0f
#endif
#ifndef MCL_LEFT_DEPTH
#define MCL_LEFT_DEPTH   6.0f
#endif
#ifndef MCL_RIGHT_PORT
#define MCL_RIGHT_PORT   0
#endif
#ifndef MCL_RIGHT_ALONG
#define MCL_RIGHT_ALONG  0.0f
#endif
#ifndef MCL_RIGHT_DEPTH
#define MCL_RIGHT_DEPTH  6.0f
#endif

// ── MCL tuning parameters ─────────────────────────────────────────────────────
#ifndef MCL_PARTICLES
#define MCL_PARTICLES    200
#endif
#ifndef MCL_SENSOR_SIGMA
#define MCL_SENSOR_SIGMA 2.5f
#endif
#ifndef MCL_OUTLIER_GAP
#define MCL_OUTLIER_GAP  6.0f
#endif
#ifndef MCL_MAX_RANGE
#define MCL_MAX_RANGE    144.0f
#endif
#ifndef EKF_Q_POS
#define EKF_Q_POS        0.02f
#endif
#ifndef EKF_Q_THETA
#define EKF_Q_THETA      0.0005f
#endif
#ifndef EKF_Q_VEL
#define EKF_Q_VEL        1.0f
#endif
#ifndef MCL_SNAP_DIVERGE
#define MCL_SNAP_DIVERGE 9.0f
#endif
#ifndef MCL_SNAP_CONVERGE
#define MCL_SNAP_CONVERGE 3.0f
#endif

#include "LightLib/main.h"
#include <functional>
#include <atomic>
#include <vector>
#include "autons.hpp"
#include "pros/motors.h"
#include "LightLib/odom.hpp"
#include "LightLib/lightcast.hpp"
#include "LightLib/custom_selector.hpp"
#include "LightLib/cstm_move.hpp"
#include "LightLib/ez_extra.hpp"
#include "ui_config.hpp"

// ── Global objects built from the #defines in main.cpp ───────────────────────

ez::Drive chassis(LEFT_PORTS, RIGHT_PORTS, IMU_PORT, _ROBOT_WHEEL_DIA, WHEEL_RPM);

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
#if VERT_LEFT_PORT != 0
static pros::Rotation _vertLeftRot(VERT_LEFT_PORT);
static TrackingWheel  _vertLeftTracker(&_vertLeftRot, VERT_WHEEL_DIA, VERT_LEFT_OFFSET);
static TrackingWheel* vertLeftPtr = &_vertLeftTracker;
#else
static TrackingWheel* vertLeftPtr = nullptr;
#endif
#if VERT_RIGHT_PORT != 0
static pros::Rotation _vertRightRot(VERT_RIGHT_PORT);
static TrackingWheel  _vertRightTracker(&_vertRightRot, VERT_WHEEL_DIA, VERT_RIGHT_OFFSET);
static TrackingWheel* vertRightPtr = &_vertRightTracker;
#else
static TrackingWheel* vertRightPtr = nullptr;
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
#if GPS_PORT != 0
static pros::Gps _gpsObj(GPS_PORT, GPS_OFFSET_X, GPS_OFFSET_Y);
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

static std::vector<DistanceSensorSpec> _build_distance_specs() {
    std::vector<DistanceSensorSpec> v;
    using light::lightcast::Face;
    using light::lightcast::fromFace;
    #if MCL_FRONT_PORT != 0
    v.push_back(fromFace(&_dist_front_obj, Face::FRONT, MCL_FRONT_ALONG, MCL_FRONT_DEPTH));
    #endif
    #if MCL_BACK_PORT != 0
    v.push_back(fromFace(&_dist_back_obj, Face::BACK, MCL_BACK_ALONG, MCL_BACK_DEPTH));
    #endif
    #if MCL_LEFT_PORT != 0
    v.push_back(fromFace(&_dist_left_obj, Face::LEFT, MCL_LEFT_ALONG, MCL_LEFT_DEPTH));
    #endif
    #if MCL_RIGHT_PORT != 0
    v.push_back(fromFace(&_dist_right_obj, Face::RIGHT, MCL_RIGHT_ALONG, MCL_RIGHT_DEPTH));
    #endif
    return v;
}

static MCLConfig _build_mcl_config() {
    MCLConfig cfg;
    cfg.numParticles  = MCL_PARTICLES;
    cfg.sensorSigmaIn = MCL_SENSOR_SIGMA;
    cfg.outlierGapIn  = MCL_OUTLIER_GAP;
    cfg.maxRangeIn    = MCL_MAX_RANGE;
    cfg.ekfQPos       = EKF_Q_POS;
    cfg.ekfQTheta     = EKF_Q_THETA;
    cfg.ekfQVel       = EKF_Q_VEL;
    cfg.snapDiverge   = MCL_SNAP_DIVERGE;
    cfg.snapConverge  = MCL_SNAP_CONVERGE;
    return cfg;
}

// Auto-select displacement wheels: prefer unpowered rotation trackers when the
// user provided them, fall back to powered motor encoders when not.
static TrackingWheel* _v1_selected = vertLeftPtr  ? vertLeftPtr  : &leftTracker;
static TrackingWheel* _v2_selected = vertRightPtr ? vertRightPtr : &rightTracker;

OdomSensors sensors(_v1_selected, _v2_selected, horiz1Ptr, horiz2Ptr,
                    &imu, imu2_ptr,
                    gpsPtr, GPS_OFFSET_X, GPS_OFFSET_Y,
                    _build_distance_specs());

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

// ── Controller screen display ────────────────────────────────────────────────
// Three configurable slots (LEFT/MID/RIGHT) chosen in ui_config.hpp print to
// one controller line. auton_time_str is set by autonomous() and auton_toggle()
// so the AutonTimer slot shows the last/current auton's elapsed time.
static char auton_time_str[16] = "";

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
static uint32_t auton_start_ms = 0;

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
            auton_start_ms = pros::millis();
            auton_running.store(true);
            auton_task = new pros::Task(auton_task_fn, nullptr, TASK_PRIORITY_DEFAULT,
                                        TASK_STACK_DEPTH_DEFAULT, "Auton Task");
        } else {
            uint32_t elapsed_ms = pros::millis() - auton_start_ms;
            uint32_t secs = elapsed_ms / 1000;
            uint32_t ms   = elapsed_ms % 1000;
            snprintf(auton_time_str, sizeof(auton_time_str), "S:%lu.%03lus", secs, ms);
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
    light::ez_extra_init(&chassis, &leftMotors, &rightMotors);

    // Use the global `sensors` built at TU scope — it carries the full config
    // (optional rotation trackers, GPS, LightCast distance specs).
    light::init(sensors, _build_mcl_config());

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

    light::auton_start_ms = pros::millis();
    light::auton_selector.run();
    uint32_t elapsed_ms = pros::millis() - light::auton_start_ms;

    uint32_t secs = elapsed_ms / 1000;
    uint32_t ms   = elapsed_ms % 1000;
    snprintf(auton_time_str, sizeof(auton_time_str), "D:%lu.%03lus", secs, ms);
}
