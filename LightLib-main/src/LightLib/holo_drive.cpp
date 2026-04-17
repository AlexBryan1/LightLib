// ┌─────────────────────────────────────────────────────────────────────────────┐
// │  holo_drive.cpp — Holonomic and H-drive chassis controllers                │
// │                                                                            │
// │  This file provides two self-contained chassis classes for robots that     │
// │  can strafe (move sideways) — something a standard tank drive can't do.   │
// │  Both are independent of EZ-Template and have their own PID controllers.  │
// │                                                                            │
// │  ┌────────────────────────────────────────────────────────────────────┐    │
// │  │  1. HoloDrive  — 4-motor holonomic (X-Drive or Mecanum)           │    │
// │  │                                                                    │    │
// │  │  All four wheels are angled so the robot can drive, strafe, and   │    │
// │  │  turn simultaneously.  Motor mixing:                               │    │
// │  │                                                                    │    │
// │  │      FL = throttle - strafe + turn                                │    │
// │  │      FR = throttle + strafe - turn                                │    │
// │  │      BL = throttle + strafe + turn                                │    │
// │  │      BR = throttle - strafe - turn                                │    │
// │  │                                                                    │    │
// │  │  Motor layout (top view, nose up):                                │    │
// │  │       FL ╲  ╱ FR                                                  │    │
// │  │           ╲╱                                                      │    │
// │  │           ╱╲                                                      │    │
// │  │       BL ╱  ╲ BR                                                  │    │
// │  └────────────────────────────────────────────────────────────────────┘    │
// │                                                                            │
// │  ┌────────────────────────────────────────────────────────────────────┐    │
// │  │  2. HDrive  — Tank drive + one center strafe wheel                │    │
// │  │                                                                    │    │
// │  │  Left and right motor groups drive normally (tank).  A single     │    │
// │  │  perpendicular center wheel handles strafing.                     │    │
// │  │                                                                    │    │
// │  │  Motor layout (top view, nose up):                                │    │
// │  │       L ──── R      (left/right groups, any number of motors)     │    │
// │  │          C          (center strafe wheel)                          │    │
// │  │       L ──── R                                                    │    │
// │  └────────────────────────────────────────────────────────────────────┘    │
// │                                                                            │
// │  SETUP (same for both classes):                                            │
// │    1. Construct with motor ports, IMU port, wheel diameter, gear ratio     │
// │    2. Call calibrate() once in initialize() to zero the IMU                │
// │    3. Call set_drive_pid(), set_strafe_pid(), set_turn_pid(),              │
// │       set_heading_pid() before using autonomous methods                    │
// │    4. In opcontrol(), call opcontrol(throttle, strafe, turn) each loop    │
// │    5. In autonomous, call drive(), strafe(), turn_to(), turn_relative()   │
// │                                                                            │
// │  PID CONTROL:                                                              │
// │    Each axis has its own PID controller (see HoloPID in the header):      │
// │      - drive_pid_   : forward/backward distance                            │
// │      - strafe_pid_  : left/right distance                                 │
// │      - turn_pid_    : absolute heading for turn_to()                      │
// │      - heading_pid_ : heading correction during drive/strafe to keep      │
// │                        the robot pointed straight                          │
// │                                                                            │
// │  EXIT CONDITIONS:                                                          │
// │    Motions exit when the error stays small for several consecutive         │
// │    10ms cycles ("settled" counter), or when the timeout expires.           │
// └─────────────────────────────────────────────────────────────────────────────┘

#include "LightLib/holo_drive.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <algorithm>
#include <cstdlib>

namespace light {

// Clamp a value between lo and hi (used throughout for speed limiting)
static inline double dclamp(double v, double lo, double hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HoloDrive — 4-motor holonomic (X-Drive / Mecanum)
// ═══════════════════════════════════════════════════════════════════════════════

HoloDrive::HoloDrive(int fl, int fr, int bl, int br,
                     int imu_port, double wheel_diameter, double gear_ratio,
                     Type type)
    : fl_(fl), fr_(fr), bl_(bl), br_(br),
      imu_(imu_port),
      type_(type),
      wheel_circumference_(M_PI * wheel_diameter),
      gear_ratio_(gear_ratio)
{}

// ── Opcontrol ────────────────────────────────────────────────────────────────
// Pass raw joystick values (-127 to 127) directly from the controller.
// The mixing math in set_holonomic() converts them to individual motor speeds.
void HoloDrive::opcontrol(int throttle, int strafe, int turn) {
    set_holonomic((double)throttle, (double)strafe, (double)turn);
}

// ── Autonomous motions ───────────────────────────────────────────────────────
// Each motion uses PID to reach a target while the heading PID keeps the robot
// pointed in the same direction it started.  The "settled" counter requires the
// error to stay small for 15+ consecutive cycles (150ms) before exiting — this
// prevents the robot from stopping while it's still coasting.

// Drive straight forward (positive inches) or backward (negative inches)
void HoloDrive::drive(double inches, int max_speed, int timeout_ms) {
    reset_sensors();
    const double target  = inches_to_deg(inches);
    const double hdg_tgt = get_heading();
    drive_pid_.reset();
    heading_pid_.reset();

    uint32_t start   = pros::millis();
    int      settled = 0;

    while ((int)(pros::millis() - start) < timeout_ms) {
        const double error      = target - drive_pos();
        const double output     = dclamp(drive_pid_.compute(error), -max_speed, max_speed);
        const double correction = heading_pid_.compute(wrap180(hdg_tgt - get_heading()));

        set_holonomic(output, 0.0, correction);

        if (std::abs(error) < 15.0) ++settled; else settled = 0;
        if (settled >= 15) break;
        pros::delay(10);
    }
    set_holonomic(0, 0, 0);
}

// Strafe right (positive inches) or left (negative inches)
void HoloDrive::strafe(double inches, int max_speed, int timeout_ms) {
    reset_sensors();
    const double target  = inches_to_deg(inches);
    const double hdg_tgt = get_heading();
    strafe_pid_.reset();
    heading_pid_.reset();

    uint32_t start   = pros::millis();
    int      settled = 0;

    while ((int)(pros::millis() - start) < timeout_ms) {
        const double error      = target - strafe_pos();
        const double output     = dclamp(strafe_pid_.compute(error), -max_speed, max_speed);
        const double correction = heading_pid_.compute(wrap180(hdg_tgt - get_heading()));

        set_holonomic(0.0, output, correction);

        if (std::abs(error) < 15.0) ++settled; else settled = 0;
        if (settled >= 15) break;
        pros::delay(10);
    }
    set_holonomic(0, 0, 0);
}

// Turn to an absolute heading (0-360 degrees, clockwise-positive).
// Tighter exit condition (1° for 20 cycles = 200ms) than drive/strafe for accuracy.
void HoloDrive::turn_to(double heading_deg, int max_speed, int timeout_ms) {
    turn_pid_.reset();
    uint32_t start   = pros::millis();
    int      settled = 0;

    while ((int)(pros::millis() - start) < timeout_ms) {
        const double error  = wrap180(heading_deg - get_heading());
        const double output = dclamp(turn_pid_.compute(error), -max_speed, max_speed);

        set_holonomic(0.0, 0.0, output);

        if (std::abs(error) < 1.0) ++settled; else settled = 0;
        if (settled >= 20) break;
        pros::delay(10);
    }
    set_holonomic(0, 0, 0);
}

// Turn by a relative amount — positive = clockwise, negative = counter-clockwise.
// Converts to an absolute heading then delegates to turn_to().
void HoloDrive::turn_relative(double degrees, int max_speed, int timeout_ms) {
    double target = std::fmod(get_heading() + degrees + 360.0, 360.0);
    turn_to(target, max_speed, timeout_ms);
}

// ── PID setters ──────────────────────────────────────────────────────────────
// Call these in initialize() before running any autonomous motion.
// Each takes (kP, kI, kD).  Start with kI = 0 and tune kP + kD first.
void HoloDrive::set_drive_pid  (double p, double i, double d) { drive_pid_.set(p,i,d); }
void HoloDrive::set_strafe_pid (double p, double i, double d) { strafe_pid_.set(p,i,d); }
void HoloDrive::set_turn_pid   (double p, double i, double d) { turn_pid_.set(p,i,d); }
void HoloDrive::set_heading_pid(double p, double i, double d) { heading_pid_.set(p,i,d); }

// ── Setup & utility ─────────────────────────────────────────────────────────

// Zero the IMU — call once in initialize().  If wait=true (default), blocks
// until calibration finishes (~2 seconds).
void HoloDrive::calibrate(bool wait) {
    imu_.reset(wait);
}

// Zero all four motor encoders — called at the start of each autonomous motion
// so that drive_pos() and strafe_pos() measure distance from the motion's start.
void HoloDrive::reset_sensors() {
    fl_.tare_position();
    fr_.tare_position();
    bl_.tare_position();
    br_.tare_position();
}

void HoloDrive::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    fl_.set_brake_mode(mode);
    fr_.set_brake_mode(mode);
    bl_.set_brake_mode(mode);
    br_.set_brake_mode(mode);
}

double HoloDrive::get_heading() const {
    return imu_.get_heading();
}

// ── Private helpers ──────────────────────────────────────────────────────────

// Core motor mixing — converts (throttle, strafe, turn) into four motor speeds.
// This is the holonomic kinematics equation.  If any motor would exceed ±127,
// all four are scaled down proportionally so the motion ratio is preserved.
//
// The signs come from the wheel geometry:
//   FL = throttle - strafe + turn    ┌──────────┐
//   FR = throttle + strafe - turn    │ FL    FR │  → positive strafe = right
//   BL = throttle + strafe + turn    │ BL    BR │  → positive turn = clockwise
//   BR = throttle - strafe - turn    └──────────┘  → positive throttle = forward
void HoloDrive::set_holonomic(double throttle, double strafe, double turn) {
    double fl = throttle - strafe + turn;
    double fr = throttle + strafe - turn;
    double bl = throttle + strafe + turn;
    double br = throttle - strafe - turn;

    double peak = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br)});
    if (peak > 127.0) {
        const double s = 127.0 / peak;
        fl *= s; fr *= s; bl *= s; br *= s;
    }

    fl_.move((int)fl);
    fr_.move((int)fr);
    bl_.move((int)bl);
    br_.move((int)br);
}

// Convert a distance in inches to motor encoder degrees, accounting for wheel
// size and gear ratio.  Used as the PID target for drive/strafe motions.
double HoloDrive::inches_to_deg(double inches) const {
    return inches * 360.0 * gear_ratio_ / wheel_circumference_;
}

// Forward/backward position estimate: average all four encoders.
// In holonomic kinematics, strafe components cancel out when averaged.
double HoloDrive::drive_pos() const {
    return (fl_.get_position() + fr_.get_position() +
            bl_.get_position() + br_.get_position()) / 4.0;
}

// Left/right (strafe) position estimate: (-FL + FR + BL - BR) / 4.
// The signs isolate the strafe component — forward/turn contributions cancel out.
double HoloDrive::strafe_pos() const {
    return (-fl_.get_position() + fr_.get_position() +
             bl_.get_position() - br_.get_position()) / 4.0;
}

// Normalize an angle to [-180, 180] so PID always takes the shortest path
double HoloDrive::wrap180(double a) {
    while (a >  180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HDrive — Tank drive + center strafe wheel
//
//  This is for robots with a standard tank drive (left + right motor groups)
//  plus one extra wheel mounted sideways in the center for strafing.
//  The center wheel only spins during strafe(); drive() and turn_to() only
//  use the tank sides.
// ═══════════════════════════════════════════════════════════════════════════════

HDrive::HDrive(std::vector<int> left_ports, std::vector<int> right_ports,
               int center_port, int imu_port,
               double wheel_diameter, double gear_ratio)
    : left_ (std::vector<int8_t>(left_ports.begin(),  left_ports.end())),
      right_(std::vector<int8_t>(right_ports.begin(), right_ports.end())),
      center_(center_port),
      imu_(imu_port),
      wheel_circumference_(M_PI * wheel_diameter),
      gear_ratio_(gear_ratio)
{}

// ── Opcontrol ────────────────────────────────────────────────────────────────
// Tank sides handle forward/backward + turning; center wheel handles strafing.
void HDrive::opcontrol(int throttle, int strafe, int turn) {
    set_tank((double)(throttle + turn), (double)(throttle - turn));
    center_.move(std::clamp(strafe, -127, 127));
}

// ── Autonomous motions ───────────────────────────────────────────────────────

// Drive forward/backward using the tank sides only (center wheel stays idle).
// Heading PID keeps the robot pointed straight.
void HDrive::drive(double inches, int max_speed, int timeout_ms) {
    reset_sensors();
    const double target  = inches_to_deg(inches);
    const double hdg_tgt = get_heading();
    drive_pid_.reset();
    heading_pid_.reset();

    uint32_t start   = pros::millis();
    int      settled = 0;

    while ((int)(pros::millis() - start) < timeout_ms) {
        const double error      = target - drive_pos();
        const double output     = dclamp(drive_pid_.compute(error), -max_speed, max_speed);
        const double correction = heading_pid_.compute(wrap180(hdg_tgt - get_heading()));

        set_tank(output + correction, output - correction);

        if (std::abs(error) < 15.0) ++settled; else settled = 0;
        if (settled >= 15) break;
        pros::delay(10);
    }
    set_tank(0, 0);
}

// Strafe using the center wheel only.  The tank sides don't drive forward —
// they only apply small corrections to counteract any heading drift caused
// by friction or weight imbalance during the strafe.
void HDrive::strafe(double inches, int max_speed, int timeout_ms) {
    center_.tare_position();
    const double target  = inches_to_deg(inches);
    const double hdg_tgt = get_heading();
    strafe_pid_.reset();
    heading_pid_.reset();

    uint32_t start   = pros::millis();
    int      settled = 0;

    while ((int)(pros::millis() - start) < timeout_ms) {
        const double error      = target - center_.get_position();
        const double output     = dclamp(strafe_pid_.compute(error), -max_speed, max_speed);
        const double correction = heading_pid_.compute(wrap180(hdg_tgt - get_heading()));

        // Tank sides counteract any heading drift while center wheel strafes.
        set_tank(correction, -correction);
        center_.move((int)output);

        if (std::abs(error) < 15.0) ++settled; else settled = 0;
        if (settled >= 15) break;
        pros::delay(10);
    }
    set_tank(0, 0);
    center_.move(0);
}

// Turn in place to an absolute heading using opposing tank sides (left fwd, right back).
void HDrive::turn_to(double heading_deg, int max_speed, int timeout_ms) {
    turn_pid_.reset();
    uint32_t start   = pros::millis();
    int      settled = 0;

    while ((int)(pros::millis() - start) < timeout_ms) {
        const double error  = wrap180(heading_deg - get_heading());
        const double output = dclamp(turn_pid_.compute(error), -max_speed, max_speed);

        // Positive output = CW: left forward, right backward.
        set_tank(output, -output);

        if (std::abs(error) < 1.0) ++settled; else settled = 0;
        if (settled >= 20) break;
        pros::delay(10);
    }
    set_tank(0, 0);
}

// Turn by a relative amount — wraps to 0-360 then delegates to turn_to().
void HDrive::turn_relative(double degrees, int max_speed, int timeout_ms) {
    double target = std::fmod(get_heading() + degrees + 360.0, 360.0);
    turn_to(target, max_speed, timeout_ms);
}

// ── PID setters ──────────────────────────────────────────────────────────────
void HDrive::set_drive_pid  (double p, double i, double d) { drive_pid_.set(p,i,d); }
void HDrive::set_strafe_pid (double p, double i, double d) { strafe_pid_.set(p,i,d); }
void HDrive::set_turn_pid   (double p, double i, double d) { turn_pid_.set(p,i,d); }
void HDrive::set_heading_pid(double p, double i, double d) { heading_pid_.set(p,i,d); }

// ── Setup & utility ─────────────────────────────────────────────────────────

void HDrive::calibrate(bool wait) {
    imu_.reset(wait);
}

void HDrive::reset_sensors() {
    left_.tare_position_all();
    right_.tare_position_all();
    center_.tare_position();
}

void HDrive::set_brake_mode(pros::motor_brake_mode_e_t mode) {
    left_.set_brake_mode_all(mode);
    right_.set_brake_mode_all(mode);
    center_.set_brake_mode(mode);
}

double HDrive::get_heading() const {
    return imu_.get_heading();
}

// ── Private helpers ──────────────────────────────────────────────────────────

// Send clamped speeds to the left and right motor groups
void HDrive::set_tank(double left_v, double right_v) {
    left_.move ((int)dclamp(left_v,  -127, 127));
    right_.move((int)dclamp(right_v, -127, 127));
}

double HDrive::inches_to_deg(double inches) const {
    return inches * 360.0 * gear_ratio_ / wheel_circumference_;
}

// Average encoder position across all left + right motors
double HDrive::drive_pos() const {
    auto lp = left_.get_position_all();
    auto rp = right_.get_position_all();
    double sum = 0;
    for (double v : lp) sum += v;
    for (double v : rp) sum += v;
    return sum / (double)(lp.size() + rp.size());
}

double HDrive::wrap180(double a) {
    while (a >  180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

} // namespace light
