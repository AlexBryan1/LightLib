// HoloDrive (4-motor X/Mecanum) and HDrive (tank + center strafe wheel).
// Independent of EZ-Template; motions exit on settled-tick counter or timeout.

#include "LightLib/drive/holo_drive.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "pros/rtos.hpp"

namespace light {

static inline double dclamp(double v, double lo, double hi) {
  return v < lo ? lo : v > hi ? hi
                              : v;
}

static constexpr double DRIVE_SETTLED_TOL_DEG = 15.0;
static constexpr double TURN_SETTLED_TOL_DEG = 1.0;
static constexpr int DRIVE_SETTLED_TICKS = 15;
static constexpr int TURN_SETTLED_TICKS = 20;

HoloDrive::HoloDrive(int fl, int fr, int bl, int br,
                     int imu_port, double wheel_diameter, double gear_ratio,
                     Type type)
    : fl_(fl), fr_(fr), bl_(bl), br_(br), imu_(imu_port), type_(type), wheel_circumference_(M_PI * wheel_diameter), gear_ratio_(gear_ratio) {}

void HoloDrive::opcontrol(int throttle, int strafe, int turn) {
  set_holonomic((double)throttle, (double)strafe, (double)turn);
}

void HoloDrive::drive(double inches, int max_speed, int timeout_ms) {
  reset_sensors();
  const double target = inches_to_deg(inches);
  const double hdg_tgt = get_heading();
  drive_pid_.reset();
  heading_pid_.reset();

  uint32_t start = pros::millis();
  int settled = 0;

  while ((int)(pros::millis() - start) < timeout_ms) {
    const double pos = drive_pos();
    const double error = target - pos;
    const double hdg = get_heading();
    const double output = dclamp(drive_pid_.compute(error, pos), -max_speed, max_speed);
    const double correction = heading_pid_.compute(wrap180(hdg_tgt - hdg), hdg);

    set_holonomic(output, 0.0, correction);

    if (std::abs(error) < DRIVE_SETTLED_TOL_DEG)
      ++settled;
    else
      settled = 0;
    if (settled >= DRIVE_SETTLED_TICKS) break;
    pros::delay(10);
  }
  set_holonomic(0, 0, 0);
}

void HoloDrive::strafe(double inches, int max_speed, int timeout_ms) {
  reset_sensors();
  const double target = inches_to_deg(inches);
  const double hdg_tgt = get_heading();
  strafe_pid_.reset();
  heading_pid_.reset();

  uint32_t start = pros::millis();
  int settled = 0;

  while ((int)(pros::millis() - start) < timeout_ms) {
    const double pos = strafe_pos();
    const double error = target - pos;
    const double hdg = get_heading();
    const double output = dclamp(strafe_pid_.compute(error, pos), -max_speed, max_speed);
    const double correction = heading_pid_.compute(wrap180(hdg_tgt - hdg), hdg);

    set_holonomic(0.0, output, correction);

    if (std::abs(error) < DRIVE_SETTLED_TOL_DEG)
      ++settled;
    else
      settled = 0;
    if (settled >= DRIVE_SETTLED_TICKS) break;
    pros::delay(10);
  }
  set_holonomic(0, 0, 0);
}

// turn_to uses a tighter exit (1° for 200 ms) than drive/strafe.
void HoloDrive::turn_to(double heading_deg, int max_speed, int timeout_ms) {
  turn_pid_.reset();
  uint32_t start = pros::millis();
  int settled = 0;

  while ((int)(pros::millis() - start) < timeout_ms) {
    const double hdg = get_heading();
    const double error = wrap180(heading_deg - hdg);
    const double output = dclamp(turn_pid_.compute(error, hdg), -max_speed, max_speed);

    set_holonomic(0.0, 0.0, output);

    if (std::abs(error) < TURN_SETTLED_TOL_DEG)
      ++settled;
    else
      settled = 0;
    if (settled >= TURN_SETTLED_TICKS) break;
    pros::delay(10);
  }
  set_holonomic(0, 0, 0);
}

void HoloDrive::turn_relative(double degrees, int max_speed, int timeout_ms) {
  double target = std::fmod(get_heading() + degrees + 360.0, 360.0);
  turn_to(target, max_speed, timeout_ms);
}

void HoloDrive::set_drive_pid(double p, double i, double d) { drive_pid_.set(p, i, d); }
void HoloDrive::set_strafe_pid(double p, double i, double d) { strafe_pid_.set(p, i, d); }
void HoloDrive::set_turn_pid(double p, double i, double d) { turn_pid_.set(p, i, d); }
void HoloDrive::set_heading_pid(double p, double i, double d) { heading_pid_.set(p, i, d); }

void HoloDrive::calibrate(bool wait) {
  imu_.reset(wait);
}

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

// Holonomic mixing: +throttle = forward, +strafe = right, +turn = CW.
// Proportional rescale preserves the motion ratio when any motor saturates.
void HoloDrive::set_holonomic(double throttle, double strafe, double turn) {
  double fl = throttle - strafe + turn;
  double fr = throttle + strafe - turn;
  double bl = throttle + strafe + turn;
  double br = throttle - strafe - turn;

  double peak = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br)});
  if (peak > 127.0) {
    const double s = 127.0 / peak;
    fl *= s;
    fr *= s;
    bl *= s;
    br *= s;
  }

  // lround (not trunc) preserves cumulative scale-down precision; skip
  // smartport writes that match the last sent value.
  int ifl = (int)std::lround(fl);
  int ifr = (int)std::lround(fr);
  int ibl = (int)std::lround(bl);
  int ibr = (int)std::lround(br);

  if (ifl != last_fl_) {
    fl_.move(ifl);
    last_fl_ = ifl;
  }
  if (ifr != last_fr_) {
    fr_.move(ifr);
    last_fr_ = ifr;
  }
  if (ibl != last_bl_) {
    bl_.move(ibl);
    last_bl_ = ibl;
  }
  if (ibr != last_br_) {
    br_.move(ibr);
    last_br_ = ibr;
  }
}

double HoloDrive::inches_to_deg(double inches) const {
  return inches * 360.0 * gear_ratio_ / wheel_circumference_;
}

// Averaging all four cancels the strafe component.
double HoloDrive::drive_pos() const {
  return (fl_.get_position() + fr_.get_position() +
          bl_.get_position() + br_.get_position()) /
         4.0;
}

// (-FL + FR + BL - BR) / 4 isolates strafe; forward/turn cancel.
double HoloDrive::strafe_pos() const {
  return (-fl_.get_position() + fr_.get_position() +
          bl_.get_position() - br_.get_position()) /
         4.0;
}

// fmod is O(1) for arbitrary-magnitude inputs (pros::Rotation accumulates
// across turns). Shared with HDrive and rotational_snap.
double wrap180(double a) {
  a = std::fmod(a, 360.0);
  if (a > 180.0) a -= 360.0;
  if (a < -180.0) a += 360.0;
  return a;
}

HDrive::HDrive(std::vector<int> left_ports, std::vector<int> right_ports,
               int center_port, int imu_port,
               double wheel_diameter, double gear_ratio)
    : left_(std::vector<int8_t>(left_ports.begin(), left_ports.end())),
      right_(std::vector<int8_t>(right_ports.begin(), right_ports.end())),
      center_(center_port),
      imu_(imu_port),
      wheel_circumference_(M_PI * wheel_diameter),
      gear_ratio_(gear_ratio) {}

void HDrive::opcontrol(int throttle, int strafe, int turn) {
  set_tank((double)(throttle + turn), (double)(throttle - turn));
  center_.move(std::clamp(strafe, -127, 127));
}

void HDrive::drive(double inches, int max_speed, int timeout_ms) {
  reset_sensors();
  const double target = inches_to_deg(inches);
  const double hdg_tgt = get_heading();
  drive_pid_.reset();
  heading_pid_.reset();

  uint32_t start = pros::millis();
  int settled = 0;

  while ((int)(pros::millis() - start) < timeout_ms) {
    const double pos = drive_pos();
    const double error = target - pos;
    const double hdg = get_heading();
    const double output = dclamp(drive_pid_.compute(error, pos), -max_speed, max_speed);
    const double correction = heading_pid_.compute(wrap180(hdg_tgt - hdg), hdg);

    set_tank(output + correction, output - correction);

    if (std::abs(error) < DRIVE_SETTLED_TOL_DEG)
      ++settled;
    else
      settled = 0;
    if (settled >= DRIVE_SETTLED_TICKS) break;
    pros::delay(10);
  }
  set_tank(0, 0);
}

// Tank sides only correct heading drift; center wheel does the strafing.
void HDrive::strafe(double inches, int max_speed, int timeout_ms) {
  center_.tare_position();
  const double target = inches_to_deg(inches);
  const double hdg_tgt = get_heading();
  strafe_pid_.reset();
  heading_pid_.reset();

  uint32_t start = pros::millis();
  int settled = 0;

  while ((int)(pros::millis() - start) < timeout_ms) {
    const double pos = center_.get_position();
    const double error = target - pos;
    const double hdg = get_heading();
    const double output = dclamp(strafe_pid_.compute(error, pos), -max_speed, max_speed);
    const double correction = heading_pid_.compute(wrap180(hdg_tgt - hdg), hdg);

    set_tank(correction, -correction);
    center_.move((int)output);

    if (std::abs(error) < DRIVE_SETTLED_TOL_DEG)
      ++settled;
    else
      settled = 0;
    if (settled >= DRIVE_SETTLED_TICKS) break;
    pros::delay(10);
  }
  set_tank(0, 0);
  center_.move(0);
}

void HDrive::turn_to(double heading_deg, int max_speed, int timeout_ms) {
  turn_pid_.reset();
  uint32_t start = pros::millis();
  int settled = 0;

  while ((int)(pros::millis() - start) < timeout_ms) {
    const double hdg = get_heading();
    const double error = wrap180(heading_deg - hdg);
    const double output = dclamp(turn_pid_.compute(error, hdg), -max_speed, max_speed);

    // Positive output = CW: left fwd, right back.
    set_tank(output, -output);

    if (std::abs(error) < TURN_SETTLED_TOL_DEG)
      ++settled;
    else
      settled = 0;
    if (settled >= TURN_SETTLED_TICKS) break;
    pros::delay(10);
  }
  set_tank(0, 0);
}

void HDrive::turn_relative(double degrees, int max_speed, int timeout_ms) {
  double target = std::fmod(get_heading() + degrees + 360.0, 360.0);
  turn_to(target, max_speed, timeout_ms);
}

void HDrive::set_drive_pid(double p, double i, double d) { drive_pid_.set(p, i, d); }
void HDrive::set_strafe_pid(double p, double i, double d) { strafe_pid_.set(p, i, d); }
void HDrive::set_turn_pid(double p, double i, double d) { turn_pid_.set(p, i, d); }
void HDrive::set_heading_pid(double p, double i, double d) { heading_pid_.set(p, i, d); }

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

void HDrive::set_tank(double left_v, double right_v) {
  left_.move((int)dclamp(left_v, -127, 127));
  right_.move((int)dclamp(right_v, -127, 127));
}

double HDrive::inches_to_deg(double inches) const {
  return inches * 360.0 * gear_ratio_ / wheel_circumference_;
}

double HDrive::drive_pos() const {
  auto lp = left_.get_position_all();
  auto rp = right_.get_position_all();
  double sum = 0;
  for (double v : lp) sum += v;
  for (double v : rp) sum += v;
  return sum / (double)(lp.size() + rp.size());
}

}  // namespace light
