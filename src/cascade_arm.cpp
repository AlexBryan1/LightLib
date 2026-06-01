#include "cascade_arm.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(v, hi));
}

int clampi(int v, int lo, int hi) {
  return std::max(lo, std::min(v, hi));
}
}

CascadeArm::CascadeArm(pros::Motor& cascade_motor, pros::Rotation& cascade_sensor,
                       pros::Motor& arm_motor,     pros::Rotation& arm_sensor,
                       const CascadeArmConfig& cfg)
    : cascade_m_(cascade_motor),
      cascade_s_(cascade_sensor),
      arm_m_(arm_motor),
      arm_s_(arm_sensor),
      cfg_(cfg) {}

void CascadeArm::zero_sensors() {
  cascade_s_.reset_position();
  arm_s_.reset_position();
  lift_tgt_in_   = 0.0f;
  arm_tgt_deg_   = 0.0f;
  target_height_ = current_height_in();
  lift_prev_err_ = lift_i_err_ = 0.0f;
  arm_prev_err_  = arm_i_err_  = 0.0f;
}

float CascadeArm::cascade_position_in() const {
  const float revs = cascade_s_.get_position() / 36000.0f;
  return revs * cfg_.cascade_in_per_rot * cfg_.cascade_stage_mult;
}

float CascadeArm::arm_angle_deg() const {
  const float revs = arm_s_.get_position() / 36000.0f;
  return revs * cfg_.arm_deg_per_rot;
}

float CascadeArm::current_height_in() const {
  return cfg_.floor_to_lift_zero_in
       + cascade_position_in()
       + cfg_.arm_pivot_y_in
       + cfg_.arm_length_in * std::sin(arm_angle_deg() * kDegToRad);
}

float CascadeArm::current_forward_in() const {
  return cfg_.arm_pivot_x_in
       + cfg_.arm_length_in * std::cos(arm_angle_deg() * kDegToRad);
}

float CascadeArm::target_height_in() const { return target_height_; }

bool CascadeArm::at_target(float tol_in) const {
  return std::fabs(current_height_in() - target_height_) < tol_in;
}

void CascadeArm::set_height_limit(float inches) {
  cfg_.max_end_height_in = std::max(0.0f, inches);
  if (target_height_ > cfg_.max_end_height_in)
    set_target_height(cfg_.max_end_height_in);
}

float CascadeArm::interp_arm_angle_(float h) const {
  const auto& curve = cfg_.height_to_arm_curve;
  if (curve.empty()) return 0.0f;
  if (h <= curve.front().first)  return curve.front().second;
  if (h >= curve.back().first)   return curve.back().second;
  for (size_t i = 1; i < curve.size(); ++i) {
    const auto& a = curve[i - 1];
    const auto& b = curve[i];
    if (h <= b.first) {
      const float span = b.first - a.first;
      const float t = (span > 0.0f) ? (h - a.first) / span : 0.0f;
      return a.second + t * (b.second - a.second);
    }
  }
  return curve.back().second;
}

void CascadeArm::solve_targets_(float h, float& out_lift_in, float& out_arm_deg) const {
  h = clampf(h, 0.0f, cfg_.max_end_height_in);

  float arm_deg = clampf(interp_arm_angle_(h), cfg_.arm_min_deg, cfg_.arm_max_deg);
  float arm_y = cfg_.arm_length_in * std::sin(arm_deg * kDegToRad);
  float lift_in = h - cfg_.floor_to_lift_zero_in - cfg_.arm_pivot_y_in - arm_y;

  if (lift_in < cfg_.cascade_min_in || lift_in > cfg_.cascade_max_in) {
    lift_in = clampf(lift_in, cfg_.cascade_min_in, cfg_.cascade_max_in);
    const float needed_arm_y =
        h - cfg_.floor_to_lift_zero_in - cfg_.arm_pivot_y_in - lift_in;
    float sin_arg = clampf(needed_arm_y / cfg_.arm_length_in, -1.0f, 1.0f);
    arm_deg = clampf(std::asin(sin_arg) / kDegToRad,
                     cfg_.arm_min_deg, cfg_.arm_max_deg);
  }

  out_lift_in = lift_in;
  out_arm_deg = arm_deg;
}

void CascadeArm::set_target_height(float inches) {
  target_height_ = clampf(inches, 0.0f, cfg_.max_end_height_in);
  solve_targets_(target_height_, lift_tgt_in_, arm_tgt_deg_);
}

void CascadeArm::manual(int cascade_mV, int arm_mV) {
  const bool active = std::abs(cascade_mV) > cfg_.manual_deadzone_mV
                   || std::abs(arm_mV)     > cfg_.manual_deadzone_mV;
  manual_active_ = active;
  if (active) {
    cascade_m_.move_voltage(clampi(cascade_mV, -cfg_.lift_max_mV, cfg_.lift_max_mV));
    arm_m_.move_voltage    (clampi(arm_mV,     -cfg_.arm_max_mV,  cfg_.arm_max_mV));
  }
}

void CascadeArm::update() {
  if (manual_active_) {
    lift_tgt_in_ = cascade_position_in();
    arm_tgt_deg_ = arm_angle_deg();
    target_height_ = current_height_in();
    lift_prev_err_ = lift_i_err_ = 0.0f;
    arm_prev_err_  = arm_i_err_  = 0.0f;
    manual_active_ = false;
    return;
  }

  const float lift_err = lift_tgt_in_ - cascade_position_in();
  lift_i_err_ += lift_err;
  const float lift_d = lift_err - lift_prev_err_;
  lift_prev_err_ = lift_err;
  float lift_out_v =
      cfg_.lift_kP * lift_err + cfg_.lift_kI * lift_i_err_ + cfg_.lift_kD * lift_d;
  int lift_mV = clampi(static_cast<int>(lift_out_v * 1000.0f),
                       -cfg_.lift_max_mV, cfg_.lift_max_mV);
  cascade_m_.move_voltage(lift_mV);

  const float arm_err = arm_tgt_deg_ - arm_angle_deg();
  arm_i_err_ += arm_err;
  const float arm_d = arm_err - arm_prev_err_;
  arm_prev_err_ = arm_err;
  float arm_out_v =
      cfg_.arm_kP * arm_err + cfg_.arm_kI * arm_i_err_ + cfg_.arm_kD * arm_d;
  int arm_mV = clampi(static_cast<int>(arm_out_v * 100.0f),
                      -cfg_.arm_max_mV, cfg_.arm_max_mV);
  arm_m_.move_voltage(arm_mV);
}
