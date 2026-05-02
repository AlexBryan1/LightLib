#include "LightLib/rotational_snap.hpp"

#include <algorithm>
#include <cmath>

#include "LightLib/holo_drive.hpp"  // light::wrap180

namespace light {

RotationalSnap::RotationalSnap(pros::AbstractMotor& motor,
                               pros::Rotation& sensor,
                               std::vector<double> snap_angles_deg,
                               double kP,
                               double tolerance_deg,
                               int max_snap_speed,
                               int input_deadband)
    : motor_(motor),
      sensor_(sensor),
      snaps_(std::move(snap_angles_deg)),
      kP_(kP),
      tol_(tolerance_deg),
      max_speed_(max_snap_speed),
      deadband_(input_deadband) {}

void RotationalSnap::set_snap_angles(std::vector<double> angles_deg) {
  snaps_ = std::move(angles_deg);
  has_target_ = false;
}

void RotationalSnap::set_enabled(bool on) {
  enabled_ = on;
  if (!on) {
    was_active_ = false;
    has_target_ = false;
  }
}

double RotationalSnap::get_position_deg() const {
  return sensor_.get_position() / 100.0;
}

double RotationalSnap::get_target_deg() const {
  return has_target_ ? target_ : std::nan("");
}

double RotationalSnap::nearest_snap_(double pos_deg) const {
  if (snaps_.empty()) return pos_deg;

  auto modDist = [](double a, double b) {
    double d = std::fmod(std::fabs(a - b), 360.0);
    return std::min(d, 360.0 - d);
  };

  double best = snaps_.front();
  double best_dist = modDist(pos_deg, best);
  for (size_t i = 1; i < snaps_.size(); ++i) {
    double d = modDist(pos_deg, snaps_[i]);
    if (d < best_dist) {
      best_dist = d;
      best = snaps_[i];
    }
  }
  return best;
}

void RotationalSnap::update(int manual_input) {
  if (!enabled_) {
    motor_.move(manual_input);
    was_active_ = false;
    has_target_ = false;
    return;
  }

  if (std::abs(manual_input) > deadband_) {
    motor_.move(manual_input);
    was_active_ = true;
    has_target_ = false;
    return;
  }

  // Operator just released: latch the nearest snap angle.
  if (was_active_) {
    target_ = nearest_snap_(get_position_deg());
    has_target_ = true;
    was_active_ = false;
  }

  if (!has_target_) {
    motor_.move(0);
    return;
  }

  double error = wrap180(target_ - get_position_deg());
  if (std::fabs(error) <= tol_) {
    motor_.move(0);  // brake-mode HOLD does the work
    return;
  }

  int cmd = static_cast<int>(kP_ * error);
  cmd = std::clamp(cmd, -max_speed_, max_speed_);
  motor_.move(cmd);
}

}  // namespace light
