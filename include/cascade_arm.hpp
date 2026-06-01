#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"

#include <utility>
#include <vector>

/**
 * \file cascade_arm.hpp
 *
 * Proprietary kinematics controller for a cascade lift carrying a
 * single-pivot 2-bar arm. Lives at user-code level (not in src/LightLib/) so
 * it is NOT bundled into liblightlib.a — strip these two files to remove the
 * feature from a fork.
 *
 * Operator gives a single absolute end-effector height; the controller solves
 * for (lift_pos, arm_angle) via a configurable height->arm-angle curve and
 * drives both motors toward their targets at full voltage simultaneously
 * ("as fast as possible"). A runtime height cap protects the mechanism.
 *
 * Frame convention (robot frame, looking down):
 *   - x positive = forward (direction the robot drives forward)
 *   - y positive = up (away from the floor)
 *   - arm angle 0 deg = pivot-to-tip vector points forward and level
 *   - arm angle  90 deg = pivot-to-tip points straight up
 *   - arm angle -90 deg = pivot-to-tip points straight down
 */

struct CascadeArmConfig {
  float cascade_in_per_rot   = 1.0f;
  float cascade_stage_mult   = 1.0f;
  float cascade_min_in       = 0.0f;
  float cascade_max_in       = 30.0f;

  float arm_length_in        = 12.0f;
  float arm_pivot_x_in       = 4.0f;
  float arm_pivot_y_in       = 2.0f;
  float arm_deg_per_rot      = 360.0f;
  float arm_min_deg          = -10.0f;
  float arm_max_deg          = 100.0f;

  float floor_to_lift_zero_in = 6.0f;

  float max_end_height_in    = 36.0f;

  std::vector<std::pair<float, float>> height_to_arm_curve = {
      { 0.0f,  -10.0f},
      {12.0f,    0.0f},
      {24.0f,   45.0f},
      {36.0f,   90.0f},
  };

  float lift_kP = 8.0f, lift_kI = 0.0f, lift_kD = 0.5f;
  float arm_kP  = 2.0f, arm_kI  = 0.0f, arm_kD  = 0.1f;

  int lift_max_mV = 12000;
  int arm_max_mV  = 12000;

  int manual_deadzone_mV = 300;
};

class CascadeArm {
 public:
  CascadeArm(pros::Motor& cascade_motor, pros::Rotation& cascade_sensor,
             pros::Motor& arm_motor,     pros::Rotation& arm_sensor,
             const CascadeArmConfig& cfg);

  void set_target_height(float inches);
  void set_height_limit(float inches);
  void zero_sensors();
  void manual(int cascade_mV, int arm_mV);

  void update();

  float cascade_position_in() const;
  float arm_angle_deg()       const;
  float current_height_in()   const;
  float current_forward_in()  const;
  float target_height_in()    const;
  bool  at_target(float lift_tol_in = 0.5f, float arm_tol_deg = 1.0f) const;

  CascadeArmConfig& config() { return cfg_; }
  const CascadeArmConfig& config() const { return cfg_; }

 private:
  pros::Motor&    cascade_m_;
  pros::Rotation& cascade_s_;
  pros::Motor&    arm_m_;
  pros::Rotation& arm_s_;
  CascadeArmConfig cfg_;

  float target_height_ = 0.0f;
  float lift_tgt_in_   = 0.0f;
  float arm_tgt_deg_   = 0.0f;

  float lift_prev_err_ = 0.0f;
  float lift_i_err_    = 0.0f;
  float arm_prev_err_  = 0.0f;
  float arm_i_err_     = 0.0f;

  bool manual_active_ = false;

  void solve_targets_(float target_height_in,
                      float& out_lift_in, float& out_arm_deg) const;
  float interp_arm_angle_(float target_height_in) const;
};
