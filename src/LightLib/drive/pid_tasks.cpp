/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "LightLib/drive/drive.hpp"
#include "LightLib/util/util.hpp"
#include "pros/misc.hpp"

using namespace light;

void Drive::ez_auto_task() {
  while (true) {
    ez_tracking_task();

    switch (drive_mode_get()) {
      case DRIVE:
        drive_pid_task();
        break;
      case TURN ... TURN_TO_POINT:
        turn_pid_task();
        break;
      case SWING:
        swing_pid_task();
        break;
      case POINT_TO_POINT:
        ptp_task();
        break;
      case PURE_PURSUIT:
        pp_task();
        break;
      case DISABLE:
        break;
      default:
        break;
    }

    util::AUTON_RAN = drive_mode_get() != DISABLE ? true : false;

    pros::delay(light::util::DELAY_TIME);
  }
}

void Drive::drive_pid_task() {
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  headingPID.compute(drive_imu_get());

  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());

  double l_drive_out = leftPID.output;
  double r_drive_out = rightPID.output;

  double max_slew_out = fmax(slew_left.output(), slew_right.output());
  double faster_side = fmax(fabs(l_drive_out), fabs(r_drive_out));
  if (faster_side > max_slew_out) {
    l_drive_out *= (max_slew_out / faster_side);
    r_drive_out *= (max_slew_out / faster_side);
  }

  double imu_out = heading_on ? headingPID.output : 0;

  double l_out = l_drive_out + imu_out;
  double r_out = r_drive_out - imu_out;

  max_slew_out = fmax(slew_left.output(), slew_right.output());
  faster_side = fmax(fabs(l_out), fabs(r_out));
  if (faster_side > max_slew_out) {
    l_out *= (max_slew_out / faster_side);
    r_out *= (max_slew_out / faster_side);
  }

  l_out += friction_ff(l_out);
  r_out += friction_ff(r_out);

  if (drive_toggle)
    private_drive_set(l_out, r_out);
}

void Drive::turn_pid_task() {
  if (mode == TURN) {
    turnPID.compute(drive_imu_get());
  } else {
    double a_target = util::absolute_angle_to_point(point_to_face[!ptf1_running], odom_pose_get());
    a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
    double error = a_target - odom_theta_get();
    turnPID.compute_error(error, odom_theta_get());
  }

  slew_turn.iterate(drive_imu_get());

  double gyro_out = util::clamp(turnPID.output, slew_turn.output(), -slew_turn.output());

  // Floor at pid_turn_min once we're inside the StartI region.
  if (turnPID.constants.ki != 0 && (fabs(turnPID.target_get()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
    if (pid_turn_min_get() != 0)
      gyro_out = util::clamp(gyro_out, pid_turn_min_get(), -pid_turn_min_get());
  }

  // friction_ff is odd in v_target, so right's negation produces the right sign.
  double turn_l = gyro_out;
  double turn_r = -gyro_out;
  turn_l += friction_ff(turn_l);
  turn_r += friction_ff(turn_r);

  if (drive_toggle)
    private_drive_set(turn_l, turn_r);
}

void Drive::swing_pid_task() {
  swingPID.compute(drive_imu_get());
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  double current = slew_swing_using_angle ? drive_imu_get() : (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
  slew_swing.iterate(current);

  double swing_out = util::clamp(swingPID.output, slew_swing.output(), -slew_swing.output());

  if (swingPID.constants.ki != 0 && (fabs(swingPID.target_get()) > swingPID.constants.start_i && fabs(swingPID.error) < swingPID.constants.start_i)) {
    if (pid_swing_min_get() != 0)
      swing_out = util::clamp(swing_out, pid_swing_min_get(), -pid_swing_min_get());
  }

  double opposite_output = 0;
  double scale = swing_out / max_speed;
  if (drive_toggle) {
    if (current_swing == LEFT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? rightPID.output : (swing_opposite_speed * scale);
      double l_v = swing_out;
      double r_v = opposite_output;
      private_drive_set(l_v + friction_ff(l_v), r_v + friction_ff(r_v));
    } else if (current_swing == RIGHT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? leftPID.output : -(swing_opposite_speed * scale);
      double l_v = opposite_output;
      double r_v = -swing_out;
      private_drive_set(l_v + friction_ff(l_v), r_v + friction_ff(r_v));
    }
  }
}

void Drive::ptp_task() {
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());
  double max_slew_out = fmax(slew_left.output(), slew_right.output());

  // is_past_target instead of distance formula avoids impossible-movement edge case.
  double temp_target = is_past_target(odom_target, odom_pose_get());
  int dir = (current_drive_direction == REV ? -1 : 1);
  int flipped = util::sgn(temp_target) != util::sgn(past_target) ? -1 : 1;

  // Synthetic "current sensor value" for the xy PID.
  new_current_fake += xy_delta_fake * ((dir * flipped));
  xyPID.compute_error(fabs(temp_target) * dir * flipped, new_current_fake);

  pose ptf = point_to_face[!ptf1_running];
  double a_target = util::absolute_angle_to_point(ptf, odom_pose_get());
  a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
  double wrapped_a_target = a_target - odom_theta_get();
  current_a_odomPID.compute_error(wrapped_a_target, odom_theta_get());

  // Scale xy down when off-heading so turning takes priority.
  double xy_out = xyPID.output;
  xy_out = util::clamp(xy_out, max_slew_out);
  double scale = 1.0 - ((1.0 - cos(util::to_rad(current_a_odomPID.error))) / odom_turn_bias_amount);
  if (odom_turn_bias_enabled())
    xy_out *= scale;
  double a_out = current_a_odomPID.output;

  double faster_side = fmax(fabs(xy_out), fabs(a_out));
  if (faster_side > max_slew_out) {
    xy_out *= (max_slew_out / faster_side);
    a_out *= (max_slew_out / faster_side);
  }

  double l_out = xy_out + a_out;
  double r_out = xy_out - a_out;

  faster_side = fmax(fabs(l_out), fabs(r_out));
  if (faster_side > max_slew_out) {
    l_out *= (max_slew_out / faster_side);
    r_out *= (max_slew_out / faster_side);
  }

  l_out += friction_ff(l_out);
  r_out += friction_ff(r_out);

  if (drive_toggle)
    private_drive_set(l_out, r_out);

  // Keep PIDs current for wait_until.
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());
}

void Drive::boomerang_task() {
  int target_index = pp_index;
  pose target = pp_movements[target_index].target;

  int dir = current_drive_direction == REV ? -1 : 1;

  double h = util::distance_to_point(target, odom_pose_get()) * odom_boomerang_dlead_get();
  double max = max_boomerang_distance;
  h = h > max ? max : h;
  h *= dir;

  pose temp = util::vector_off_point(-h, pp_movements[target_index].target);
  temp.theta = target.theta;

  if (util::distance_to_point(target, odom_pose_get()) < odom_look_ahead_get() / 2.0) {
    temp = target;
  }

  if (odom_target.x != temp.x || odom_target.y != temp.y) {
    bool slew_on = slew_left.enabled() || slew_right.enabled() ? true : false;
    raw_pid_odom_ptp_set({temp, pp_movements[target_index].drive_direction, pp_movements[target_index].max_xy_speed}, slew_on);
  }

  ptp_task();
}

void Drive::pp_task() {
  if (fabs(util::distance_to_point(pp_movements[pp_index].target, odom_pose_get())) < odom_look_ahead_get()) {
    if (pp_index < pp_movements.size() - 1) {
      pp_index = pp_index >= pp_movements.size() - 1 ? pp_index : pp_index + 1;
      bool slew_on = slew_left.enabled() || slew_right.enabled() ? true : false;
      if (!current_slew_on) slew_on = false;
      raw_pid_odom_ptp_set(pp_movements[pp_index], slew_on);
    }
  }

  if (pp_movements[pp_index].target.theta != ANGLE_NOT_SET) {
    boomerang_task();
  } else {
    ptp_task();
  }
}