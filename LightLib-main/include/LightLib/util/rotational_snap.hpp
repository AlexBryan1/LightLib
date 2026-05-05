#pragma once

#include <vector>

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

/**
 * \file rotational_snap.hpp
 *
 * Manual-then-snap mechanism controller.
 *
 * Drives a motor manually while the operator is pressing a button or
 * deflecting a joystick, and automatically snaps to the nearest preset
 * angle the instant the operator stops.
 */

namespace light {

/**
 * Manual-then-snap controller for a motor + rotation-sensor pair.
 *
 * Pair any `pros::Motor` with a `pros::Rotation` sensor on the same axle.
 * Call update() once per opcontrol tick where `manual_input` is the blended
 * button + joystick value in the range -127..127.
 *
 * While `|input| > deadband` the motor runs manually. The first tick where
 * input transitions back to zero, the controller latches onto the nearest
 * snap angle and runs a P controller to hold it.
 */
class RotationalSnap {
 public:
  /**
   * \param motor
   *        motor to drive
   * \param sensor
   *        rotation sensor on the same axis
   * \param snap_angles_deg
   *        list of preset angles (degrees) to snap to
   * \param kP
   *        proportional gain on the snap controller
   * \param tolerance_deg
   *        considered settled within this many degrees
   * \param max_snap_speed
   *        speed cap while snapping, 0..127
   * \param input_deadband
   *        |input| ≤ this is treated as zero
   */
  RotationalSnap(pros::AbstractMotor& motor,
                 pros::Rotation& sensor,
                 std::vector<double> snap_angles_deg,
                 double kP = 1.5,
                 double tolerance_deg = 1.0,
                 int max_snap_speed = 80,
                 int input_deadband = 5);

  /**
   * Advance one tick.
   *
   * Call once per opcontrol loop. `|input| > deadband` → manual control.
   * Going from non-zero to zero latches the nearest snap angle and runs a
   * P-controller to it.
   *
   * \param manual_input
   *        operator input, -127..127
   *
   * \note INVARIANT: must be called every tick while the controller is
   *       connected. If a tick is missed between non-zero and zero input,
   *       the latch may be skipped (the snap target won't engage until the
   *       next non-zero → zero transition).
   */
  void update(int manual_input);

  /**
   * Replace the snap angle list at runtime.
   *
   * \param angles_deg
   *        new list of angles in degrees
   */
  void set_snap_angles(std::vector<double> angles_deg);

  /**
   * Enable or disable snap behavior.
   *
   * \param on
   *        true = normal behavior, false = pure passthrough (manual only)
   */
  void set_enabled(bool on);

  /** \return Current sensor reading in degrees. */
  double get_position_deg() const;

  /** \return Current snap target in degrees, NaN if none latched. */
  double get_target_deg() const;

  /** \return true while the snap controller is actively driving to a target. */
  bool is_snapping() const { return has_target_; }

 private:
  double nearest_snap_(double pos_deg) const;

  pros::AbstractMotor& motor_;
  pros::Rotation& sensor_;
  std::vector<double> snaps_;

  double kP_;
  double tol_;
  double target_ = 0.0;
  int max_speed_;
  int deadband_;
  bool enabled_ = true;
  bool was_active_ = false;
  bool has_target_ = false;
};

}  // namespace light
