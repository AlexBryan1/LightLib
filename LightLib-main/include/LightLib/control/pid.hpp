/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "LightLib/api.h"
#include "LightLib/util/util.hpp"

/**
 * \file pid.hpp
 *
 * Discrete PID controller with rich exit-condition support.
 *
 * Computes a control output from an error signal, with optional integral
 * windup guard, derivative-kick suppression, and a configurable family of
 * exit conditions (small/big error timers, velocity exit, current limit).
 */

namespace light {
/**
 * Discrete PID controller with rich exit-condition support.
 *
 * Configure once via the constructor or constants_set(), then call compute()
 * each cycle with the latest sensor reading. Use exit_condition() to detect
 * when the controller has settled.
 */
class PID {
 public:
  /**
   * Default constructor.
   */
  PID();

  /**
   * Constructor with constants.
   *
   * \param p
   *        kP
   * \param i
   *        ki
   * \param d
   *        kD
   * \param p_start_i
   *        error value that i starts within
   * \param name
   *        std::string of name that prints
   */
  PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");

  /**
   * Set constants for PID.
   *
   * \param p
   *        kP
   * \param i
   *        ki
   * \param d
   *        kD
   * \param p_start_i
   *        error value that i starts within
   */
  void constants_set(double p, double i = 0, double d = 0, double p_start_i = 0);

  /**
   * PID gain constants.
   */
  struct Constants {
    double kp;       ///< Proportional gain.
    double ki;       ///< Integral gain.
    double kd;       ///< Derivative gain.
    double start_i;  ///< Error threshold below which integral begins accumulating.
  };

  /**
   * Exit condition timing and thresholds.
   */
  struct exit_condition_ {
    int small_exit_time = 0;     ///< ms to exit when error stays within `small_error`.
    double small_error = 0;      ///< Error band that starts the small-exit timer.
    int big_exit_time = 0;       ///< ms to exit when error stays within `big_error`.
    double big_error = 0;        ///< Error band that starts the big-exit timer.
    int velocity_exit_time = 0;  ///< ms of zero velocity before exiting.
    int mA_timeout = 0;          ///< ms above current limit before exiting.
  };

  /**
   * Set's constants for exit conditions.
   *
   * \param p_small_exit_time
   *        sets small_exit_time, timer for to exit within smalL_error
   * \param p_small_error
   *        sets smalL_error, timer will start when error is within this
   * \param p_big_exit_time
   *        sets big_exit_time, timer for to exit within big_error
   * \param p_big_error
   *        sets big_error, timer will start when error is within this
   * \param p_velocity_exit_time
   *        sets velocity_exit_time, timer will start when velocity is 0
   */
  void exit_condition_set(int p_small_exit_time, double p_small_error, int p_big_exit_time = 0, double p_big_error = 0, int p_velocity_exit_time = 0, int p_mA_timeout = 0);

  /**
   * Sets PID target.
   *
   * \param target
   *        new target for PID
   */
  void target_set(double input);

  /**
   * Computes PID.
   *
   * \param current
   *        current sensor value
   */
  double compute(double current);

  /**
   * Computes PID, but you compute the error yourself.
   *
   * Current is only used here for calculative derivative to solve derivative kick.
   *
   * \param err
   *        error for the PID, you need to calculate this yourself
   * \param current
   *        current sensor value
   */
  double compute_error(double err, double current);

  /**
   * Returns target value.
   */
  double target_get();

  /**
   * Returns constants.
   */
  Constants constants_get();

  /**
   * Returns true if PID constants are set, returns false if they're all 0.
   */
  bool constants_set_check();

  /**
   * Resets all variables to 0.  This does not reset constants.
   */
  void variables_reset();

  /** Active PID gain constants (read/write). */
  Constants constants;

  /** Active exit-condition thresholds (read/write). */
  exit_condition_ exit;

  /**
   * Updates a secondary sensor for velocity exiting.  Ideal use is IMU during normal drive motions.
   *
   * \param secondary_sensor
   *        secondary sensor value
   */
  void velocity_sensor_secondary_set(double secondary_sensor);

  /**
   * Returns the updated secondary sensor for velocity exiting.
   */
  double velocity_sensor_secondary_get();

  /**
   * Boolean for if the secondary sensor will be updated or not.  True uses this sensor, false does not.
   *
   * \param toggle
   *        true uses this sensor, false does not
   */
  void velocity_sensor_secondary_toggle_set(bool toggle);

  /**
   * Returns the boolean for if the secondary sensor will be updated or not.  True uses this sensor, false does not.
   */
  bool velocity_sensor_secondary_toggle_get();

  /**
   * Sets the threshold that the main sensor will return 0 velocity within.
   *
   * \param zero
   *        a small double
   */
  void velocity_sensor_main_exit_set(double zero);

  /**
   * Returns the threshold that the main sensor will return 0 velocity within.
   */
  double velocity_sensor_main_exit_get();

  /**
   * Sets the threshold that the secondary sensor will return 0 velocity within.
   *
   * \param zero
   *        a small double
   */
  void velocity_sensor_secondary_exit_set(double zero);

  /**
   * Returns the threshold that the secondary sensor will return 0 velocity within.
   */
  double velocity_sensor_secondary_exit_get();

  /**
   * Iterative exit condition for PID.
   *
   * \param print = false
   *        if true, prints when complete
   */
  light::exit_output exit_condition(bool print = false);

  /**
   * Iterative exit condition for PID.
   *
   * \param sensor
   *        a pros motor on your mechanism
   * \param print = false
   *        if true, prints when complete
   */
  light::exit_output exit_condition(pros::Motor sensor, bool print = false);

  /**
   * Iterative exit condition for PID.
   *
   * \param sensor
   *        pros motors on your mechanism
   * \param print = false
   *        if true, prints when complete
   */
  light::exit_output exit_condition(std::vector<pros::Motor> sensor, bool print = false);

  /**
   * Iterative exit condition for PID.
   *
   * \param sensor
   *        pros motor group on your mechanism
   * \param print = false
   *        if true, prints when complete
   */
  light::exit_output exit_condition(pros::MotorGroup sensor, bool print = false);

  /**
   * Sets the name of the PID that prints during exit conditions.
   *
   * \param name
   *        the name of the mechanism for printing
   */
  void name_set(std::string name);

  /**
   * Returns the name of the PID that prints during exit conditions.
   */
  std::string name_get();

  /**
   * Enables / disables i resetting when sgn of error changes.
   *
   * True resets, false doesn't.
   *
   * \param toggle
   *        true resets, false doesn't
   */
  void i_reset_toggle(bool toggle);

  /**
   * Returns if i will reset when sgn of error changes.
   *
   * True resets, false doesn't.
   */
  bool i_reset_get();

  /**
   * Resets all timers for exit conditions.
   */
  void timers_reset();

  /**
   * \name Live PID state
   * Updated each call to compute() / compute_error().
   * @{
   */
  double output = 0.0;        ///< Last computed control output.
  double cur = 0.0;           ///< Last sensor value seen.
  double error = 0.0;         ///< Current error (target − cur).
  double target = 0.0;        ///< Current target.
  double prev_error = 0.0;    ///< Previous-cycle error.
  double prev_current = 0.0;  ///< Previous-cycle sensor value.
  double integral = 0.0;      ///< Accumulated integral term.
  double derivative = 0.0;    ///< Last derivative term.
  long time = 0;              ///< Timestamp of last compute (ms).
  long prev_time = 0;         ///< Timestamp of previous compute (ms).
  /** @} */

 private:
  double velocity_zero_main = 0.05;
  double velocity_zero_secondary = 0.075;
  int i = 0, j = 0, k = 0, l = 0, m = 0;
  bool is_mA = false;
  double second_sensor = 0.0;

  std::string name;
  bool name_active = false;
  void exit_condition_print(light::exit_output exit_type);
  bool reset_i_sgn = true;
  double raw_compute();
  bool use_second_sensor = false;
};
};  // namespace light