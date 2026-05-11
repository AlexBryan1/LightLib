/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "LightLib/api.h"

/**
 * \file piston.hpp
 *
 * Stateful wrapper around `pros::adi::DigitalOut` that tracks toggle state
 * and offers convenient button-driven activation modes.
 */

namespace light {
/**
 * Stateful 3-wire pneumatic piston with toggle helpers.
 *
 * Wraps a `pros::adi::DigitalOut` and remembers whether the piston is
 * currently extended. Use button_toggle() for one-button toggling or
 * buttons() for two-button extend/retract.
 */
class Piston {
 public:
  /** Underlying PROS digital output. Exposed for advanced use. */
  pros::adi::DigitalOut piston;

  /**
   * Piston constructor.
   *
   * The starting position of your piston defaults to false.
   *
   * \param input_port
   *        the ports of your pistons
   * \param default_state
   *        starting state of your piston
   */
  Piston(int input_port, bool default_state = false);

  /**
   * Piston constructor in 3 wire expander.
   *
   * The starting position of your piston defaults to false.
   *
   * \param input_ports
   *        the ports of your pistons
   * \param default_state
   *        starting state of your piston
   */
  Piston(int input_port, int expander_smart_port, bool default_state = false);

  /**
   * Sets the piston to the input.
   *
   * \param input
   *        true sets to the opposite of the starting position
   */
  void set(bool input);

  /**
   * Returns current piston state.
   */
  bool get();

  /**
   * One button toggle for the piston.
   *
   * \param toggle
   *        an input button
   */
  void button_toggle(int toggle);

  /**
   * Two-button trigger: one button extends, the other retracts.
   *
   * \param active
   *        button id that sets the piston to true
   * \param deactive
   *        button id that sets the piston to false
   */
  void buttons(int active, int deactive);

 private:
  bool reversed = false;
  bool current = false;
  int last_press = 0;
};
};  // namespace light