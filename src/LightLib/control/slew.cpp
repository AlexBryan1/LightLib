/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "LightLib/lib.hpp"

using namespace light;

// Constructor
slew::slew() {}
slew::slew(double distance, int minimum_speed) {
  constants_set(distance, minimum_speed);
}

// Set constants
void slew::constants_set(double distance, int minimum_speed) {
  constants.min_speed = minimum_speed;
  constants.distance_to_travel = distance;
}

void slew::speed_max_set(double speed) { max_speed = speed; }
double slew::speed_max_get() { return max_speed; }

bool slew::enabled() { return is_enabled; }                  // Is slew currently enabled?
double slew::output() { return last_output; }                // Returns output
slew::Constants slew::constants_get() { return constants; }  // Get constants

// Initialize for the movement
void slew::initialize(bool enabled, double maximum_speed, double target, double current) {
  is_enabled = maximum_speed < constants.min_speed ? false : enabled;
  max_speed = maximum_speed;

  // Guard: a zero slew distance (or target == current) makes the slope formula
  // divide by zero (x_intercept - current == 0). Disable slew in that case —
  // we'll just return max_speed every iterate().
  if (constants.distance_to_travel <= 0 || target == current) {
    is_enabled = false;
    sign = 0;
    return;
  }

  sign = util::sgn(target - current);
  x_intercept = current + ((constants.distance_to_travel * sign));
  y_intercept = max_speed * sign;
  slope = ((sign * constants.min_speed) - y_intercept) / (x_intercept - 0 - current);  // y2-y1 / x2-x1
}

// Iterate the slew throughout the movement
double slew::iterate(double current) {
  // Is slew still on?
  if (is_enabled) {
    // Error is distance away from completed slew
    error = x_intercept - current;

    // When the sign of error flips, slew is completed
    if (util::sgn(error) != sign)
      is_enabled = false;

    // Return y=mx+b
    else if (util::sgn(error) == sign)
      last_output = ((slope * error) + y_intercept) * sign;
  } else {
    // When slew is completed, return max speed
    last_output = max_speed;
  }

  return last_output;
}
