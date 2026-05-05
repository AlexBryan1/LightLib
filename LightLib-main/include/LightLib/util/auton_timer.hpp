#pragma once
#include <cstdint>

/**
 * \file auton_timer.hpp
 *
 * Simple wall-clock helpers anchored at the start of the autonomous routine.
 * Useful for time-budgeting auton sequences.
 */

namespace light {

/**
 * Millisecond timestamp captured right before the selected auton routine
 * runs. Set by LightLib in `autonomous()`; read by the helpers below.
 */
extern uint32_t auton_start_ms;

/** \return Milliseconds since the selected auton routine began executing. */
uint32_t auton_elapsed();

/**
 * Block until `ms` milliseconds have elapsed since auton started. If that
 * point is already in the past, returns immediately.
 *
 * \param ms
 *        wall-clock target, in ms since auton start
 */
void wait_until_auton(uint32_t ms);

}  // namespace light
