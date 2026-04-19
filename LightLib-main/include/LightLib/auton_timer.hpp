#pragma once
#include <cstdint>

namespace light {

// Millis timestamp captured right before the selected auton routine runs.
// Set by LightLib in autonomous(); read by the helpers below.
extern uint32_t auton_start_ms;

// Milliseconds since the selected auton routine began executing.
uint32_t auton_elapsed();

// Block until `ms` milliseconds have elapsed since auton started.
// If that point is already in the past, return immediately.
void wait_until_auton(uint32_t ms);

}  // namespace light
