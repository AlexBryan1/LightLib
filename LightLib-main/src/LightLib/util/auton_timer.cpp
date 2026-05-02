#include "LightLib/util/auton_timer.hpp"

#include "pros/rtos.hpp"

namespace light {

uint32_t auton_start_ms = 0;

uint32_t auton_elapsed() {
  return (uint32_t)(pros::millis() - auton_start_ms);
}

void wait_until_auton(uint32_t ms) {
  int32_t remain = (int32_t)((auton_start_ms + ms) - pros::millis());
  if (remain > 0) pros::delay((uint32_t)remain);
}

}  // namespace light
