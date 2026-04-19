#include "LightLib/auton_timer.hpp"
#include "pros/rtos.hpp"

namespace light {

uint32_t auton_start_ms = 0;

uint32_t auton_elapsed() {
    return pros::millis() - auton_start_ms;
}

void wait_until_auton(uint32_t ms) {
    uint32_t target = auton_start_ms + ms;
    uint32_t now = pros::millis();
    if (now < target) pros::delay(target - now);
}

}  // namespace light
