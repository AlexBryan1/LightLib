#include "LightLib/lightcast.hpp"
#include "pros/rtos.hpp"

// 5 Hz LightCast update task. Spawned by light::init() after lightcast::init().
// Kept as a separate TU so the ~20 ms-per-tick ray-cast work stays off the
// 100 Hz odom task — predict() runs inline on that task; only update() (the
// expensive part) runs here.

namespace light::lightcast {

static pros::Task* lightcastTask_ = nullptr;

void startTask() {
    if (lightcastTask_ != nullptr) return;
    lightcastTask_ = new pros::Task([] {
        while (true) {
            update();
            pros::delay(200);
        }
    }, "lightcast_task");
}

}  // namespace light::lightcast
