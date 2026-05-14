#include "LightLib/path/autotune.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "LightLib/util/extras.hpp"
#include "autotune_common.hpp"

namespace light {

void autotune_drive_pid(float driveDistIn, float overshootThreshIn, int maxIters) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!chassis || !L || !R) {
    printf("[AUTOTUNE][drive] Failed: No chassis or motor groups resolved.\n");
    return;
  }

  VelocityTunerStep s;
  s.setConsts = [&](float p, float d) {
    chassis->pid_drive_constants_set((double)p, 0.0, (double)d);
  };

  s.getConsts = std::function<std::pair<float, float>()>([chassis]() -> std::pair<float, float> {
    PID::Constants c = chassis->pid_drive_constants_get();
    return std::make_pair((float)c.kp, (float)c.kd);
  });

  s.setTarget   = [&](float sgn) { chassis->pid_drive_set(sgn * (double)driveDistIn, 127); };
  s.readPos     = [&]() -> float {
    return (float)(0.5 * (chassis->drive_sensor_left() + chassis->drive_sensor_right()));
  };
  s.resetSensor = [&]() { chassis->drive_sensor_reset(); };
  s.target      = driveDistIn;
  s.targetMag   = std::fabs(driveDistIn);
  s.overshootThresh = overshootThreshIn;
  s.label       = "drive";

  runVelocityAutotune(chassis, L, R, s, maxIters);
}

}  // namespace light
