#include "LightLib/drive/autotune.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "LightLib/util/extras.hpp"
#include "autotune_common.hpp"

namespace light {

void autotune_turn_pid(float turnAngleDeg, float overshootThreshDeg, int maxIters) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!chassis || !L || !R) {
    printf("[AUTOTUNE][turn] Failed: No chassis or motor groups resolved.\n");
    return;
  }

  VelocityTunerStep s;
  s.setConsts = [&](float p, float d) {
    chassis->pid_turn_constants_set((double)p, 0.0, (double)d);
  };

  s.getConsts = std::function<std::pair<float, float>()>([chassis]() -> std::pair<float, float> {
    PID::Constants c = chassis->pid_turn_constants_get();
    return std::make_pair((float)c.kp, (float)c.kd);
  });

  s.setTarget   = [&](float sgn) { chassis->pid_turn_set(sgn * (double)turnAngleDeg, 127); };
  s.readPos     = [&]() -> float { return (float)chassis->drive_imu_get(); };
  s.resetSensor = [&]() { chassis->drive_imu_reset(0.0); };
  s.target      = turnAngleDeg;
  s.targetMag   = std::fabs(turnAngleDeg);
  s.overshootThresh = overshootThreshDeg;
  s.label       = "turn";

  runVelocityAutotune(chassis, L, R, s, maxIters);
}

}  // namespace light
