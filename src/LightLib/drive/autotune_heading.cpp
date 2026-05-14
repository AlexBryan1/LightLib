#include "LightLib/drive/autotune.hpp"

#include <cstdio>
#include <utility>

#include "LightLib/util/extras.hpp"
#include "autotune_common.hpp"

namespace light {

// Heading autotuner. Drives straight with heading correction on, scoring
// oscillation on the |L−R| motor voltage differential — i.e. the heading
// PID's own controller effort. Reuses the shared runVelocityAutotune()
// (regula falsi bracket+bisect over kP, then kD) so behavior matches
// drive/turn/swing tuners.
void autotune_heading_pid(float driveDistIn, float peakDiffMv, int maxIters) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!chassis || !L || !R) {
    printf("[AUTOTUNE][heading] Failed: No chassis or motor groups resolved.\n");
    return;
  }

  VelocityTunerStep s;
  s.setConsts = [chassis](float p, float d) {
    chassis->pid_heading_constants_set((double)p, 0.0, (double)d, 0.0);
  };
  s.getConsts = [chassis]() -> std::pair<float, float> {
    PID::Constants c = chassis->pid_heading_constants_get();
    return std::make_pair((float)c.kp, (float)c.kd);
  };
  s.setTarget = [&](float sgn) {
    chassis->pid_drive_set(sgn * (double)driveDistIn, 127,
                           /*slew_on=*/false, /*toggle_heading=*/true);
  };
  // Score on the heading PID's output: signed differential motor voltage.
  // Zero = straight; sign flips and large peaks = oscillation.
  s.readPos = [L, R]() -> float {
    return (float)(L->get_voltage() - R->get_voltage());
  };
  s.resetSensor = [chassis]() { chassis->drive_angle_set(0.0); };
  s.target          = 0.0f;
  s.targetMag       = 0.0f;
  s.overshootThresh = peakDiffMv;
  s.label           = "heading";

  runVelocityAutotune(chassis, L, R, s, maxIters,
                      /*kpMaxCap=*/10.0f,
                      /*kpBoostFactor=*/1.3f,
                      /*kdMaxCapMul=*/25.0f,
                      /*tolFrac=*/0.05f,
                      /*maxMotionMs=*/8000);
}

}  // namespace light
