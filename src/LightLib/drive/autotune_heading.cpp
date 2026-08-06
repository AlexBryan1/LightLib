#include "LightLib/drive/autotune.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "LightLib/util/extras.hpp"
#include "autotune_common.hpp"

namespace light {

// Heading autotuner. Drives fwd/back with heading correction on while injecting a
// small heading-target step, and scores on the drivetrain-side encoder offset
// (drive_sensor_left - drive_sensor_right) — a physical straightness measure that
// equals trackWidth * heading(rad). Reuses the shared runVelocityAutotune()
// (regula falsi bracket+bisect over kP, then kD) so behavior matches the
// drive/turn/swing tuners.
void autotune_heading_pid(float driveDistIn, float headingStepDeg,
                          float overshootThreshDeg, int maxIters) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!chassis || !L || !R) {
    printf("[AUTOTUNE][heading] Failed: No chassis or motor groups resolved.\n");
    return;
  }

  // Side-offset (L_enc - R_enc) = trackWidth * heading_rad, so a heading step of
  // headingStepDeg settles the offset at trackWidth * step_rad. Guard against an
  // unconfigured track width.
  double trackW = chassis->drive_width_get();
  if (trackW < 0.1) trackW = 11.0;  // sane fallback (in)
  const float targetOffsetIn = (float)(trackW * headingStepDeg     * M_PI / 180.0);
  const float threshOffsetIn = (float)(trackW * overshootThreshDeg * M_PI / 180.0);

  const int kProbeSpeed = 90;  // moderate: leaves voltage headroom for the correction

  VelocityTunerStep s;
  s.setConsts = [chassis](float p, float d) {
    chassis->pid_heading_constants_set((double)p, 0.0, (double)d, 0.0);
  };
  s.getConsts = [chassis]() -> std::pair<float, float> {
    PID::Constants c = chassis->pid_heading_constants_get();
    return std::make_pair((float)c.kp, (float)c.kd);
  };
  // Inject a heading-target step, then drive so the heading loop must chase it
  // under real drive-coupling. Alternating sgn ping-pongs the robot back.
  s.setTarget = [=](float sgn) {
    chassis->headingPID.target_set(sgn * (double)headingStepDeg);
    chassis->pid_drive_set(sgn * (double)driveDistIn, kProbeSpeed,
                           /*slew_on=*/false, /*toggle_heading=*/true);
  };
  // Score on how far the two drive sides diverge = straightness error.
  s.readPos = [chassis]() -> float {
    return (float)(chassis->drive_sensor_left() - chassis->drive_sensor_right());
  };
  // Zero encoders (offset baseline) AND IMU (heading) before each step.
  s.resetSensor = [chassis]() {
    chassis->drive_sensor_reset();
    chassis->drive_imu_reset(0.0);
  };
  s.target          = targetOffsetIn;
  s.targetMag       = targetOffsetIn;
  s.overshootThresh = threshOffsetIn;
  s.label           = "heading";

  runVelocityAutotune(chassis, L, R, s, maxIters,
                      /*kpMaxCap=*/25.0f,        // was 10 — below the kP=11 default; raised
                      /*kpBoostFactor=*/1.3f,
                      /*kdMaxCapMul=*/25.0f,
                      /*tolFrac=*/0.05f,
                      /*maxMotionMs=*/8000);
}

}  // namespace light
