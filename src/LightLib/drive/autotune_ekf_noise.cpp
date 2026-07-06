#include "LightLib/drive/autotune.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

#include "LightLib/drive/ekf.hpp"
#include "LightLib/drive/odometry.hpp"
#include "LightLib/util/extras.hpp"
#include "LightLib/util/util.hpp"
#include "path_runtime_internal.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace light {

// Robot parked. var(per-tick deltas)/dt is the process-noise floor; pushes
// to the live EKF.
void autotune_ekf_noise(int sampleMs, int durationMs, int warmupMs) {
  light::Drive* chassis = light::getChassis();
  if (!chassis) {
    printf("[AUTOTUNE][ekf] no chassis\n");
    autotune_result_set("EKF FAILED");
    return;
  }

  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  EzPauseGuard guard(chassis, L, R);
  if (L) L->move_voltage(0);
  if (R) R->move_voltage(0);

  pros::delay(warmupMs);

  Pose prev = light::getPose();
  double prevTheta = chassis->drive_imu_get();
  std::vector<float> dx, dy, dth;

  uint32_t t0 = pros::millis();
  while ((int)(pros::millis() - t0) < durationMs) {
    pros::delay(sampleMs);
    Pose p = light::getPose();
    double t = chassis->drive_imu_get();
    dx.push_back(p.x - prev.x);
    dy.push_back(p.y - prev.y);
    dth.push_back((float)((t - prevTheta) * units::RAD_PER_DEG));
    prev = p;
    prevTheta = t;
  }

  auto var = [](const std::vector<float>& v) -> float {
    if (v.size() < 2) return 0.0f;
    double m = 0;
    for (float x : v) m += x;
    m /= v.size();
    double s = 0;
    for (float x : v) s += (x - m) * (x - m);
    return (float)(s / (v.size() - 1));
  };

  float dt = sampleMs / 1000.0f;
  float Qpos = 0.5f * (var(dx) + var(dy)) / dt;
  float Qtheta = var(dth) / dt;
  float Qvel = (dt > 1e-6f) ? (Qpos / dt) : Qpos;

  MCLConfig cfg = light::ekf::config();
  cfg.ekfQPos = Qpos;
  cfg.ekfQTheta = Qtheta;
  cfg.ekfQVel = Qvel;
  light::ekf::setConfig(cfg);

  printf("[AUTOTUNE][ekf] samples=%d  Q_pos=%.5f in^2/s  Q_theta=%.6f rad^2/s  Q_vel=%.4f\n",
         (int)dx.size(), Qpos, Qtheta, Qvel);
  printf("[AUTOTUNE][ekf] applied live. Transcribe into EKF_Q_* macros in main.cpp to persist.\n");
  // %.1g keeps both values ≤5 chars (e.g. 0.001 / 1e-06) so the line fits 18.
  autotune_result_set("Qp%.1g Qt%.1g", Qpos, Qtheta);
}

}  // namespace light
