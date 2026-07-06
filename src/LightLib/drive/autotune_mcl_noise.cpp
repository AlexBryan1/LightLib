#include "LightLib/drive/autotune.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "LightLib/drive/lightcast.hpp"
#include "LightLib/util/extras.hpp"
#include "LightLib/util/util.hpp"
#include "path_runtime_internal.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace light {

// Robot parked with each distance sensor pointed at a wall in range.
// sensorSigmaIn = mean per-sensor stddev (in); outlierGapIn = max(3·σ, 1.0).
void autotune_mcl_noise(int sampleMs, int durationMs, int warmupMs) {
  light::Drive* chassis = light::getChassis();
  if (!chassis) {
    printf("[AUTOTUNE][mcl] no chassis\n");
    autotune_result_set("MCL FAILED");
    return;
  }

  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  EzPauseGuard guard(chassis, L, R);
  if (L) L->move_voltage(0);
  if (R) R->move_voltage(0);

  const auto& specs = light::lightcast::sensors();
  if (specs.empty()) {
    printf("[AUTOTUNE][mcl] no distance sensors configured — nothing to tune\n");
    autotune_result_set("MCL FAILED");
    return;
  }

  pros::delay(warmupMs);

  std::vector<std::vector<float>> samples(specs.size());
  uint32_t t0 = pros::millis();
  while ((int)(pros::millis() - t0) < durationMs) {
    pros::delay(sampleMs);
    for (size_t k = 0; k < specs.size(); ++k) {
      if (!specs[k].sensor) continue;
      int mm = specs[k].sensor->get();
      if (mm <= 0 || mm >= 9999) continue;
      samples[k].push_back(mm * units::IN_PER_MM);
    }
  }

  auto stddev = [](const std::vector<float>& v) -> float {
    if (v.size() < 2) return 0.0f;
    double m = 0;
    for (float x : v) m += x;
    m /= v.size();
    double s = 0;
    for (float x : v) s += (x - m) * (x - m);
    return (float)std::sqrt(s / (v.size() - 1));
  };

  float sigmaSum = 0.0f;
  int contributing = 0;
  for (size_t k = 0; k < specs.size(); ++k) {
    if (samples[k].size() < 5) {
      printf("[AUTOTUNE][mcl] sensor %d: only %d samples — skipped\n",
             (int)k, (int)samples[k].size());
      continue;
    }
    float s = stddev(samples[k]);
    printf("[AUTOTUNE][mcl] sensor %d: n=%d  sigma=%.3f in\n",
           (int)k, (int)samples[k].size(), s);
    sigmaSum += s;
    contributing++;
  }
  if (contributing == 0) {
    printf("[AUTOTUNE][mcl] no usable sensor data — aborting\n");
    autotune_result_set("MCL FAILED");
    return;
  }

  float sigma = sigmaSum / contributing;
  if (sigma < 0.25f) sigma = 0.25f;  // honest precision floor
  float gap = std::max(3.0f * sigma, 1.0f);

  MCLConfig cfg = light::lightcast::config();
  cfg.sensorSigmaIn = sigma;
  cfg.outlierGapIn = gap;
  light::lightcast::setConfig(cfg);

  printf("[AUTOTUNE][mcl] sensorSigmaIn=%.3f in  outlierGapIn=%.3f in  (avg of %d sensors)\n",
         sigma, gap, contributing);
  printf("[AUTOTUNE][mcl] applied live. Transcribe into mcl.sensorSigmaIn / mcl.outlierGapIn in default_constants() (autons.cpp) to persist.\n");
  autotune_result_set("MCL sg%.2f gp%.1f", sigma, gap);
}

}  // namespace light
