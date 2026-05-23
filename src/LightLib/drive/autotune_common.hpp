#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>

#include "LightLib/drive/drive.hpp"
#include "pros/motor_group.hpp"

namespace light {

// Per-tuner-label record of whether the last run completed phase 2 cleanly.
// Lives in memory only — resets on reboot, no SD card involved.
extern std::map<std::string, bool> g_autotunePrevSuccess;

struct VelocityMotionSample {
  float peakAbsVel = 0.0f;   // peak |dpos/dt| in units/sec
  float peakAbsPos = 0.0f;   // peak |pos − resetSensor zero|
  int   crossings  = 0;      // number of (pos − target) sign flips
  int   settleMs   = -1;     // ms from probe start to 3-stop confirm; -1 = never settled
};

VelocityMotionSample sampleMotion(std::function<float()> readPos,
                                  pros::MotorGroup* L, pros::MotorGroup* R,
                                  float target,
                                  int maxMotionMs,
                                  int settleMs = 250,
                                  float settleVoltMv = 1500.0f);

struct VelocityTunerStep {
  std::function<void(float, float)> setConsts;
  std::function<std::pair<float, float>()> getConsts = []() {
    return std::make_pair(0.0f, 0.0f);
  };
  std::function<void(float)> setTarget;   // arg = sign (+1 / -1); flips each probe to skip back-drive
  std::function<float()> readPos;
  std::function<void()> resetSensor;
  float target;              // unsigned magnitude (default per entry point)
  float targetMag;
  float overshootThresh;
  const char* label;
};

// Two-phase bracket-and-bisect tuner.
// Phase 1: find largest kP that doesn't overshoot (kD=0). Bracket by doubling
//          kP until overshoot, then bisect.
// Phase 2: boost kP past the boundary (×kpBoostFactor) so the robot
//          intentionally overshoots/oscillates. With that aggressive kP fixed,
//          bracket+bisect kD upward to find the smallest kD that damps the
//          overshoot back under threshold.
// Result: aggressive kP with just-enough kD — faster response than pure-P
// "barely no overshoot", still no residual overshoot. kI stays 0.
void runVelocityAutotune(light::Drive* chassis,
                         pros::MotorGroup* L, pros::MotorGroup* R,
                         const VelocityTunerStep& s,
                         int maxItersPerPhase,
                         float kpMaxCap = 25.0f,
                         float kpBoostFactor = 1.3f,
                         float kdMaxCapMul = 25.0f,
                         float tolFrac = 0.05f,
                         int maxMotionMs = 4000,
                         float kpSuccessBoost = 1.25f);

}  // namespace light
