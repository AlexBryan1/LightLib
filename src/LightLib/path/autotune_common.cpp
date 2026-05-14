#include "autotune_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "pros/rtos.hpp"

namespace light {

std::map<std::string, bool> g_autotunePrevSuccess;

VelocityMotionSample sampleMotion(std::function<float()> readPos,
                                  pros::MotorGroup* L, pros::MotorGroup* R,
                                  float target,
                                  int maxMotionMs,
                                  int settleMs,
                                  float settleVoltMv) {
  VelocityMotionSample s;
  uint32_t t0 = pros::millis();
  float prevPos = readPos();
  float prevDelta = prevPos - target;
  uint32_t prevMs = t0;

  int consecutiveStops = 0;
  uint32_t currentStopStart = 0;
  bool isCurrentlyStopped = false;

  pros::delay(40);  // Let the PID start commanding

  while ((int)(pros::millis() - t0) < maxMotionMs) {
    uint32_t now = pros::millis();
    float pos = readPos();
    float dt = (now - prevMs) / 1000.0f;

    if (dt > 0.0f) {
      float v = (pos - prevPos) / dt;
      if (std::fabs(v) > s.peakAbsVel) s.peakAbsVel = std::fabs(v);
    }
    if (std::fabs(pos) > s.peakAbsPos) s.peakAbsPos = std::fabs(pos);

    float delta = pos - target;
    if ((prevDelta < 0.0f) != (delta < 0.0f)) ++s.crossings;
    prevDelta = delta;

    prevPos = pos;
    prevMs = now;

    float vL = (float)std::abs(L->get_voltage());
    float vR = (float)std::abs(R->get_voltage());

    if (vL < settleVoltMv && vR < settleVoltMv) {
      if (!isCurrentlyStopped) {
        currentStopStart = now;
        isCurrentlyStopped = true;
      } else if ((int)(now - currentStopStart) >= settleMs) {
        consecutiveStops++;
        printf("[AUTOTUNE] Good stop confirmed (%d/3)\n", consecutiveStops);

        if (consecutiveStops >= 3) {
          break;
        }
        currentStopStart = now;
      }
    } else {
      if (consecutiveStops > 0) {
        printf("[AUTOTUNE] Stability broken! Resetting stop counter.\n");
      }
      consecutiveStops = 0;
      isCurrentlyStopped = false;
      currentStopStart = 0;
    }

    pros::delay(10);
  }
  return s;
}

void runVelocityAutotune(light::Drive* chassis,
                         pros::MotorGroup* L, pros::MotorGroup* R,
                         const VelocityTunerStep& s,
                         int maxItersPerPhase,
                         float kpMaxCap,
                         float kpBoostFactor,
                         float kdMaxCapMul,
                         float tolFrac,
                         int maxMotionMs,
                         float kpSuccessBoost) {

  // Fetch live constants directly from the chassis — start tuning from
  // whatever the user has in default_constants() rather than a fallback.
  auto currentVals = s.getConsts();
  float kpStart = currentVals.first;
  float kdLive  = currentVals.second;
  printf("[AUTOTUNE][%s] starting from live PID: kP=%.3f kD=%.3f\n",
         s.label, kpStart, kdLive);

  // Safeguard if the chassis was never configured (kP=0 → no motion possible).
  if (kpStart <= 0.001f) {
    kpStart = 0.5f;
    printf("[AUTOTUNE][%s] live kP was 0; seeding kpStart=0.5\n", s.label);
  }

  // In-memory boost on prior success. kpStart already came back via getConsts()
  // because the previous run wrote (kP, kD) to the chassis via setConsts().
  // Here we only decide whether to scale that starting kP up before phase 1.
  auto prevIt = g_autotunePrevSuccess.find(s.label);
  bool prevSucceeded = (prevIt != g_autotunePrevSuccess.end()) && prevIt->second;
  if (prevSucceeded) {
    float boosted = kpStart * kpSuccessBoost;
    printf("[AUTOTUNE][%s] previous run succeeded; boosting kP from %.3f to %.3f\n",
           s.label, kpStart, boosted);
    kpStart = boosted;
  }

  // Alternating-direction probe: each call flips sgn so the previous-probe's
  // end pose becomes the next probe's start pose. No back-drive needed.
  float sgn = +1.0f;
  auto runOnce = [&](float kP, float kD) -> VelocityMotionSample {
    s.setConsts(kP, kD);
    auto verify = s.getConsts();
    printf("[AUTOTUNE][%s]   wrote kP=%.3f kD=%.3f -> live reads kP=%.3f kD=%.3f\n",
           s.label, kP, kD, verify.first, verify.second);

    // Park the chassis BEFORE resetting the sensor. Otherwise the PID would
    // still be chasing the previous probe's target, see a sudden huge error
    // when the sensor zeros, and slam the motors in the old direction for a
    // tick before setTarget overrides it.
    chassis->drive_mode_set(light::DISABLE, /*stop_drive=*/true);
    pros::delay(50);

    s.resetSensor();
    pros::delay(50);

    s.setTarget(sgn);   // re-enables DRIVE/TURN/SWING mode via pid_*_set
    VelocityMotionSample m = sampleMotion(s.readPos, L, R, sgn * s.target, maxMotionMs);
    chassis->pid_wait();

    // Park again so motors don't keep nudging during the settle window.
    chassis->drive_mode_set(light::DISABLE, /*stop_drive=*/true);
    pros::delay(200);

    sgn = -sgn;
    return m;
  };

  const int oscThresh = 3;
  float pLow = 0.0f, pHigh = 0.0f;
  float peakLow = 0.0f, peakHigh = 0.0f;
  float kP = kpStart;
  int iter = 0;

  // ═══ Phase 1: Bracket kP using current value as floor ═════════
  while (iter < maxItersPerPhase) {
    VelocityMotionSample m = runOnce(kP, 0.0f);
    float absoluteOvershoot = m.peakAbsPos - s.targetMag;
    bool osc = (m.crossings >= oscThresh) || (absoluteOvershoot > s.overshootThresh);

    printf("[AUTOTUNE][%s] kP-bracket iter=%d kP=%.3f peakAbs=%.2f crossings=%d %s\n",
           s.label, iter, kP, m.peakAbsPos, m.crossings, osc ? "OSCILLATES" : "stable");

    ++iter;
    if (osc) { pHigh = kP; peakHigh = m.peakAbsPos; break; }
    pLow = kP; peakLow = m.peakAbsPos;
    kP *= 2.0f;
    if (kP >= kpMaxCap) { pHigh = kpMaxCap; peakHigh = m.peakAbsPos + 1.0f; break; }
  }

  // If even the live starting kP immediately overshoots, seed pLow with a tiny floor.
  if (pLow == 0.0f) {
    pLow = 0.01f;
    pHigh = kpStart;
    peakLow = 0.0f;
    if (peakHigh <= 0.0f) peakHigh = s.targetMag + s.overshootThresh + 1.0f;
  }

  // Regula falsi: interpolate to where peakAbsPos − targetMag == overshootThresh.
  while (iter < maxItersPerPhase && (pHigh - pLow) / pHigh > tolFrac) {
    float yLow  = peakLow  - s.targetMag;
    float yHigh = peakHigh - s.targetMag;
    float yTarget = s.overshootThresh;
    float frac = (yHigh - yLow) > 1e-3f
                   ? (yTarget - yLow) / (yHigh - yLow)
                   : 0.5f;
    frac = std::clamp(frac, 0.15f, 0.85f);
    kP = pLow + frac * (pHigh - pLow);

    VelocityMotionSample m = runOnce(kP, 0.0f);
    float absoluteOvershoot = m.peakAbsPos - s.targetMag;
    bool osc = (m.crossings >= oscThresh) || (absoluteOvershoot > s.overshootThresh);
    printf("[AUTOTUNE][%s] kP-bisect iter=%d kP=%.3f peakAbs=%.2f crossings=%d frac=%.2f %s\n",
           s.label, iter, kP, m.peakAbsPos, m.crossings, frac, osc ? "OSCILLATES" : "stable");
    if (osc) { pHigh = kP; peakHigh = m.peakAbsPos; }
    else     { pLow  = kP; peakLow  = m.peakAbsPos; }
    ++iter;
  }

  float kpBoundary = pLow;
  float kpBoosted  = kpBoundary * kpBoostFactor;

  // ═══ Phase 2: Find smallest kD that damps oscillation at boosted kP ════
  float dLow = 0.0f, dHigh = 0.0f;
  float peakDLow = 0.0f, peakDHigh = 0.0f;
  float kdMaxCap = kpBoosted * kdMaxCapMul;

  float kD = (kdLive > 0.01f) ? kdLive : (kpBoosted * 0.25f);
  int dIter = 0;

  while (dIter < maxItersPerPhase) {
    VelocityMotionSample m = runOnce(kpBoosted, kD);
    float absoluteOvershoot = m.peakAbsPos - s.targetMag;
    bool osc = (m.crossings >= oscThresh) || (absoluteOvershoot > s.overshootThresh);

    printf("[AUTOTUNE][%s] kD-bracket iter=%d kD=%.3f peakAbs=%.2f crossings=%d %s\n",
           s.label, dIter, kD, m.peakAbsPos, m.crossings, osc ? "OSCILLATES" : "damped");
    ++dIter;

    if (!osc) { dHigh = kD; peakDHigh = m.peakAbsPos; break; }
    dLow = kD; peakDLow = m.peakAbsPos;
    kD *= 2.5f;
    if (kD >= kdMaxCap) { dHigh = kdMaxCap; peakDHigh = m.peakAbsPos; break; }
  }

  if (dHigh <= 0.0f) {
    s.setConsts(kpBoundary, 0.0f);
    g_autotunePrevSuccess[s.label] = false;
    printf("[AUTOTUNE][%s] phase 2 could not damp at kP=%.3f — marked failed; "
           "next run will not boost\n", s.label, kpBoosted);
    return;
  }

  // Regula falsi for kD: peak DECREASES with kD, so yLow > yHigh.
  // Find kD where peak deviation == overshootThresh.
  while (dIter < maxItersPerPhase && (dHigh - dLow) / dHigh > tolFrac) {
    float yLow  = peakDLow  - s.targetMag;   // larger (oscillating)
    float yHigh = peakDHigh - s.targetMag;   // smaller (damped)
    float yTarget = s.overshootThresh;
    float frac = (yLow - yHigh) > 1e-3f
                   ? (yLow - yTarget) / (yLow - yHigh)
                   : 0.5f;
    frac = std::clamp(frac, 0.15f, 0.85f);
    kD = dLow + frac * (dHigh - dLow);

    VelocityMotionSample m = runOnce(kpBoosted, kD);
    float absoluteOvershoot = m.peakAbsPos - s.targetMag;
    bool osc = (m.crossings >= oscThresh) || (absoluteOvershoot > s.overshootThresh);
    printf("[AUTOTUNE][%s] kD-bisect iter=%d kD=%.3f peakAbs=%.2f crossings=%d frac=%.2f %s\n",
           s.label, dIter, kD, m.peakAbsPos, m.crossings, frac, osc ? "OSCILLATES" : "damped");
    if (osc) { dLow  = kD; peakDLow  = m.peakAbsPos; }
    else     { dHigh = kD; peakDHigh = m.peakAbsPos; }
    ++dIter;
  }

  printf("[AUTOTUNE][%s] COMPLETED. Old kP: %.2f -> New kP: %.3f | Old kD: %.2f -> New kD: %.3f\n",
         s.label, kpStart, kpBoosted, kdLive, dHigh);
  s.setConsts(kpBoosted, dHigh);
  auto applied = s.getConsts();
  printf("[AUTOTUNE][%s] APPLIED VERIFY: live PID now reads kP=%.3f kD=%.3f\n",
         s.label, applied.first, applied.second);
  g_autotunePrevSuccess[s.label] = true;
}

}  // namespace light
