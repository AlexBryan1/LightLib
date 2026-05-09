// Lookahead-carrot follower. Shares Trajectory/RamseteConfig/DriveFF and the
// EzPauseGuard motor-ownership pattern with ramsete.cpp; differs only in the
// inner control law (carrot curvature vs RAMSETE pose-error correction).
//
// Invariants: θ is radians (use getPoseRad at boundaries); EzPauseGuard owns
// motors for the call; do not call chassis.pid_* while PP is running.

#include "LightLib/path/pure_pursuit.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "LightLib/lib.hpp"
#include "LightLib/path/ramsete.hpp"
#include "path_runtime_internal.hpp"
#include "LightLib/util/extras.hpp"
#include "LightLib/drive/odometry.hpp"
#include "LightLib/util/util.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace light {

// Separate from g_rc/g_ff (chassis-shared with RAMSETE).
static PurePursuitConfig g_pp;
static bool g_pp_configured = false;

void pure_pursuit_configure(PurePursuitConfig cfg) {
  g_pp = cfg;
  g_pp_configured = true;
}

// Walk forward summing chord lengths until cum ≥ lookaheadIn. reachedEnd is
// set when the carrot lands on the final trajectory point.
static std::size_t carrotIndex(const Trajectory& traj,
                               std::size_t startIdx,
                               float lookaheadIn,
                               bool& reachedEnd) {
  reachedEnd = false;
  if (traj.pts.size() < 2) {
    reachedEnd = true;
    return traj.pts.empty() ? 0 : traj.pts.size() - 1;
  }
  float cum = 0.0f;
  for (std::size_t i = startIdx; i + 1 < traj.pts.size(); ++i) {
    float dx = traj.pts[i + 1].x - traj.pts[i].x;
    float dy = traj.pts[i + 1].y - traj.pts[i].y;
    cum += std::hypot(dx, dy);
    if (cum >= lookaheadIn) return i + 1;
  }
  reachedEnd = true;
  return traj.pts.size() - 1;
}

// Monotonic forward search in a bounded window keeps the inner loop O(1).
static std::size_t closestIndex(const Trajectory& traj,
                                float px, float py,
                                std::size_t lastIdx,
                                float& outDist) {
  constexpr std::size_t kWindow = 80;  // ~0.8 s at 10 ms cadence
  std::size_t best = lastIdx;
  float bestD2 = 1e18f;
  std::size_t end = std::min(traj.pts.size(), lastIdx + kWindow);
  for (std::size_t i = lastIdx; i < end; ++i) {
    float dx = traj.pts[i].x - px;
    float dy = traj.pts[i].y - py;
    float d2 = dx * dx + dy * dy;
    if (d2 < bestD2) {
      bestD2 = d2;
      best = i;
    }
  }
  outDist = std::sqrt(bestD2);
  return best;
}

// Resolve event times to sample indices so the inner loop can fire by index.
struct PPEvent {
  std::size_t idx;
  std::function<void()> action;
};

static std::vector<PPEvent> bindEventsToIndex(const Trajectory& traj,
                                              std::vector<ResolvedEvent> events) {
  std::vector<PPEvent> out;
  out.reserve(events.size());
  for (auto& e : events) {
    if (!e.action) continue;
    // Samples are 10 ms apart from t=0.
    int idx = (int)std::lround(e.t * 100.0f);
    if (idx < 0) idx = 0;
    if ((std::size_t)idx >= traj.pts.size()) idx = (int)traj.pts.size() - 1;
    out.push_back({(std::size_t)idx, std::move(e.action)});
  }
  std::sort(out.begin(), out.end(),
            [](const PPEvent& a, const PPEvent& b) { return a.idx < b.idx; });
  return out;
}

bool followPurePursuitCore(const Trajectory& traj,
                           std::vector<ResolvedEvent> events,
                           int timeoutMs,
                           float poseErrBailIn) {
  if (!g_configured) {
    printf("[PP] ramsete_configure() has not been called\n");
    return false;
  }
  if (!g_pp_configured) {
    printf("[PP] pure_pursuit_configure() has not been called\n");
    return false;
  }
  if (traj.empty() || traj.duration <= 0.0f) {
    printf("[PP] empty trajectory\n");
    return false;
  }

  light::Drive* chassis = light::getChassis();
  pros::MotorGroup* left = nullptr;
  pros::MotorGroup* right = nullptr;
  light::getDriveMotorGroups(&left, &right);
  if (!left || !right) {
    printf("[PP] motor groups not registered via extras_init\n");
    return false;
  }

  float battV = (float)pros::battery::get_voltage();
  if (battV < 10500.0f) {
    printf("[PP] battery too low (%.0f mV) — aborting\n", battV);
    return false;
  }

  {
    Pose p0 = light::getPoseRad();
    float dx = traj.pts.front().x - p0.x;
    float dy = traj.pts.front().y - p0.y;
    if (std::hypot(dx, dy) > 3.0f) {
      printf("[PP] initial pose %.1f,%.1f not within 3\" of traj start %.1f,%.1f\n",
             p0.x, p0.y, traj.pts.front().x, traj.pts.front().y);
      return false;
    }
  }

  EzPauseGuard guard(chassis, left, right);

  float peakErr = 0.0f;
  float rmsNum = 0.0f;
  int rmsCnt = 0;
  int satTicks = 0;
  int totTicks = 0;

  uint32_t bailStart = 0;
  bool bailArmed = false;
  uint32_t divergeStart = 0;
  bool divergeArmed = false;

  WheelState wsL, wsR;

  uint32_t t0 = pros::millis();
  uint32_t next = t0;

  const uint32_t hardTimeoutMs =
      (timeoutMs > 0) ? (uint32_t)timeoutMs
                      : (uint32_t)(traj.duration * 1000.0f) + 500;

  bool success = false;
  const float W = g_rc.trackWidthIn;
  const float lookahead = g_pp.lookaheadIn;

  std::vector<PPEvent> ppEvents = bindEventsToIndex(traj, std::move(events));
  std::size_t nextEvent = 0;

  std::size_t closestIdx = 0;
  bool atEnd = false;

  while (true) {
    uint32_t now = pros::millis();
    uint32_t elapsed = now - t0;
    if (elapsed > hardTimeoutMs) break;

    Pose pose = light::getPoseRad();

    float closestDist = 0.0f;
    closestIdx = closestIndex(traj, pose.x, pose.y, closestIdx, closestDist);

    while (nextEvent < ppEvents.size() && closestIdx >= ppEvents[nextEvent].idx) {
      if (ppEvents[nextEvent].action) ppEvents[nextEvent].action();
      ++nextEvent;
    }

    peakErr = std::max(peakErr, closestDist);
    rmsNum += closestDist * closestDist;
    rmsCnt++;

    if (closestDist > poseErrBailIn) {
      if (!bailArmed) {
        bailArmed = true;
        bailStart = now;
      } else if (now - bailStart > 150) {
        printf("[PP] bail: pose err %.2f > %.2f for >150ms\n",
               closestDist, poseErrBailIn);
        break;
      }
    } else {
      bailArmed = false;
    }

    // After the carrot runs off the end it stays pinned to the last point
    // for the termination check.
    bool reachedEnd = false;
    std::size_t carrotI =
        carrotIndex(traj, closestIdx, lookahead, reachedEnd);
    if (reachedEnd) atEnd = true;

    const TrajState& closest = traj.pts[closestIdx];
    const TrajState& carrot = traj.pts[carrotI];

    // Robot frame: forward = +Y at θ=0; +e_lat = right (CW-rotation side).
    float dx = carrot.x - pose.x;
    float dy = carrot.y - pose.y;
    float cosT = std::cos(pose.theta);
    float sinT = std::sin(pose.theta);
    float e_fwd = sinT * dx + cosT * dy;
    float e_lat = cosT * dx - sinT * dy;

    // Pure-pursuit curvature: κ = 2·e_lat / L². LightLib θ is CW-positive,
    // so when the carrot is to the right (e_lat > 0) we want κ > 0 → ω > 0,
    // which the wheel split below turns into vL > vR → robot rotates CW (right).
    float L2 = e_fwd * e_fwd + e_lat * e_lat;
    float kappa = (L2 > 1e-6f) ? (2.0f * e_lat / L2) : 0.0f;

    // v < 0 when reversed=true; ω = v·κ flips with it, so reversed paths
    // turn the correct way, thank you ethanmik.
    float vTarget = closest.v;
    float aTarget = closest.a;
    float omegaTarget = vTarget * kappa;

    float vL_target = vTarget + omegaTarget * W * 0.5f;
    float vR_target = vTarget - omegaTarget * W * 0.5f;

    float vL_actual = motorRpmToInPerSec((float)left->get_actual_velocity());
    float vR_actual = motorRpmToInPerSec((float)right->get_actual_velocity());

    // Wheel-EMA vs odom-local-speed divergence watchdog.
    {
      Pose vl = light::getLocalSpeed(true);
      float vAvgWheel = 0.5f * (wsL.vFiltered + wsR.vFiltered);
      float vAvgOdom = vl.y;
      float vCmd = std::fabs(vTarget);
      float divThresh = std::max(4.0f, 0.25f * vCmd);
      if (std::fabs(vAvgWheel - vAvgOdom) > divThresh) {
        if (!divergeArmed) {
          divergeArmed = true;
          divergeStart = now;
        } else if (now - divergeStart > 200) {
          printf("[PP] bail: wheel/odom speed divergence (|%.1f-%.1f| > %.1f)\n",
                 vAvgWheel, vAvgOdom, divThresh);
          break;
        }
      } else {
        divergeArmed = false;
      }
    }

    float batt_mV = (float)pros::battery::get_voltage();
    float battScale = g_charVoltage_mV / std::max(batt_mV, 1.0f);
    battScale = std::clamp(battScale, 0.85f, 1.00f);

    float vL_volts = wheelVoltageCmd(vL_target, aTarget, vL_actual, wsL, battScale);
    float vR_volts = wheelVoltageCmd(vR_target, aTarget, vR_actual, wsR, battScale);

    if (balanceSaturation(vL_volts, vR_volts)) satTicks++;
    totTicks++;

    left->move_voltage((int)(vL_volts * 1000.0f));
    right->move_voltage((int)(vR_volts * 1000.0f));

    if (atEnd) {
      float ex = traj.pts.back().x - pose.x;
      float ey = traj.pts.back().y - pose.y;
      float endErr = std::hypot(ex, ey);
      if (endErr < std::min(1.5f, poseErrBailIn * 0.5f)) {
        success = true;
        break;
      }
    }

    next += 10;
    uint32_t wait = (next > pros::millis()) ? (next - pros::millis()) : 0;
    pros::delay(wait == 0 ? 1 : wait);
  }

  float rms = (rmsCnt > 0) ? std::sqrt(rmsNum / rmsCnt) : 0.0f;
  float satPct = (totTicks > 0) ? (100.0f * satTicks / totTicks) : 0.0f;
  Pose finalPose = light::getPoseRad();
  float finalErr = std::hypot(traj.pts.back().x - finalPose.x,
                              traj.pts.back().y - finalPose.y);
  printf("[PP] %s dur=%.2fs peak=%.2f\" rms=%.2f\" sat=%.1f%% finalErr=%.2f\"\n",
         success ? "OK" : "FAIL",
         (pros::millis() - t0) / 1000.0f,
         peakErr, rms, satPct, finalErr);

  return success;
}

}  // namespace light
