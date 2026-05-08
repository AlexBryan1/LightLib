// ─── pure_pursuit.cpp — lookahead-carrot path follower ──────────────────────
//
// PP shares almost everything with the RAMSETE follower:
//   * Same Waypoint / Spline / Trajectory pipeline produces the path.
//   * Same RamseteConfig (chassis geometry) + DriveFF (per-wheel FF) gains
//     registered via ramsete_configure().
//   * Same EzPauseGuard motor-ownership pattern and per-wheel velocity
//     feedforward formula.
//   * Same telemetry shape (peak/RMS pose error, saturation %, final err).
//
// What differs: the inner control law. RAMSETE uses time-parameterized
// pose-error correction; PP uses a lookahead carrot and the curvature it
// implies.
//
// INVARIANTS (do not violate):
//   * θ is radians inside this file. Convert at the boundary via
//     light::getPoseRad().
//   * EzPauseGuard owns the motors for the duration of the call — every
//     exit path passes through its destructor.
//   * Do not call any chassis.pid_* motion while PP is running.

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

// PP-only configuration. Lives separate from g_rc / g_ff (which describe the
// chassis and apply to both followers).
static PurePursuitConfig g_pp;
static bool g_pp_configured = false;

void pure_pursuit_configure(PurePursuitConfig cfg) {
  g_pp = cfg;
  g_pp_configured = true;
}

// Walk the trajectory forward from `startIdx`, summing chord lengths, until
// cumulative distance >= lookaheadIn. Returns the carrot index. Sets
// `reachedEnd` when we ran off the end before accumulating enough distance,
// in which case the carrot is the final trajectory point.
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

// Locate the trajectory sample whose x,y is closest to `pose`. Search is
// monotonic forward from `lastIdx` over a bounded window so the inner loop
// stays O(1) amortized.
static std::size_t closestIndex(const Trajectory& traj,
                                float px, float py,
                                std::size_t lastIdx,
                                float& outDist) {
  constexpr std::size_t kWindow = 80;  // ~0.8 s of trajectory at 10 ms cadence
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

// Convert each ResolvedEvent's time `t` into the trajectory-sample index
// nearest that time. Lets the inner loop fire events by index comparison.
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
    // Trajectory samples are 10 ms apart starting at t=0. Round to nearest.
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
  // Pre-flight checks — same shape as ramsete's, so error messages stay
  // consistent across followers.
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

  // Telemetry accumulators (same fields as RAMSETE for easy A/B comparison).
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

    // Fire any events whose sample index has been reached.
    while (nextEvent < ppEvents.size() && closestIdx >= ppEvents[nextEvent].idx) {
      if (ppEvents[nextEvent].action) ppEvents[nextEvent].action();
      ++nextEvent;
    }

    peakErr = std::max(peakErr, closestDist);
    rmsNum += closestDist * closestDist;
    rmsCnt++;

    // Sustained pose-error bail — same threshold/duration as RAMSETE.
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

    // Carrot lookup. Once we run off the end, we stay "atEnd" and the carrot
    // is permanently the last point — used for the termination check.
    bool reachedEnd = false;
    std::size_t carrotI =
        carrotIndex(traj, closestIdx, lookahead, reachedEnd);
    if (reachedEnd) atEnd = true;

    const TrajState& closest = traj.pts[closestIdx];
    const TrajState& carrot = traj.pts[carrotI];

    // Carrot in robot frame. Same axes the RAMSETE controller uses
    // (forward = +Y at θ=0, +e_y means target is on the CW-rotation side).
    float dx = carrot.x - pose.x;
    float dy = carrot.y - pose.y;
    float cosT = std::cos(pose.theta);
    float sinT = std::sin(pose.theta);
    float e_fwd = sinT * dx + cosT * dy;
    float e_lat = cosT * dx - sinT * dy;

    // Pure-pursuit curvature: κ = 2·e_lat / L². Sign matches LightLib's
    // CW-positive heading convention because e_lat > 0 means carrot is to
    // the right (CW direction), and ω = v·κ then drives heading CW = right.
    float L2 = e_fwd * e_fwd + e_lat * e_lat;
    float kappa = (L2 > 1e-6f) ? (2.0f * e_lat / L2) : 0.0f;

    // Velocity from the trajectory's profile. Negative when the trajectory
    // was generated with reversed=true; ω = v·κ flips correctly so the
    // robot turns the right way going backward.
    float vTarget = closest.v;
    float aTarget = closest.a;
    float omegaTarget = vTarget * kappa;

    float vL_target = vTarget - omegaTarget * W * 0.5f;
    float vR_target = vTarget + omegaTarget * W * 0.5f;

    float vL_actual = motorRpmToInPerSec((float)left->get_actual_velocity());
    float vR_actual = motorRpmToInPerSec((float)right->get_actual_velocity());

    // Wheel/odom divergence watchdog (lifted from RAMSETE).
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

    // Termination — carrot has fully run off the end and we're inside the
    // RAMSETE-equivalent close-enough envelope.
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
