// Invariants:
//   * θ is radians inside this file; convert at boundaries via getPoseRad().
//     fmod/fmodf are banned — use wrap_rad().
//   * EzPauseGuard owns the motors for the duration of the call; every exit
//     path (success/timeout/bail/exception) must pass through its destructor.
//   * Do not call chassis.pid_* while a trajectory is running.

#include "LightLib/path/ramsete.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "LightLib/drive/ekf.hpp"
#include "LightLib/lib.hpp"
#include "LightLib/util/extras.hpp"
#include "path_runtime_internal.hpp"
#include "LightLib/drive/lightcast.hpp"
#include "LightLib/drive/odometry.hpp"
#include "LightLib/util/util.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace light {

RamseteConfig g_rc;
DriveFF g_ff;
TrajConstraints g_defaultCons;
bool g_configured = false;
float g_charVoltage_mV = 12000.0f;  // assume fresh cell until characterized

// Cached so the 100 Hz inner loop doesn't re-multiply gearRatio·π·diam/60.
float s_rpmToInPerSec = 0.0f;

// Trajectory generation takes 1–2 ms, enough to stall the 100 Hz loop;
// also blocks the two followers from re-entering each other.
std::atomic<bool> g_genLocked{false};

using ::light::util::wrap_rad;

// Taylor series for |x| < 1e-4 to avoid 0/0 in sin(x)/x.
static inline float sincGuarded(float x) {
  if (std::fabs(x) < 1e-4f) {
    float x2 = x * x;
    return 1.0f - x2 / 6.0f + x2 * x2 / 120.0f;
  }
  return std::sin(x) / x;
}

float motorRpmToInPerSec(float rpm) {
  return rpm * s_rpmToInPerSec;
}

// Park EZ in DISABLE on entry; zero voltage and leave EZ disabled on exit.
// We don't restore the prior mode because mid-motion state is stale; the
// next EZ motion call re-arms cleanly.
EzPauseGuard::EzPauseGuard(light::Drive* c, pros::MotorGroup* l, pros::MotorGroup* r)
    : chassis(c), left(l), right(r) {
  if (chassis) {
    chassis->pid_drive_toggle(false);
    chassis->drive_mode_set(light::DISABLE, /*stop_drive=*/true);
  }
}

EzPauseGuard::~EzPauseGuard() {
  if (left) left->move_voltage(0);
  if (right) right->move_voltage(0);
  pros::delay(20);  // let zero-command land before releasing
}

void ramsete_configure(RamseteConfig rc, DriveFF ff, TrajConstraints defaultCons) {
  g_rc = rc;
  g_ff = ff;
  g_defaultCons = defaultCons;
  g_configured = true;
  g_charVoltage_mV = (float)pros::battery::get_voltage();
  if (g_charVoltage_mV < 8000.0f) g_charVoltage_mV = 12000.0f;
  s_rpmToInPerSec = g_rc.gearRatio * (float)M_PI * g_rc.wheelDiamIn / 60.0f;
}

// α = 0.4 → ~25 ms effective window at 10 ms cadence; smooths the 1 RPM
// quantization of get_actual_velocity() without meaningful phase lag.
static constexpr float WHEEL_EMA_ALPHA = 0.4f;

float wheelVoltageCmd(float vTarget, float aTarget,
                      float vActual, WheelState& ws,
                      float batteryScale) {
  if (!ws.seeded) {
    ws.vFiltered = vActual;
    ws.seeded = true;
  } else {
    ws.vFiltered = WHEEL_EMA_ALPHA * vActual + (1.0f - WHEEL_EMA_ALPHA) * ws.vFiltered;
  }

  float kV = g_ff.kV * batteryScale;
  float kA = g_ff.kA * batteryScale;

  float sign = (vTarget > 0.05f) ? 1.0f : (vTarget < -0.05f ? -1.0f : 0.0f);
  float ff = g_ff.kS * sign + kV * vTarget + kA * aTarget;
  float fb = g_ff.kP * (vTarget - ws.vFiltered);
  float V = ff + fb;
  return std::clamp(V, -12.0f, 12.0f);
}

// Proportional rescale preserves steering ratio when a side saturates.
bool balanceSaturation(float& vL, float& vR, float limit) {
  float m = std::max(std::fabs(vL), std::fabs(vR));
  if (m > limit) {
    vL = vL / m * limit;
    vR = vR / m * limit;
    return true;
  }
  return false;
}

// O(N) scan, runs at trajectory-gen time (N ≈ 1500 samples / 100 wpts).
float waypointTime(const std::vector<Waypoint>& wps,
                   int idx, const Trajectory& traj) {
  float bestD2 = 1e18f;
  float bestT = 0.0f;
  const float wx = wps[idx].x;
  const float wy = wps[idx].y;
  for (const auto& p : traj.pts) {
    float dx = p.x - wx;
    float dy = p.y - wy;
    float d2 = dx * dx + dy * dy;
    if (d2 < bestD2) {
      bestD2 = d2;
      bestT = p.t;
    }
  }
  return bestT;
}

bool followTrajectoryCore(const Trajectory& traj,
                          std::vector<ResolvedEvent> events,
                          int timeoutMs,
                          float poseErrBailIn) {
  if (!g_configured) {
    printf("[RAMSETE] ramsete_configure() has not been called\n");
    return false;
  }
  if (traj.empty() || traj.duration <= 0.0f) {
    printf("[RAMSETE] empty trajectory\n");
    return false;
  }

  light::Drive* chassis = light::getChassis();
  pros::MotorGroup* left = nullptr;
  pros::MotorGroup* right = nullptr;
  light::getDriveMotorGroups(&left, &right);
  if (!left || !right) {
    printf("[RAMSETE] motor groups not registered via extras_init\n");
    return false;
  }

  float battV = (float)pros::battery::get_voltage();
  if (battV < 10500.0f) {
    printf("[RAMSETE] battery too low (%.0f mV) — aborting\n", battV);
    return false;
  }

  // Huge e_x at t=0 causes a dangerous lurch — refuse to start.
  {
    Pose p0 = light::getPoseRad();
    float dx = traj.pts.front().x - p0.x;
    float dy = traj.pts.front().y - p0.y;
    if (std::hypot(dx, dy) > 3.0f) {
      printf("[RAMSETE] initial pose %.1f,%.1f not within 3\" of traj start %.1f,%.1f\n",
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

  // wheel-EMA vs odom-local-speed divergence watchdog
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
  const float b = g_rc.b;
  const float zeta = g_rc.zeta;

  // Sorted so the per-tick check only peeks at the head; cursor monotonic.
  std::sort(events.begin(), events.end(),
            [](const ResolvedEvent& a, const ResolvedEvent& b) { return a.t < b.t; });
  std::size_t nextEvent = 0;

  while (true) {
    uint32_t now = pros::millis();
    uint32_t elapsed = now - t0;
    float tRel = elapsed / 1000.0f;

    if (elapsed > hardTimeoutMs) break;

    // Callbacks run on the control thread — a slow callback delays the next tick.
    while (nextEvent < events.size() && tRel >= events[nextEvent].t) {
      if (events[nextEvent].action) events[nextEvent].action();
      ++nextEvent;
    }

    TrajState s = traj.sample(tRel);

    Pose pose = light::getPoseRad();

    // LightLib θ: 0 faces +Y, CW-positive. Forward = (sinθ, cosθ); left = (cosθ, −sinθ).
    float dx = s.x - pose.x;
    float dy = s.y - pose.y;
    float cosT = std::cos(pose.theta);
    float sinT = std::sin(pose.theta);
    float e_x = sinT * dx + cosT * dy;
    float e_y = cosT * dx - sinT * dy;
    float e_th = wrap_rad(s.theta - pose.theta);

    // 0.3" deadband kills sub-inch odom noise that thrashes v_cmd on straights.
    if (std::fabs(e_x) < 0.3f) e_x = 0.0f;
    if (std::fabs(e_y) < 0.3f) e_y = 0.0f;

    float posErr = std::hypot(dx, dy);
    peakErr = std::max(peakErr, posErr);
    rmsNum += posErr * posErr;
    rmsCnt++;

    // Sustained pose-error bail: above threshold for >150 ms = lost the path.
    if (posErr > poseErrBailIn) {
      if (!bailArmed) {
        bailArmed = true;
        bailStart = now;
      } else if (now - bailStart > 150) {
        printf("[RAMSETE] bail: pose err %.2f > %.2f for >150ms\n",
               posErr, poseErrBailIn);
        break;
      }
    } else {
      bailArmed = false;
    }

    // RAMSETE control law (Corke / Bradski).
    float vr = s.v;
    float wr = s.omega;
    float k = 2.0f * zeta * std::sqrt(wr * wr + b * vr * vr);
    float sinc_e = sincGuarded(e_th);
    float v_cmd = vr * std::cos(e_th) + k * e_x;
    float w_cmd = wr + b * vr * sinc_e * e_y + k * e_th;

    // Tank kinematics — split (v, ω) into per-wheel linear speed.
    // LightLib θ is CW-positive, so positive ω turns CW (right): left wheel
    // speeds up, right wheel slows down.
    float vL_target = v_cmd + w_cmd * W * 0.5f;
    float vR_target = v_cmd - w_cmd * W * 0.5f;

    float vL_actual = motorRpmToInPerSec((float)left->get_actual_velocity());
    float vR_actual = motorRpmToInPerSec((float)right->get_actual_velocity());

    // Wheel-EMA vs odom mismatch >200 ms = lost tracking wheel or motor cable.
    {
      Pose vl = light::getLocalSpeed(true);
      float vAvgWheel = 0.5f * (wsL.vFiltered + wsR.vFiltered);
      float vAvgOdom = vl.y;
      float vCmd = std::fabs(s.v);
      float divThresh = std::max(4.0f, 0.25f * vCmd);
      if (std::fabs(vAvgWheel - vAvgOdom) > divThresh) {
        if (!divergeArmed) {
          divergeArmed = true;
          divergeStart = now;
        } else if (now - divergeStart > 200) {
          printf("[RAMSETE] bail: wheel/odom speed divergence (|%.1f-%.1f| > %.1f)\n",
                 vAvgWheel, vAvgOdom, divThresh);
          break;
        }
      } else {
        divergeArmed = false;
      }
    }

    // Clamp battery scaling so a critically low cell can't command unsafely
    // large voltages.
    float batt_mV = (float)pros::battery::get_voltage();
    float battScale = g_charVoltage_mV / std::max(batt_mV, 1.0f);
    battScale = std::clamp(battScale, 0.85f, 1.00f);

    // ω̇ contribution to per-wheel accel is 2nd-order small at this cadence.
    float aL = s.a, aR = s.a;

    float vL_volts = wheelVoltageCmd(vL_target, aL, vL_actual, wsL, battScale);
    float vR_volts = wheelVoltageCmd(vR_target, aR, vR_actual, wsR, battScale);

    if (balanceSaturation(vL_volts, vR_volts)) satTicks++;
    totTicks++;

    left->move_voltage((int)(vL_volts * 1000.0f));
    right->move_voltage((int)(vR_volts * 1000.0f));

    if (tRel >= traj.duration && posErr < std::min(1.5f, poseErrBailIn * 0.5f)) {
      success = true;
      break;
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
  printf("[RAMSETE] %s dur=%.2fs peak=%.2f\" rms=%.2f\" sat=%.1f%% finalErr=%.2f\"\n",
         success ? "OK" : "FAIL",
         (pros::millis() - t0) / 1000.0f,
         peakErr, rms, satPct, finalErr);

  return success;
}

static bool dispatchFollower(const Trajectory& traj,
                             std::vector<ResolvedEvent> events,
                             int timeoutMs,
                             float poseErrBailIn,
                             Follower follower) {
  if (follower == Follower::PURE_PURSUIT) {
    return followPurePursuitCore(traj, std::move(events), timeoutMs, poseErrBailIn);
  }
  return followTrajectoryCore(traj, std::move(events), timeoutMs, poseErrBailIn);
}

bool followTrajectory(const Trajectory& traj,
                      int timeoutMs,
                      float poseErrBailIn,
                      Follower follower) {
  return dispatchFollower(traj, {}, timeoutMs, poseErrBailIn, follower);
}

static bool followTrajectoryWithEvents(const std::vector<Waypoint>& wps,
                                       const TrajConstraints& cons,
                                       std::vector<PathEvent> events,
                                       bool reversed,
                                       int timeoutMs,
                                       float poseErrBailIn,
                                       Follower follower) {
  bool expected = false;
  if (!g_genLocked.compare_exchange_strong(expected, true)) {
    printf("[PATH] trajectory generation already in progress\n");
    return false;
  }
  Trajectory traj = generateTrajectory(wps, cons, reversed);
  g_genLocked.store(false);

  if (traj.empty()) return false;

  std::vector<ResolvedEvent> resolved;
  resolved.reserve(events.size());
  for (auto& e : events) {
    if (e.atWaypoint < 0 || e.atWaypoint >= (int)wps.size()) {
      printf("[PATH] event waypoint idx %d out of range [0, %u)\n",
             e.atWaypoint, (unsigned)wps.size());
      continue;
    }
    if (!e.action) continue;
    resolved.push_back({waypointTime(wps, e.atWaypoint, traj),
                        std::move(e.action)});
  }
  return dispatchFollower(traj, std::move(resolved), timeoutMs, poseErrBailIn, follower);
}

bool followTrajectory(const std::vector<Waypoint>& wps,
                      const TrajConstraints& cons,
                      bool reversed,
                      int timeoutMs,
                      float poseErrBailIn,
                      Follower follower) {
  return followTrajectoryWithEvents(wps, cons, {}, reversed,
                                    timeoutMs, poseErrBailIn, follower);
}

bool followTrajectory(const std::vector<Waypoint>& wps,
                      const TrajConstraints& cons,
                      std::vector<PathEvent> events,
                      bool reversed,
                      int timeoutMs,
                      float poseErrBailIn,
                      Follower follower) {
  return followTrajectoryWithEvents(wps, cons, std::move(events),
                                    reversed, timeoutMs, poseErrBailIn, follower);
}

// No-cons overloads — fall back to the TrajConstraints registered via
// ramsete_configure(). Mirrors the runJerryioPath behavior so users can call
// followTrajectory(path) without re-declaring constraints at every call site.
bool followTrajectory(const std::vector<Waypoint>& wps,
                      bool reversed,
                      int timeoutMs,
                      float poseErrBailIn,
                      Follower follower) {
  if (!g_configured) {
    printf("[PATH] ramsete_configure() has not been called\n");
    return false;
  }
  return followTrajectory(wps, g_defaultCons, reversed,
                          timeoutMs, poseErrBailIn, follower);
}

bool followTrajectory(const std::vector<Waypoint>& wps,
                      std::vector<PathEvent> events,
                      bool reversed,
                      int timeoutMs,
                      float poseErrBailIn,
                      Follower follower) {
  if (!g_configured) {
    printf("[PATH] ramsete_configure() has not been called\n");
    return false;
  }
  return followTrajectory(wps, g_defaultCons, std::move(events),
                          reversed, timeoutMs, poseErrBailIn, follower);
}

// Auto-detects two formats: sparse "x,y[,heading_rad]" or Jerryio densified
// (header "#PATH-POINTS-START", "x,y,speed[,heading_deg]"). Speed/heading
// cols are ignored on densified paths — dense x,y defines the curve and the
// trajectory generator computes its own profile. Densified paths are
// translated to start at (0,0) in the robot frame. UTF-8 BOM tolerated.
static bool parseJerryio(const char* csv,
                         std::vector<Waypoint>& wps,
                         std::vector<int>& anchorIdx,
                         bool& densified) {
  if (!csv) {
    printf("[JERRYIO] null csv\n");
    return false;
  }

  const char* p = csv;

  if ((unsigned char)p[0] == 0xEF &&
      (unsigned char)p[1] == 0xBB &&
      (unsigned char)p[2] == 0xBF) {
    p += 3;
  }

  densified = (std::strstr(p, "#PATH-POINTS-START") != nullptr);
  const int maxCols = densified ? 4 : 3;
  const int headingCol = densified ? 3 : 2;

  wps.clear();
  anchorIdx.clear();

  while (*p) {
    const char* lineStart = p;
    while (*p && *p != '\n' && *p != '\r') ++p;
    const char* lineEnd = p;
    if (*p == '\r') ++p;
    if (*p == '\n') ++p;

    const char* c = lineStart;
    while (c < lineEnd && (*c == ' ' || *c == '\t')) ++c;
    if (c >= lineEnd || *c == '#') continue;

    float vals[4] = {0, 0, 0, 0};
    int n = 0;
    bool malformed = false;
    while (n < maxCols && c < lineEnd) {
      char* endp = nullptr;
      float v = std::strtof(c, &endp);
      if (endp == c) break;
      if (endp < lineEnd && *endp != ',' && *endp != ' ' && *endp != '\t') {
        printf("[JERRYIO] malformed number near col %d — line skipped\n", n);
        malformed = true;
        break;
      }
      vals[n++] = v;
      c = endp;
      while (c < lineEnd && (*c == ',' || *c == ' ' || *c == '\t')) ++c;
    }

    if (malformed) continue;
    if (n < 2) continue;

    Waypoint w;
    w.x = vals[0];
    w.y = vals[1];
    bool hasHeading = (n > headingCol);
    if (hasHeading && !densified) {
      // Sparse CSV column-3 is heading in radians; Waypoint stores degrees.
      w.headingDeg = vals[headingCol] * 180.0f / (float)M_PI;
    }
    int idx = (int)wps.size();
    wps.push_back(w);
    if (hasHeading) anchorIdx.push_back(idx);
  }

  if (wps.size() < 2) {
    printf("[JERRYIO] need >=2 waypoints, got %u\n", (unsigned)wps.size());
    return false;
  }

  if (anchorIdx.empty()) {
    anchorIdx.reserve(wps.size());
    for (int i = 0; i < (int)wps.size(); ++i) anchorIdx.push_back(i);
  }

  if (densified) {
    // Translate path to robot-frame origin; reset pose first if already aligned to Jerryio field.
    float x0 = wps[0].x, y0 = wps[0].y;
    for (auto& w : wps) {
      w.x -= x0;
      w.y -= y0;
    }
  }

  return true;
}

static std::vector<PathEvent> remapAnchorEvents(std::vector<PathEvent> events,
                                                const std::vector<int>& anchorIdx) {
  std::vector<PathEvent> out;
  out.reserve(events.size());
  for (auto& e : events) {
    if (e.atWaypoint < 0 || e.atWaypoint >= (int)anchorIdx.size()) {
      printf("[JERRYIO] event anchor idx %d out of range [0, %u)\n",
             e.atWaypoint, (unsigned)anchorIdx.size());
      continue;
    }
    e.atWaypoint = anchorIdx[e.atWaypoint];
    out.push_back(std::move(e));
  }
  return out;
}

bool runJerryioPath(const char* csv,
                    bool reversed,
                    int timeoutMs,
                    float poseErrBailIn,
                    Follower follower) {
  std::vector<Waypoint> wps;
  std::vector<int> anchorIdx;
  bool densified = false;
  if (!parseJerryio(csv, wps, anchorIdx, densified)) return false;
  return followTrajectory(wps, g_defaultCons, reversed,
                          timeoutMs, poseErrBailIn, follower);
}

bool runJerryioPath(const char* csv,
                    std::vector<PathEvent> events,
                    bool reversed,
                    int timeoutMs,
                    float poseErrBailIn,
                    Follower follower) {
  std::vector<Waypoint> wps;
  std::vector<int> anchorIdx;
  bool densified = false;
  if (!parseJerryio(csv, wps, anchorIdx, densified)) return false;
  auto mapped = remapAnchorEvents(std::move(events), anchorIdx);
  return followTrajectory(wps, g_defaultCons, std::move(mapped),
                          reversed, timeoutMs, poseErrBailIn, follower);
}

static bool readSDFile(const char* filePath, char* buf, std::size_t bufSize) {
  if (!filePath) {
    printf("[JERRYIO] null file path\n");
    return false;
  }

  FILE* f = std::fopen(filePath, "r");
  if (!f) {
    printf("[JERRYIO] cannot open %s\n", filePath);
    return false;
  }

  std::size_t n = std::fread(buf, 1, bufSize - 1, f);
  bool overflow = (n == bufSize - 1) && !std::feof(f);
  std::fclose(f);

  if (overflow) {
    printf("[JERRYIO] %s larger than %u-byte buffer\n",
           filePath, (unsigned)(bufSize - 1));
    return false;
  }
  buf[n] = '\0';
  return true;
}

bool runJerryioPathFromSD(const char* filePath,
                          bool reversed,
                          int timeoutMs,
                          float poseErrBailIn,
                          Follower follower) {
  constexpr std::size_t BUF_SIZE = 4096;
  char buf[BUF_SIZE];
  if (!readSDFile(filePath, buf, BUF_SIZE)) return false;
  return runJerryioPath(buf, reversed, timeoutMs, poseErrBailIn, follower);
}

bool runJerryioPathFromSD(const char* filePath,
                          std::vector<PathEvent> events,
                          bool reversed,
                          int timeoutMs,
                          float poseErrBailIn,
                          Follower follower) {
  constexpr std::size_t BUF_SIZE = 4096;
  char buf[BUF_SIZE];
  if (!readSDFile(filePath, buf, BUF_SIZE)) return false;
  return runJerryioPath(buf, std::move(events),
                        reversed, timeoutMs, poseErrBailIn, follower);
}

// Characterization routines: each one is the body of its own selectable auton.
// Values are printf'd; user transcribes into ramsete_configure().

static void driveAllVolts(pros::MotorGroup* L, pros::MotorGroup* R, float volts) {
  L->move_voltage((int)(volts * 1000.0f));
  R->move_voltage((int)(volts * 1000.0f));
}

void characterize_kV_kA_kS(float maxVoltage, float rampVps) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup* L = nullptr;
  pros::MotorGroup* R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!L || !R) {
    printf("[CHAR] no motor groups\n");
    return;
  }

  EzPauseGuard guard(chassis, L, R);

  // kS: slow 0.05 V / 200 ms ramp until a side moves.
  float v = 0.0f;
  float kS = 0.0f;
  while (v < 4.0f) {
    driveAllVolts(L, R, v);
    pros::delay(200);
    float sL = std::fabs(motorRpmToInPerSec((float)L->get_actual_velocity()));
    float sR = std::fabs(motorRpmToInPerSec((float)R->get_actual_velocity()));
    if (sL > 1.0f || sR > 1.0f) {
      kS = v;
      break;
    }
    v += 0.05f;
  }
  printf("[CHAR] kS ≈ %.3f V\n", kS);
  driveAllVolts(L, R, 0);
  pros::delay(500);

  // kV: quasi-static ramp, regress v_actual vs (V - kS).
  float sumVV = 0, sumVv = 0, sumv2 = 0;
  int samples = 0;
  float vApplied = kS;
  uint32_t t0 = pros::millis();
  while (vApplied < maxVoltage) {
    driveAllVolts(L, R, vApplied);
    pros::delay(20);
    float sL = motorRpmToInPerSec((float)L->get_actual_velocity());
    float sR = motorRpmToInPerSec((float)R->get_actual_velocity());
    float vel = 0.5f * (std::fabs(sL) + std::fabs(sR));
    float effV = vApplied - kS;
    if (vel > 2.0f && effV > 0.1f) {
      sumVV += effV * effV;
      sumVv += effV * vel;
      sumv2 += vel * vel;
      samples++;
    }
    vApplied += rampVps * 0.02f;
  }
  float kV = (samples > 5 && sumVV > 1e-3f) ? (sumVv / sumVV) : 0.0f;
  printf("[CHAR] kV ≈ %.4f V/(in/s) over %d samples\n", kV, samples);
  driveAllVolts(L, R, 0);
  pros::delay(500);

  // kA: step 0 → 8 V, capture accel in first 300 ms.
  driveAllVolts(L, R, 8.0f);
  uint32_t tStep = pros::millis();
  float vPrev = 0.0f;
  float aMax = 0.0f;
  while (pros::millis() - tStep < 300) {
    pros::delay(10);
    float sL = motorRpmToInPerSec((float)L->get_actual_velocity());
    float sR = motorRpmToInPerSec((float)R->get_actual_velocity());
    float vel = 0.5f * (std::fabs(sL) + std::fabs(sR));
    float a = (vel - vPrev) / 0.010f;
    if (a > aMax) aMax = a;
    vPrev = vel;
  }
  driveAllVolts(L, R, 0);
  // During the initial step vel ≪ steady-state, so kA ≈ (8 - kS)/aMax.
  float kA = aMax > 1.0f ? (8.0f - kS) / aMax : 0.0f;
  printf("[CHAR] kA ≈ %.4f V/(in/s^2) (peak a = %.1f in/s^2)\n", kA, aMax);

  if (kV > 1e-4f && kA > 1e-4f) {
    g_ff.kS = kS;
    g_ff.kV = kV;
    g_ff.kA = kA;
    printf("[CHAR][APPLIED] DriveFF kS=%.3f kV=%.4f kA=%.4f (kP unchanged)\n",
           kS, kV, kA);
  } else {
    printf("[CHAR][FAILED] kV/kA invalid — DriveFF left unchanged\n");
  }

  (void)t0;
}

float characterize_track_width(int rotations) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup* L = nullptr;
  pros::MotorGroup* R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!L || !R) {
    printf("[CHAR] no motor groups\n");
    return 0.0f;
  }

  EzPauseGuard guard(chassis, L, R);

  L->tare_position();
  R->tare_position();
  Pose start = light::getPoseRad();
  (void)start;

  driveAllVolts(L, R, 0);
  L->move_voltage(3000);
  R->move_voltage(-3000);

  pros::delay((uint32_t)(rotations * 800));  // ~0.8 s per rotation at 3 V

  L->move_voltage(0);
  R->move_voltage(0);
  pros::delay(200);

  float posL = std::fabs(L->get_position()) * g_rc.gearRatio * (float)M_PI * g_rc.wheelDiamIn / 360.0f;
  float posR = std::fabs(R->get_position()) * g_rc.gearRatio * (float)M_PI * g_rc.wheelDiamIn / 360.0f;
  float arc = 0.5f * (posL + posR);

  Pose now = light::getPoseRad();
  float turnedRad = std::fabs(wrap_rad(now.theta - start.theta));
  if (turnedRad < 0.1f) {
    printf("[CHAR] turn too small\n");
    return 0.0f;
  }

  float W = arc / (turnedRad * 0.5f);
  printf("[CHAR] track width ≈ %.2f in (arc=%.1f, rad=%.2f)\n", W, arc, turnedRad);

  if (W > 1.0f && W < 40.0f) {
    g_rc.trackWidthIn = W;
    printf("[CHAR][APPLIED] RamseteConfig.trackWidthIn = %.2f in\n", W);
  } else {
    printf("[CHAR][FAILED] track width out of range — config unchanged\n");
  }
  return W;
}

// Constant-radius circle; ramp v until odom radius deviates >15%.
// Conservative aLatMax = 0.7 · v²/R.
float characterize_a_lat_max() {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup* L = nullptr;
  pros::MotorGroup* R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!L || !R) {
    printf("[CHAR] no motor groups\n");
    return 0.0f;
  }

  EzPauseGuard guard(chassis, L, R);

  const float R_target = 24.0f;
  float v = 12.0f;
  float aLatSafe = 0.0f;
  while (v < 60.0f) {
    // Wheel-speed ratio for radius R_target.
    float W = g_rc.trackWidthIn;
    float vL = v * (R_target - W * 0.5f) / R_target;
    float vR = v * (R_target + W * 0.5f) / R_target;

    float Vl = g_ff.kS + g_ff.kV * vL;
    float Vr = g_ff.kS + g_ff.kV * vR;
    L->move_voltage((int)(std::clamp(Vl, -12.0f, 12.0f) * 1000));
    R->move_voltage((int)(std::clamp(Vr, -12.0f, 12.0f) * 1000));

    pros::delay(500);
    Pose p0 = light::getPoseRad();
    pros::delay(500);
    Pose p1 = light::getPoseRad();
    float dth = std::fabs(wrap_rad(p1.theta - p0.theta));
    float dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
    float Rmeas = (dth > 0.05f) ? (dist / dth) : 1e6f;

    if (std::fabs(Rmeas - R_target) / R_target > 0.15f) {
      aLatSafe = 0.7f * (v - 3.0f) * (v - 3.0f) / R_target;
      break;
    }
    v += 3.0f;
  }
  printf("[CHAR] aLatMax ≈ %.1f in/s^2\n", aLatSafe);

  if (aLatSafe > 5.0f) {
    g_defaultCons.aLatMax = aLatSafe;
    printf("[CHAR][APPLIED] TrajConstraints.aLatMax = %.1f in/s^2\n", aLatSafe);
  } else {
    printf("[CHAR][FAILED] aLatMax invalid — TrajConstraints unchanged\n");
  }
  return aLatSafe;
}

// Open-loop voltage sweep fits V = kS·sgn(v) + kV·v + kQ·v·|v| in motor-cmd
// units (−127..127). Feeds chassis.friction_constants_set() (the FF for the
// non-trajectory PIDs; RAMSETE has its own per-wheel FF). Raw samples dumped
// to /usd/friction_id.csv when SD present.
void characterize_friction(float maxVoltage, float stepV) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup* L = nullptr;
  pros::MotorGroup* R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!L || !R || !chassis) {
    printf("[FRIC] no motor groups / chassis\n");
    return;
  }

  double maxRpm = chassis->drive_rpm_get();
  if (maxRpm < 1.0) {
    printf("[FRIC] invalid cartridge rpm (%g) — aborting\n", maxRpm);
    return;
  }

  EzPauseGuard guard(chassis, L, R);

  FILE* f = nullptr;
  if (light::util::SD_CARD_ACTIVE) {
    f = fopen("/usd/friction_id.csv", "w");
    if (f) fputs("v_cmd_applied,v_cmd_velocity\n", f);
  }

  std::vector<double> v_arr;       // measured velocity, motor-cmd units
  std::vector<double> Vapp_arr;    // applied voltage, motor-cmd units
  v_arr.reserve(64);
  Vapp_arr.reserve(64);

  // Quasi-static ramp; skip samples below noise floor (encoder quantization).
  for (float vApplied = 0.0f; vApplied <= maxVoltage + 1e-3f; vApplied += stepV) {
    driveAllVolts(L, R, vApplied);
    pros::delay(800);

    double sumRpm = 0.0;
    int n = 0;
    uint32_t tStart = pros::millis();
    while (pros::millis() - tStart < 200) {
      double sL = (double)L->get_actual_velocity();
      double sR = (double)R->get_actual_velocity();
      sumRpm += 0.5 * (std::fabs(sL) + std::fabs(sR));
      n++;
      pros::delay(10);
    }
    double rpmAvg = (n > 0) ? sumRpm / n : 0.0;
    double v_cmd_velocity = (rpmAvg / maxRpm) * 127.0;
    double Vapp_cmd = vApplied * (127.0 / 12.0);

    if (f) fprintf(f, "%.3f,%.3f\n", Vapp_cmd, v_cmd_velocity);

    if (v_cmd_velocity > 2.0) {  // encoder-quantization noise floor
      v_arr.push_back(v_cmd_velocity);
      Vapp_arr.push_back(Vapp_cmd);
    }
  }

  driveAllVolts(L, R, 0);
  pros::delay(500);
  if (f) fclose(f);

  size_t N = v_arr.size();
  if (N < 5) {
    printf("[FRIC] not enough usable samples (%u) — fit aborted\n",
           (unsigned)N);
    return;
  }

  // Normal equations for [kS, kV, kQ]. All v > 0, so sgn(v) = 1; kV/kQ
  // separation comes from curvature of the data.
  double M[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  double bvec[3] = {0, 0, 0};
  for (size_t i = 0; i < N; ++i) {
    double v = v_arr[i];
    double s = 1.0;
    double vv = v * v;
    double Va = Vapp_arr[i];
    double basis[3] = {s, v, vv};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) M[r][c] += basis[r] * basis[c];
      bvec[r] += basis[r] * Va;
    }
  }

  auto det3 = [](double m[3][3]) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  };
  double D = det3(M);
  if (std::fabs(D) < 1e-9) {
    printf("[FRIC] singular fit matrix — aborting\n");
    return;
  }
  auto subDet = [&](int col) {
    double T[3][3];
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        T[r][c] = (c == col) ? bvec[r] : M[r][c];
    return det3(T);
  };
  double kS = subDet(0) / D;
  double kV = subDet(1) / D;
  double kQ = subDet(2) / D;

  printf("[FRIC] kS=%.3f kV=%.4f kQ=%.5f over %u samples\n",
         kS, kV, kQ, (unsigned)N);

  if (kS >= 0.0 && kV >= 0.0) {
    chassis->friction_constants_set(kS, kV, kQ);
    printf("[FRIC][APPLIED] friction_constants_set(%.3f, %.4f, %.5f)\n",
           kS, kV, kQ);
  } else {
    printf("[FRIC][FAILED] negative coefficients — chassis unchanged\n");
  }
  if (light::util::SD_CARD_ACTIVE) {
    printf("[FRIC] raw data: /usd/friction_id.csv\n");
  }
}

// Bang-bang oscillator. readError returns a signed error in PID units;
// applyRelay drives ±h·sign. Ku, Pu via Åström: Ku = 4h/(π·a), Pu = mean
// period between matched zero-crossings.
struct RelayResult {
  float Ku = 0.0f;
  float Pu = 0.0f;
  int cyclesSeen = 0;
  bool ok = false;
};

static RelayResult runRelay(std::function<float()> readError,
                            std::function<void(int)> applyRelay,
                            float h, int cycles, int timeoutMs,
                            int chunkCycles = 0, int coolMs = 0) {
  RelayResult out;
  if (!readError || !applyRelay || h <= 0.0f || cycles <= 0) return out;

  // ~2% hysteresis band ignores sensor jitter around zero.
  const float band = 0.02f * h + 1e-3f;
  int sign = (readError() >= 0.0f) ? -1 : 1;
  applyRelay(sign);

  uint32_t t0 = pros::millis();
  uint32_t tLastZX = t0;
  float curMax = -1e9f;
  float curMin = 1e9f;
  bool haveHalfCycle = false;

  std::vector<float> amps;  // per-half-cycle extrema spread
  std::vector<float> pers;  // half-period between zero-crossings

  // Motor-cooldown chunking: t0 bumped by coolMs so cooldown doesn't count
  // against timeoutMs.
  const int halfPerChunk = (chunkCycles > 0 && coolMs > 0) ? (2 * chunkCycles) : 0;
  int halfInChunk = 0;

  while ((int)(pros::millis() - t0) < timeoutMs) {
    float e = readError();
    if (e > curMax) curMax = e;
    if (e < curMin) curMin = e;

    // Hysteresis zero-crossing: flip only after crossing through opposite band.
    bool flip = false;
    if (sign > 0 && e > band) flip = true;
    if (sign < 0 && e < -band) flip = true;
    if (flip) {
      uint32_t now = pros::millis();
      if (haveHalfCycle) {
        float amp = 0.5f * (curMax - curMin);
        amps.push_back(amp);
        pers.push_back((now - tLastZX) / 1000.0f);
      }
      curMax = -1e9f;
      curMin = 1e9f;
      tLastZX = now;
      sign = -sign;
      applyRelay(sign);
      haveHalfCycle = true;
      halfInChunk++;

      if ((int)amps.size() >= 2 * cycles) break;

      if (halfPerChunk > 0 && halfInChunk >= halfPerChunk) {
        // Throw away first half-cycle after resume (re-priming transient).
        applyRelay(0);
        printf("[AUTOTUNE] chunk done (%d/%d cycles), cooling %dms\n",
               (int)amps.size() / 2, cycles, coolMs);
        pros::delay(coolMs);
        t0 += coolMs;
        halfInChunk = 0;
        sign = (readError() >= 0.0f) ? -1 : 1;
        applyRelay(sign);
        tLastZX = pros::millis();
        curMax = -1e9f;
        curMin = 1e9f;
        haveHalfCycle = false;
      }
    }

    pros::delay(10);
  }

  applyRelay(0);
  if (amps.size() < 3 || pers.size() < 3) return out;

  // Drop the first half-cycle (initial-push transient).
  float aSum = 0, pSum = 0;
  int n = 0;
  for (std::size_t i = 1; i < amps.size(); ++i) {
    aSum += amps[i];
    pSum += pers[i];
    ++n;
  }
  float aMean = aSum / n;
  float pHalf = pSum / n;
  float Pu = 2.0f * pHalf;

  if (aMean < 1e-4f || Pu < 0.02f) return out;
  out.Ku = 4.0f * h / ((float)M_PI * aMean);
  out.Pu = Pu;
  out.cyclesSeen = n / 2;
  out.ok = true;
  return out;
}

static void applyZnAndPrint(const char* tag, const RelayResult& r,
                            std::function<void(double, double, double)> setter) {
  if (!r.ok) {
    printf("[AUTOTUNE][%s] FAILED — no stable oscillation (reliefV too low?)\n", tag);
    return;
  }
  float kP = 0.6f * r.Ku;
  float kI = 2.0f * kP / r.Pu;
  float kD = kP * r.Pu / 8.0f;
  printf("[AUTOTUNE][%s] Ku=%.3f  Pu=%.3fs  (cycles=%d) -> kP=%.3f  kI=%.4f  kD=%.3f\n",
         tag, r.Ku, r.Pu, r.cyclesSeen, kP, kI, kD);
  if (setter) setter((double)kP, (double)kI, (double)kD);
  printf("[AUTOTUNE][%s] applied to live EZ chassis (start_i=0).\n", tag);
}

// Velocity-based iterative autotune. Drives the chassis through its own PID,
// samples peak |velocity|, peak |position|, and counts target zero-crossings.
// Oscillation criterion: crossings >= 3 (one approach + return-through +
// re-cross). kI stays at zero.
struct VelocityMotionSample {
  float peakAbsVel = 0.0f;   // peak |dpos/dt| in units/sec
  float peakAbsPos = 0.0f;   // peak |pos − resetSensor zero|
  int   crossings  = 0;      // number of (pos − target) sign flips
};

static VelocityMotionSample sampleMotion(std::function<float()> readPos,
                                         pros::MotorGroup* L, pros::MotorGroup* R,
                                         float target,
                                         int maxMotionMs,
                                         int settleMs = 250,
                                         float settleVoltMv = 1500.0f) {
  VelocityMotionSample s;
  uint32_t t0 = pros::millis();
  float prevPos = readPos();
  float prevDelta = prevPos - target;
  uint32_t prevMs = t0;
  uint32_t lowVoltStart = 0;

  pros::delay(40);  // let the PID start commanding before we judge "settled"

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

    // Chassis-done heuristic: both motor commands stay near 0 for settleMs.
    float vL = (float)std::abs(L->get_voltage());
    float vR = (float)std::abs(R->get_voltage());
    if (vL < settleVoltMv && vR < settleVoltMv) {
      if (lowVoltStart == 0) lowVoltStart = now;
      else if ((int)(now - lowVoltStart) >= settleMs) break;
    } else {
      lowVoltStart = 0;
    }
    pros::delay(10);
  }
  return s;
}

struct VelocityTunerStep {
  std::function<void(float, float)> setConsts;  // (kP, kD); kI always 0
  std::function<void()> setTarget;              // drive/turn/swing to target
  std::function<void()> driveBack;              // return to start
  std::function<float()> readPos;
  std::function<void()> resetSensor;
  float target;             // signed target (matches the value handed to pid_*_set)
  float targetMag;          // |target|, for printout / safety
  float overshootThresh;    // kept for printout context
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
static void runVelocityAutotune(light::Drive* chassis,
                                pros::MotorGroup* L, pros::MotorGroup* R,
                                const VelocityTunerStep& s,
                                float kpStart, int maxItersPerPhase,
                                float kpMaxCap = 50.0f,
                                float kpBoostFactor = 2.0f,
                                float kdStartFrac = 0.10f,
                                float kdMaxCapMul = 10.0f,
                                float tolFrac = 0.05f,
                                int maxMotionMs = 4000) {
  auto runOnce = [&](float kP, float kD) -> VelocityMotionSample {
    s.setConsts(kP, kD);
    s.resetSensor();
    pros::delay(80);
    s.setTarget();
    VelocityMotionSample m = sampleMotion(s.readPos, L, R, s.target, maxMotionMs);
    chassis->pid_wait();
    pros::delay(500);  // catch any late ring
    s.driveBack();
    chassis->pid_wait();
    pros::delay(250);
    return m;
  };

  const int oscThresh = 3;  // crossings >= 3 ⇒ oscillating

  // ═══ Phase 1: find kP boundary (largest non-oscillating value) ═════════
  float pLow = 0.0f, pHigh = 0.0f;
  float kP = kpStart;
  int iter = 0;
  while (iter < maxItersPerPhase) {
    VelocityMotionSample m = runOnce(kP, 0.0f);
    bool osc = m.crossings >= oscThresh;
    printf("[AUTOTUNE][%s] kP-bracket iter=%d kP=%.3f peakAbs=%.2f crossings=%d %s\n",
           s.label, iter, kP, m.peakAbsPos, m.crossings, osc ? "OSCILLATES" : "stable");
    ++iter;
    if (osc) { pHigh = kP; break; }
    pLow = kP;
    kP *= 2.0f;
    if (kP >= kpMaxCap) { pHigh = kpMaxCap; break; }
  }

  // Never oscillated → use the largest tested kP, no kD.
  if (pHigh <= 0.0f) {
    printf("[AUTOTUNE][%s] FINAL kP=%.3f kI=0 kD=0 (no oscillation in kP bracket; capped)\n",
           s.label, pLow);
    s.setConsts(pLow, 0.0f);
    return;
  }

  while (iter < maxItersPerPhase && (pHigh - pLow) / pHigh > tolFrac) {
    kP = 0.5f * (pLow + pHigh);
    VelocityMotionSample m = runOnce(kP, 0.0f);
    bool osc = m.crossings >= oscThresh;
    printf("[AUTOTUNE][%s] kP-bisect iter=%d kP=%.3f peakAbs=%.2f crossings=%d %s [low=%.3f high=%.3f]\n",
           s.label, iter, kP, m.peakAbsPos, m.crossings,
           osc ? "OSCILLATES" : "stable", pLow, pHigh);
    if (osc) pHigh = kP; else pLow = kP;
    ++iter;
  }

  float kpBoundary = pLow;  // largest kP that didn't oscillate
  float kpBoosted  = kpBoundary * kpBoostFactor;
  printf("[AUTOTUNE][%s] kP boundary=%.3f → boosting to kP=%.3f (×%.2f) for kD search\n",
         s.label, kpBoundary, kpBoosted, kpBoostFactor);

  // ═══ Phase 2: find smallest kD that stops oscillation at boosted kP ════
  float dLow = 0.0f, dHigh = 0.0f;
  float kdMaxCap = kpBoosted * kdMaxCapMul;
  float kD = std::max(0.1f, kpBoosted * kdStartFrac);
  int dIter = 0;
  while (dIter < maxItersPerPhase) {
    VelocityMotionSample m = runOnce(kpBoosted, kD);
    bool osc = m.crossings >= oscThresh;
    printf("[AUTOTUNE][%s] kD-bracket iter=%d kD=%.3f peakAbs=%.2f crossings=%d %s\n",
           s.label, dIter, kD, m.peakAbsPos, m.crossings,
           osc ? "OSCILLATES" : "stable");
    ++dIter;
    if (!osc) { dHigh = kD; break; }
    dLow = kD;
    kD *= 2.0f;
    if (kD >= kdMaxCap) { dHigh = kdMaxCap; break; }
  }

  // kD couldn't damp even at cap → fall back to safe pure-P tune.
  if (dHigh <= 0.0f) {
    printf("[AUTOTUNE][%s] WARN kD didn't damp at kP=%.3f; falling back to boundary kP\n",
           s.label, kpBoosted);
    printf("[AUTOTUNE][%s] FINAL kP=%.3f kI=0 kD=0\n", s.label, kpBoundary);
    s.setConsts(kpBoundary, 0.0f);
    return;
  }

  while (dIter < maxItersPerPhase && (dHigh - dLow) / dHigh > tolFrac) {
    kD = 0.5f * (dLow + dHigh);
    VelocityMotionSample m = runOnce(kpBoosted, kD);
    bool osc = m.crossings >= oscThresh;
    printf("[AUTOTUNE][%s] kD-bisect iter=%d kD=%.3f peakAbs=%.2f crossings=%d %s [low=%.3f high=%.3f]\n",
           s.label, dIter, kD, m.peakAbsPos, m.crossings,
           osc ? "OSCILLATES" : "stable", dLow, dHigh);
    if (osc) dLow = kD; else dHigh = kD;
    ++dIter;
  }

  printf("[AUTOTUNE][%s] FINAL kP=%.3f kI=0 kD=%.3f\n", s.label, kpBoosted, dHigh);
  s.setConsts(kpBoosted, dHigh);
}

void autotune_turn_pid(float turnAngleDeg, float overshootThreshDeg,
                       float kpStart, int maxIters) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!chassis || !L || !R) {
    printf("[AUTOTUNE][turn] no chassis/motors\n");
    return;
  }
  // pid_*_set's speed-arg default is the user-owned light::TURN_SPEED /
  // DRIVE_SPEED / SWING_SPEED globals (defined in autons.cpp, hot package).
  // Pass 127 (full) explicitly so the cold-package link doesn't need them.
  VelocityTunerStep s;
  s.setConsts    = [&](float p, float d) { chassis->pid_turn_constants_set(p, 0.0, d); };
  s.setTarget    = [&]() { chassis->pid_turn_set((double)turnAngleDeg, 127); };
  s.driveBack    = [&]() { chassis->pid_turn_set(0.0, 127); };
  s.readPos      = [&]() -> float { return (float)chassis->drive_imu_get(); };
  s.resetSensor  = [&]() { chassis->drive_imu_reset(0.0); };
  s.target           = turnAngleDeg;
  s.targetMag        = std::fabs(turnAngleDeg);
  s.overshootThresh  = overshootThreshDeg;
  s.label            = "turn";
  runVelocityAutotune(chassis, L, R, s, kpStart, maxIters);
}

void autotune_drive_pid(float driveDistIn, float overshootThreshIn,
                        float kpStart, int maxIters) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!chassis || !L || !R) {
    printf("[AUTOTUNE][drive] no chassis/motors\n");
    return;
  }
  VelocityTunerStep s;
  s.setConsts   = [&](float p, float d) { chassis->pid_drive_constants_set(p, 0.0, d); };
  s.setTarget   = [&]() { chassis->pid_drive_set((double)driveDistIn, 127); };
  s.driveBack   = [&]() { chassis->pid_drive_set((double)-driveDistIn, 127); };
  s.readPos     = [&]() -> float {
    return (float)(0.5 * (chassis->drive_sensor_left() + chassis->drive_sensor_right()));
  };
  s.resetSensor = [&]() { chassis->drive_sensor_reset(); };
  s.target           = driveDistIn;
  s.targetMag        = std::fabs(driveDistIn);
  s.overshootThresh  = overshootThreshIn;
  s.label            = "drive";
  runVelocityAutotune(chassis, L, R, s, kpStart, maxIters);
}

void autotune_swing_pid(float swingAngleDeg, float overshootThreshDeg,
                        float kpStart, int maxIters) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup *L = nullptr, *R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!chassis || !L || !R) {
    printf("[AUTOTUNE][swing] no chassis/motors\n");
    return;
  }
  VelocityTunerStep s;
  s.setConsts   = [&](float p, float d) { chassis->pid_swing_constants_set(p, 0.0, d); };
  s.setTarget   = [&]() { chassis->pid_swing_set(light::LEFT_SWING, (double)swingAngleDeg, 127); };
  s.driveBack   = [&]() { chassis->pid_swing_set(light::LEFT_SWING, 0.0, 127); };
  s.readPos     = [&]() -> float { return (float)chassis->drive_imu_get(); };
  s.resetSensor = [&]() { chassis->drive_imu_reset(0.0); };
  s.targetMag        = std::fabs(swingAngleDeg);
  s.overshootThresh  = overshootThreshDeg;
  s.label            = "swing";
  runVelocityAutotune(chassis, L, R, s, kpStart, maxIters);
}

void autotune_heading_pid(float forwardV, float reliefV, int cycles, int timeoutMs,
                          int chunkCycles, int coolMs) {
  light::Drive* chassis = light::getChassis();
  pros::MotorGroup* L = nullptr;
  pros::MotorGroup* R = nullptr;
  light::getDriveMotorGroups(&L, &R);
  if (!chassis || !L || !R) {
    printf("[AUTOTUNE][heading] no chassis/motors\n");
    return;
  }

  EzPauseGuard guard(chassis, L, R);
  chassis->drive_imu_reset(0.0);
  pros::delay(50);

  // Robot creeps at forwardV; relay modulates L/R differential to keep
  // heading at 0 — tunes the heading-correction PID that runs in parallel
  // with drive PID.
  auto readErr = [&]() -> float {
    return -(float)chassis->drive_imu_get();
  };
  auto apply = [&](int sign) {
    float vL = forwardV - sign * reliefV;
    float vR = forwardV + sign * reliefV;
    L->move_voltage((int)(vL * 1000.0f));
    R->move_voltage((int)(vR * 1000.0f));
  };
  RelayResult r = runRelay(readErr, apply, reliefV, cycles, timeoutMs,
                           chunkCycles, coolMs);
  applyZnAndPrint("heading", r, [&](double p, double i, double d) {
    chassis->pid_heading_constants_set(p, i, d, 0.0);
  });
}

// Robot parked. var(per-tick deltas)/dt is the process-noise floor; pushes
// to the live EKF.
void autotune_ekf_noise(int sampleMs, int durationMs, int warmupMs) {
  light::Drive* chassis = light::getChassis();
  if (!chassis) {
    printf("[AUTOTUNE][ekf] no chassis\n");
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
}

// Robot parked with each distance sensor pointed at a wall in range.
// sensorSigmaIn = mean per-sensor stddev (in); outlierGapIn = max(3·σ, 1.0).
void autotune_mcl_noise(int sampleMs, int durationMs, int warmupMs) {
  light::Drive* chassis = light::getChassis();
  if (!chassis) {
    printf("[AUTOTUNE][mcl] no chassis\n");
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
}

}  // namespace light
