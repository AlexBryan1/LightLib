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

#include "LightLib/util/extras.hpp"
#include "path_runtime_internal.hpp"
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

}  // namespace light
