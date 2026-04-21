// ─── ramsete.cpp — RAMSETE path-following control loop ──────────────────────
//
// This subsystem commands `move_voltage` (battery-compensated) on the drive
// motor groups, taking exclusive ownership for the duration of a trajectory.
//
// INVARIANTS (do not violate):
//   * θ is radians everywhere inside this file. Convert once at the boundary
//     via light::getPoseRad(). `fmod` and `fmodf` are banned — use wrapRad().
//   * EZ-Template drive ownership is handed to us by EzPauseGuard on entry,
//     and released unconditionally by the guard's destructor — every exit
//     path (success, timeout, bailout, exception) must pass through it.
//   * RAMSETE owns the motors exclusively. Do not call any chassis.pid_*
//     motion while a trajectory is running.

#include "LightLib/ramsete.hpp"
#include "LightLib/ez_extra.hpp"
#include "LightLib/odom.hpp"
#include "EZ-Template/api.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include <atomic>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace light {

// ── Module configuration ────────────────────────────────────────────────────
static RamseteConfig    g_rc;
static DriveFF          g_ff;
static TrajConstraints  g_defaultCons;
static bool             g_configured     = false;
static float            g_charVoltage_mV = 12000.0f;   // assume fresh cell until characterized

// Guard against generating a new trajectory while one is being followed —
// on-brain trajectory generation takes 1–2 ms, long enough to stall a 100 Hz
// control loop. We force the user to cache trajectories outside of the
// follower by rejecting nested use.
static std::atomic<bool> g_genLocked{false};

// ── Small math helpers ──────────────────────────────────────────────────────
static inline float wrapRad(float a) {
    return std::atan2(std::sin(a), std::cos(a));
}

// Taylor-guarded sinc: sin(x)/x with series when |x| < 1e-4.
static inline float sincGuarded(float x) {
    if (std::fabs(x) < 1e-4f) {
        float x2 = x * x;
        return 1.0f - x2 / 6.0f + x2 * x2 / 120.0f;
    }
    return std::sin(x) / x;
}

// Convert wheel angular velocity (rpm at the motor) to linear speed in/s.
// vLinear = motorRPM * gearRatio * π * wheelDiam / 60
static inline float motorRpmToInPerSec(float rpm) {
    return rpm * g_rc.gearRatio * (float)M_PI * g_rc.wheelDiamIn / 60.0f;
}

// ── EZ pause/resume RAII guard ──────────────────────────────────────────────
//
// On construction: save EZ drive state, then force the drive task to write
// nothing and park it in DISABLE. On destruction: zero voltage, leave EZ
// DISABLED — the next EZ motion call re-arms cleanly. We never try to
// "restore" the previous mode because mid-motion state is stale.
struct EzPauseGuard {
    ez::Drive* chassis;
    pros::MotorGroup* left;
    pros::MotorGroup* right;

    EzPauseGuard(ez::Drive* c, pros::MotorGroup* l, pros::MotorGroup* r)
        : chassis(c), left(l), right(r) {
        if (chassis) {
            chassis->pid_drive_toggle(false);
            chassis->drive_mode_set(ez::DISABLE, /*stop_drive=*/true);
        }
    }
    ~EzPauseGuard() {
        if (left)  left->move_voltage(0);
        if (right) right->move_voltage(0);
        pros::delay(20);   // let zero-command actually land before releasing
    }
};

void ramsete_configure(RamseteConfig rc, DriveFF ff, TrajConstraints defaultCons) {
    g_rc          = rc;
    g_ff          = ff;
    g_defaultCons = defaultCons;
    g_configured  = true;
    g_charVoltage_mV = (float)pros::battery::get_voltage();
    if (g_charVoltage_mV < 8000.0f) g_charVoltage_mV = 12000.0f;
}

// ── Per-wheel controller state ──────────────────────────────────────────────
struct WheelState {
    float vFiltered = 0.0f;   // EMA-filtered actual velocity (in/s)
};

// α = 0.4 gives ~25 ms effective window at 10 ms control period — enough to
// smooth the 1 RPM quantization of get_actual_velocity() without adding
// meaningful phase lag to the velocity P term.
static constexpr float WHEEL_EMA_ALPHA = 0.4f;

static float wheelVoltageCmd(float vTarget, float aTarget,
                             float vActual, WheelState& ws,
                             float batteryScale) {
    ws.vFiltered = WHEEL_EMA_ALPHA * vActual + (1.0f - WHEEL_EMA_ALPHA) * ws.vFiltered;

    float kV = g_ff.kV * batteryScale;
    float kA = g_ff.kA * batteryScale;

    float sign = (vTarget > 0.05f) ? 1.0f : (vTarget < -0.05f ? -1.0f : 0.0f);
    float ff   = g_ff.kS * sign + kV * vTarget + kA * aTarget;
    float fb   = g_ff.kP * (vTarget - ws.vFiltered);
    float V    = ff + fb;
    return std::clamp(V, -12.0f, 12.0f);
}

// Scale both sides together when one would saturate — preserves the commanded
// steering ratio, same trick as light::moveToPoint. Returns true if clipped.
static bool balanceSaturation(float& vL, float& vR, float limit = 12.0f) {
    float m = std::max(std::fabs(vL), std::fabs(vR));
    if (m > limit) {
        vL = vL / m * limit;
        vR = vR / m * limit;
        return true;
    }
    return false;
}

// ── Event resolution ────────────────────────────────────────────────────────
// Mid-path events are supplied by the caller as (waypointIndex, action) and
// resolved to (time, action) once before the control loop starts. The loop
// then only does O(1) time comparisons per tick.
struct ResolvedEvent {
    float t;
    std::function<void()> action;
};

// Find the sample time of the traj.pts entry closest in (x,y) to wps[idx].
// Linear scan — O(N) per event; N ≈ 1500 samples for a 100-point path. Runs
// at trajectory-gen time, not inside the control loop.
static float waypointTime(const std::vector<Waypoint>& wps,
                          int idx, const Trajectory& traj) {
    float bestD2 = 1e18f;
    float bestT  = 0.0f;
    const float wx = wps[idx].x;
    const float wy = wps[idx].y;
    for (const auto& p : traj.pts) {
        float dx = p.x - wx;
        float dy = p.y - wy;
        float d2 = dx*dx + dy*dy;
        if (d2 < bestD2) { bestD2 = d2; bestT = p.t; }
    }
    return bestT;
}

static bool followTrajectoryCore(const Trajectory& traj,
                                 std::vector<ResolvedEvent> events,
                                 int timeoutMs,
                                 float poseErrBailIn) {
    // Pre-flight checks — fail loudly, don't silently run on bad inputs.
    if (!g_configured) {
        printf("[RAMSETE] ramsete_configure() has not been called\n");
        return false;
    }
    if (traj.empty() || traj.duration <= 0.0f) {
        printf("[RAMSETE] empty trajectory\n");
        return false;
    }

    ez::Drive* chassis = light::getChassis();
    pros::MotorGroup* left  = nullptr;
    pros::MotorGroup* right = nullptr;
    light::getDriveMotorGroups(&left, &right);
    if (!left || !right) {
        printf("[RAMSETE] motor groups not registered via ez_extra_init\n");
        return false;
    }

    float battV = (float)pros::battery::get_voltage();
    if (battV < 10500.0f) {
        printf("[RAMSETE] battery too low (%.0f mV) — aborting\n", battV);
        return false;
    }

    // Initial pose sanity — huge e_x at t=0 causes a dangerous lurch.
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

    // Telemetry accumulators.
    float peakErr  = 0.0f;
    float rmsNum   = 0.0f;
    int   rmsCnt   = 0;
    int   satTicks = 0;
    int   totTicks = 0;

    // Sustained-error watchdog.
    uint32_t bailStart = 0;
    bool bailArmed = false;

    // Divergence watchdog (wheel EMA vs odom local speed).
    uint32_t divergeStart = 0;
    bool divergeArmed = false;

    WheelState wsL, wsR;

    uint32_t t0   = pros::millis();
    uint32_t next = t0;

    const uint32_t hardTimeoutMs =
        (timeoutMs > 0) ? (uint32_t)timeoutMs
                        : (uint32_t)(traj.duration * 1000.0f) + 500;

    bool success = false;
    const float W = g_rc.trackWidthIn;
    const float b = g_rc.b;
    const float zeta = g_rc.zeta;

    // Sort events by firing time so the per-tick check only needs to peek at
    // the head. Cursor advances monotonically.
    std::sort(events.begin(), events.end(),
              [](const ResolvedEvent& a, const ResolvedEvent& b){ return a.t < b.t; });
    std::size_t nextEvent = 0;

    while (true) {
        uint32_t now = pros::millis();
        uint32_t elapsed = now - t0;
        float tRel = elapsed / 1000.0f;

        if (elapsed > hardTimeoutMs) break;

        // Fire any events whose sampled time has arrived. Callbacks run on
        // the control thread — a slow callback will delay the next tick.
        while (nextEvent < events.size() && tRel >= events[nextEvent].t) {
            if (events[nextEvent].action) events[nextEvent].action();
            ++nextEvent;
        }

        TrajState s = traj.sample(tRel);

        Pose pose = light::getPoseRad();

        // Pose error in the robot's current frame.
        // LightLib θ: 0 faces +Y, CW-positive. Rotate world (dx, dy) into robot
        // frame (e_x forward, e_y left) with standard 2-D rotation, adjusted so
        // that "forward" is along +Y.
        float dx = s.x - pose.x;
        float dy = s.y - pose.y;
        float cosT = std::cos(pose.theta);
        float sinT = std::sin(pose.theta);
        // forward = along (sin θ, cos θ); left  = along (cos θ, -sin θ)
        float e_x =  sinT * dx + cosT * dy;
        float e_y =  cosT * dx - sinT * dy;
        float e_th = wrapRad(s.theta - pose.theta);

        // 0.3" deadband on position error — kills sub-inch odom noise that
        // otherwise thrashes the commanded velocity on long straights.
        if (std::fabs(e_x) < 0.3f) e_x = 0.0f;
        if (std::fabs(e_y) < 0.3f) e_y = 0.0f;

        float posErr = std::hypot(dx, dy);
        peakErr = std::max(peakErr, posErr);
        rmsNum += posErr * posErr;
        rmsCnt++;

        // Sustained-error bailout: >threshold for >150 ms → we've lost the path.
        if (posErr > poseErrBailIn) {
            if (!bailArmed) { bailArmed = true; bailStart = now; }
            else if (now - bailStart > 150) {
                printf("[RAMSETE] bail: pose err %.2f > %.2f for >150ms\n",
                       posErr, poseErrBailIn);
                break;
            }
        } else {
            bailArmed = false;
        }

        // RAMSETE control law (see Corke / Bradski RAMSETE derivation).
        float vr = s.v;
        float wr = s.omega;
        float k  = 2.0f * zeta * std::sqrt(wr*wr + b * vr * vr);
        float sinc_e = sincGuarded(e_th);
        float v_cmd = vr * std::cos(e_th)  + k * e_x;
        float w_cmd = wr + b * vr * sinc_e * e_y + k * e_th;

        // Tank kinematics — split (v, ω) into per-wheel linear speed.
        float vL_target = v_cmd - w_cmd * W * 0.5f;
        float vR_target = v_cmd + w_cmd * W * 0.5f;

        // Per-wheel actual velocity from the motors.
        float vL_actual = motorRpmToInPerSec((float)left->get_actual_velocity());
        float vR_actual = motorRpmToInPerSec((float)right->get_actual_velocity());

        // Divergence check — EMA disagreeing with odom longitudinal speed for
        // >200 ms means we've lost a tracking wheel or a motor cable.
        {
            Pose vl = light::getLocalSpeed(true);  // radians variant — theta unused
            float vAvgWheel = 0.5f * (wsL.vFiltered + wsR.vFiltered);
            float vAvgOdom  = vl.y;   // +Y-forward local speed
            if (std::fabs(vAvgWheel - vAvgOdom) > 4.0f) {
                if (!divergeArmed) { divergeArmed = true; divergeStart = now; }
                else if (now - divergeStart > 200) {
                    printf("[RAMSETE] bail: wheel/odom speed divergence\n");
                    break;
                }
            } else {
                divergeArmed = false;
            }
        }

        // Battery scaling on kV/kA — clamp so a critically low cell can't
        // command unsafely large voltages.
        float batt_mV = (float)pros::battery::get_voltage();
        float battScale = g_charVoltage_mV / std::max(batt_mV, 1.0f);
        battScale = std::clamp(battScale, 0.85f, 1.10f);

        // Acceleration is the same for both wheels on a diff-drive at first
        // order — the ω̇ contribution is 2nd-order small for the cadence here.
        float aL = s.a, aR = s.a;

        float vL_volts = wheelVoltageCmd(vL_target, aL, vL_actual, wsL, battScale);
        float vR_volts = wheelVoltageCmd(vR_target, aR, vR_actual, wsR, battScale);

        if (balanceSaturation(vL_volts, vR_volts)) satTicks++;
        totTicks++;

        left->move_voltage((int)(vL_volts * 1000.0f));
        right->move_voltage((int)(vR_volts * 1000.0f));

        // Success condition: trajectory has played out AND we're close.
        if (tRel >= traj.duration && posErr < std::min(1.5f, poseErrBailIn * 0.5f)) {
            success = true;
            break;
        }

        // Control-rate pacing.
        next += 10;
        uint32_t wait = (next > pros::millis()) ? (next - pros::millis()) : 0;
        pros::delay(wait == 0 ? 1 : wait);
    }

    // EzPauseGuard dtor zeros both motors and delays 20 ms before returning.

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

bool followTrajectory(const Trajectory& traj,
                      int timeoutMs,
                      float poseErrBailIn) {
    return followTrajectoryCore(traj, {}, timeoutMs, poseErrBailIn);
}

// Shared body for the wps-overloads: generate the trajectory, resolve events
// to sample times, then hand off to the core loop.
static bool followTrajectoryWithEvents(const std::vector<Waypoint>& wps,
                                       const TrajConstraints& cons,
                                       std::vector<PathEvent> events,
                                       bool reversed,
                                       int timeoutMs,
                                       float poseErrBailIn) {
    bool expected = false;
    if (!g_genLocked.compare_exchange_strong(expected, true)) {
        printf("[RAMSETE] trajectory generation already in progress\n");
        return false;
    }
    Trajectory traj = generateTrajectory(wps, cons, reversed);
    g_genLocked.store(false);

    if (traj.empty()) return false;

    std::vector<ResolvedEvent> resolved;
    resolved.reserve(events.size());
    for (auto& e : events) {
        if (e.atWaypoint < 0 || e.atWaypoint >= (int)wps.size()) {
            printf("[RAMSETE] event waypoint idx %d out of range [0, %u)\n",
                   e.atWaypoint, (unsigned)wps.size());
            continue;
        }
        if (!e.action) continue;
        resolved.push_back({ waypointTime(wps, e.atWaypoint, traj),
                             std::move(e.action) });
    }
    return followTrajectoryCore(traj, std::move(resolved), timeoutMs, poseErrBailIn);
}

bool followTrajectory(const std::vector<Waypoint>& wps,
                      const TrajConstraints& cons,
                      bool reversed,
                      int timeoutMs,
                      float poseErrBailIn) {
    return followTrajectoryWithEvents(wps, cons, {}, reversed, timeoutMs, poseErrBailIn);
}

bool followTrajectory(const std::vector<Waypoint>& wps,
                      const TrajConstraints& cons,
                      std::vector<PathEvent> events,
                      bool reversed,
                      int timeoutMs,
                      float poseErrBailIn) {
    return followTrajectoryWithEvents(wps, cons, std::move(events),
                                      reversed, timeoutMs, poseErrBailIn);
}

// ── Jerryio CSV loader ──────────────────────────────────────────────────────
//
// Parses a waypoint CSV and runs it through followTrajectory using the
// defaults from ramsete_configure(). Two formats are auto-detected:
//
//   1. Simple waypoint list: "x, y" or "x, y, heading_rad" per line.
//      Path is treated as already relative to the robot's start pose.
//
//   2. Jerryio densified export (begins with "#PATH-POINTS-START"):
//      "x, y, speed[, heading_deg]" per line. Speed/heading columns are
//      ignored — the dense x,y points fully define the curve, and the
//      trajectory generator computes its own velocity profile. The path is
//      translated so its first point lands at the robot's current pose
//      (VGPS-style absolute coords wouldn't match LightLib's relative frame).
//
// Blank lines and '#'-comments are skipped. A UTF-8 BOM at the start is
// tolerated.
// Parses the CSV into waypoints + anchor indices (indices into wps that
// represent "key" points — heading-bearing lines for a Jerryio densified
// export, or all lines for a sparse hand-written CSV). Returns false on
// any parse-level failure (null input, <2 waypoints).
static bool parseJerryio(const char* csv,
                         std::vector<Waypoint>& wps,
                         std::vector<int>& anchorIdx,
                         bool& densified) {
    if (!csv) { printf("[JERRYIO] null csv\n"); return false; }

    const char* p = csv;

    // UTF-8 BOM sometimes prepended by browser-side exporters.
    if ((unsigned char)p[0] == 0xEF &&
        (unsigned char)p[1] == 0xBB &&
        (unsigned char)p[2] == 0xBF) {
        p += 3;
    }

    // Jerryio's dense-sampled export begins with this marker. When present,
    // column 3 is speed (not heading in radians), so we parse x, y, speed,
    // heading_deg but only use x, y for the waypoint. Heading column presence
    // flags the point as an anchor (a Jerryio end-point / user click).
    densified = (std::strstr(p, "#PATH-POINTS-START") != nullptr);
    const int maxCols     = densified ? 4 : 3;
    const int headingCol  = densified ? 3 : 2;   // 0-based

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
        while (n < maxCols && c < lineEnd) {
            char* endp = nullptr;
            float v = std::strtof(c, &endp);
            if (endp == c) break;
            vals[n++] = v;
            c = endp;
            while (c < lineEnd && (*c == ',' || *c == ' ' || *c == '\t')) ++c;
        }

        if (n < 2) continue;

        Waypoint w;
        w.x = vals[0];
        w.y = vals[1];
        bool hasHeading = (n > headingCol);
        if (hasHeading && !densified) {
            // Sparse CSV: column 3 is heading in radians (LightLib convention).
            w.headingRad = vals[headingCol];
        }
        // For densified paths the column-4 heading is in degrees in Jerryio's
        // own frame — we don't pin headings on densified paths (the spline
        // chord tangent is what the trajectory generator uses anyway).
        int idx = (int)wps.size();
        wps.push_back(w);
        if (hasHeading) anchorIdx.push_back(idx);
    }

    if (wps.size() < 2) {
        printf("[JERRYIO] need >=2 waypoints, got %u\n", (unsigned)wps.size());
        return false;
    }

    // Fallback for CSVs that never set a heading: every line is an anchor.
    if (anchorIdx.empty()) {
        anchorIdx.reserve(wps.size());
        for (int i = 0; i < (int)wps.size(); ++i) anchorIdx.push_back(i);
    }

    if (densified) {
        // Subtract first point from every waypoint so path starts at (0, 0)
        // in the robot's frame. Call resetPose yourself beforehand if your
        // odometry is already aligned to Jerryio's field origin.
        float x0 = wps[0].x, y0 = wps[0].y;
        for (auto& w : wps) { w.x -= x0; w.y -= y0; }
    }

    return true;
}

// Remap PathEvent.atWaypoint from "anchor index" (user-facing) to the raw
// index into the waypoint list that followTrajectory expects. Out-of-range
// entries are dropped with a warning.
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
                    float poseErrBailIn) {
    std::vector<Waypoint> wps;
    std::vector<int> anchorIdx;
    bool densified = false;
    if (!parseJerryio(csv, wps, anchorIdx, densified)) return false;
    return followTrajectory(wps, g_defaultCons, reversed, timeoutMs, poseErrBailIn);
}

bool runJerryioPath(const char* csv,
                    std::vector<PathEvent> events,
                    bool reversed,
                    int timeoutMs,
                    float poseErrBailIn) {
    std::vector<Waypoint> wps;
    std::vector<int> anchorIdx;
    bool densified = false;
    if (!parseJerryio(csv, wps, anchorIdx, densified)) return false;
    auto mapped = remapAnchorEvents(std::move(events), anchorIdx);
    return followTrajectory(wps, g_defaultCons, std::move(mapped),
                            reversed, timeoutMs, poseErrBailIn);
}

// Reads the SD file into `buf` (size BUF_SIZE). Returns false on any failure
// (null path, open fail, overflow).
static bool readSDFile(const char* filePath, char* buf, std::size_t bufSize) {
    if (!filePath) { printf("[JERRYIO] null file path\n"); return false; }

    FILE* f = std::fopen(filePath, "r");
    if (!f) { printf("[JERRYIO] cannot open %s\n", filePath); return false; }

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
                          float poseErrBailIn) {
    constexpr std::size_t BUF_SIZE = 4096;
    char buf[BUF_SIZE];
    if (!readSDFile(filePath, buf, BUF_SIZE)) return false;
    return runJerryioPath(buf, reversed, timeoutMs, poseErrBailIn);
}

bool runJerryioPathFromSD(const char* filePath,
                          std::vector<PathEvent> events,
                          bool reversed,
                          int timeoutMs,
                          float poseErrBailIn) {
    constexpr std::size_t BUF_SIZE = 4096;
    char buf[BUF_SIZE];
    if (!readSDFile(filePath, buf, BUF_SIZE)) return false;
    return runJerryioPath(buf, std::move(events), reversed, timeoutMs, poseErrBailIn);
}

// ── Characterization routines — thin wrappers kept here so they share the
//    battery, motor-group, and EzPauseGuard infrastructure. Each one is
//    intended to be the body of its own selectable auton. Values are
//    printf'd — user transcribes into ramsete_configure() in default_constants.

static void driveAllVolts(pros::MotorGroup* L, pros::MotorGroup* R, float volts) {
    L->move_voltage((int)(volts * 1000.0f));
    R->move_voltage((int)(volts * 1000.0f));
}

void characterize_kV_kA_kS(float maxVoltage, float rampVps) {
    ez::Drive* chassis = light::getChassis();
    pros::MotorGroup* L = nullptr; pros::MotorGroup* R = nullptr;
    light::getDriveMotorGroups(&L, &R);
    if (!L || !R) { printf("[CHAR] no motor groups\n"); return; }

    EzPauseGuard guard(chassis, L, R);

    // Phase 1 — kS: slow 0.05 V / 200 ms ramp until either side moves.
    float v = 0.0f;
    float kS = 0.0f;
    while (v < 4.0f) {
        driveAllVolts(L, R, v);
        pros::delay(200);
        float sL = std::fabs(motorRpmToInPerSec((float)L->get_actual_velocity()));
        float sR = std::fabs(motorRpmToInPerSec((float)R->get_actual_velocity()));
        if (sL > 1.0f || sR > 1.0f) { kS = v; break; }
        v += 0.05f;
    }
    printf("[CHAR] kS ≈ %.3f V\n", kS);
    driveAllVolts(L, R, 0);
    pros::delay(500);

    // Phase 2 — kV: quasi-static ramp, regress v_actual vs (V - kS).
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

    // Phase 3 — kA: step 0 → 8 V, capture accel in first 300 ms.
    driveAllVolts(L, R, 8.0f);
    uint32_t tStep = pros::millis();
    float vPrev = 0.0f;
    float aMax  = 0.0f;
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
    // Estimate kA using final measurements. We don't know vel at t=0 exactly,
    // but during the initial step vel << steady-state, so kA ≈ (8 - kS)/aMax.
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
    ez::Drive* chassis = light::getChassis();
    pros::MotorGroup* L = nullptr; pros::MotorGroup* R = nullptr;
    light::getDriveMotorGroups(&L, &R);
    if (!L || !R) { printf("[CHAR] no motor groups\n"); return 0.0f; }

    EzPauseGuard guard(chassis, L, R);

    L->tare_position();
    R->tare_position();
    Pose start = light::getPoseRad();
    (void)start;

    // Spin in place at low voltage for long enough to integrate the encoder
    // readings. The user watches the robot and the terminal.
    driveAllVolts(L, R, 0);
    L->move_voltage( 3000);
    R->move_voltage(-3000);

    pros::delay((uint32_t)(rotations * 800));  // ~0.8 s per rotation at 3 V

    L->move_voltage(0); R->move_voltage(0);
    pros::delay(200);

    float posL = std::fabs(L->get_position()) * g_rc.gearRatio
                 * (float)M_PI * g_rc.wheelDiamIn / 360.0f;
    float posR = std::fabs(R->get_position()) * g_rc.gearRatio
                 * (float)M_PI * g_rc.wheelDiamIn / 360.0f;
    float arc = 0.5f * (posL + posR);

    Pose now = light::getPoseRad();
    float turnedRad = std::fabs(wrapRad(now.theta - start.theta));
    if (turnedRad < 0.1f) { printf("[CHAR] turn too small\n"); return 0.0f; }

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

float characterize_a_lat_max() {
    // Commanded constant-radius circle; ramp v until odom radius deviates
    // more than 15%. Conservative aLatMax = 0.7 · v²/R.
    ez::Drive* chassis = light::getChassis();
    pros::MotorGroup* L = nullptr; pros::MotorGroup* R = nullptr;
    light::getDriveMotorGroups(&L, &R);
    if (!L || !R) { printf("[CHAR] no motor groups\n"); return 0.0f; }

    EzPauseGuard guard(chassis, L, R);

    const float R_target = 24.0f;
    float v = 12.0f;
    float aLatSafe = 0.0f;
    while (v < 60.0f) {
        // vL/vR for circle of radius R_target: ratio = (R - W/2)/(R + W/2)
        float W = g_rc.trackWidthIn;
        float vL = v * (R_target - W*0.5f) / R_target;
        float vR = v * (R_target + W*0.5f) / R_target;

        // crude volts-from-speed — use kV feedforward only
        float Vl = g_ff.kS + g_ff.kV * vL;
        float Vr = g_ff.kS + g_ff.kV * vR;
        L->move_voltage((int)(std::clamp(Vl, -12.0f, 12.0f) * 1000));
        R->move_voltage((int)(std::clamp(Vr, -12.0f, 12.0f) * 1000));

        // Let it settle, then measure odom radius over 500 ms.
        pros::delay(500);
        Pose p0 = light::getPoseRad();
        pros::delay(500);
        Pose p1 = light::getPoseRad();
        float dth = std::fabs(wrapRad(p1.theta - p0.theta));
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

// ── Relay-feedback PID auto-tune ────────────────────────────────────────────
//
// Generic bang-bang oscillator. `readError` returns a signed error in whatever
// unit the PID sees (degrees, inches, etc.). `applyRelay(sign)` drives the
// actuator at ±h · sign. Runs at 10 ms ticks until `cycles` zero-crossings
// are collected or `timeoutMs` elapses. Ku and Pu come from Åström's formula:
//   Ku = 4h / (π·a),  Pu = mean period between matched zero-crossings.

struct RelayResult {
    float Ku    = 0.0f;
    float Pu    = 0.0f;
    int   cyclesSeen = 0;
    bool  ok    = false;
};

static RelayResult runRelay(std::function<float()> readError,
                            std::function<void(int)> applyRelay,
                            float h, int cycles, int timeoutMs,
                            int chunkCycles = 0, int coolMs = 0) {
    RelayResult out;
    if (!readError || !applyRelay || h <= 0.0f || cycles <= 0) return out;

    // Hysteresis band ~2% of h to ignore sensor jitter around zero.
    const float band = 0.02f * h + 1e-3f;
    int sign = (readError() >= 0.0f) ? -1 : 1;  // push toward zero first
    applyRelay(sign);

    uint32_t t0      = pros::millis();
    uint32_t tLastZX = t0;
    float    curMax  = -1e9f;
    float    curMin  =  1e9f;
    bool     haveHalfCycle = false;

    std::vector<float> amps;   // per-half-cycle extrema spread
    std::vector<float> pers;   // period between like-signed zero-crossings

    // Motor-cooldown chunking: every `halfPerChunk` half-cycles, kill the
    // relay for `coolMs` to keep motors from cooking on long tunes. Active
    // time (not cooldown time) counts against timeoutMs, so we bump t0 by
    // coolMs after each pause.
    const int halfPerChunk = (chunkCycles > 0 && coolMs > 0) ? (2 * chunkCycles) : 0;
    int halfInChunk = 0;

    while ((int)(pros::millis() - t0) < timeoutMs) {
        float e = readError();
        if (e > curMax) curMax = e;
        if (e < curMin) curMin = e;

        // Zero-crossing with hysteresis: sign flips only after error crosses
        // through the opposite band.
        bool flip = false;
        if (sign > 0 && e >  band) { flip = true; }  // moved positive → push down
        if (sign < 0 && e < -band) { flip = true; }  // moved negative → push up
        if (flip) {
            uint32_t now = pros::millis();
            if (haveHalfCycle) {
                // full half-cycle captured: min→max spread / 2 = amplitude
                float amp = 0.5f * (curMax - curMin);
                amps.push_back(amp);
                pers.push_back((now - tLastZX) / 1000.0f);  // half-period in s
            }
            curMax = -1e9f;
            curMin =  1e9f;
            tLastZX = now;
            sign = -sign;
            applyRelay(sign);
            haveHalfCycle = true;
            halfInChunk++;

            if ((int)amps.size() >= 2 * cycles) break;   // cycles full periods

            if (halfPerChunk > 0 && halfInChunk >= halfPerChunk) {
                // Chunk full — cool off before next chunk. Throw away the
                // first half-cycle after resume (transient from re-priming).
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
                curMin =  1e9f;
                haveHalfCycle = false;
            }
        }

        pros::delay(10);
    }

    applyRelay(0);
    if (amps.size() < 3 || pers.size() < 3) return out;

    // Drop the first half-cycle — it's a transient from the initial push.
    float aSum = 0, pSum = 0;
    int   n    = 0;
    for (std::size_t i = 1; i < amps.size(); ++i) { aSum += amps[i]; pSum += pers[i]; ++n; }
    float aMean = aSum / n;
    float pHalf = pSum / n;              // seconds per half-period
    float Pu    = 2.0f * pHalf;

    if (aMean < 1e-4f || Pu < 0.02f) return out;
    out.Ku = 4.0f * h / ((float)M_PI * aMean);
    out.Pu = Pu;
    out.cyclesSeen = n / 2;
    out.ok = true;
    return out;
}

static void applyZnAndPrint(const char* tag, const RelayResult& r,
                            std::function<void(double,double,double)> setter) {
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

void autotune_turn_pid(float reliefV, int cycles, int timeoutMs,
                       int chunkCycles, int coolMs) {
    ez::Drive* chassis = light::getChassis();
    pros::MotorGroup* L = nullptr; pros::MotorGroup* R = nullptr;
    light::getDriveMotorGroups(&L, &R);
    if (!chassis || !L || !R) { printf("[AUTOTUNE][turn] no chassis/motors\n"); return; }

    EzPauseGuard guard(chassis, L, R);
    chassis->drive_imu_reset(0.0);
    pros::delay(50);

    auto readErr = [&]() -> float {
        return -(float)chassis->drive_imu_get();   // setpoint = 0°
    };
    auto apply = [&](int sign) {
        int mV = (int)(sign * reliefV * 1000.0f);
        L->move_voltage( mV);
        R->move_voltage(-mV);
    };
    RelayResult r = runRelay(readErr, apply, reliefV, cycles, timeoutMs,
                             chunkCycles, coolMs);
    applyZnAndPrint("turn", r, [&](double p, double i, double d) {
        chassis->pid_turn_constants_set(p, i, d, 0.0);
    });
}

void autotune_drive_pid(float reliefV, int cycles, int timeoutMs,
                        int chunkCycles, int coolMs) {
    ez::Drive* chassis = light::getChassis();
    pros::MotorGroup* L = nullptr; pros::MotorGroup* R = nullptr;
    light::getDriveMotorGroups(&L, &R);
    if (!chassis || !L || !R) { printf("[AUTOTUNE][drive] no chassis/motors\n"); return; }

    EzPauseGuard guard(chassis, L, R);
    chassis->drive_sensor_reset();
    pros::delay(50);

    const float target = 12.0f;   // inches ahead of start
    auto readErr = [&]() -> float {
        double avg = 0.5 * (chassis->drive_sensor_left() + chassis->drive_sensor_right());
        return target - (float)avg;
    };
    auto apply = [&](int sign) {
        int mV = (int)(sign * reliefV * 1000.0f);
        L->move_voltage(mV);
        R->move_voltage(mV);
    };
    RelayResult r = runRelay(readErr, apply, reliefV, cycles, timeoutMs,
                             chunkCycles, coolMs);
    applyZnAndPrint("drive", r, [&](double p, double i, double d) {
        chassis->pid_drive_constants_set(p, i, d, 0.0);
    });
}

void autotune_swing_pid(float reliefV, int cycles, int timeoutMs,
                        int chunkCycles, int coolMs) {
    ez::Drive* chassis = light::getChassis();
    pros::MotorGroup* L = nullptr; pros::MotorGroup* R = nullptr;
    light::getDriveMotorGroups(&L, &R);
    if (!chassis || !L || !R) { printf("[AUTOTUNE][swing] no chassis/motors\n"); return; }

    EzPauseGuard guard(chassis, L, R);
    chassis->drive_imu_reset(0.0);
    pros::delay(50);

    // Left-swing variant: only the left motors drive; right side is held at 0 V.
    auto readErr = [&]() -> float {
        return -(float)chassis->drive_imu_get();
    };
    auto apply = [&](int sign) {
        int mV = (int)(sign * reliefV * 1000.0f);
        L->move_voltage(mV);
        R->move_voltage(0);
    };
    RelayResult r = runRelay(readErr, apply, reliefV, cycles, timeoutMs,
                             chunkCycles, coolMs);
    applyZnAndPrint("swing", r, [&](double p, double i, double d) {
        chassis->pid_swing_constants_set(p, i, d, 0.0);
    });
}

void autotune_heading_pid(float forwardV, float reliefV, int cycles, int timeoutMs,
                          int chunkCycles, int coolMs) {
    ez::Drive* chassis = light::getChassis();
    pros::MotorGroup* L = nullptr; pros::MotorGroup* R = nullptr;
    light::getDriveMotorGroups(&L, &R);
    if (!chassis || !L || !R) { printf("[AUTOTUNE][heading] no chassis/motors\n"); return; }

    EzPauseGuard guard(chassis, L, R);
    chassis->drive_imu_reset(0.0);
    pros::delay(50);

    // Robot creeps forward at forwardV; relay modulates L/R differential to
    // bring heading back to 0. Effectively tunes the heading-correction PID
    // which runs in parallel with drive PID during a drive_pid move.
    auto readErr = [&]() -> float {
        return -(float)chassis->drive_imu_get();
    };
    auto apply = [&](int sign) {
        float vL = forwardV + sign * reliefV;
        float vR = forwardV - sign * reliefV;
        L->move_voltage((int)(vL * 1000.0f));
        R->move_voltage((int)(vR * 1000.0f));
    };
    RelayResult r = runRelay(readErr, apply, reliefV, cycles, timeoutMs,
                             chunkCycles, coolMs);
    applyZnAndPrint("heading", r, [&](double p, double i, double d) {
        chassis->pid_heading_constants_set(p, i, d, 0.0);
    });
}

} // namespace light
