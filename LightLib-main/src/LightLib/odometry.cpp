// +-----------------------------------------------------------------------------+
// |  odometry.cpp -- 2-D position tracking for the robot                        |
// |                                                                             |
// |  This file implements a classic arc-based odometry system.  It reads        |
// |  tracking wheels (and optionally an IMU) to figure out where the robot      |
// |  is on the field at all times.                                              |
// |                                                                             |
// |  COORDINATE SYSTEM:                                                         |
// |                                                                             |
// |          +Y (forward)                                                       |
// |           ^                                                                 |
// |           |                                                                 |
// |   -X <----+----> +X (right)                                                 |
// |           |                                                                 |
// |           v                                                                 |
// |          -Y                                                                 |
// |                                                                             |
// |    Theta = 0 facing +Y, increases clockwise (radians internally).           |
// |                                                                             |
// |  SENSOR LAYOUT (top-down view):                                             |
// |                                                                             |
// |      V1 |   | V2        V1, V2 = vertical tracking wheels                   |
// |      +--+---+--+        H1, H2 = horizontal tracking wheels                 |
// |   H1-+  | C |  +-H2    C      = center of rotation                          |
// |      +--+---+--+                                                            |
// |         |   |            Offset = signed dist from wheel to C               |
// |                          (positive = right / forward of center)             |
// |                                                                             |
// |  HOW IT WORKS (each tick, every 10 ms):                                     |
// |    1. Read raw encoder positions from all tracking wheels + IMU.            |
// |    2. Compute deltas (change since last tick).                              |
// |    3. Determine heading change (deltaHeading) using best source:            |
// |       - Two horizontal wheels  -> arc formula (most accurate)               |
// |       - Two unpowered verticals -> arc formula                              |
// |       - IMU -> gyro integration                                             |
// |       - Two powered verticals  -> arc formula (fallback)                    |
// |    4. Convert encoder deltas to local displacement (localX, localY)         |
// |       using the arc approximation:                                          |
// |         localX = 2 sin(dH/2) * (dX/dH + horizOffset)                        |
// |         localY = 2 sin(dH/2) * (dY/dH + vertOffset)                         |
// |       If the robot drove straight (dH ~ 0), simplifies to raw deltas.       |
// |    5. Rotate local displacement into global frame using avgHeading:         |
// |         globalX +=  localY sin(h_avg) - localX cos(h_avg)                   |
// |         globalY +=  localY cos(h_avg) + localX sin(h_avg)                   |
// |    6. Update speed estimates with exponential moving average (EMA)          |
// |       so they are smooth enough for motion prediction.                      |
// |                                                                             |
// |  SETUP:                                                                     |
// |    1. Create TrackingWheel objects for each sensor (see odom.hpp).          |
// |    2. Bundle them into an OdomSensors struct.                               |
// |    3. Call light::init(sensors) once in initialize().                       |
// |    4. Use light::getPose() anywhere to read the robot position.             |
// |    5. Use light::setPose() to set a known starting position.                |
// |                                                                             |
// |  EXAMPLE:                                                                   |
// |    // In initialize():                                                      |
// |    TrackingWheel leftWheel(&leftRot,  2.75, -2.5);  // 2.75in wheel,        |
// |    TrackingWheel rightWheel(&rightRot, 2.75,  2.5); // offsets from ctr     |
// |    TrackingWheel backWheel(&backRot,  2.75, -3.0);                          |
// |    OdomSensors sensors(&leftWheel, &rightWheel,                             |
// |                        &backWheel, nullptr, &imu);                          |
// |    light::init(sensors);                                                    |
// |    light::setPose(Pose(0, 0, 0));                                           |
// |                                                                             |
// |    // Anywhere:                                                             |
// |    Pose pos = light::getPose();  // x, y in inches, theta in degrees        |
// +-----------------------------------------------------------------------------+

#include "LightLib/odom.hpp"
#include "LightLib/ekf.hpp"
#include "LightLib/lightcast.hpp"
#include "EZ-Template/drive/drive.hpp"
#include <cmath>

// ─── Helpers ─────────────────────────────────────────────────────────────────

static float degToRad(float deg) { return deg * M_PI / 180.0f; }
static float radToDeg(float rad) { return rad * 180.0f / M_PI; }

// ─── State ───────────────────────────────────────────────────────────────────
// All odometry state is file-static — there's one global robot position.
// odomPose:       current (x, y, θ) in inches and radians
// odomSpeed:      global-frame velocity (smoothed via EMA), in inches/sec
// odomLocalSpeed: robot-frame velocity (forward/strafe/turn), in inches/sec
static OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr);
static MCLConfig   odomCfg;

static Pose odomPose;
static Pose odomSpeed;
static Pose odomLocalSpeed;

// "prev" values store the last-read raw encoder/IMU positions so we can
// compute deltas each tick.  Two sets exist:
//   prevVertical1/2, prevHorizontal1/2  — per-sensor, used for heading calc
//   prevVertical, prevHorizontal        — per-chosen-wheel, used for displacement
static float prevVertical    = 0;
static float prevVertical1   = 0;
static float prevVertical2   = 0;
static float prevHorizontal  = 0;
static float prevHorizontal1 = 0;
static float prevHorizontal2 = 0;
static float prevImu         = 0;

static pros::Task* trackingTask = nullptr;

// ─── Public API ───────────────────────────────────────────────────────────────

// getPose() — returns the robot's current position.
// By default returns degrees for theta; pass true for radians.
Pose light::getPose(bool radians) {
    if (radians) return odomPose;
    return Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

// setPose() — teleport the robot's tracked position (e.g., at the start of auton).
// Pass degrees by default, or radians if you set the flag.
void light::setPose(Pose pose, bool radians) {
    odomPose = radians ? pose : Pose(pose.x, pose.y, degToRad(pose.theta));
    // Propagate to the fused estimators so the next update() doesn't pull the
    // pose back to the stale EKF mean.
    light::ekf::reset(odomPose);
    light::lightcast::init(odomPose, odomSensors.distanceSensors);
}

// getSpeed() — global-frame velocity (how fast x, y, θ are changing).
// Useful for feed-forward or motion profiling.
Pose light::getSpeed(bool radians) {
    if (radians) return odomSpeed;
    return Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

// getLocalSpeed() — robot-frame velocity (forward, strafe, turn rate).
// "Local" means relative to the robot's own heading, not the field.
Pose light::getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    return Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

// estimatePose() — predict where the robot will be `time` seconds from now,
// assuming it keeps its current local velocity.  Useful for leading a target
// (e.g., shooting a game piece at a moving goal) or for latency compensation.
Pose light::estimatePose(float time, bool radians) {
    Pose curPose    = getPose(true);
    Pose localSpeed = getLocalSpeed(true);
    Pose deltaLocal = localSpeed * time;

    float avgHeading = curPose.theta + deltaLocal.theta / 2.0f;
    Pose future = curPose;
    future.x +=  deltaLocal.y * sinf(avgHeading);
    future.y +=  deltaLocal.y * cosf(avgHeading);
    future.x += -deltaLocal.x * cosf(avgHeading);
    future.y +=  deltaLocal.x * sinf(avgHeading);
    if (!radians) future.theta = radToDeg(future.theta);
    return future;
}

// ─── Update ──────────────────────────────────────────────────────────────────
// This is the core odometry loop body — called every 10 ms by the tracking task.
//
// Step 1: Read all raw sensor positions
// Step 2: Compute deltas (change since last tick)
// Step 3: Determine heading change from the best available source
// Step 4: Pick which single vertical / horizontal wheel to use for displacement
// Step 5: Arc approximation → local displacement (localX, localY)
// Step 6: Rotate into global frame → update odomPose
// Step 7: Update smoothed speed estimates (EMA)
void light::update() {
    // ── Step 1: Read raw sensor positions ──
    float vertical1Raw   = odomSensors.vertical1   ? odomSensors.vertical1->getDistanceTraveled()   : 0;
    float vertical2Raw   = odomSensors.vertical2   ? odomSensors.vertical2->getDistanceTraveled()   : 0;
    float horizontal1Raw = odomSensors.horizontal1 ? odomSensors.horizontal1->getDistanceTraveled() : 0;
    float horizontal2Raw = odomSensors.horizontal2 ? odomSensors.horizontal2->getDistanceTraveled() : 0;
    float imuRaw = 0;
    if (odomSensors.imu != nullptr && odomSensors.imu2 != nullptr) {
    // Average both IMUs — uncorrelated drift partially cancels out,
    // giving a more stable heading than a single IMU.
        imuRaw = degToRad((odomSensors.imu->get_rotation() +
                       odomSensors.imu2->get_rotation()) / 2.0f);
}   else if (odomSensors.imu != nullptr) {
        imuRaw = degToRad(odomSensors.imu->get_rotation());
}

    // ── Step 2: Compute deltas ──
    float deltaV1  = vertical1Raw   - prevVertical1;
    float deltaV2  = vertical2Raw   - prevVertical2;
    float deltaH1  = horizontal1Raw - prevHorizontal1;
    float deltaH2  = horizontal2Raw - prevHorizontal2;
    float deltaImu = imuRaw         - prevImu;

    prevVertical1   = vertical1Raw;
    prevVertical2   = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    prevImu         = imuRaw;

    // ── Step 3: Determine heading change ──
    // Priority order (most accurate → least):
    //   1. Two horizontal tracking wheels (best — dedicated unpowered sensors)
    //   2. Two unpowered vertical wheels  (good — no motor backlash)
    //   3. IMU gyro                       (decent — drifts over time)
    //   4. Two powered vertical wheels    (fallback — motor encoders have slop)
    //
    // The formula  Δθ = (Δleft − Δright) / (offsetLeft − offsetRight)
    // is the arc-length relationship: two wheels at different offsets from the
    // center trace different arc lengths when the robot turns.
    float heading = odomPose.theta;

    bool h1ok        = (odomSensors.horizontal1 != nullptr);
    bool h2ok        = (odomSensors.horizontal2 != nullptr);
    bool v1unpowered = odomSensors.vertical1 && !odomSensors.vertical1->isPowered();
    bool v2unpowered = odomSensors.vertical2 && !odomSensors.vertical2->isPowered();

    if (h1ok && h2ok) {
        heading -= (deltaH1 - deltaH2) /
                   (odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset());
    } else if (v1unpowered && v2unpowered) {
        heading -= (deltaV1 - deltaV2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    } else if (odomSensors.imu != nullptr) {
        heading += deltaImu;
    } else if (odomSensors.vertical1 && odomSensors.vertical2) {
        heading -= (deltaV1 - deltaV2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    }

    float deltaHeading = heading - odomPose.theta;
    // Average heading during this tick — used to rotate local → global.
    // Using the midpoint (not the final heading) reduces integration error
    // when the robot is turning, because the displacement happened across
    // the full arc, not just at the endpoint.
    float avgHeading   = odomPose.theta + deltaHeading / 2.0f;

    // ── Step 4: Pick which wheel to use for displacement ──
    // Prefer unpowered (rotation sensor) wheels — they don't have motor
    // backlash or gear slop, so they give cleaner distance readings.
    TrackingWheel* vertWheel  = nullptr;
    TrackingWheel* horizWheel = nullptr;

    if (v1unpowered)                                          vertWheel = odomSensors.vertical1;
    else if (v2unpowered)                                     vertWheel = odomSensors.vertical2;
    else if (odomSensors.vertical1)                           vertWheel = odomSensors.vertical1;

    if (h1ok)      horizWheel = odomSensors.horizontal1;
    else if (h2ok) horizWheel = odomSensors.horizontal2;

    float rawVertical   = vertWheel  ? vertWheel->getDistanceTraveled()  : 0;
    float rawHorizontal = horizWheel ? horizWheel->getDistanceTraveled() : 0;
    float vertOffset    = vertWheel  ? vertWheel->getOffset()  : 0;
    float horizOffset   = horizWheel ? horizWheel->getOffset() : 0;

    float deltaX = rawHorizontal - prevHorizontal;
    float deltaY = rawVertical   - prevVertical;
    prevHorizontal = rawHorizontal;
    prevVertical   = rawVertical;

    // ── Step 5: Arc approximation → local displacement ──
    // When the robot turns, the tracking wheel traces an arc, not a straight
    // line.  The chord length of that arc is:
    //     chord = 2 sin(Δθ/2) × radius
    // where radius = (Δencoder / Δθ) + offset.
    // If Δθ ≈ 0 (driving straight), the formula would divide by zero,
    // so we just use the raw deltas — at tiny angles the chord ≈ arc length.
    float localX, localY;
    if (deltaHeading == 0.0f) {
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2.0f * sinf(deltaHeading / 2.0f) * (deltaX / deltaHeading + horizOffset);
        localY = 2.0f * sinf(deltaHeading / 2.0f) * (deltaY / deltaHeading + vertOffset);
    }

    // ── Step 6/7: EKF predict + measurement updates + LightCast predict ──
    // Arc math becomes the motion model; EKF handles mean + covariance
    // propagation and Kalman-weighted measurement fusion. LightCast runs its
    // (cheap) predict here synced to the same deltas; its (expensive) update
    // runs at 5 Hz in lightcast_task.cpp.
    const float dt = 0.01f;

    // Pose measurement derived from this tick's arc — used as the "wheel
    // pose" update when GPS + unpowered rotation sensors are both absent.
    // Compute it against the prior fused mean so we don't race with the EKF.
    Pose prior = light::ekf::mean();
    Pose wheelDerived;
    wheelDerived.x     = prior.x + localY * sinf(avgHeading) - localX * cosf(avgHeading);
    wheelDerived.y     = prior.y + localY * cosf(avgHeading) + localX * sinf(avgHeading);
    wheelDerived.theta = heading;

    light::ekf::predict(localX, localY, deltaHeading, dt);

    // IMU: small variance so it dominates heading when other sources don't beat it.
    if (odomSensors.imu != nullptr) {
        const float R_imu = 0.000076f;  // ~(0.5°)² in rad²
        light::ekf::updateHeadingIMU(imuRaw, R_imu);
    }

    // GPS: only fuse when sensor reports low error. get_error() ≈ meters;
    // variance scales roughly with error². Convert GPS meters → inches.
    if (odomSensors.gps != nullptr) {
        double err_m = odomSensors.gps->get_error();
        if (err_m > 0.0 && err_m < 0.5) {
            const float M_TO_IN = 39.3701f;
            float gx = static_cast<float>(odomSensors.gps->get_position_x()) * M_TO_IN;
            float gy = static_cast<float>(odomSensors.gps->get_position_y()) * M_TO_IN;
            float R_gps = static_cast<float>(err_m * err_m) * M_TO_IN * M_TO_IN;
            light::ekf::updateGPS(gx, gy, R_gps);
        }
    }

    // Powered-wheel-only fallback: only weight this in when there's nothing
    // better (i.e. no unpowered vert + no horizontal trackers). High R so
    // good sensors dominate when present.
    bool haveUnpowered = v1unpowered || v2unpowered || h1ok || h2ok;
    if (!haveUnpowered) {
        const float R_wheel = 4.0f;  // (2 in)² position, ignored for heading in this path
        light::ekf::updateWheelPose(wheelDerived, R_wheel);
    }

    light::lightcast::predict(localX, localY, deltaHeading);

    // Divergence snap: if EKF covariance blows up AND LightCast has a tight
    // cluster AND enough sensors participate, pull the EKF to the LightCast
    // best estimate. Rate-limited to prevent oscillation.
    static int snapCooldown = 0;
    if (snapCooldown > 0) --snapCooldown;
    // Read snap thresholds from ekf::config() so on-brain tuner edits take
    // effect without re-init.
    MCLConfig liveCfg = light::ekf::config();
    if (snapCooldown == 0 &&
        light::lightcast::sensorCount() >= 2 &&
        light::ekf::diverged(liveCfg.snapDiverge) &&
        light::lightcast::converged(liveCfg.snapConverge)) {
        light::ekf::reset(light::lightcast::best());
        snapCooldown = 50;  // 500 ms at 100 Hz
    }

    // Read fused state back out into the public pose/speed fields.
    odomPose  = light::ekf::mean();
    odomSpeed = light::ekf::velocity();

    // Local-frame speed = world speed rotated by -theta.
    float ct = cosf(odomPose.theta);
    float st = sinf(odomPose.theta);
    odomLocalSpeed.x     = odomSpeed.x * (-ct) + odomSpeed.y *  st;
    odomLocalSpeed.y     = odomSpeed.x *   st  + odomSpeed.y *  ct;
    odomLocalSpeed.theta = odomSpeed.theta;
}

// ─── Task management ─────────────────────────────────────────────────────────

// init() — call once in initialize() to start the odometry background task.
// Pass in your OdomSensors struct with all tracking wheels and IMU(s).
// The task runs update() every 10 ms (100 Hz) in the background.
void light::init(OdomSensors sensors, MCLConfig cfg) {
    odomSensors = sensors;
    odomCfg     = cfg;

    // Bring the fused pose estimators online before the tick task starts so
    // the first update() has valid state to update.
    light::ekf::init(odomPose, cfg);
    light::lightcast::init(odomPose, odomSensors.distanceSensors, cfg);
    if (!odomSensors.distanceSensors.empty()) {
        light::lightcast::startTask();
    }

    if (trackingTask == nullptr) {
        trackingTask = new pros::Task([] {
            while (true) {
                light::update();
                pros::delay(10);
            }
        });
    }

    // Route EZ-Template's odom_*_get / odom_*_set through the fused pose so
    // pid_odom_* motion closes the loop on the EKF/MCL estimate instead of the
    // wheel integrator. Both APIs use inches/degrees and CCW-from-+y, so no
    // unit/frame conversion is needed.
    ez::register_pose_source(
        []() -> ez::pose {
            Pose p = light::getPose();
            return ez::pose{p.x, p.y, p.theta};
        },
        [](ez::pose p) {
            light::setPose({(float)p.x, (float)p.y, (float)p.theta});
        });
}
// reset() — zero out the "previous" encoder snapshots so the next update()
// doesn't see a massive delta and teleport the robot.  Call this after
// setPose() if you've physically moved the robot or reset encoders.
void light::reset() {
    prevVertical1   = odomSensors.vertical1   ? odomSensors.vertical1->getDistanceTraveled()   : 0;
    prevVertical2   = odomSensors.vertical2   ? odomSensors.vertical2->getDistanceTraveled()   : 0;
    prevHorizontal1 = odomSensors.horizontal1 ? odomSensors.horizontal1->getDistanceTraveled() : 0;
    prevHorizontal2 = odomSensors.horizontal2 ? odomSensors.horizontal2->getDistanceTraveled() : 0;
    prevImu         = odomSensors.imu         ? degToRad(odomSensors.imu->get_rotation())      : 0;
    prevVertical    = 0;
    prevHorizontal  = 0;
}

// stop() — kill the background tracking task.  Call if you need to
// shut down odom (e.g., switching to a different control mode).
void light::stop() {
    if (trackingTask != nullptr) {
        trackingTask->remove();
        delete trackingTask;
        trackingTask = nullptr;
    }
}