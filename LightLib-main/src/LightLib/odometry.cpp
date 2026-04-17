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
#include <cmath>

// ─── Helpers ─────────────────────────────────────────────────────────────────

// Exponential Moving Average — smooths noisy speed readings.
// `smooth` near 1.0 = very smooth (slow to respond), near 0.0 = raw/noisy.
// We use 0.95 for speed estimates, which gives a ~200 ms effective window.
static float ema(float newVal, float oldVal, float smooth) {
    return smooth * oldVal + (1.0f - smooth) * newVal;
}
static float degToRad(float deg) { return deg * M_PI / 180.0f; }
static float radToDeg(float rad) { return rad * 180.0f / M_PI; }

// ─── State ───────────────────────────────────────────────────────────────────
// All odometry state is file-static — there's one global robot position.
// odomPose:       current (x, y, θ) in inches and radians
// odomSpeed:      global-frame velocity (smoothed via EMA), in inches/sec
// odomLocalSpeed: robot-frame velocity (forward/strafe/turn), in inches/sec
static OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr);

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

    // ── Step 6: Rotate local → global and update pose ──
    // localY (forward) maps to both global X and Y via sin/cos of heading.
    // localX (strafe) maps similarly but rotated 90°.
    Pose prevPose = odomPose;
    odomPose.x +=  localY * sinf(avgHeading);
    odomPose.y +=  localY * cosf(avgHeading);
    odomPose.x += -localX * cosf(avgHeading);
    odomPose.y +=  localX * sinf(avgHeading);
    odomPose.theta = heading;

    // ── Step 7: Update smoothed speed estimates ──
    // dt = 0.01 because update() runs every 10 ms.
    // EMA with α=0.95 smooths out encoder noise while still tracking
    // velocity changes within ~200 ms — good enough for motion prediction
    // (see estimatePose()) and feed-forward control.
    const float dt = 0.01f;
    odomSpeed.x     = ema((odomPose.x     - prevPose.x)     / dt, odomSpeed.x,     0.95f);
    odomSpeed.y     = ema((odomPose.y     - prevPose.y)     / dt, odomSpeed.y,     0.95f);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / dt, odomSpeed.theta, 0.95f);

    odomLocalSpeed.x     = ema(localX       / dt, odomLocalSpeed.x,     0.95f);
    odomLocalSpeed.y     = ema(localY       / dt, odomLocalSpeed.y,     0.95f);
    odomLocalSpeed.theta = ema(deltaHeading / dt, odomLocalSpeed.theta, 0.95f);
}

// ─── Task management ─────────────────────────────────────────────────────────

// init() — call once in initialize() to start the odometry background task.
// Pass in your OdomSensors struct with all tracking wheels and IMU(s).
// The task runs update() every 10 ms (100 Hz) in the background.
void light::init(OdomSensors sensors) {
    odomSensors = sensors;
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task([] {
            while (true) {
                light::update();
                pros::delay(10);
            }
        });
    }
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