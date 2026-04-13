#include "LightLib/odom.hpp"
#include <cmath>

// ─── Helpers ─────────────────────────────────────────────────────────────────
static float ema(float newVal, float oldVal, float smooth) {
    return smooth * oldVal + (1.0f - smooth) * newVal;
}
static float degToRad(float deg) { return deg * M_PI / 180.0f; }
static float radToDeg(float rad) { return rad * 180.0f / M_PI; }

// ─── State ───────────────────────────────────────────────────────────────────
static OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr);

static Pose odomPose;
static Pose odomSpeed;
static Pose odomLocalSpeed;

static float prevVertical    = 0;
static float prevVertical1   = 0;
static float prevVertical2   = 0;
static float prevHorizontal  = 0;
static float prevHorizontal1 = 0;
static float prevHorizontal2 = 0;
static float prevImu         = 0;

static pros::Task* trackingTask = nullptr;

// ─── Public API ───────────────────────────────────────────────────────────────
Pose light::getPose(bool radians) {
    if (radians) return odomPose;
    return Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

void light::setPose(Pose pose, bool radians) {
    odomPose = radians ? pose : Pose(pose.x, pose.y, degToRad(pose.theta));
}

Pose light::getSpeed(bool radians) {
    if (radians) return odomSpeed;
    return Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

Pose light::getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    return Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

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
void light::update() {
    float vertical1Raw   = odomSensors.vertical1   ? odomSensors.vertical1->getDistanceTraveled()   : 0;
    float vertical2Raw   = odomSensors.vertical2   ? odomSensors.vertical2->getDistanceTraveled()   : 0;
    float horizontal1Raw = odomSensors.horizontal1 ? odomSensors.horizontal1->getDistanceTraveled() : 0;
    float horizontal2Raw = odomSensors.horizontal2 ? odomSensors.horizontal2->getDistanceTraveled() : 0;
    float imuRaw = 0;
    if (odomSensors.imu != nullptr && odomSensors.imu2 != nullptr) {
    // average both IMUs — uncorrelated drift partially cancels out
        imuRaw = degToRad((odomSensors.imu->get_rotation() +
                       odomSensors.imu2->get_rotation()) / 2.0f);
}   else if (odomSensors.imu != nullptr) {
        imuRaw = degToRad(odomSensors.imu->get_rotation());
}

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
    float avgHeading   = odomPose.theta + deltaHeading / 2.0f;

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

    float localX, localY;
    if (deltaHeading == 0.0f) {
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2.0f * sinf(deltaHeading / 2.0f) * (deltaX / deltaHeading + horizOffset);
        localY = 2.0f * sinf(deltaHeading / 2.0f) * (deltaY / deltaHeading + vertOffset);
    }

    Pose prevPose = odomPose;
    odomPose.x +=  localY * sinf(avgHeading);
    odomPose.y +=  localY * cosf(avgHeading);
    odomPose.x += -localX * cosf(avgHeading);
    odomPose.y +=  localX * sinf(avgHeading);
    odomPose.theta = heading;

    const float dt = 0.01f;
    odomSpeed.x     = ema((odomPose.x     - prevPose.x)     / dt, odomSpeed.x,     0.95f);
    odomSpeed.y     = ema((odomPose.y     - prevPose.y)     / dt, odomSpeed.y,     0.95f);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / dt, odomSpeed.theta, 0.95f);

    odomLocalSpeed.x     = ema(localX       / dt, odomLocalSpeed.x,     0.95f);
    odomLocalSpeed.y     = ema(localY       / dt, odomLocalSpeed.y,     0.95f);
    odomLocalSpeed.theta = ema(deltaHeading / dt, odomLocalSpeed.theta, 0.95f);
}

// ─── Task management ─────────────────────────────────────────────────────────
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
void light::reset() {
    // Re-read sensors so next update delta is 0 instead of a huge jump
    prevVertical1   = odomSensors.vertical1   ? odomSensors.vertical1->getDistanceTraveled()   : 0;
    prevVertical2   = odomSensors.vertical2   ? odomSensors.vertical2->getDistanceTraveled()   : 0;
    prevHorizontal1 = odomSensors.horizontal1 ? odomSensors.horizontal1->getDistanceTraveled() : 0;
    prevHorizontal2 = odomSensors.horizontal2 ? odomSensors.horizontal2->getDistanceTraveled() : 0;
    prevImu         = odomSensors.imu         ? degToRad(odomSensors.imu->get_rotation())      : 0;
    prevVertical    = 0;
    prevHorizontal  = 0;
}

void light::stop() {
    if (trackingTask != nullptr) {
        trackingTask->remove();
        delete trackingTask;
        trackingTask = nullptr;
    }
}