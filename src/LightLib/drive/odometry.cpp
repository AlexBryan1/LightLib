// Convention: +Y forward, +X right, θ = 0 facing +Y, CW-positive (radians).

#include <cmath>

#include "LightLib/drive/drive.hpp"
#include "LightLib/drive/ekf.hpp"
#include "LightLib/drive/lightcast.hpp"
#include "LightLib/drive/odometry.hpp"
#include "LightLib/drive/sensor_aux.hpp"
#include "LightLib/util/extras.hpp"
#include "LightLib/util/util.hpp"
#include "pros/rtos.hpp"

static float degToRad(float deg) { return deg * M_PI / 180.0f; }
static float radToDeg(float rad) { return rad * 180.0f / M_PI; }

using ::light::util::wrap_rad;

static OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr);
static MCLConfig odomCfg;

static Pose odomPose;
static Pose odomSpeed;
static Pose odomLocalSpeed;

static pros::Mutex pose_mtx_;

// Per-sensor (1/2) used for heading calc; per-chosen-wheel for displacement.
static float prevVertical = 0;
static float prevVertical1 = 0;
static float prevVertical2 = 0;
static float prevHorizontal = 0;
static float prevHorizontal1 = 0;
static float prevHorizontal2 = 0;
static float prevImu = 0;

static pros::Task* trackingTask = nullptr;

Pose light::getPose(bool radians) {
  pose_mtx_.take(TIMEOUT_MAX);
  Pose p = odomPose;
  pose_mtx_.give();
  if (radians) return p;
  return Pose(p.x, p.y, radToDeg(p.theta));
}

void light::setPose(Pose pose, bool radians) {
  Pose newPose = radians ? pose : Pose(pose.x, pose.y, degToRad(pose.theta));
  pose_mtx_.take(TIMEOUT_MAX);
  odomPose = newPose;
  pose_mtx_.give();
  // Propagate so the next update() doesn't pull pose back to the stale EKF mean.
  light::ekf::reset(newPose);
  light::lightcast::init(newPose, odomSensors.distanceSensors, light::lightcast::config());
}

Pose light::getSpeed(bool radians) {
  pose_mtx_.take(TIMEOUT_MAX);
  Pose s = odomSpeed;
  pose_mtx_.give();
  if (radians) return s;
  return Pose(s.x, s.y, radToDeg(s.theta));
}

Pose light::getLocalSpeed(bool radians) {
  pose_mtx_.take(TIMEOUT_MAX);
  Pose s = odomLocalSpeed;
  pose_mtx_.give();
  if (radians) return s;
  return Pose(s.x, s.y, radToDeg(s.theta));
}

Pose light::estimatePose(float time, bool radians) {
  Pose curPose = getPose(true);
  Pose localSpeed = getLocalSpeed(true);
  Pose deltaLocal = localSpeed * time;

  float avgHeading = curPose.theta + deltaLocal.theta / 2.0f;
  Pose future = curPose;
  future.x += deltaLocal.y * sinf(avgHeading);
  future.y += deltaLocal.y * cosf(avgHeading);
  future.x += -deltaLocal.x * cosf(avgHeading);
  future.y += deltaLocal.x * sinf(avgHeading);
  if (!radians) future.theta = radToDeg(future.theta);
  return future;
}

// IMU yaw-scale correction (set via Drive::drive_imu_scaler_set). A factor > 1
// compensates a tilted mount whose spin axis is off vertical and therefore
// under-counts real yaw. Reading it from the chassis keeps a single knob shared
// with the PID path (drive_imu_get) so heading is consistent across the stack.
static float imuScaler() {
  light::Drive* c = light::getChassis();
  return c ? (float)c->drive_imu_scaler_get() : 1.0f;
}

// Scaled, dual-IMU-averaged heading in radians. Used by both update() and
// reset() so their baselines can't drift apart (a mismatch would teleport the
// first post-reset delta). Averaging both IMUs lets uncorrelated drift cancel.
static float readOdomImuRad() {
  const float scaler = imuScaler();
  if (odomSensors.imu != nullptr && odomSensors.imu2 != nullptr) {
    return degToRad((odomSensors.imu->get_rotation() +
                     odomSensors.imu2->get_rotation()) /
                    2.0f * scaler);
  } else if (odomSensors.imu != nullptr) {
    return degToRad(odomSensors.imu->get_rotation() * scaler);
  }
  return 0.0f;
}

void light::update() {
  float vertical1Raw = odomSensors.vertical1 ? odomSensors.vertical1->getDistanceTraveled() : 0;
  float vertical2Raw = odomSensors.vertical2 ? odomSensors.vertical2->getDistanceTraveled() : 0;
  float horizontal1Raw = odomSensors.horizontal1 ? odomSensors.horizontal1->getDistanceTraveled() : 0;
  float horizontal2Raw = odomSensors.horizontal2 ? odomSensors.horizontal2->getDistanceTraveled() : 0;
  float imuRaw = readOdomImuRad();

  float deltaV1 = vertical1Raw - prevVertical1;
  float deltaV2 = vertical2Raw - prevVertical2;
  float deltaH1 = horizontal1Raw - prevHorizontal1;
  float deltaH2 = horizontal2Raw - prevHorizontal2;
  float deltaImu = imuRaw - prevImu;
  // ZUPT-estimated gyro bias. Zero before first stationary sample.
  deltaImu -= light::sensor_aux::imuBiasRadPerSec() * 0.01f;

  prevVertical1 = vertical1Raw;
  prevVertical2 = vertical2Raw;
  prevHorizontal1 = horizontal1Raw;
  prevHorizontal2 = horizontal2Raw;
  prevImu = imuRaw;

  // Heading-source priority: two-horiz > two-unpowered-vert > IMU > two-powered-vert.
  // Arc relation: Δθ = (Δleft − Δright) / (offsetLeft − offsetRight).
  float heading = odomPose.theta;

  bool h1ok = (odomSensors.horizontal1 != nullptr);
  bool h2ok = (odomSensors.horizontal2 != nullptr);
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

  float deltaHeading = wrap_rad(heading - odomPose.theta);
  // Midpoint heading for arc integration — endpoint would understate the rotation.
  float avgHeading = odomPose.theta + deltaHeading / 2.0f;

  // Prefer unpowered (rotation sensor) wheels — no motor backlash.
  TrackingWheel* vertWheel = nullptr;
  TrackingWheel* horizWheel = nullptr;

  if (v1unpowered)
    vertWheel = odomSensors.vertical1;
  else if (v2unpowered)
    vertWheel = odomSensors.vertical2;
  else if (odomSensors.vertical1)
    vertWheel = odomSensors.vertical1;

  if (h1ok)
    horizWheel = odomSensors.horizontal1;
  else if (h2ok)
    horizWheel = odomSensors.horizontal2;

  float rawVertical = vertWheel ? vertWheel->getDistanceTraveled() : 0;
  float rawHorizontal = horizWheel ? horizWheel->getDistanceTraveled() : 0;
  float vertOffset = vertWheel ? vertWheel->getOffset() : 0;
  float horizOffset = horizWheel ? horizWheel->getOffset() : 0;

  float deltaX = rawHorizontal - prevHorizontal;
  float deltaY = rawVertical - prevVertical;
  prevHorizontal = rawHorizontal;
  prevVertical = rawVertical;

  // Arc-chord approximation: chord = 2 sin(Δθ/2) × (Δenc/Δθ + offset).
  // Falls back to raw deltas for Δθ ≈ 0 to avoid divide-by-zero.
  float localX, localY;
  if (deltaHeading == 0.0f) {
    localX = deltaX;
    localY = deltaY;
  } else {
    localX = 2.0f * sinf(deltaHeading / 2.0f) * (deltaX / deltaHeading + horizOffset);
    localY = 2.0f * sinf(deltaHeading / 2.0f) * (deltaY / deltaHeading + vertOffset);
  }

  // Single-wheel rotation correction: with only one wheel on an axis the
  // chord formula's offset term can't compensate for cross-axis rotation;
  // subtract dθ × offset as linear approximation.
  if (light::sensor_aux::singleVertMode()) {
    localY -= deltaHeading * light::sensor_aux::singleVertOffset();
  }
  if (light::sensor_aux::singleHorizMode()) {
    localX -= deltaHeading * light::sensor_aux::singleHorizOffset();
  }

  // Slip detected: wheel translation untrustworthy this tick. EKF still
  // advances heading via the IMU update path.
  if (light::sensor_aux::slipping()) {
    localX = 0.0f;
    localY = 0.0f;
  }

  const float dt = 0.01f;

  // Wheel-derived pose against the prior fused mean (avoids racing the EKF).
  Pose prior = light::ekf::mean();
  Pose wheelDerived;
  wheelDerived.x = prior.x + localY * sinf(avgHeading) - localX * cosf(avgHeading);
  wheelDerived.y = prior.y + localY * cosf(avgHeading) + localX * sinf(avgHeading);
  wheelDerived.theta = heading;

  light::ekf::predict(localX, localY, deltaHeading, dt);

  if (odomSensors.imu != nullptr) {
    const float R_imu = 0.000076f;  // ~(0.5°)² in rad²
    light::ekf::updateHeadingIMU(imuRaw, R_imu);
  }

  // GPS variance scales with reported error²; meters → inches.
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

  // Powered-wheel-only fallback when no unpowered sensors are present.
  bool haveUnpowered = v1unpowered || v2unpowered || h1ok || h2ok;
  if (!haveUnpowered) {
    const float R_wheel = 4.0f;  // (2 in)² position
    light::ekf::updateWheelPose(wheelDerived, R_wheel);
  }

  light::lightcast::predict(localX, localY, deltaHeading);

  // Snap EKF to LightCast best when EKF diverges and LightCast has a tight
  // multi-sensor cluster; rate-limited against oscillation.
  static int snapCooldown = 0;
  if (snapCooldown > 0) --snapCooldown;
  // Read live config so on-brain tuner edits take effect without re-init.
  MCLConfig liveCfg = light::ekf::config();
  if (snapCooldown == 0 &&
      light::lightcast::sensorCount() >= 2 &&
      light::ekf::diverged(liveCfg.snapDiverge) &&
      light::lightcast::converged(liveCfg.snapConverge)) {
    light::ekf::reset(light::lightcast::best());
    snapCooldown = 50;  // 500 ms at 100 Hz
  }

  Pose newPose = light::ekf::mean();
  Pose rawSpeed = light::ekf::velocity();

  constexpr float SPEED_EMA_ALPHA = 0.3f;
  static bool speed_seeded = false;
  static Pose smoothedSpeed;
  if (!speed_seeded) {
    smoothedSpeed = rawSpeed;
    speed_seeded = true;
  } else {
    smoothedSpeed.x = SPEED_EMA_ALPHA * rawSpeed.x + (1.0f - SPEED_EMA_ALPHA) * smoothedSpeed.x;
    smoothedSpeed.y = SPEED_EMA_ALPHA * rawSpeed.y + (1.0f - SPEED_EMA_ALPHA) * smoothedSpeed.y;
    smoothedSpeed.theta = SPEED_EMA_ALPHA * rawSpeed.theta + (1.0f - SPEED_EMA_ALPHA) * smoothedSpeed.theta;
  }

  float ct = cosf(newPose.theta);
  float st = sinf(newPose.theta);
  Pose newLocalSpeed;
  newLocalSpeed.x = smoothedSpeed.x * (-ct) + smoothedSpeed.y * st;
  newLocalSpeed.y = smoothedSpeed.x * st + smoothedSpeed.y * ct;
  newLocalSpeed.theta = smoothedSpeed.theta;

  pose_mtx_.take(TIMEOUT_MAX);
  odomPose = newPose;
  odomSpeed = smoothedSpeed;
  odomLocalSpeed = newLocalSpeed;
  pose_mtx_.give();

  // Wall-snap + bias/slip detectors run after the fresh pose is published.
  light::sensor_aux::tick();
}

void light::init(OdomSensors sensors, MCLConfig cfg) {
  odomSensors = sensors;
  odomCfg = cfg;

  // Estimators must be live before the first tick.
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

  // Route EZ-Template's odom_*_get/set through the fused pose so pid_odom_*
  // closes on the EKF/MCL estimate, not the wheel integrator. Same units/frame.
  light::register_pose_source(
      []() -> light::pose {
        Pose p = light::getPose();
        return light::pose{p.x, p.y, p.theta};
      },
      [](light::pose p) {
        light::setPose({(float)p.x, (float)p.y, (float)p.theta});
      });
}

// Re-snapshot previous encoder/IMU values so the next update() doesn't
// integrate a teleport-sized delta. Call after setPose() or sensor reset.
void light::reset() {
  prevVertical1 = odomSensors.vertical1 ? odomSensors.vertical1->getDistanceTraveled() : 0;
  prevVertical2 = odomSensors.vertical2 ? odomSensors.vertical2->getDistanceTraveled() : 0;
  prevHorizontal1 = odomSensors.horizontal1 ? odomSensors.horizontal1->getDistanceTraveled() : 0;
  prevHorizontal2 = odomSensors.horizontal2 ? odomSensors.horizontal2->getDistanceTraveled() : 0;
  prevImu = readOdomImuRad();
  prevVertical = 0;
  prevHorizontal = 0;
}

void light::stop() {
  if (trackingTask != nullptr) {
    trackingTask->remove();
    delete trackingTask;
    trackingTask = nullptr;
  }
}