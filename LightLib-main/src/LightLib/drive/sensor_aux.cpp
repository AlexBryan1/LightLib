#include "LightLib/drive/sensor_aux.hpp"

#include <cmath>
#include <mutex>

#include "LightLib/path/field_map.hpp"
#include "LightLib/util/util.hpp"
#include "pros/rtos.hpp"

// Tunable defaults — overridable by the user defining them before
// #including robot_impl.inl. Match the silent-defaults style of the
// MCL_* tuning block.
#ifndef AUX_ZUPT_DWELL_MS
#define AUX_ZUPT_DWELL_MS         300
#endif
#ifndef AUX_ZUPT_VEL_THRESH
#define AUX_ZUPT_VEL_THRESH       0.5f
#endif
#ifndef AUX_ZUPT_OMEGA_THRESH
#define AUX_ZUPT_OMEGA_THRESH     0.0175f
#endif
#ifndef AUX_ZUPT_SAMPLE_MS
#define AUX_ZUPT_SAMPLE_MS        200
#endif
#ifndef AUX_SLIP_DEG_PER_100MS
#define AUX_SLIP_DEG_PER_100MS    5.0f
#endif
#ifndef AUX_SLIP_CLEAR_DEG
#define AUX_SLIP_CLEAR_DEG        1.0f
#endif
#ifndef AUX_WALLSNAP_HEADING_TOL
#define AUX_WALLSNAP_HEADING_TOL  0.0873f
#endif
#ifndef AUX_WALLSNAP_DIST_MIN_IN
#define AUX_WALLSNAP_DIST_MIN_IN  3.0f
#endif
#ifndef AUX_WALLSNAP_DIST_MAX_IN
#define AUX_WALLSNAP_DIST_MAX_IN  60.0f
#endif
#ifndef AUX_WALLSNAP_RESID_TOL_IN
#define AUX_WALLSNAP_RESID_TOL_IN 1.5f
#endif

namespace light::sensor_aux {
namespace {

using ::light::util::wrap_rad;

OdomSensors odomSensors_(nullptr, nullptr, nullptr, nullptr, nullptr);

// Algorithm A — IMU bias (ZUPT)
float imuBias_ = 0.0f;
uint32_t stationarySinceMs_ = 0;
bool inSampleWindow_ = false;
uint32_t sampleStartMs_ = 0;
float sampleSum_ = 0.0f;
int sampleCount_ = 0;
float prevImuRad_ = 0.0f;
uint32_t prevTickMs_ = 0;

// Algorithm B — single-wheel mode
bool singleVert_ = false;
float singleVertOff_ = 0.0f;
bool singleHoriz_ = false;
float singleHorizOff_ = 0.0f;

// Algorithm C — slip detection
bool slipFlag_ = false;
float slipAccumDeg_ = 0.0f;
float prevImuForSlip_ = 0.0f;
float prevV1_ = 0.0f;
float prevV2_ = 0.0f;

// Algorithm D — wall-snap diagnostic
int wallSnapCount_ = 0;

pros::Mutex mtx_;

float radToDeg(float r) { return r * 180.0f / M_PI; }
float degToRad(float d) { return d * M_PI / 180.0f; }

// Read the (averaged) IMU heading in radians, matching the convention
// in odometry.cpp:193-201.
float readImuRad() {
  if (odomSensors_.imu && odomSensors_.imu2) {
    return degToRad((odomSensors_.imu->get_rotation() +
                     odomSensors_.imu2->get_rotation()) /
                    2.0f);
  }
  if (odomSensors_.imu) {
    return degToRad(odomSensors_.imu->get_rotation());
  }
  return 0.0f;
}

// Snap one axis of the pose without touching the others.
void snapAxis(char axis, float value) {
  Pose cur = light::getPose(true);  // radians, so we can write back without conversion
  if (axis == 'x') cur.x = value;
  else if (axis == 'y') cur.y = value;
  light::setPose(cur, true);
  ++wallSnapCount_;
}

// Try the wall-snap on a single distance sensor. Returns true if it fired.
// Angle convention follows odometry.cpp: theta=0 faces +Y, CCW positive,
// so the unit ray vector is (sin(worldAng), cos(worldAng)).
bool tryWallSnap(const DistanceSensorSpec& spec, const Pose& curPose) {
  if (!spec.sensor) return false;
  int mm = spec.sensor->get();
  if (mm <= 0 || mm > 9000) return false;
  float measuredIn = mm / 25.4f;
  if (measuredIn < AUX_WALLSNAP_DIST_MIN_IN) return false;
  if (measuredIn > AUX_WALLSNAP_DIST_MAX_IN) return false;

  float worldAng = wrap_rad(curPose.theta + spec.angleRad);
  float rayX = std::sin(worldAng);   // X component of unit ray vector
  float rayY = std::cos(worldAng);   // Y component
  float minCos = std::cos(AUX_WALLSNAP_HEADING_TOL);

  // Sensor world position. Mirror the chord-formula rotation used in
  // lightcast.cpp:166-167:
  //   sx = px + offY * sin(theta) - offX * cos(theta)
  //   sy = py + offY * cos(theta) + offX * sin(theta)
  float ct = std::cos(curPose.theta);
  float st = std::sin(curPose.theta);
  float sensorWorldX = curPose.x + spec.offsetY * st - spec.offsetX * ct;
  float sensorWorldY = curPose.y + spec.offsetY * ct + spec.offsetX * st;

  float expected = light::field::raycast(sensorWorldX, sensorWorldY, worldAng,
                                         AUX_WALLSNAP_DIST_MAX_IN + 24.0f);
  if (expected <= 0.0f) return false;
  if (std::fabs(measuredIn - expected) > AUX_WALLSNAP_RESID_TOL_IN) return false;

  // Determine which wall the ray hits and snap the matching axis.
  // For a ray pointing +Y (rayY ≈ +1), the hit point's Y =
  //   sensorWorldY + measuredIn * rayY = sensorWorldY + measuredIn.
  // That equals wallY = +FIELD_HALF, so:
  //   curPose.y = wallY - measuredIn - (sensorWorldY - curPose.y)
  //             = wallY - measuredIn - (offY*ct + offX*st).
  if (rayY > minCos) {
    float wallY = light::field::FIELD_HALF;
    float newY = wallY - measuredIn - (spec.offsetY * ct + spec.offsetX * st);
    snapAxis('y', newY);
    return true;
  }
  if (rayY < -minCos) {
    float wallY = -light::field::FIELD_HALF;
    float newY = wallY + measuredIn - (spec.offsetY * ct + spec.offsetX * st);
    snapAxis('y', newY);
    return true;
  }
  if (rayX > minCos) {
    float wallX = light::field::FIELD_HALF;
    float newX = wallX - measuredIn - (spec.offsetY * st - spec.offsetX * ct);
    snapAxis('x', newX);
    return true;
  }
  if (rayX < -minCos) {
    float wallX = -light::field::FIELD_HALF;
    float newX = wallX + measuredIn - (spec.offsetY * st - spec.offsetX * ct);
    snapAxis('x', newX);
    return true;
  }
  return false;
}

}  // namespace

void init(const OdomSensors& sensors) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  odomSensors_ = sensors;

  // Algorithm B — detect single-wheel modes. For vertical, only count
  // unpowered (rotation-sensor) wheels — drivetrain fallbacks aren't
  // mounted at offset.
  bool v1up = sensors.vertical1 && !sensors.vertical1->isPowered();
  bool v2up = sensors.vertical2 && !sensors.vertical2->isPowered();
  if (v1up && !v2up) {
    singleVert_ = true;
    singleVertOff_ = sensors.vertical1->getOffset();
  } else if (!v1up && v2up) {
    singleVert_ = true;
    singleVertOff_ = sensors.vertical2->getOffset();
  } else {
    singleVert_ = false;
    singleVertOff_ = 0.0f;
  }

  bool h1 = sensors.horizontal1 != nullptr;
  bool h2 = sensors.horizontal2 != nullptr;
  if (h1 && !h2) {
    singleHoriz_ = true;
    singleHorizOff_ = sensors.horizontal1->getOffset();
  } else if (!h1 && h2) {
    singleHoriz_ = true;
    singleHorizOff_ = sensors.horizontal2->getOffset();
  } else {
    singleHoriz_ = false;
    singleHorizOff_ = 0.0f;
  }

  // Reset state
  imuBias_ = 0.0f;
  stationarySinceMs_ = 0;
  inSampleWindow_ = false;
  sampleSum_ = 0.0f;
  sampleCount_ = 0;
  prevImuRad_ = readImuRad();
  prevImuForSlip_ = prevImuRad_;
  prevV1_ = sensors.vertical1 ? sensors.vertical1->getDistanceTraveled() : 0.0f;
  prevV2_ = sensors.vertical2 ? sensors.vertical2->getDistanceTraveled() : 0.0f;
  prevTickMs_ = pros::millis();
  slipFlag_ = false;
  slipAccumDeg_ = 0.0f;
  wallSnapCount_ = 0;
}

void tick() {
  std::lock_guard<pros::Mutex> lock(mtx_);

  uint32_t nowMs = pros::millis();
  uint32_t dtMs = nowMs - prevTickMs_;
  if (dtMs == 0) dtMs = 10;  // first call after init
  prevTickMs_ = nowMs;
  float dt = dtMs / 1000.0f;

  float imuRad = readImuRad();
  float imuDelta = wrap_rad(imuRad - prevImuRad_);
  prevImuRad_ = imuRad;

  // ── Algorithm A: ZUPT IMU bias estimation ────────────────────────────
  if (odomSensors_.imu) {
    Pose ls = light::getLocalSpeed(true);  // forward / strafe / omega in rad/s
    float linSpeed = std::fabs(ls.x) + std::fabs(ls.y);
    bool stationary = (linSpeed < AUX_ZUPT_VEL_THRESH) &&
                      (std::fabs(ls.theta) < AUX_ZUPT_OMEGA_THRESH);

    if (!stationary) {
      stationarySinceMs_ = 0;
      inSampleWindow_ = false;
    } else {
      if (stationarySinceMs_ == 0) stationarySinceMs_ = nowMs;
      uint32_t stillFor = nowMs - stationarySinceMs_;
      if (!inSampleWindow_ && stillFor >= AUX_ZUPT_DWELL_MS) {
        inSampleWindow_ = true;
        sampleStartMs_ = nowMs;
        sampleSum_ = 0.0f;
        sampleCount_ = 0;
      }
      if (inSampleWindow_) {
        // Accumulate raw gyro rate (uncorrected — we want to measure
        // the bias, not the corrected reading). dt-based rate matches
        // what odometry.cpp will subtract per tick.
        if (dt > 1e-6f) {
          sampleSum_ += imuDelta / dt;
          ++sampleCount_;
        }
        if (nowMs - sampleStartMs_ >= AUX_ZUPT_SAMPLE_MS && sampleCount_ > 0) {
          // Folded into the running bias as a 50/50 blend so a single
          // bad sample (e.g. someone bumped the robot mid-sample) can't
          // permanently corrupt the estimate.
          float fresh = sampleSum_ / sampleCount_;
          imuBias_ = 0.5f * imuBias_ + 0.5f * fresh;
          inSampleWindow_ = false;
        }
      }
    }
  }

  // ── Algorithm C: wheel-IMU slip detection ────────────────────────────
  // Eligible only when both vertical slots are present (need both for
  // arc-derived dtheta) AND at least one is powered (drivetrain). A
  // pure unpowered-tracking-wheel config doesn't slip in the same way.
  bool eligibleSlip = odomSensors_.imu && odomSensors_.vertical1 && odomSensors_.vertical2 &&
                      (odomSensors_.vertical1->isPowered() ||
                       odomSensors_.vertical2->isPowered());
  if (eligibleSlip) {
    float v1 = odomSensors_.vertical1->getDistanceTraveled();
    float v2 = odomSensors_.vertical2->getDistanceTraveled();
    float dV1 = v1 - prevV1_;
    float dV2 = v2 - prevV2_;
    prevV1_ = v1;
    prevV2_ = v2;

    float off1 = odomSensors_.vertical1->getOffset();
    float off2 = odomSensors_.vertical2->getOffset();
    float denom = off1 - off2;
    if (std::fabs(denom) > 1e-3f) {
      // Sign convention matches odometry.cpp:241-243: heading -= (dV1-dV2)/(off1-off2).
      float wheelDtheta = -(dV1 - dV2) / denom;
      float imuDthetaForSlip = wrap_rad(imuRad - prevImuForSlip_);
      prevImuForSlip_ = imuRad;

      float disagreementDeg = std::fabs(radToDeg(wheelDtheta - imuDthetaForSlip));
      // Decay the rolling sum at ~10%/tick (≈100 ms half-life at 100 Hz).
      slipAccumDeg_ = 0.9f * slipAccumDeg_ + disagreementDeg;
      if (!slipFlag_ && slipAccumDeg_ > AUX_SLIP_DEG_PER_100MS) slipFlag_ = true;
      else if (slipFlag_ && slipAccumDeg_ < AUX_SLIP_CLEAR_DEG) slipFlag_ = false;
    }
  } else {
    slipFlag_ = false;
    slipAccumDeg_ = 0.0f;
    if (odomSensors_.vertical1) prevV1_ = odomSensors_.vertical1->getDistanceTraveled();
    if (odomSensors_.vertical2) prevV2_ = odomSensors_.vertical2->getDistanceTraveled();
    prevImuForSlip_ = imuRad;
  }

  // ── Algorithm D: single-axis wall-snap ───────────────────────────────
  if (!odomSensors_.distanceSensors.empty()) {
    Pose cur = light::getPose(true);
    for (const auto& spec : odomSensors_.distanceSensors) {
      if (tryWallSnap(spec, cur)) {
        // Refresh `cur` after a snap — subsequent sensors should test
        // against the new pose to avoid double-correcting.
        cur = light::getPose(true);
      }
    }
  }
}

float imuBiasRadPerSec() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return imuBias_;
}

bool singleVertMode() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return singleVert_;
}
float singleVertOffset() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return singleVertOff_;
}
bool singleHorizMode() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return singleHoriz_;
}
float singleHorizOffset() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return singleHorizOff_;
}
bool slipping() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return slipFlag_;
}
int wallSnapsThisAuton() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return wallSnapCount_;
}
void resetCounters() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  wallSnapCount_ = 0;
}

}  // namespace light::sensor_aux
