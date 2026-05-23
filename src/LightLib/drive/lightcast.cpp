#include "LightLib/drive/lightcast.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>

#include "LightLib/drive/field_map.hpp"
#include "LightLib/util/util.hpp"
#include "pros/rtos.hpp"

// Mutex protects the particle array so 100 Hz predict() (odom task) and
// 10 Hz update() (lightcast task) can interleave safely.

namespace light::lightcast {
namespace {

constexpr float kWeightEpsilon = 1e-12f;

struct Particle {
  float x, y, theta, w;
};

MCLConfig cfg_;
std::vector<Particle> particles_;
std::vector<DistanceSensorSpec> sensors_;
pros::Mutex mtx_;
std::mt19937 rng_{0xC0FFEE};
uint32_t degenerateCount_ = 0;

using ::light::util::wrap_rad;

float gaussian(float sigma) {
  static std::normal_distribution<float> d(0.0f, 1.0f);
  return sigma * d(rng_);
}

// Low-variance resampling preserves diversity better than random when a few
// heavy particles dominate the weight.
void resample() {
  int n = (int)particles_.size();
  std::vector<Particle> out(n);
  float totalW = 0.0f;
  for (int i = 0; i < n; ++i) totalW += particles_[i].w;
  if (totalW < kWeightEpsilon) {
    for (int i = 0; i < n; ++i) particles_[i].w = 1.0f / n;
    return;
  }
  std::uniform_real_distribution<float> u(0.0f, totalW / n);
  float r = u(rng_);
  float c = particles_[0].w;
  int i = 0;
  for (int m = 0; m < n; ++m) {
    float U = r + m * (totalW / n);
    while (U > c && i < n - 1) {
      ++i;
      c += particles_[i].w;
    }
    out[m] = particles_[i];
    out[m].w = 1.0f / n;
  }
  std::swap(particles_, out);
}

}  // namespace

void init(const Pose& initial, const std::vector<DistanceSensorSpec>& sensors, MCLConfig cfg) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  cfg_ = cfg;
  sensors_ = sensors;
  particles_.resize(cfg_.numParticles);
  for (int i = 0; i < cfg_.numParticles; ++i) {
    particles_[i].x = initial.x + gaussian(cfg_.initPosSigmaIn);
    particles_[i].y = initial.y + gaussian(cfg_.initPosSigmaIn);
    particles_[i].theta = wrap_rad(initial.theta + gaussian(cfg_.initHeadingSigmaRad));
    particles_[i].w = 1.0f / cfg_.numParticles;
  }
}

void predict(float dLocalX, float dLocalY, float dTheta) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  // Noise grows with motion: 0.05 in/in slip + 0.02 in static jitter,
  // 0.1 rad/rad heading drift + 0.002 rad static IMU noise.
  float motionMag = std::sqrt(dLocalX * dLocalX + dLocalY * dLocalY);
  float posNoise = 0.02f + 0.05f * motionMag;
  float angNoise = 0.002f + 0.1f * std::fabs(dTheta);

  for (int i = 0; i < (int)particles_.size(); ++i) {
    Particle& p = particles_[i];
    float thetaMid = p.theta + dTheta * 0.5f;
    float s = std::sin(thetaMid);
    float c = std::cos(thetaMid);
    float nx = dLocalX + gaussian(posNoise);
    float ny = dLocalY + gaussian(posNoise);
    p.x += ny * s - nx * c;
    p.y += ny * c + nx * s;
    p.theta = wrap_rad(p.theta + dTheta + gaussian(angNoise));
  }
}

void update() {
  if (sensors_.empty()) return;

  std::lock_guard<pros::Mutex> lock(mtx_);

  static std::vector<float> measured;
  measured.assign(sensors_.size(), -1.0f);
  for (size_t k = 0; k < sensors_.size(); ++k) {
    if (!sensors_[k].sensor) continue;
    int mm = sensors_[k].sensor->get();
    if (mm <= 0 || mm > 9000) continue;
    measured[k] = mm / 25.4f;
  }

  const float twoSigSq = 2.0f * cfg_.sensorSigmaIn * cfg_.sensorSigmaIn;
  int n = (int)particles_.size();

  // Log-sum-exp normalization: sums log-likelihoods, then exponentiates
  // relative to max so very negative weights don't underflow to zero.
  std::vector<float> logWeights(n, 0.0f);
  std::vector<int> contribCounts(n, 0);
  float maxLogW = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < n; ++i) {
    const Particle& p = particles_[i];
    float logW = 0.0f;
    int contributed = 0;
    for (size_t k = 0; k < sensors_.size(); ++k) {
      if (measured[k] < 0.0f) continue;
      // Sensor mount → world frame via particle pose.
      float s = std::sin(p.theta);
      float c = std::cos(p.theta);
      float sx = p.x + sensors_[k].offsetY * s - sensors_[k].offsetX * c;
      float sy = p.y + sensors_[k].offsetY * c + sensors_[k].offsetX * s;
      float sAng = p.theta + sensors_[k].angleRad;

      // raycastFn null → perimeter map fallback (raw-spec, diagonal mounts).
      auto fn = sensors_[k].raycastFn ? sensors_[k].raycastFn : &field::raycast;
      float expected = fn(sx, sy, sAng, cfg_.maxRangeIn);

      float residual = measured[k] - expected;
      // Negative residual past outlierGap = ball/object between sensor and
      // wall. Treat as neutral so we don't localize against moving objects.
      if (residual < -cfg_.outlierGapIn) continue;
      logW += -(residual * residual) / twoSigSq;
      ++contributed;
    }
    logWeights[i] = logW;
    contribCounts[i] = contributed;
    if (contributed > 0 && logW > maxLogW) maxLogW = logW;
  }

  // No contributing particle = uninformative tick. Uniform-reset and bail.
  if (maxLogW == -std::numeric_limits<float>::infinity()) {
    for (int i = 0; i < n; ++i) particles_[i].w = 1.0f / n;
    return;
  }

  // exp(logW − maxLogW) keeps heaviest at w=1; underflow-free relative scale.
  for (int i = 0; i < n; ++i) {
    if (contribCounts[i] == 0)
      particles_[i].w = 1.0f / n;
    else
      particles_[i].w = std::exp(logWeights[i] - maxLogW);
  }

  // Normalize, then resample if effective sample size drops below n/2.
  float sumW = 0.0f;
  for (int i = 0; i < n; ++i) sumW += particles_[i].w;
  if (sumW < kWeightEpsilon) {
    ++degenerateCount_;
    for (int i = 0; i < n; ++i) particles_[i].w = 1.0f / n;
    return;
  }
  float sumWSq = 0.0f;
  for (int i = 0; i < n; ++i) {
    particles_[i].w /= sumW;
    sumWSq += particles_[i].w * particles_[i].w;
  }
  float ess = 1.0f / sumWSq;
  if (ess < n / 2.0f) resample();
}

Pose best() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  // Weighted mean; circular mean on theta dodges the wrap discontinuity.
  float sx = 0.0f, sy = 0.0f, ss = 0.0f, sc = 0.0f, wSum = 0.0f;
  for (int i = 0; i < (int)particles_.size(); ++i) {
    const Particle& p = particles_[i];
    sx += p.x * p.w;
    sy += p.y * p.w;
    ss += std::sin(p.theta) * p.w;
    sc += std::cos(p.theta) * p.w;
    wSum += p.w;
  }
  if (wSum < kWeightEpsilon) return {0, 0, 0};
  return {sx / wSum, sy / wSum, std::atan2(ss, sc)};
}

float convergence() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  // Position stddev — how tight the cloud is.
  int n = (int)particles_.size();
  if (n == 0) return 0.0f;
  float mx = 0.0f, my = 0.0f;
  for (int i = 0; i < n; ++i) {
    mx += particles_[i].x;
    my += particles_[i].y;
  }
  mx /= n;
  my /= n;
  float var = 0.0f;
  for (int i = 0; i < n; ++i) {
    float dx = particles_[i].x - mx;
    float dy = particles_[i].y - my;
    var += dx * dx + dy * dy;
  }
  return std::sqrt(var / n);
}

bool converged(float threshold_in) { return convergence() < threshold_in; }

int sensorCount() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return static_cast<int>(sensors_.size());
}
std::vector<DistanceSensorSpec> sensors() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return sensors_;
}

MCLConfig config() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return cfg_;
}
void setConfig(const MCLConfig& cfg) {
  std::lock_guard<pros::Mutex> lock(mtx_);
  cfg_ = cfg;
}

uint32_t degenerateTickCount() {
  std::lock_guard<pros::Mutex> lock(mtx_);
  return degenerateCount_;
}

// Update is the expensive raycast work (~20 ms/tick); kept off the odom
// task. predict() runs inline on the 100 Hz odom task.
void startTask() {
  // Function-static: thread-safe single init, no heap, no re-entry leak.
  static pros::Task lightcastTask([] {
    while (true) {
      update();
      pros::delay(100);
    }
  },
                                  "lightcast_task");
  (void)lightcastTask;
}

}  // namespace light::lightcast
