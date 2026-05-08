#include "LightLib/drive/lightcast.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>

#include "LightLib/path/field_map.hpp"
#include "LightLib/util/util.hpp"
#include "pros/rtos.hpp"

// LightCast particle-filter implementation. The particle array is protected by
// a mutex so the 100 Hz predict() (called from the odom task) and the 10 Hz
// update() (called from the lightcast task in startTask() below) can
// interleave safely.

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

// Low-variance resampling — prefers this over random because it preserves
// particle diversity better when a few heavy particles dominate the weight.
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

DistanceSensorSpec fromFace(pros::Distance* s, Face face, float along, float depth) {
  DistanceSensorSpec spec{s, 0.0f, 0.0f, 0.0f};
  switch (face) {
    case Face::FRONT:
      spec.offsetX = along;
      spec.offsetY = depth;
      spec.angleRad = 0.0f;
      spec.raycastFn = &light::field::raycast_front;
      break;
    case Face::BACK:
      spec.offsetX = -along;
      spec.offsetY = -depth;
      spec.angleRad = M_PI;
      spec.raycastFn = &light::field::raycast_back;
      break;
    case Face::LEFT:
      spec.offsetX = -depth;
      spec.offsetY = along;
      spec.angleRad = M_PI / 2.0f;
      spec.raycastFn = &light::field::raycast_left;
      break;
    case Face::RIGHT:
      spec.offsetX = depth;
      spec.offsetY = -along;
      spec.angleRad = -M_PI / 2.0f;
      spec.raycastFn = &light::field::raycast_right;
      break;
  }
  return spec;
}

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
  // Noise scales with motion magnitude (more motion = more uncertainty).
  // Coefficients model: 0.05 in/in odometry slip + 0.02 in static jitter,
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

  // Read each distance sensor once per tick (values are mm). Skip disconnected
  // sensors (get() returns a large error code on fault).
  static std::vector<float> measured;
  measured.assign(sensors_.size(), -1.0f);
  for (size_t k = 0; k < sensors_.size(); ++k) {
    if (!sensors_[k].sensor) continue;
    int mm = sensors_[k].sensor->get();
    if (mm <= 0 || mm > 9000) continue;
    measured[k] = mm / 25.4f;  // mm → inches
  }

  const float twoSigSq = 2.0f * cfg_.sensorSigmaIn * cfg_.sensorSigmaIn;
  int n = (int)particles_.size();

  // Two-pass to enable log-sum-exp normalization. First pass accumulates
  // log-likelihood; second pass exponentiates relative to the max so even
  // very negative log-weights survive instead of underflowing to 0.
  std::vector<float> logWeights(n, 0.0f);
  std::vector<int> contribCounts(n, 0);
  float maxLogW = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < n; ++i) {
    const Particle& p = particles_[i];
    float logW = 0.0f;
    int contributed = 0;
    for (size_t k = 0; k < sensors_.size(); ++k) {
      if (measured[k] < 0.0f) continue;
      // Transform sensor mount into world frame via particle pose.
      float s = std::sin(p.theta);
      float c = std::cos(p.theta);
      float sx = p.x + sensors_[k].offsetY * s - sensors_[k].offsetX * c;
      float sy = p.y + sensors_[k].offsetY * c + sensors_[k].offsetX * s;
      float sAng = p.theta + sensors_[k].angleRad;

      // Per-sensor map dispatch: fromFace() binds raycastFn to the matching
      // light::field::raycast_<face>; raw spec construction (e.g. diagonal
      // mounts) leaves raycastFn null and falls back to the perimeter map.
      auto fn = sensors_[k].raycastFn ? sensors_[k].raycastFn : &field::raycast;
      float expected = fn(sx, sy, sAng, cfg_.maxRangeIn);

      float residual = measured[k] - expected;
      // Game-element outlier: a ball between sensor and wall reads short.
      // Treat as neutral instead of penalizing — don't localize against
      // moving objects.
      if (residual < -cfg_.outlierGapIn) continue;
      logW += -(residual * residual) / twoSigSq;
      ++contributed;
    }
    logWeights[i] = logW;
    contribCounts[i] = contributed;
    if (contributed > 0 && logW > maxLogW) maxLogW = logW;
  }

  // If no particle contributed anything, the entire tick is uninformative —
  // reset to uniform and bail before resampling.
  if (maxLogW == -std::numeric_limits<float>::infinity()) {
    for (int i = 0; i < n; ++i) particles_[i].w = 1.0f / n;
    return;
  }

  // Second pass: exp(logW - maxLogW) so the heaviest particle gets w=1 and
  // others get well-scaled relative weights. Mathematically identical to
  // normalize-after-exp, but underflow-free.
  for (int i = 0; i < n; ++i) {
    if (contribCounts[i] == 0)
      particles_[i].w = 1.0f / n;
    else
      particles_[i].w = std::exp(logWeights[i] - maxLogW);
  }

  // Normalize + effective sample size check for resample decision.
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
  // Weighted mean; theta uses circular mean to avoid wrap discontinuity.
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
  // Position standard deviation — how tight the particle cloud is.
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

// 10 Hz LightCast update task. Spawned by light::init() after lightcast::init().
// predict() runs inline on the 100 Hz odom task; only update() (the expensive
// ray-cast work, ~20 ms/tick) runs on this task so it stays off odom.
void startTask() {
  // Function-static pros::Task: constructed exactly once on first call,
  // standard C++ thread-safe init handles concurrent starts. No heap
  // allocation, no leak path on re-entry.
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
