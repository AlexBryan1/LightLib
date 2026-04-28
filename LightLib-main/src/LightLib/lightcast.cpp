#include "LightLib/lightcast.hpp"
#include "LightLib/field_map.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <random>

// LightCast particle-filter implementation. The particle array is protected by
// a mutex so the 100 Hz predict() (called from the odom task) and the 5 Hz
// update() (called from lightcast_task.cpp) can interleave safely.

namespace light::lightcast {
namespace {

struct Particle { float x, y, theta, w; };

MCLConfig cfg_;
std::vector<Particle> particles_;
std::vector<DistanceSensorSpec> sensors_;
pros::Mutex mtx_;
std::mt19937 rng_{0xC0FFEE};

float wrapRad(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

float gaussian(float sigma) {
    std::normal_distribution<float> d(0.0f, sigma);
    return d(rng_);
}

// Low-variance resampling — prefers this over random because it preserves
// particle diversity better when a few heavy particles dominate the weight.
void resample() {
    int n = (int)particles_.size();
    std::vector<Particle> out(n);
    float totalW = 0.0f;
    for (int i = 0; i < n; ++i) totalW += particles_[i].w;
    if (totalW < 1e-12f) {
        for (int i = 0; i < n; ++i) particles_[i].w = 1.0f / n;
        return;
    }
    std::uniform_real_distribution<float> u(0.0f, totalW / n);
    float r = u(rng_);
    float c = particles_[0].w;
    int   i = 0;
    for (int m = 0; m < n; ++m) {
        float U = r + m * (totalW / n);
        while (U > c && i < n - 1) { ++i; c += particles_[i].w; }
        out[m] = particles_[i];
        out[m].w = 1.0f / n;
    }
    particles_ = std::move(out);
}

}  // namespace

DistanceSensorSpec fromFace(pros::Distance* s, Face face, float along, float depth) {
    DistanceSensorSpec spec{s, 0.0f, 0.0f, 0.0f};
    switch (face) {
        case Face::FRONT: spec.offsetX =  along; spec.offsetY =  depth; spec.angleRad = 0.0f;          break;
        case Face::BACK:  spec.offsetX = -along; spec.offsetY = -depth; spec.angleRad = M_PI;          break;
        case Face::LEFT:  spec.offsetX = -depth; spec.offsetY =  along; spec.angleRad =  M_PI / 2.0f;  break;
        case Face::RIGHT: spec.offsetX =  depth; spec.offsetY = -along; spec.angleRad = -M_PI / 2.0f;  break;
    }
    return spec;
}

void init(const Pose& initial, const std::vector<DistanceSensorSpec>& sensors, MCLConfig cfg) {
    std::lock_guard<pros::Mutex> lock(mtx_);
    cfg_ = cfg;
    sensors_ = sensors;
    particles_.resize(cfg_.numParticles);
    for (int i = 0; i < cfg_.numParticles; ++i) {
        particles_[i].x     = initial.x     + gaussian(1.0f);
        particles_[i].y     = initial.y     + gaussian(1.0f);
        particles_[i].theta = wrapRad(initial.theta + gaussian(0.05f));
        particles_[i].w     = 1.0f / cfg_.numParticles;
    }
}

void predict(float dLocalX, float dLocalY, float dTheta) {
    std::lock_guard<pros::Mutex> lock(mtx_);
    // Noise scales with motion magnitude (more motion = more uncertainty).
    float motionMag = std::sqrt(dLocalX*dLocalX + dLocalY*dLocalY);
    float posNoise  = 0.02f + 0.05f * motionMag;
    float angNoise  = 0.002f + 0.1f * std::fabs(dTheta);

    for (int i = 0; i < (int)particles_.size(); ++i) {
        Particle& p = particles_[i];
        float thetaMid = p.theta + dTheta * 0.5f;
        float s = std::sin(thetaMid);
        float c = std::cos(thetaMid);
        float nx = dLocalX + gaussian(posNoise);
        float ny = dLocalY + gaussian(posNoise);
        p.x    += ny * s - nx * c;
        p.y    += ny * c + nx * s;
        p.theta = wrapRad(p.theta + dTheta + gaussian(angNoise));
    }
}

void update() {
    if (sensors_.empty()) return;

    std::lock_guard<pros::Mutex> lock(mtx_);

    // Read each distance sensor once per tick (values are mm). Skip disconnected
    // sensors (get() returns a large error code on fault).
    std::vector<float> measured(sensors_.size(), -1.0f);
    for (size_t k = 0; k < sensors_.size(); ++k) {
        if (!sensors_[k].sensor) continue;
        int mm = sensors_[k].sensor->get();
        if (mm <= 0 || mm > 9000) continue;
        measured[k] = mm / 25.4f;  // mm → inches
    }

    const float twoSigSq = 2.0f * cfg_.sensorSigmaIn * cfg_.sensorSigmaIn;
    int n = (int)particles_.size();

    for (int i = 0; i < n; ++i) {
        Particle& p = particles_[i];
        float logW = 0.0f;
        int   contributed = 0;
        for (size_t k = 0; k < sensors_.size(); ++k) {
            if (measured[k] < 0.0f) continue;
            // Transform sensor mount into world frame via particle pose.
            float s = std::sin(p.theta);
            float c = std::cos(p.theta);
            float sx = p.x + sensors_[k].offsetY * s - sensors_[k].offsetX * c;
            float sy = p.y + sensors_[k].offsetY * c + sensors_[k].offsetX * s;
            float sAng = p.theta + sensors_[k].angleRad;

            float expected = field::raycast(sx, sy, sAng, cfg_.maxRangeIn);

            float residual = measured[k] - expected;
            // Game-element outlier: a ball between sensor and wall reads short.
            // Treat as neutral instead of penalizing — don't localize against
            // moving objects.
            if (residual < -cfg_.outlierGapIn) continue;
            logW += -(residual * residual) / twoSigSq;
            ++contributed;
        }
        if (contributed == 0) p.w = 1.0f / n;
        else                  p.w = std::exp(logW);
    }

    // Normalize + effective sample size check for resample decision.
    float sumW = 0.0f;
    for (int i = 0; i < n; ++i) sumW += particles_[i].w;
    if (sumW < 1e-12f) return;
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
    if (wSum < 1e-12f) return {0, 0, 0};
    return { sx / wSum, sy / wSum, std::atan2(ss, sc) };
}

float convergence() {
    std::lock_guard<pros::Mutex> lock(mtx_);
    // Position standard deviation — how tight the particle cloud is.
    int n = (int)particles_.size();
    if (n == 0) return 0.0f;
    float mx = 0.0f, my = 0.0f;
    for (int i = 0; i < n; ++i) { mx += particles_[i].x; my += particles_[i].y; }
    mx /= n; my /= n;
    float var = 0.0f;
    for (int i = 0; i < n; ++i) {
        float dx = particles_[i].x - mx;
        float dy = particles_[i].y - my;
        var += dx*dx + dy*dy;
    }
    return std::sqrt(var / n);
}

bool converged(float threshold_in) { return convergence() < threshold_in; }

int sensorCount() { return static_cast<int>(sensors_.size()); }
const std::vector<DistanceSensorSpec>& sensors() { return sensors_; }

MCLConfig config() { return cfg_; }
void      setConfig(const MCLConfig& cfg) { cfg_ = cfg; }

}  // namespace light::lightcast
