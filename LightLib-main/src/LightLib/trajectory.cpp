// ─── trajectory.cpp — Time-parameterize a Spline with a velocity profile ─────
//
// Three passes over the path (sampled every PROFILE_DS inches):
//   1. Per-sample speed ceiling from curvature: v² · |κ| ≤ aLatMax.
//   2. Forward sweep enforces accel ceiling: v[i+1]² ≤ v[i]² + 2·aMax·ds.
//   3. Backward sweep enforces decel ceiling: v[i-1]² ≤ v[i]² + 2·aDecMax·ds.
// Endpoints are clamped to zero so the robot starts and stops at rest.
//
// Then we walk the arclength-domain profile and emit control-rate samples:
//   dt = 2·ds / (v[i] + v[i+1])
// accumulating time until the path runs out.

#include "LightLib/trajectory.hpp"
#include <algorithm>
#include <cmath>

namespace light {

static constexpr float PROFILE_DS = 0.5f;  // inches
static constexpr float CONTROL_DT = 0.01f; // seconds

// 3-sample median filter on a vector — smooths curvature spikes that come
// from the finite-difference nature of the spline derivatives. We do *not*
// filter velocity itself, since that's what the forward/backward passes do.
static void medianFilter3(std::vector<float>& v) {
    if (v.size() < 3) return;
    std::vector<float> out(v.size());
    out.front() = v.front();
    out.back()  = v.back();
    for (std::size_t i = 1; i + 1 < v.size(); ++i) {
        float a = v[i-1], b = v[i], c = v[i+1];
        // median of three
        out[i] = std::max(std::min(a, b), std::min(std::max(a, b), c));
    }
    v.swap(out);
}

Trajectory generateTrajectory(const std::vector<Waypoint>& wps,
                              const TrajConstraints& cons,
                              bool reversed) {
    Trajectory traj;
    if (wps.size() < 2) return traj;

    Spline spline(wps);
    if (!spline.valid()) return traj;

    const float L = spline.totalArcLen();
    if (L < 1e-3f) return traj;

    // Sample the path every PROFILE_DS inches.
    const int N = std::max(2, (int)std::ceil(L / PROFILE_DS) + 1);
    std::vector<float> sArr(N), kArr(N), vArr(N, cons.vMax);
    for (int i = 0; i < N; ++i) {
        float s = (i == N - 1) ? L : (float)i * PROFILE_DS;
        sArr[i] = s;
        kArr[i] = spline.curvatureAt(s);
    }

    medianFilter3(kArr);

    // 1. Curvature → lateral-accel ceiling per sample.
    for (int i = 0; i < N; ++i) {
        float k = std::abs(kArr[i]);
        if (k > 1e-4f) {
            float vCap = std::sqrt(cons.aLatMax / k);
            vArr[i] = std::min(vArr[i], vCap);
        }
    }

    // Endpoints at rest.
    vArr.front() = 0.0f;
    vArr.back()  = 0.0f;

    // 2. Forward pass — limit by forward accel.
    for (int i = 0; i + 1 < N; ++i) {
        float ds = sArr[i+1] - sArr[i];
        float vNext = std::sqrt(vArr[i]*vArr[i] + 2.0f * cons.aMax * ds);
        vArr[i+1] = std::min(vArr[i+1], vNext);
    }

    // 3. Backward pass — limit by decel.
    for (int i = N - 1; i > 0; --i) {
        float ds = sArr[i] - sArr[i-1];
        float vPrev = std::sqrt(vArr[i]*vArr[i] + 2.0f * cons.aDecMax * ds);
        vArr[i-1] = std::min(vArr[i-1], vPrev);
    }

    // Emit control-rate states by walking arclength with dt = 2·ds/(v_a+v_b).
    // We keep a floating "arclength cursor" and advance it by v·dt each step,
    // interpolating into (sArr, vArr) for v and a.
    traj.pts.reserve((std::size_t)(L / (cons.vMax * CONTROL_DT) * 2.0f) + 64);

    float s = 0.0f;
    float t = 0.0f;
    int idx = 0;  // index into sArr such that sArr[idx] <= s < sArr[idx+1]

    while (s <= L) {
        // Advance idx to bracket s.
        while (idx + 1 < N && sArr[idx+1] < s) ++idx;

        // Interpolate v at s.
        float v;
        if (idx + 1 >= N) {
            v = vArr.back();
        } else {
            float denom = sArr[idx+1] - sArr[idx];
            float frac  = denom > 1e-6f ? (s - sArr[idx]) / denom : 0.0f;
            v = vArr[idx] + frac * (vArr[idx+1] - vArr[idx]);
        }

        // Sample geometry at s.
        SplineSample samp = spline.sampleAt(s);
        float kappa = spline.curvatureAt(s);
        float theta = std::atan2(samp.dx, samp.dy);

        // Build state — flip heading and sign for reversed trajectories.
        TrajState st;
        st.t     = t;
        st.x     = samp.x;
        st.y     = samp.y;
        st.theta = reversed ? (theta + (float)M_PI) : theta;
        st.v     = reversed ? -v : v;
        st.kappa = kappa;   // curvature is geometric — doesn't flip
        st.omega = st.v * kappa;

        traj.pts.push_back(st);

        if (s >= L) break;

        // Next step: one control tick of arclength.
        float advance = std::max(0.01f, v) * CONTROL_DT;
        s += advance;
        if (s > L) s = L;
        t += CONTROL_DT;
    }

    // Fill in acceleration by central difference on v.
    const std::size_t M = traj.pts.size();
    for (std::size_t i = 0; i < M; ++i) {
        float a;
        if (i == 0 && M > 1)        a = (traj.pts[1].v     - traj.pts[0].v)     / CONTROL_DT;
        else if (i == M - 1 && M>1) a = (traj.pts[M-1].v   - traj.pts[M-2].v)   / CONTROL_DT;
        else if (M > 2)             a = (traj.pts[i+1].v   - traj.pts[i-1].v)   / (2.0f * CONTROL_DT);
        else                        a = 0.0f;
        traj.pts[i].a = a;
    }

    traj.duration = M > 0 ? traj.pts.back().t : 0.0f;
    return traj;
}

TrajState Trajectory::sample(float t) const {
    if (pts.empty()) return TrajState{};
    if (t <= pts.front().t) return pts.front();
    if (t >= pts.back().t)  return pts.back();

    // Binary search for the bracketing states.
    std::size_t lo = 0, hi = pts.size() - 1;
    while (hi - lo > 1) {
        std::size_t mid = (lo + hi) / 2;
        if (pts[mid].t <= t) lo = mid; else hi = mid;
    }
    const TrajState& a = pts[lo];
    const TrajState& b = pts[hi];
    float denom = b.t - a.t;
    float frac  = denom > 1e-6f ? (t - a.t) / denom : 0.0f;

    TrajState out;
    out.t     = t;
    out.x     = a.x     + frac * (b.x     - a.x);
    out.y     = a.y     + frac * (b.y     - a.y);
    // Heading: interpolate as a vector to avoid π-seam issues.
    {
        float sx = std::sin(a.theta) + frac * (std::sin(b.theta) - std::sin(a.theta));
        float cy = std::cos(a.theta) + frac * (std::cos(b.theta) - std::cos(a.theta));
        out.theta = std::atan2(sx, cy);
    }
    out.v     = a.v     + frac * (b.v     - a.v);
    out.omega = a.omega + frac * (b.omega - a.omega);
    out.a     = a.a     + frac * (b.a     - a.a);
    out.kappa = a.kappa + frac * (b.kappa - a.kappa);
    return out;
}

} // namespace light
