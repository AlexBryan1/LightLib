// Three passes at PROFILE_DS spacing: lateral-accel ceiling (v²·|κ| ≤ aLatMax),
// forward accel sweep, backward decel sweep. Endpoints clamped to zero. Then
// the arclength profile is walked at dt = 2·ds/(v[i]+v[i+1]) to emit samples.

#include "Lightlib/drive/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace light {

static constexpr float PROFILE_DS = 0.5f;   // inches
static constexpr float CONTROL_DT = 0.01f;  // seconds

// Smooths curvature spikes from finite-difference spline derivatives.
// Velocity itself is left alone — that's what the forward/backward passes do.
static void medianFilter3(std::vector<float>& v) {
  if (v.size() < 3) return;
  std::vector<float> out(v.size());
  out.front() = v.front();
  out.back() = v.back();
  for (std::size_t i = 1; i + 1 < v.size(); ++i) {
    float a = v[i - 1], b = v[i], c = v[i + 1];
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

  const int N = std::max(2, (int)std::ceil(L / PROFILE_DS) + 1);
  std::vector<float> sArr(N), kArr(N), vArr(N, cons.vMax);
  for (int i = 0; i < N; ++i) {
    float s = (i == N - 1) ? L : (float)i * PROFILE_DS;
    sArr[i] = s;
    kArr[i] = spline.curvatureAt(s);
  }

  medianFilter3(kArr);

  for (int i = 0; i < N; ++i) {
    float k = std::abs(kArr[i]);
    if (k > 1e-4f) {
      float vCap = std::sqrt(cons.aLatMax / k);
      vArr[i] = std::min(vArr[i], vCap);
    }
  }

  vArr.front() = 0.0f;
  vArr.back() = 0.0f;

  for (int i = 0; i + 1 < N; ++i) {
    float ds = sArr[i + 1] - sArr[i];
    float vNext = std::sqrt(vArr[i] * vArr[i] + 2.0f * cons.aMax * ds);
    vArr[i + 1] = std::min(vArr[i + 1], vNext);
  }

  for (int i = N - 1; i > 0; --i) {
    float ds = sArr[i] - sArr[i - 1];
    float vPrev = std::sqrt(vArr[i] * vArr[i] + 2.0f * cons.aDecMax * ds);
    vArr[i - 1] = std::min(vArr[i - 1], vPrev);
  }

  traj.pts.reserve((std::size_t)(L / (cons.vMax * CONTROL_DT) * 2.0f) + 64);

  float s = 0.0f;
  float t = 0.0f;
  int idx = 0;

  while (s <= L) {
    while (idx + 1 < N && sArr[idx + 1] < s) ++idx;

    float v;
    if (idx + 1 >= N) {
      v = vArr.back();
    } else {
      float denom = sArr[idx + 1] - sArr[idx];
      float frac = denom > 1e-6f ? (s - sArr[idx]) / denom : 0.0f;
      v = vArr[idx] + frac * (vArr[idx + 1] - vArr[idx]);
    }

    SplineSample samp = spline.sampleAt(s);
    float kappa = spline.curvatureAt(s);
    float theta = std::atan2(samp.dx, samp.dy);

    TrajState st;
    st.t = t;
    st.x = samp.x;
    st.y = samp.y;
    st.theta = reversed ? (theta + (float)M_PI) : theta;
    st.v = reversed ? -v : v;
    st.kappa = kappa;  // geometric — sign doesn't flip on reverse
    st.omega = st.v * kappa;

    traj.pts.push_back(st);

    if (s >= L) break;

    float advance = std::max(0.01f, v) * CONTROL_DT;
    s += advance;
    if (s > L) s = L;
    t += CONTROL_DT;
  }

  const std::size_t M = traj.pts.size();
  for (std::size_t i = 0; i < M; ++i) {
    float a;
    if (M < 3 || i == 0 || i == M - 1)
      a = 0.0f;
    else
      a = (traj.pts[i + 1].v - traj.pts[i - 1].v) / (2.0f * CONTROL_DT);
    traj.pts[i].a = a;
  }

  traj.duration = M > 0 ? traj.pts.back().t : 0.0f;
  return traj;
}

TrajState Trajectory::sample(float t) const {
  if (pts.empty()) return TrajState{};
  if (t <= pts.front().t) return pts.front();
  if (t >= pts.back().t) return pts.back();

  std::size_t lo = 0, hi = pts.size() - 1;
  while (hi - lo > 1) {
    std::size_t mid = (lo + hi) / 2;
    if (pts[mid].t <= t)
      lo = mid;
    else
      hi = mid;
  }
  const TrajState& a = pts[lo];
  const TrajState& b = pts[hi];
  float denom = b.t - a.t;
  float frac = denom > 1e-6f ? (t - a.t) / denom : 0.0f;

  TrajState out;
  out.t = t;
  out.x = a.x + frac * (b.x - a.x);
  out.y = a.y + frac * (b.y - a.y);
  // Heading interpolated as a vector to dodge the π-seam.
  {
    float sx = std::sin(a.theta) + frac * (std::sin(b.theta) - std::sin(a.theta));
    float cy = std::cos(a.theta) + frac * (std::cos(b.theta) - std::cos(a.theta));
    out.theta = std::atan2(sx, cy);
  }
  out.v = a.v + frac * (b.v - a.v);
  out.omega = a.omega + frac * (b.omega - a.omega);
  out.a = a.a + frac * (b.a - a.a);
  out.kappa = a.kappa + frac * (b.kappa - a.kappa);
  return out;
}

}  // namespace light
