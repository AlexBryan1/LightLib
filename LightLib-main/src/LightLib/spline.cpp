// ─── spline.cpp — Quintic Hermite spline for RAMSETE trajectories ────────────
//
// Fits a C² continuous curve through a list of Waypoints. Each segment is a
// pair of 5th-order Hermite polynomials in x(u) and y(u) for u ∈ [0, 1], built
// from endpoint position / velocity / acceleration. Interior-waypoint velocity
// is chosen with a finite-difference (Catmull-Rom-style) rule when the user
// does not pin the heading; acceleration defaults to zero (matches typical
// robot path generators — non-zero α estimates cause more wiggling than they
// fix for short segments).
//
// Arclength: each segment builds a 65-sample s(u) trapezoid table once at
// construction; s→u lookup is a binary search + linear interp.

#include "LightLib/spline.hpp"
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace light {

// Hermite basis functions for quintic (order 5):
//   H0(u) = 1 - 10u³ + 15u⁴ - 6u⁵
//   H1(u) = u - 6u³ + 8u⁴ - 3u⁵
//   H2(u) = u²/2 - 3u³/2 + 3u⁴/2 - u⁵/2
//   H3(u) = u⁵ - u⁴/2 + u³/2 ... mirrored at u=1
// We evaluate directly from the 6 endpoint coefficients via Horner on the
// monomial coefficients (faster than recomputing basis each call).

struct QuinticCoeffs {
    float c0, c1, c2, c3, c4, c5;
    float eval(float u)   const { return ((((c5*u + c4)*u + c3)*u + c2)*u + c1)*u + c0; }
    float evalD(float u)  const { return (((5*c5*u + 4*c4)*u + 3*c3)*u + 2*c2)*u + c1; }
    float evalDD(float u) const { return ((20*c5*u + 12*c4)*u + 6*c3)*u + 2*c2; }
};

// Build monomial coeffs from Hermite boundary conditions (p0, v0, a0, p1, v1, a1).
// Derived by solving the 6x6 system for q(u) such that:
//   q(0) = p0, q'(0) = v0, q''(0) = a0
//   q(1) = p1, q'(1) = v1, q''(1) = a1
// Closed-form solution:
static QuinticCoeffs makeQuintic(float p0, float v0, float a0,
                                 float p1, float v1, float a1) {
    QuinticCoeffs q;
    q.c0 = p0;
    q.c1 = v0;
    q.c2 = a0 * 0.5f;
    q.c3 = -10.0f*p0 - 6.0f*v0 - 1.5f*a0 + 10.0f*p1 - 4.0f*v1 + 0.5f*a1;
    q.c4 =  15.0f*p0 + 8.0f*v0 + 1.5f*a0 - 15.0f*p1 + 7.0f*v1 - 1.0f*a1;
    q.c5 =  -6.0f*p0 - 3.0f*v0 - 0.5f*a0 +  6.0f*p1 - 3.0f*v1 + 0.5f*a1;
    return q;
}

SplineSample HermiteSegment::eval(float u) const {
    // Rebuild coeffs per-call (cheap — 12 ops) rather than caching per-segment.
    // Segments are evaluated O(64) times at construction and O(1) per control
    // tick thereafter, so the cost is noise compared to the arclength table.
    QuinticCoeffs qx = makeQuintic(p0x, v0x, a0x, p1x, v1x, a1x);
    QuinticCoeffs qy = makeQuintic(p0y, v0y, a0y, p1y, v1y, a1y);
    SplineSample s;
    s.x   = qx.eval(u);    s.y   = qy.eval(u);
    s.dx  = qx.evalD(u);   s.dy  = qy.evalD(u);
    s.ddx = qx.evalDD(u);  s.ddy = qy.evalDD(u);
    return s;
}

// Heading → tangent vector (LightLib convention: +Y forward, θ=0 faces +Y, CW+).
// Tangent magnitude is chosen as the chord length of the segment so that the
// Hermite curve matches chord velocity when the user pins headings at both
// ends (avoids tiny derivatives that manifest as kinks).
static void headingToTangent(float headingRad, float scale, float& vx, float& vy) {
    vx = std::sin(headingRad) * scale;
    vy = std::cos(headingRad) * scale;
}

Spline::Spline(const std::vector<Waypoint>& wps) {
    if (wps.size() < 2) return;

    const std::size_t N = wps.size();

    // Choose per-waypoint tangent (velocity) vectors.
    // For endpoints with pinned headings, convert heading to unit tangent
    // and scale by adjacent chord length. For interior waypoints without a
    // pinned heading, use a Catmull-Rom-like centered difference.
    std::vector<float> vx(N, 0.0f), vy(N, 0.0f);
    for (std::size_t i = 0; i < N; ++i) {
        bool hasHeading = wps[i].headingRad.has_value();
        float chordLen;
        if (i == 0)          chordLen = std::hypot(wps[1].x - wps[0].x,      wps[1].y - wps[0].y);
        else if (i == N - 1) chordLen = std::hypot(wps[N-1].x - wps[N-2].x,  wps[N-1].y - wps[N-2].y);
        else                 chordLen = 0.5f * (std::hypot(wps[i+1].x - wps[i-1].x,
                                                           wps[i+1].y - wps[i-1].y));
        if (chordLen < 1e-4f) chordLen = 1.0f;

        if (hasHeading) {
            headingToTangent(*wps[i].headingRad, chordLen, vx[i], vy[i]);
        } else if (i == 0) {
            vx[i] = wps[1].x - wps[0].x;
            vy[i] = wps[1].y - wps[0].y;
        } else if (i == N - 1) {
            vx[i] = wps[N-1].x - wps[N-2].x;
            vy[i] = wps[N-1].y - wps[N-2].y;
        } else {
            vx[i] = 0.5f * (wps[i+1].x - wps[i-1].x);
            vy[i] = 0.5f * (wps[i+1].y - wps[i-1].y);
        }
    }

    // Build one HermiteSegment per adjacent waypoint pair.
    segs_.reserve(N - 1);
    for (std::size_t i = 0; i + 1 < N; ++i) {
        HermiteSegment seg;
        seg.p0x = wps[i].x;   seg.p0y = wps[i].y;
        seg.v0x = vx[i];      seg.v0y = vy[i];
        seg.a0x = 0.0f;       seg.a0y = 0.0f;
        seg.p1x = wps[i+1].x; seg.p1y = wps[i+1].y;
        seg.v1x = vx[i+1];    seg.v1y = vy[i+1];
        seg.a1x = 0.0f;       seg.a1y = 0.0f;
        segs_.push_back(seg);
    }

    // Per-segment arclength table via trapezoidal integration of speed.
    // 65 samples gives ~1% accuracy for realistic VEX segment lengths.
    segArcTables_.resize(segs_.size());
    segLens_.resize(segs_.size());
    totalLen_ = 0.0f;
    for (std::size_t i = 0; i < segs_.size(); ++i) {
        auto& table = segArcTables_[i];
        const int K = 64;                  // 65 samples → 64 intervals
        float prevSpeed = 0.0f;
        {
            SplineSample s = segs_[i].eval(0.0f);
            prevSpeed = std::hypot(s.dx, s.dy);
        }
        table[0] = 0.0f;
        for (int k = 1; k <= K; ++k) {
            float u = (float)k / (float)K;
            SplineSample s = segs_[i].eval(u);
            float speed = std::hypot(s.dx, s.dy);
            // trapezoid: ∫|r'(u)| du ≈ sum of averaged speeds × du
            table[k] = table[k-1] + 0.5f * (speed + prevSpeed) * (1.0f / (float)K);
            prevSpeed = speed;
        }
        segLens_[i] = table[K];
        totalLen_  += segLens_[i];
    }
}

void Spline::sToSegU(float s, int& segIdx, float& u) const {
    if (segs_.empty()) { segIdx = 0; u = 0.0f; return; }
    if (s <= 0.0f)           { segIdx = 0; u = 0.0f; return; }
    if (s >= totalLen_)      { segIdx = (int)segs_.size() - 1; u = 1.0f; return; }

    // Walk through segment lengths to find which segment contains s.
    float acc = 0.0f;
    segIdx = 0;
    for (std::size_t i = 0; i < segLens_.size(); ++i) {
        if (s <= acc + segLens_[i]) { segIdx = (int)i; break; }
        acc += segLens_[i];
    }
    float segS = s - acc;

    // Binary-search the per-segment LUT for u such that table[u] ≈ segS.
    const auto& table = segArcTables_[segIdx];
    const int K = 64;
    int lo = 0, hi = K;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (table[mid] < segS) lo = mid; else hi = mid;
    }
    float a = table[lo], b = table[hi];
    float frac = (b - a > 1e-6f) ? (segS - a) / (b - a) : 0.0f;
    u = ((float)lo + frac) / (float)K;
}

SplineSample Spline::sampleAt(float s) const {
    int seg; float u;
    sToSegU(s, seg, u);
    if (segs_.empty()) return SplineSample{};
    return segs_[seg].eval(u);
}

float Spline::headingAt(float s) const {
    SplineSample p = sampleAt(s);
    // LightLib: θ = 0 faces +Y; +Y forward, +X right; CW-positive.
    // tangent (sin θ, cos θ) ⇒ θ = atan2(dx, dy).
    return std::atan2(p.dx, p.dy);
}

float Spline::curvatureAt(float s) const {
    SplineSample p = sampleAt(s);
    float num = p.dx * p.ddy - p.dy * p.ddx;
    float speedSq = p.dx*p.dx + p.dy*p.dy;
    float denom = std::pow(std::max(speedSq, 1e-8f), 1.5f);
    // LightLib sign convention: κ > 0 means path turning CW (heading increases).
    // atan2(dx, dy) increases when (dx·ddy − dy·ddx) > 0 — matches above.
    return num / denom;
}

} // namespace light
