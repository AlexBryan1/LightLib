#pragma once
// ─── Quintic Hermite spline over 2-D waypoints ───────────────────────────────
//
// Used by RAMSETE trajectory generation. Produces a C² continuous curve with
// controllable heading at endpoints and optional heading at interior waypoints.
//
// CONVENTION (LightLib): +Y forward, +X right. Theta = 0 faces +Y, CW-positive.
// Tangent direction for a heading θ is (sin θ, cos θ). Curvature sign: positive
// curvature = path turning CW (robot heading increases).

#include <array>
#include <optional>
#include <vector>

namespace light {

struct Waypoint {
    float x = 0.0f;
    float y = 0.0f;
    std::optional<float> headingRad;   // if absent, tangent taken from adjacent chord
    std::optional<float> speed;        // optional per-waypoint speed cap (not used yet)
};

struct SplineSample {
    float x = 0.0f, y = 0.0f;
    float dx = 0.0f, dy = 0.0f;        // d/du
    float ddx = 0.0f, ddy = 0.0f;      // d²/du²
};

struct HermiteSegment {
    float p0x, p0y, v0x, v0y, a0x, a0y;
    float p1x, p1y, v1x, v1y, a1x, a1y;
    SplineSample eval(float u) const;
};

class Spline {
public:
    Spline() = default;
    explicit Spline(const std::vector<Waypoint>& wps);

    bool   valid() const { return !segs_.empty(); }
    float  totalArcLen() const { return totalLen_; }
    size_t segmentCount() const { return segs_.size(); }

    // Global-arclength queries. s is clamped to [0, totalArcLen()].
    SplineSample sampleAt(float s) const;
    float        headingAt(float s) const;    // atan2(dx, dy) — LightLib convention
    float        curvatureAt(float s) const;  // positive = CW (matches theta sign)

private:
    std::vector<HermiteSegment> segs_;
    std::vector<std::array<float, 65>> segArcTables_;  // per-segment u→s LUT
    std::vector<float> segLens_;
    float totalLen_ = 0.0f;

    void sToSegU(float s, int& segIdx, float& u) const;
};

} // namespace light
