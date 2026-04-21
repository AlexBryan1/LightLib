#include "LightLib/field_map.hpp"
#include <cmath>
#include <algorithm>

namespace light::field {

// Slab-based ray-vs-AABB: for the outer field walls we intersect the ray with
// each of the four axis-aligned lines and keep the smallest positive t.
// More robust than segment intersection when the ray origin is near a wall,
// and cheap enough to run 200 particles × N sensors per LightCast tick.
float raycast(float x, float y, float angleRad, float max_range) {
    if (x < -FIELD_HALF || x > FIELD_HALF ||
        y < -FIELD_HALF || y > FIELD_HALF) return 0.0f;

    const float dx = std::sin(angleRad);
    const float dy = std::cos(angleRad);

    float tMin = max_range;

    auto hit = [&](float denom, float num) {
        if (std::fabs(denom) < 1e-6f) return;
        float t = num / denom;
        if (t > 0.0f && t < tMin) tMin = t;
    };

    hit(dx,  FIELD_HALF - x);  // right wall
    hit(dx, -FIELD_HALF - x);  // left wall
    hit(dy,  FIELD_HALF - y);  // front wall
    hit(dy, -FIELD_HALF - y);  // back wall

    return std::min(tMin, max_range);
}

}  // namespace light::field
