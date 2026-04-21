#pragma once
// ─── Named-path registry ─────────────────────────────────────────────────────
//
// Wraps light::runJerryioPath so autons can run a path by name:
//
//     light::runPath("test_path");
//
// Adding a new path:
//   1. Drop a header into include/paths/ with a constexpr const char*
//      named after the path. E.g. include/paths/red_left.hpp:
//
//        namespace light::paths {
//        inline constexpr const char* red_left = R"JERRYIO(
//          ...paste Jerryio export here...
//        )JERRYIO";
//        }
//
//   2. Register it in include/paths/all.hpp — add one #include and one row
//      to the kAll[] table. No edits to paths.cpp required.
//
// Lookup is O(N) string-compare against kAll — fine for the small path
// counts a single robot carries. Unknown names printf a warning and return
// false without invoking the follower.

#include <vector>
#include "LightLib/ramsete.hpp"

namespace light {

bool runPath(const char* name,
             bool reversed = false,
             int  timeoutMs = -1,
             float poseErrBailIn = 8.0f);

bool runPath(const char* name,
             std::vector<PathEvent> events,
             bool reversed = false,
             int  timeoutMs = -1,
             float poseErrBailIn = 8.0f);

} // namespace light
