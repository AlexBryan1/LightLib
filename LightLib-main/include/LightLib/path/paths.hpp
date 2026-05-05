#pragma once

#include <vector>

#include "LightLib/path/ramsete.hpp"

/**
 * \file paths.hpp
 *
 * Named-path registry. Wraps `light::runJerryioPath` so autons can run a
 * path by name:
 *
 * \code
 *   light::runPath("test_path");
 * \endcode
 *
 * \par Adding a new path
 *   1. Drop a header into `include/paths/` with a `constexpr const char*`
 *      named after the path. E.g. `include/paths/red_left.hpp`:
 *      \code
 *        namespace light::paths {
 *        inline constexpr const char* red_left = R"JERRYIO(
 *          ...paste Jerryio export here...
 *        )JERRYIO";
 *        }
 *      \endcode
 *   2. Register it in `include/paths/all.hpp` — add one `#include` and one
 *      row to the `kAll[]` table. No edits to `paths.cpp` required.
 *
 * Lookup is O(N) string-compare against `kAll` — fine for the small path
 * counts a single robot carries. Unknown names printf a warning and return
 * false without invoking the follower.
 */

namespace light {

/**
 * Run a registered path by name.
 *
 * \param name
 *        path name (must match a row in `kAll[]`)
 * \param reversed
 *        drive the path in reverse
 * \param timeoutMs
 *        wall-time bail in ms (-1 = no timeout)
 * \param poseErrBailIn
 *        bail if pose error exceeds this many inches
 * \return  true on completion, false on bail/timeout/unknown name
 */
bool runPath(const char* name,
             bool reversed = false,
             int timeoutMs = -1,
             float poseErrBailIn = 8.0f);

/** Variant accepting mid-path event triggers. See PathEvent. */
bool runPath(const char* name,
             std::vector<PathEvent> events,
             bool reversed = false,
             int timeoutMs = -1,
             float poseErrBailIn = 8.0f);

}  // namespace light
