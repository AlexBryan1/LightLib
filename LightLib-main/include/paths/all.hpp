#pragma once
// ─── Universal path registry ─────────────────────────────────────────────
// Add a new path in two steps:
//   1. Drop `include/paths/<name>.hpp` with `inline constexpr const char* <name>`
//      under `namespace light::paths`.
//   2. Add TWO lines to this file — one #include, one row in kAll[].
// That's it. No edits to paths.cpp required.

#include "paths/test_path.hpp"
// add more here, e.g. #include "paths/red_left.hpp"

namespace light::paths {

struct PathEntry {
    const char* name;
    const char* csv;
};

inline constexpr PathEntry kAll[] = {
    { "test_path", test_path },
    // add more here, e.g. { "red_left", red_left },
};

} // namespace light::paths
