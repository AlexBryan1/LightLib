// ─── paths.cpp — named-path dispatcher ───────────────────────────────────────
//
// The registry itself (kAll[]) lives in include/paths/all.hpp so users only
// touch one file when adding a new path. This file just looks the name up
// and hands the CSV off to the RAMSETE follower via runJerryioPath.

#include "Lightlib/paths.hpp"
#include "paths/all.hpp"

#include <cstdio>
#include <cstring>

namespace light {
namespace {

const paths::PathEntry* findPath(const char* name) {
    if (!name) return nullptr;
    for (const auto& p : paths::kAll) {
        if (std::strcmp(p.name, name) == 0) return &p;
    }
    return nullptr;
}

void reportUnknown(const char* name) {
    printf("[paths] unknown path '%s' — known names:", name ? name : "(null)");
    for (const auto& p : paths::kAll) printf(" %s", p.name);
    printf("\n");
}

} // namespace

bool runPath(const char* name,
             bool reversed,
             int  timeoutMs,
             float poseErrBailIn) {
    const paths::PathEntry* e = findPath(name);
    if (!e) { reportUnknown(name); return false; }
    return runJerryioPath(e->csv, reversed, timeoutMs, poseErrBailIn);
}

bool runPath(const char* name,
             std::vector<PathEvent> events,
             bool reversed,
             int  timeoutMs,
             float poseErrBailIn) {
    const paths::PathEntry* e = findPath(name);
    if (!e) { reportUnknown(name); return false; }
    return runJerryioPath(e->csv, std::move(events),
                          reversed, timeoutMs, poseErrBailIn);
}

} // namespace light
