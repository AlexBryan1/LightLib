// Named-path dispatcher. Registry kAll[] lives in include/paths/all.hpp.

#include "LightLib/path/paths.hpp"

#include <cstdio>
#include <cstring>

#include "paths/all.hpp"

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

}  // namespace

bool runPath(const char* name,
             bool reversed,
             int timeoutMs,
             float poseErrBailIn,
             Follower follower) {
  const paths::PathEntry* e = findPath(name);
  if (!e) {
    reportUnknown(name);
    return false;
  }
  return runJerryioPath(e->csv, reversed, timeoutMs, poseErrBailIn, follower);
}

bool runPath(const char* name,
             std::vector<PathEvent> events,
             bool reversed,
             int timeoutMs,
             float poseErrBailIn,
             Follower follower) {
  const paths::PathEntry* e = findPath(name);
  if (!e) {
    reportUnknown(name);
    return false;
  }
  return runJerryioPath(e->csv, std::move(events),
                        reversed, timeoutMs, poseErrBailIn, follower);
}

}  // namespace light
