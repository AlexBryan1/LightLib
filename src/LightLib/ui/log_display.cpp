#include "LightLib/ui/log_display.hpp"

#include <cstdarg>
#include <cstdio>
#include <deque>
#include <mutex>

#include "pros/rtos.hpp"

namespace light {
namespace {

pros::Mutex g_mtx;
std::deque<std::string> g_lines;
uint32_t g_revision = 0;

}  // namespace

void log_emit(const char* fmt, ...) {
  char buf[LOG_LINE_BUF];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;

  // Strip a single trailing newline so the UI can join lines cleanly.
  size_t len = (n < (int)sizeof(buf) - 1) ? (size_t)n : sizeof(buf) - 1;
  if (len > 0 && buf[len - 1] == '\n') buf[--len] = '\0';

  {
    std::lock_guard<pros::Mutex> lock(g_mtx);
    g_lines.emplace_back(buf, len);
    while ((int)g_lines.size() > LOG_MAX_LINES) g_lines.pop_front();
    ++g_revision;
  }

  // printf outside the lock — a slow PROS terminal must not stall the
  // autotuner task that is producing log lines.
  printf("%s\n", buf);
}

void log_clear() {
  std::lock_guard<pros::Mutex> lock(g_mtx);
  g_lines.clear();
  ++g_revision;
}

std::string log_snapshot() {
  std::lock_guard<pros::Mutex> lock(g_mtx);
  std::string out;
  for (size_t i = 0; i < g_lines.size(); ++i) {
    if (i) out.push_back('\n');
    out.append(g_lines[i]);
  }
  return out;
}

uint32_t log_revision() {
  std::lock_guard<pros::Mutex> lock(g_mtx);
  return g_revision;
}

}  // namespace light
