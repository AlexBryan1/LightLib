#pragma once

#include <cstdint>
#include <string>

/**
 * \file log_display.hpp
 *
 * Tiny ring buffer for messages that should appear on the brain-screen
 * "Log" panel. Used by the autotuners to surface their final-result lines
 * (COMPLETED / FAILED / APPLIED VERIFY / EKF / MCL summaries) so the
 * driver can read them off the brain without a terminal.
 *
 * log_emit() is the only producer call site — it both stores the line in
 * the ring buffer AND printfs it, so PROS-terminal output is unchanged.
 */

namespace light {

constexpr int LOG_MAX_LINES = 15;  ///< Lines kept in the ring buffer.
constexpr int LOG_LINE_BUF = 96;   ///< Max formatted bytes per emit call.

/** Append one line and mirror it to printf. printf-style varargs. */
void log_emit(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/** Drop every stored line. Bumps the revision counter. */
void log_clear();

/** Join the buffer with '\n's. Returns a fresh copy taken under the mutex. */
std::string log_snapshot();

/** Bumped on every emit/clear so the UI timer can skip redundant redraws. */
uint32_t log_revision();

}  // namespace light
