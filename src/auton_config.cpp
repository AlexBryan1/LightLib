#include "LightLib/ui/auton_selector.hpp"
#include "LightLib/drive/autotune.hpp"
#include "LightLib/drive/ramsete.hpp"
#include "autons.hpp"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       AUTON REGISTRATION                                │
// │                                                                         │
// │  Add every autonomous routine to the on-brain selector here.            │
// │                                                                         │
// │  Syntax:                                                                │
// │    light::auton_selector.add("Button Label", "Description", function);  │
// │                                                                         │
// │  Arguments:                                                             │
// │    "Button Label"  — text shown on the brain screen (keep it short)     │
// │    "Description"   — Describe your auton, add any notes or reminders    │
// │     function       — the void() function declared in autons.hpp         │
// └─────────────────────────────────────────────────────────────────────────┘

void register_autons() {
  light::auton_selector.add("Test Path", "funny", run_jerryio_path_1);

  // ── PID auto-tune ────────────────────────────────────────────────────────
  // Two-phase bracket+bisect on oscillation (≥3 target zero-crossings).
  // Phase 1: find largest kP that doesn't oscillate (kD=0, double-then-halve).
  // Phase 2: boost that kP, then bracket+bisect kD upward until oscillation
  // damps. kI=0. Heading still uses relay-feedback. Each prints per-iter
  // stats; the final kP/kD pair is applied to the live chassis on exit.
  light::auton_selector.add("Tune: Turn", "180° back-and-forth; bisect turn kP then kD on oscillation",
                            [] { light::autotune_turn_pid(); });
  light::auton_selector.add("Tune: Drive", "24\" forward/back; bisect drive kP then kD on oscillation (needs 6 ft)",
                            [] { light::autotune_drive_pid(); });
  light::auton_selector.add("Tune: Swing", "90° left-side swing; bisect swing kP then kD on oscillation",
                            [] { light::autotune_swing_pid(); });
  light::auton_selector.add("Tune: Heading", "48\" fwd/back, scores on |L-R| motor differential (needs 10 ft; tune drive first)",
                            [] { light::autotune_heading_pid(); });
  light::auton_selector.add("Tune: EKF", "Park robot, measure noise floor (5 s)",
                            [] { light::autotune_ekf_noise(); });
  light::auton_selector.add("Tune: MCL", "Park facing walls, measure dist-sensor sigma (4 s)",
                            [] { light::autotune_mcl_noise(); });

  // ── RAMSETE characterization (prints + auto-applies to live config) ─────
  // Transcribe the printed numbers into ramsete_configure() in
  // default_constants() to make them permanent across reboots.
  light::auton_selector.add("Char: kV/kA/kS", "Ramp drive, compute DriveFF (needs 8 ft)",
                            [] { light::characterize_kV_kA_kS(); });
  light::auton_selector.add("Char: Track W.", "Spin in place, compute trackWidthIn",
                            [] { light::characterize_track_width(); });
  light::auton_selector.add("Char: aLatMax", "Constant-radius circle, find lat-accel cap",
                            [] { light::characterize_a_lat_max(); });
  light::auton_selector.add("Char: Friction", "Sweep voltage, fit kS/kV/kQ for PID FF (needs 8 ft)",
                            [] { light::characterize_friction(); });
}
