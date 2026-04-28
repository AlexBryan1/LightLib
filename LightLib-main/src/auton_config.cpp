#include "LightLib/auton_selector.hpp"
#include "LightLib/ramsete.hpp"
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
    light::auton_selector.add("Test Path",           "funny", run_jerryio_path_1);

    // ── Relay-feedback PID auto-tune ────────────────────────────────────────
    // Each runs ~6-10 s, printf's Ku/Pu/kP/kI/kD, and auto-applies to EZ.
    light::auton_selector.add("Tune: Turn",    "Relay-tune turn PID (in-place)",
                              []{ light::autotune_turn_pid(); });
    light::auton_selector.add("Tune: Drive",   "Relay-tune drive PID (needs 8 ft)",
                              []{ light::autotune_drive_pid(); });
    light::auton_selector.add("Tune: Swing",   "Relay-tune swing PID (left-side)",
                              []{ light::autotune_swing_pid(); });
    light::auton_selector.add("Tune: Heading", "Relay-tune heading-correct PID (needs lane)",
                              []{ light::autotune_heading_pid(); });
    light::auton_selector.add("Tune: EKF",     "Park robot, measure noise floor (5 s)",
                              []{ light::autotune_ekf_noise(); });
    light::auton_selector.add("Tune: MCL",     "Park facing walls, measure dist-sensor sigma (4 s)",
                              []{ light::autotune_mcl_noise(); });

    // ── RAMSETE characterization (prints + auto-applies to live config) ─────
    // Transcribe the printed numbers into ramsete_configure() in
    // default_constants() to make them permanent across reboots.
    light::auton_selector.add("Char: kV/kA/kS", "Ramp drive, compute DriveFF (needs 8 ft)",
                              []{ light::characterize_kV_kA_kS(); });
    light::auton_selector.add("Char: Track W.", "Spin in place, compute trackWidthIn",
                              []{ light::characterize_track_width(); });
    light::auton_selector.add("Char: aLatMax",  "Constant-radius circle, find lat-accel cap",
                              []{ light::characterize_a_lat_max(); });
}
