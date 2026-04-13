#pragma once

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                         AUTON DECLARATIONS                              │
// │  Declare every autonomous function here so it can be used in both       │
// │  autons.cpp (where you write the code) and auton_config.cpp             │
// │  (where you register it with the selector).                             │
// │                                                                         │
// │  Pattern:  void my_auton_name();                                        │
// └─────────────────────────────────────────────────────────────────────────┘

// ── Setup functions ───────────────────────────────────────────────────────────
// Called once from initialize() in main.cpp — do not rename these.
void default_constants();   // PID / exit-condition / slew tuning
void default_positions();   // starting piston / mechanism positions

// ── Autonomous routines ───────────────────────────────────────────────────────
// Add a declaration for every routine you write in autons.cpp,
// then register it in auton_config.cpp with light::auton_selector.add().

void split_left();
void split_right();

void sevenball_right();
void sevenball_left();

void rush_mid_left();
void rush_mid_right();

void rush_left();
void rush_right();

void sixball_left();
void sixball_right();

void skills();

// ── Test / debug routines ─────────────────────────────────────────────────────
// Useful during practice for tuning — remove or comment out for competition.
void con_test();
void drive_test(int inches);
void turn_test(int degrees);
void swing_test(int degrees);
void heading_test(int degrees);
void odom_test(int degrees);
