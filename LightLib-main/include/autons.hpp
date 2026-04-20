#pragma once

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                         AUTON DECLARATIONS                              │
// │                                                                         │
// │  Three files work together to set up autonomous:                        │
// │                                                                         │
// │    autons.hpp       ← YOU ARE HERE — declare every auton function       │
// │    autons.cpp       ← write the actual autonomous code                  │
// │    auton_config.cpp ← register each auton with the brain-screen picker  │
// │                                                                         │
// │  Workflow for adding a new routine:                                     │
// │    1. Declare it here:   void my_auton();                               │
// │    2. Write it in autons.cpp                                            │
// │    3. Register it in auton_config.cpp:                                  │
// │         light::auton_selector.add("Label", "na", my_auton);            │
// └─────────────────────────────────────────────────────────────────────────┘


// ── Setup functions ───────────────────────────────────────────────────────────
// Called once by initialize() in main — do not rename these.

void default_constants();   // tune PID gains, exit conditions, and slew rates
void default_positions();   // set starting piston / mechanism states


// ── Autonomous routines ───────────────────────────────────────────────────────
void run_jerryio_path_1();

// ── Test / tuning routines ────────────────────────────────────────────────────
// Useful during practice for dialing in PID.  Register these in
// auton_config.cpp during tuning sessions; remove them before competition.

void con_test();                    // basic connectivity / control check
void drive_test(int inches);        // drive straight N inches — tune drive PID
void turn_test(int degrees);        // turn to N degrees — tune turn PID
void swing_test(int degrees);       // swing to N degrees — tune swing PID
void heading_test(int degrees);     // drive while fighting a heading offset
void odom_test(int degrees);        // zero odom then drive 24 in — test odometry
