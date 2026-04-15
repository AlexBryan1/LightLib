#include "LightLib/auton_selector.hpp"
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
// │    "Description"   — reserved for future display, use "na" for now      │
// │     function       — the void() function declared in autons.hpp         │
// └─────────────────────────────────────────────────────────────────────────┘
void register_autons() {
    light::auton_selector.add("Test",             "na", split_left);
    light::auton_selector.add("Left Rush",        "na", rush_left);
    light::auton_selector.add("Right Rush",       "na", rush_right);
    light::auton_selector.add("Left Mid Rush",    "na", rush_mid_left);
    light::auton_selector.add("SAWP",             "na", sawp);
    light::auton_selector.add("Left Sevenball",   "na", sevenball_left);
    light::auton_selector.add("Right Sevenball",  "na", sevenball_right);
    light::auton_selector.add("Left Split",       "na", split_left);
    light::auton_selector.add("Right Split",      "na", split_right);
    light::auton_selector.add("Skills",           "na", skills);
}
