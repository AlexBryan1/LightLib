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
    light::auton_selector.add("L 7",              "na", sevenball_left);
    light::auton_selector.add("R 7",              "na", sevenball_right);
    light::auton_selector.add("L 6/3 Mid",        "na", split_left);
    light::auton_selector.add("R 6/3 Long",       "na", split_right);
    light::auton_selector.add("L 6/3 Long",       "na", delayed_split);
    light::auton_selector.add("SAWP",             "na", sawp);
    light::auton_selector.add("Secret",           "na", rush_mid_left);
    light::auton_selector.add("Skills",           "na", skills);
}
