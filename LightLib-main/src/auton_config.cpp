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
// │    "Description"   — Describe your auton, add any notes or reminders    │
// │     function       — the void() function declared in autons.hpp         │
// └─────────────────────────────────────────────────────────────────────────┘

void register_autons() {
    light::auton_selector.add("L 7",              "7 balls in long goal, gains control", sevenball_left);
    light::auton_selector.add("R 7",              "7 balls in long goal, gains control", sevenball_right);
    light::auton_selector.add("L 6/3 Long",       "3 in mid then anti lever then 6 in long", delayed_split);
    light::auton_selector.add("R 6/3 Long",       "3 in mid then anti lever then 6 in long", split_right);
    light::auton_selector.add("L 6/3 Mid",        "Anti lever 6 in long with then 3 in mid", split_left);
    light::auton_selector.add("SAWP",             "Solo Auton Win Point(5+0+3+6)", sawp);
    light::auton_selector.add("L 7/2 Delayed",    "7 ball then go to mid after delay", seven_two_left);
    light::auton_selector.add("R 7/2 Delayed",    "7 ball then go to mid after delay", seven_two_right);
    light::auton_selector.add("Secret",           "6 mid rush, one time auton to suprise opponents", rush_mid_left);
    light::auton_selector.add("Skills",           "102 max, realistically not even hitting 80 lol", skills);
}
