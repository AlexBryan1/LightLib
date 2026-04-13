#include "LightLib/auton_selector.hpp"
#include "autons.hpp"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       AUTON REGISTRATION                                │
// │  Add every autonomous routine to the on-brain selector here.            │
// │                                                                         │
// │  Syntax:                                                                │
// │    light::auton_selector.add("Button Label", "Description", function); │
// │                                                                         │
// │  - "Button Label"  shown on the selector screen (keep it short)        │
// │  - "Description"   currently unused — reserved for future display      │
// │  - function        the void() auton function declared in autons.hpp     │
// │                                                                         │
// │  Autons appear on screen in the order they are added here.              │
// └─────────────────────────────────────────────────────────────────────────┘
void register_autons() {
    light::auton_selector.add("Left Rush",        "na", rush_left);
    light::auton_selector.add("Right Rush",        "na", rush_right);
    light::auton_selector.add("Left Mid Rush",    "na", rush_mid_left);
    light::auton_selector.add("Right Mid Rush",   "na", rush_mid_right);
    light::auton_selector.add("Left Sevenball",   "na", sevenball_left);
    light::auton_selector.add("Right Sevenball",  "na", sevenball_right);
    light::auton_selector.add("Left SixBall",     "na", sixball_left);
    light::auton_selector.add("Right SixBall",    "na", sixball_right);
    light::auton_selector.add("Left Split",       "na", split_left);
    light::auton_selector.add("Right Split",      "na", split_right);
    light::auton_selector.add("Skills",           "na", skills);
}
