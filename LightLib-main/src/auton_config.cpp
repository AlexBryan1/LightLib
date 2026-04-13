#include "auton_selector.hpp"
#include "autons.hpp"  // your auton function declarations

// ─────────────────────────────────────────────────────────────────────────────
//  register_autons()
//  Call this once at the top of initialize() instead of cluttering main.cpp
// ─────────────────────────────────────────────────────────────────────────────
void register_autons() {
    // light::auton_selector.add( "Button Label", "Short description", function );
    // TODO: replace with your real auton functions

    light::auton_selector.add("Left Rush",       "na", rush_left);
    light::auton_selector.add("Right Rush",        "na", rush_right);
    light::auton_selector.add("Left Mid Rush",   "na", rush_mid_left);
    light::auton_selector.add("Right Mid Rush",   "na", rush_mid_right);
    light::auton_selector.add("Left Sevenball",   "na", sevenball_left);
    light::auton_selector.add("Right Sevenball",  "na", sevenball_right);
    light::auton_selector.add("Left SixBall",  "na", sixball_left);
    light::auton_selector.add("Right SixBall",  "na", sixball_right);
    light::auton_selector.add("Left Split",       "na", split_left);
    light::auton_selector.add("Right Split",      "na", split_right);
    light::auton_selector.add("Skills",           "na", skills);
}
