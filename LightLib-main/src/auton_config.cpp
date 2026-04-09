#include "auton_selector.hpp"
#include "autons.hpp"  // your auton function declarations

// ─────────────────────────────────────────────────────────────────────────────
//  register_autons()
//  Call this once at the top of initialize() instead of cluttering main.cpp
// ─────────────────────────────────────────────────────────────────────────────
void register_autons() {
    // light::auton_selector.add( "Button Label", "Short description", function );
    // TODO: replace with your real auton functions

    light::auton_selector.add("Red AWP",    "Rush + stake + 2 rings",  rush_right);
    light::auton_selector.add("Red Elim",   "Max rings route",         rush_left);
    light::auton_selector.add("Blue AWP",   "Rush + stake + 2 rings",  sevenball_left);
    light::auton_selector.add("Blue Elim",  "Max rings route",         sevenball_right);
    light::auton_selector.add("Red Safe",   "Conservative AWP",        split_left);
    light::auton_selector.add("Blue Safe",  "Conservative AWP",        split_right);
    light::auton_selector.add("Skills",     "Full field run",          skills);
    light::auton_selector.add("Do Nothing", "",                        [](){});
}
