#pragma once
#include <functional>
#include <string>
#include <vector>

#include "LightLib/main.h"

/**
 * \file custom_selector.hpp
 *
 * Minimal alternative to AutonSelector: name + function only, no banner /
 * preview / PID panel. Useful for projects that ship their own picker UI
 * but still want a registry.
 */

/** A bare-bones registered auton: just a name and a callback. */
struct CustomAuton {
  std::string name;             ///< Display name.
  std::function<void()> fn;     ///< Routine to invoke when selected.
};

/**
 * Render a splash banner showing the active mode.
 *
 * \param mode
 *        short mode string (e.g. "AUTON", "OPCONTROL")
 */
void display_splash(const char* mode);

/**
 * Append a routine to the custom selector.
 *
 * \param name
 *        display name
 * \param fn
 *        routine to invoke when selected
 */
void custom_selector_add(const std::string& name, std::function<void()> fn);

/** Build the custom selector UI. Call once in `initialize()`. */
void custom_selector_init();

/** \return Index of the currently selected custom auton. */
int custom_selector_get();

/** Invoke the currently selected custom auton (call from `autonomous()`). */
void custom_selector_run();
