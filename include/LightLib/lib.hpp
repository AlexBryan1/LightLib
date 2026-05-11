#pragma once

/**
 * \file lib.hpp
 *
 * Umbrella header for LightLib's own classes.
 *
 * Pull this in (or `LightLib/main.h` in user code) to get the chassis API,
 * PID / slew controllers, tracking wheels, pistons, and shared utilities
 * in one include. Combined with `LightLib/api.h` (which carries the PROS
 * C++ API), this is what most LightLib sources need.
 *
 * \note Order matters: `util.hpp` transitively includes `LightLib/api.h`,
 *       so it is included first to break the cycle that would form if
 *       `api.h` tried to pull these in directly.
 */

#include "LightLib/util/util.hpp"
#include "LightLib/control/pid.hpp"
#include "LightLib/control/slew.hpp"
#include "LightLib/drive/drive.hpp"
#include "LightLib/drive/tracking_wheel.hpp"
#include "LightLib/util/piston.hpp"
