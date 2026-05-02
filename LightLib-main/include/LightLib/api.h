/**
 * \file api.h
 *
 * PROS API header provides high-level user functionality
 *
 * Contains declarations for use by typical VEX programmers using PROS.
 *
 * This file should not be modified by users, since it gets replaced whenever
 * a kernel upgrade occurs.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_API_H_
#define _PROS_API_H_

#ifdef __cplusplus
#include <cerrno>
#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#else /* (not) __cplusplus */
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#endif /* __cplusplus */

#define PROS_VERSION_MAJOR 4
#define PROS_VERSION_MINOR 1
#define PROS_VERSION_PATCH 1
#define PROS_VERSION_STRING "4.1.1"

#ifndef PROS_USE_SIMPLE_NAMES
#define PROS_USE_SIMPLE_NAMES
#endif

#include "light/adi.h"
#include "light/colors.h"
#include "light/device.h"
#include "light/distance.h"
#include "light/error.h"
#include "light/ext_adi.h"
#include "light/gps.h"
#include "light/imu.h"
#include "light/link.h"
#include "light/llemu.h"
#include "light/misc.h"
#include "light/motors.h"
#include "light/optical.h"
#include "light/rotation.h"
#include "light/rtos.h"
#include "light/screen.h"
#include "light/vision.h"

#ifdef __cplusplus
#include "light/adi.hpp"
#include "light/colors.hpp"
#include "light/device.hpp"
#include "light/distance.hpp"
#include "light/gps.hpp"
#include "light/imu.hpp"
#include "light/link.hpp"
#include "light/llemu.hpp"
#include "light/misc.hpp"
#include "light/motor_group.hpp"
#include "light/motors.hpp"
#include "light/optical.hpp"
#include "light/rotation.hpp"
#include "light/rtos.hpp"
#include "light/screen.hpp"
#include "light/vision.hpp"
#include "light/units.hpp"
#include "light/lvgl.h"
#include "light/namespaces.hpp"
#endif

#endif  // _PROS_API_H_
