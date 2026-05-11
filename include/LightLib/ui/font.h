#pragma once

/**
 * \file font.h
 *
 * Custom LVGL font symbol exposed to both C and C++ translation units.
 */

#ifdef __cplusplus
extern "C" {
#endif
#include "liblvgl/lvgl.h"

/** Custom UI font baked into the firmware. Defined in `font.c`. */
extern lv_font_t font;

#ifdef __cplusplus
}
#endif
