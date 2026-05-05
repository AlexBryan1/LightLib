#pragma once

#include "liblvgl/lvgl.h"

/**
 * \file ui_config.hpp
 *
 * Central place to tweak the on-brain UI look. Edit colors / font here and
 * both the auton selector screen and the LVGL theme update together.
 *
 * Covers:
 *   - Color palette used by the auton selector (purple/gold theme)
 *   - Default LVGL theme font
 *   - Theme primary / secondary background colors
 *
 * \note The live PID tuner (`pid_tuner.cpp`) has its own separate palette.
 */

/**
 * \name Color palette
 * 24-bit hex `0xRRGGBB`. `lv_color_hex()` wraps at use sites; not constexpr
 * so these are `#define`s.
 * @{
 */
#define COL_BG lv_color_hex(0x68468F)        ///< Screen background.
#define COL_PANEL lv_color_hex(0x503570)     ///< Right info panel.
#define COL_ACCENT lv_color_hex(0xFFDF61)    ///< Gold (headers, selected).
#define COL_ACCENT2 lv_color_hex(0xCCAA30)   ///< Gold gradient bottom.
#define COL_BTN_IDLE lv_color_hex(0x553878)  ///< Unselected button background.
#define COL_BORDER lv_color_hex(0xFFDF61)    ///< Border lines.
#define COL_TEXT lv_color_hex(0xFFDF61)      ///< Primary text (gold).
#define COL_TEXT_DIM lv_color_hex(0xCCAA30)  ///< Dim text.
#define COL_TEXT_SEL lv_color_hex(0x3A2055)  ///< Dark text on gold background.
#define COL_YELLOW lv_color_hex(0xFFDF61)    ///< Generic yellow.
#define COL_RED lv_color_hex(0xFF4444)       ///< Warning / error red.
/** @} */

/**
 * \name LVGL theme
 * Applied in `lvgl_theme.cpp` before PROS applies its default blue theme.
 * @{
 */
#define COL_THEME_PRIMARY lv_color_hex(0x0D0D10)   ///< Primary theme background.
#define COL_THEME_SECONDARY lv_color_hex(0x0D0D10) ///< Secondary theme background.
#define UI_THEME_DARK true                          ///< Dark-mode flag for the theme.

/**
 * Default UI font. Must be one LVGL has compiled in (see `lv_conf.h`).
 * Options: `lv_font_montserrat_14`, `_16`, `_18`, `_20`, `_22`, `_24`, …
 */
#define UI_DEFAULT_FONT (&lv_font_montserrat_20)
/** @} */

/**
 * \name Animation timing (ms)
 * Bigger values = slower animations. Change here to retune the feel of the
 * selector / run screens.
 * @{
 */
constexpr int UI_ANIM_RUN_ZOOM_IN_MS = 550;     ///< Zoom when opening run screen.
constexpr int UI_ANIM_RUN_ZOOM_OUT_MS = 450;    ///< Zoom when pressing BACK.
constexpr int UI_ANIM_BANNER_PX_MS = 7;         ///< Per-pixel scroll duration.
constexpr int UI_ANIM_BANNER_STAGGER_MS = 400;  ///< Delay between buttons.
constexpr int UI_ANIM_BANNER_REPEAT_DELAY_MS = 800; ///< Delay before banner repeats.
constexpr int UI_ODOM_REFRESH_MS = 100;         ///< Odom X/Y/Angle label refresh period.
/** @} */

/**
 * \name Controller screen slots
 * One controller line, three text slots: LEFT | MID | RIGHT.
 * Pick one `CtrlSlot` per position. Line width ≈ 19 chars; slots fit 6 / 5 / 5.
 * @{
 */
/** Per-slot content options for the controller status line. */
enum class CtrlSlot {
  None,           ///< Empty slot.
  MaxMotorTempC,  ///< `"55C"` — hottest drive/score motor.
  AutonTimer,     ///< `"15.2s"` — last auton elapsed time (set by `autonomous()`).
  BatteryPct,     ///< `"87%"` — main battery capacity.
  OdomX,          ///< `"X24.5"` — current odom X (inches).
  OdomY,          ///< `"Y12.0"` — current odom Y (inches).
  OdomTheta,      ///< `"T90"` — current odom heading (degrees).
};

#define UI_CTRL_LINE 1          ///< V5 controller line, 0..2.
#define UI_CTRL_REFRESH_MS 500  ///< Controller serial is slow; keep ≥ 500.
#define UI_CTRL_SLOT_LEFT CtrlSlot::MaxMotorTempC ///< Left slot content.
#define UI_CTRL_SLOT_MID CtrlSlot::AutonTimer     ///< Middle slot content.
#define UI_CTRL_SLOT_RIGHT CtrlSlot::None         ///< Right slot content.
/** @} */
