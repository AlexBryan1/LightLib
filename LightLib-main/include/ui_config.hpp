// ─── UI Config ───────────────────────────────────────────────────────────────
// Central place to tweak the on-brain UI look.
// Edit colors/font here → selector screen + LVGL theme both update.
//
// Covers:
//   - Color palette used by the auton selector (purple/gold theme)
//   - Default LVGL theme font
//   - Theme primary/secondary background colors
//
// NOTE: the live PID tuner (pid_tuner.cpp) has its own separate palette.

#pragma once

#include "liblvgl/lvgl.h"

// ─── Color palette ───────────────────────────────────────────────────────────
// 24-bit hex 0xRRGGBB. lv_color_hex() wraps at use sites; not constexpr so #defines.

#define COL_BG        lv_color_hex(0x68468F)  // screen background
#define COL_PANEL     lv_color_hex(0x503570)  // right info panel
#define COL_ACCENT    lv_color_hex(0xFFDF61)  // gold (headers, selected)
#define COL_ACCENT2   lv_color_hex(0xCCAA30)  // gold gradient bottom
#define COL_BTN_IDLE  lv_color_hex(0x553878)  // unselected button bg
#define COL_BORDER    lv_color_hex(0xFFDF61)  // border lines
#define COL_TEXT      lv_color_hex(0xFFDF61)  // primary text (gold)
#define COL_TEXT_DIM  lv_color_hex(0xCCAA30)  // dim text
#define COL_TEXT_SEL  lv_color_hex(0x3A2055)  // dark text on gold bg
#define COL_YELLOW    lv_color_hex(0xFFDF61)
#define COL_RED       lv_color_hex(0xFF4444)

// ─── LVGL theme ──────────────────────────────────────────────────────────────
// Applied in lvgl_theme.cpp before PROS applies its default blue theme.
#define COL_THEME_PRIMARY    lv_color_hex(0x0D0D10)
#define COL_THEME_SECONDARY  lv_color_hex(0x0D0D10)
#define UI_THEME_DARK        true

// Default UI font. Must be one LVGL has compiled in (see lv_conf.h).
// Options: lv_font_montserrat_14, _16, _18, _20, _22, _24, ...
#define UI_DEFAULT_FONT      (&lv_font_montserrat_20)

// ─── Animation timing (ms) ───────────────────────────────────────────────────
// Bigger = slower. Change here to retune feel of selector/run screens.
constexpr int UI_ANIM_RUN_ZOOM_IN_MS      = 550;  // zoom when opening run screen
constexpr int UI_ANIM_RUN_ZOOM_OUT_MS     = 450;  // zoom when pressing BACK
constexpr int UI_ANIM_BANNER_PX_MS        = 7;    // per-pixel scroll duration
constexpr int UI_ANIM_BANNER_STAGGER_MS   = 400;  // delay between buttons
constexpr int UI_ANIM_BANNER_REPEAT_DELAY_MS = 800;
constexpr int UI_ODOM_REFRESH_MS          = 100;  // odom X/Y/Angle label refresh

// ─── Controller screen slots ─────────────────────────────────────────────────
// One controller line, three text slots: LEFT | MID | RIGHT.
// Pick one CtrlSlot per position. Line width ≈19 chars; slots fit 6/5/5.
enum class CtrlSlot {
    None,            // empty slot
    MaxMotorTempC,   // "55C"   hottest drive/score motor
    AutonTimer,      // "15.2s" last auton elapsed time (set by autonomous())
    BatteryPct,      // "87%"   main battery capacity
    OdomX,           // "X24.5" current odom X (inches)
    OdomY,           // "Y12.0" current odom Y (inches)
    OdomTheta,       // "T90"   current odom heading (degrees)
};

#define UI_CTRL_LINE         1      // V5 controller line, 0..2
#define UI_CTRL_REFRESH_MS   500    // controller serial is slow; keep ≥500
#define UI_CTRL_SLOT_LEFT    CtrlSlot::MaxMotorTempC
#define UI_CTRL_SLOT_MID     CtrlSlot::AutonTimer
#define UI_CTRL_SLOT_RIGHT   CtrlSlot::None
