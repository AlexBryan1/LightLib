// Override PROS's weak lvgl_init() hook.
// This file intentionally does NOT include main.h or screen.hpp — including
// screen.hpp would pull in the weak inline definition and cause a redefinition
// error. By keeping this TU clean, the linker replaces the weak symbol with
// our strong definition, which runs before PROS applies its blue material theme.
#include "liblvgl/lvgl.h"
#include "liblvgl/extra/themes/default/lv_theme_default.h"
#include "ui_config.hpp"

void lvgl_init() {
    lv_disp_t* disp = lv_disp_get_default();
    lv_theme_t* th = lv_theme_default_init(
        disp,
        COL_THEME_PRIMARY,
        COL_THEME_SECONDARY,
        UI_THEME_DARK,
        UI_DEFAULT_FONT);
    lv_disp_set_theme(disp, th);
}
