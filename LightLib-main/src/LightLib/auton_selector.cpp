#include "auton_selector.hpp"
#include "pid_tuner.hpp"

// ─── Layout ──────────────────────────────────────────────────────────────────
static constexpr int SCREEN_W  = 480;
static constexpr int SCREEN_H  = 240;
static constexpr int PANEL_W   = 290;
static constexpr int PREVIEW_W = SCREEN_W - PANEL_W;
static constexpr int HEADER_H  = 36;
static constexpr int BTN_COLS  = 2;
static constexpr int BTN_PAD   = 6;
static constexpr int BTN_H     = 42;

#define COL_BG        lv_color_make(0x0D, 0x0D, 0x10)
#define COL_PANEL     lv_color_make(0x16, 0x16, 0x1E)
#define COL_ACCENT    lv_color_make(0x00, 0xC8, 0x6E)
#define COL_ACCENT2   lv_color_make(0x00, 0x88, 0x50)
#define COL_BTN_IDLE  lv_color_make(0x22, 0x22, 0x2E)
#define COL_BORDER    lv_color_make(0x33, 0x33, 0x44)
#define COL_TEXT      lv_color_make(0xE8, 0xE8, 0xEE)
#define COL_TEXT_DIM  lv_color_make(0x88, 0x88, 0x99)
#define COL_TEXT_SEL  lv_color_make(0x08, 0x08, 0x10)

namespace light {

AutonSelector auton_selector;

static lv_style_t s_btn_idle, s_btn_sel;

// ─── Public ──────────────────────────────────────────────────────────────────

void AutonSelector::add(const std::string& name,
                        const std::string& desc,
                        std::function<void()> fn) {
    autons_.push_back({name, desc, std::move(fn)});
}

void AutonSelector::init() {
    if (autons_.empty()) return;
    build_ui();
    select(0);
}

void AutonSelector::run() {
    if (selected_idx_ >= 0 && selected_idx_ < (int)autons_.size())
        autons_[selected_idx_].fn();
}

void AutonSelector::show() {
    if (screen_) lv_scr_load(screen_);
}

// ─── UI build ────────────────────────────────────────────────────────────────

void AutonSelector::build_ui() {
    lv_style_init(&s_btn_idle);
    lv_style_set_bg_color(&s_btn_idle, COL_BTN_IDLE);
    lv_style_set_bg_opa(&s_btn_idle, LV_OPA_COVER);
    lv_style_set_border_color(&s_btn_idle, COL_BORDER);
    lv_style_set_border_width(&s_btn_idle, 1);
    lv_style_set_radius(&s_btn_idle, 5);
    lv_style_set_text_color(&s_btn_idle, COL_TEXT);

    lv_style_init(&s_btn_sel);
    lv_style_set_bg_color(&s_btn_sel, COL_ACCENT);
    lv_style_set_bg_grad_color(&s_btn_sel, COL_ACCENT2);
    lv_style_set_bg_grad_dir(&s_btn_sel, LV_GRAD_DIR_VER);
    lv_style_set_bg_opa(&s_btn_sel, LV_OPA_COVER);
    lv_style_set_border_color(&s_btn_sel, lv_color_white());
    lv_style_set_border_width(&s_btn_sel, 2);
    lv_style_set_radius(&s_btn_sel, 5);
    lv_style_set_text_color(&s_btn_sel, COL_TEXT_SEL);

    // Screen
    screen_ = lv_obj_create(NULL);
    lv_obj_set_size(screen_, SCREEN_W, SCREEN_H);
    lv_obj_set_style_bg_color(screen_, COL_BG, 0);
    lv_obj_set_style_bg_opa(screen_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screen_, 0, 0);
    lv_obj_set_style_pad_all(screen_, 0, 0);

    // Header
    lv_obj_t* header = lv_obj_create(screen_);
    lv_obj_set_size(header, SCREEN_W, HEADER_H);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_style_bg_color(header, COL_ACCENT, 0);
    lv_obj_set_style_bg_grad_color(header, COL_ACCENT2, 0);
    lv_obj_set_style_bg_grad_dir(header, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_radius(header, 0, 0);
    lv_obj_set_style_pad_all(header, 0, 0);

    lv_obj_t* title = lv_label_create(header);
    lv_label_set_text(title, LV_SYMBOL_LIST "  Cactus Auton Selector");
    lv_obj_set_style_text_color(title, COL_TEXT_SEL, 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 10, 0);

    // PID Tune button
    lv_obj_t* pid_btn = lv_btn_create(header);
    lv_obj_set_size(pid_btn, 88, HEADER_H - 10);
    lv_obj_align(pid_btn, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_set_style_bg_color(pid_btn, lv_color_make(0x08, 0x08, 0x10), 0);
    lv_obj_set_style_bg_opa(pid_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(pid_btn, COL_ACCENT, 0);
    lv_obj_set_style_border_width(pid_btn, 1, 0);
    lv_obj_set_style_radius(pid_btn, 4, 0);
    lv_obj_set_style_bg_color(pid_btn, lv_color_make(0x20, 0x20, 0x28), LV_STATE_PRESSED);
    lv_obj_add_event_cb(pid_btn, [](lv_event_t*) { pid_tuner.open(); },
                        LV_EVENT_CLICKED, NULL);
    lv_obj_t* pid_lbl = lv_label_create(pid_btn);
    lv_label_set_text(pid_lbl, LV_SYMBOL_SETTINGS " PID Tune");
    lv_obj_set_style_text_color(pid_lbl, COL_ACCENT, 0);
    lv_obj_center(pid_lbl);

    // Right preview panel
    lv_obj_t* preview = lv_obj_create(screen_);
    lv_obj_set_size(preview, PREVIEW_W - 10, SCREEN_H - HEADER_H - 8);
    lv_obj_set_pos(preview, PANEL_W + 4, HEADER_H + 4);
    lv_obj_set_style_bg_color(preview, COL_PANEL, 0);
    lv_obj_set_style_bg_opa(preview, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(preview, COL_ACCENT, 0);
    lv_obj_set_style_border_width(preview, 1, 0);
    lv_obj_set_style_radius(preview, 4, 0);
    lv_obj_set_style_pad_all(preview, 4, 0);

    img_obj_ = lv_img_create(preview);
    lv_img_set_src(img_obj_, &Web_Photo_Editor);
    lv_obj_align(img_obj_, LV_ALIGN_TOP_MID, 0, 4);

    desc_label_ = lv_label_create(preview);
    lv_label_set_long_mode(desc_label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(desc_label_, PREVIEW_W - 18);
    lv_obj_set_style_text_color(desc_label_, COL_TEXT_DIM, 0);
    lv_obj_set_style_text_align(desc_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(desc_label_, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_label_set_text(desc_label_, "");

    // Button grid
    int grid_top = HEADER_H + BTN_PAD;
    int avail_w  = PANEL_W - BTN_PAD * 2;
    int btn_w    = (avail_w - BTN_PAD * (BTN_COLS - 1)) / BTN_COLS;

    for (int i = 0; i < (int)autons_.size(); i++) {
        int col = i % BTN_COLS;
        int row = i / BTN_COLS;
        int x   = BTN_PAD + col * (btn_w + BTN_PAD);
        int y   = grid_top + row * (BTN_H  + BTN_PAD);

        lv_obj_t* btn = lv_btn_create(screen_);
        lv_obj_set_size(btn, btn_w, BTN_H);
        lv_obj_set_pos(btn, x, y);
        lv_obj_add_style(btn, &s_btn_idle, 0);
        lv_obj_add_style(btn, &s_btn_sel,  LV_STATE_CHECKED);
        lv_obj_set_user_data(btn, (void*)(intptr_t)i);
        lv_obj_add_event_cb(btn, btn_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, autons_[i].name.c_str());
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, btn_w - 8);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(lbl);

        btn_objs_.push_back(btn);
    }

    lv_scr_load(screen_);
}

// ─── Selection ───────────────────────────────────────────────────────────────

void AutonSelector::select(int idx) {
    if (idx < 0 || idx >= (int)autons_.size()) return;
    for (int i = 0; i < (int)btn_objs_.size(); i++) {
        if (i == idx) lv_obj_add_state(btn_objs_[i],   LV_STATE_CHECKED);
        else          lv_obj_clear_state(btn_objs_[i], LV_STATE_CHECKED);
    }
    lv_label_set_text(desc_label_, autons_[idx].description.c_str());
    selected_idx_ = idx;
}

void AutonSelector::btn_cb(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);
    int idx = (int)(intptr_t)lv_obj_get_user_data(btn);
    auton_selector.select(idx);
}

} // namespace cactus