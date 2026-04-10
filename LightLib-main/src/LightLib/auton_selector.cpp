#include "auton_selector.hpp"
#include "pid_tuner.hpp"

LV_IMG_DECLARE(Balls);
LV_IMG_DECLARE(radiant_scroll_banner);

// ─── Layout ──────────────────────────────────────────────────────────────────
static constexpr int SCREEN_W   = 480;
static constexpr int SCREEN_H   = 240;
static constexpr int PANEL_W    = 290;
static constexpr int PREVIEW_W  = SCREEN_W - PANEL_W;
static constexpr int HEADER_H   = 36;
static constexpr int BTN_COLS   = 2;
static constexpr int BTN_PAD    = 6;
static constexpr int BTN_H      = 42;

// compact PID panel
static constexpr double      PID_STEP[4]      = { 0.1, 0.001, 0.01, 0.5 };
static constexpr const char* PID_SLOT_NAMES[4]= { "kP", "kI", "kD", "si" };
static constexpr const char* PID_TAB_NAMES[4] = { "Drive", "Turn", "Swing", "Hdng" };

#define COL_BG        lv_color_make(0x68, 0x46, 0x8F)
#define COL_PANEL     lv_color_make(0x50, 0x35, 0x70)
#define COL_ACCENT    lv_color_make(0xFF, 0xDF, 0x61)
#define COL_ACCENT2   lv_color_make(0xCC, 0xAA, 0x30)
#define COL_BTN_IDLE  lv_color_make(0x55, 0x38, 0x78)
#define COL_BORDER    lv_color_make(0xFF, 0xDF, 0x61)
#define COL_TEXT      lv_color_make(0xFF, 0xDF, 0x61)
#define COL_TEXT_DIM  lv_color_make(0xCC, 0xAA, 0x30)
#define COL_TEXT_SEL  lv_color_make(0x3A, 0x20, 0x55)
#define COL_YELLOW    lv_color_make(0xFF, 0xDF, 0x61)
#define COL_RED       lv_color_make(0xFF, 0x44, 0x44)

// pack pid-type (0-3), slot (0-3), sign (+/-1) into a pointer-sized cookie
static void* sel_pack(int pid, int slot, int sign) {
    return (void*)(intptr_t)(((pid & 3) << 3) | ((slot & 3) << 1) | (sign > 0 ? 1 : 0));
}
static void sel_unpack(void* ud, int& pid, int& slot, int& sign) {
    intptr_t v = (intptr_t)ud;
    sign = (v & 1) ? +1 : -1;
    slot = (v >> 1) & 3;
    pid  = (v >> 3) & 3;
}

namespace light {

AutonSelector auton_selector;

static lv_style_t s_btn_idle, s_btn_sel;

// ─── Public ──────────────────────────────────────────────────────────────────

void AutonSelector::add(const std::string& name,
                        const std::string& desc,
                        std::function<void()> fn) {
    autons_.push_back({name, desc, std::move(fn), nullptr});
}

void AutonSelector::add(const std::string& name,
                        const std::string& desc,
                        std::function<void()> fn,
                        const lv_img_dsc_t* banner) {
    autons_.push_back({name, desc, std::move(fn), banner});
}

void AutonSelector::init() {
    if (autons_.empty()) return;
    build_ui();
    select(0);
}

void AutonSelector::run() {
    if (selected_idx_ < 0 || selected_idx_ >= (int)autons_.size()) return;
    if (!run_screen_) build_run_screen();
    lv_scr_load(run_screen_);
    // Show layer-top strip labels
    lv_obj_clear_flag(run_back_lbl_,   LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(run_toggle_lbl_, LV_OBJ_FLAG_HIDDEN);
    start_run_anim();   // replay animation every time
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
    lv_label_set_text(title, LV_SYMBOL_LIST "  Auton Selector");
    lv_obj_set_style_text_color(title, COL_TEXT_SEL, 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 10, 0);

    // Toggle button (Preview ↔ PID Tune)
    lv_obj_t* toggle_btn = lv_btn_create(header);
    lv_obj_set_size(toggle_btn, 100, HEADER_H - 10);
    lv_obj_align(toggle_btn, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_set_style_bg_color(toggle_btn, lv_color_make(0x50, 0x35, 0x70), 0);
    lv_obj_set_style_bg_opa(toggle_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(toggle_btn, COL_ACCENT, 0);
    lv_obj_set_style_border_width(toggle_btn, 1, 0);
    lv_obj_set_style_radius(toggle_btn, 4, 0);
    lv_obj_set_style_bg_color(toggle_btn, lv_color_make(0x20, 0x20, 0x28), LV_STATE_PRESSED);
    lv_obj_add_event_cb(toggle_btn, toggle_cb, LV_EVENT_CLICKED, this);

    toggle_lbl_ = lv_label_create(toggle_btn);
    lv_label_set_text(toggle_lbl_, LV_SYMBOL_SETTINGS " PID Tune");
    lv_obj_set_style_text_color(toggle_lbl_, COL_ACCENT, 0);
    lv_obj_center(toggle_lbl_);

    // Right panel container
    int rp_x = PANEL_W + 4;
    int rp_w = PREVIEW_W - 8;
    int rp_h = SCREEN_H - HEADER_H - 8;
    int rp_y = HEADER_H + 4;

    lv_obj_t* right_panel = lv_obj_create(screen_);
    lv_obj_set_size(right_panel, rp_w, rp_h);
    lv_obj_set_pos(right_panel, rp_x, rp_y);
    lv_obj_set_style_bg_color(right_panel, COL_PANEL, 0);
    lv_obj_set_style_bg_opa(right_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(right_panel, COL_ACCENT, 0);
    lv_obj_set_style_border_width(right_panel, 1, 0);
    lv_obj_set_style_radius(right_panel, 4, 0);
    lv_obj_set_style_pad_all(right_panel, 0, 0);

    // Build preview and PID sub-panels (both created, one hidden)
    preview_cont_ = lv_obj_create(right_panel);
    lv_obj_set_size(preview_cont_, rp_w, rp_h);
    lv_obj_set_pos(preview_cont_, 0, 0);
    lv_obj_set_style_bg_opa(preview_cont_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(preview_cont_, 0, 0);
    lv_obj_set_style_pad_all(preview_cont_, 0, 0);
    build_right_preview(preview_cont_);

    pid_cont_ = lv_obj_create(right_panel);
    lv_obj_set_size(pid_cont_, rp_w, rp_h);
    lv_obj_set_pos(pid_cont_, 0, 0);
    lv_obj_set_style_bg_opa(pid_cont_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(pid_cont_, 0, 0);
    lv_obj_set_style_pad_all(pid_cont_, 0, 0);
    build_right_pid(pid_cont_);
    lv_obj_add_flag(pid_cont_, LV_OBJ_FLAG_HIDDEN);

    // Button grid (left panel) — scrollable container
    int avail_w  = PANEL_W - BTN_PAD * 2;
    int btn_w    = (avail_w - BTN_PAD * (BTN_COLS - 1)) / BTN_COLS;
    int rows     = ((int)autons_.size() + BTN_COLS - 1) / BTN_COLS;
    int content_h = rows * (BTN_H + BTN_PAD) + BTN_PAD;
    int panel_h   = SCREEN_H - HEADER_H;

    lv_obj_t* scroll_cont = lv_obj_create(screen_);
    lv_obj_set_size(scroll_cont, PANEL_W, panel_h);
    lv_obj_set_pos(scroll_cont, 0, HEADER_H);
    lv_obj_set_style_bg_opa(scroll_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(scroll_cont, 0, 0);
    lv_obj_set_style_pad_all(scroll_cont, 0, 0);
    lv_obj_set_style_radius(scroll_cont, 0, 0);
    // Only enable scrolling when content exceeds panel height
    if (content_h > panel_h) {
        lv_obj_add_flag(scroll_cont, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(scroll_cont, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(scroll_cont, LV_SCROLLBAR_MODE_ACTIVE);
        lv_obj_set_style_width(scroll_cont, 4, LV_PART_SCROLLBAR);
        lv_obj_set_style_bg_color(scroll_cont, COL_ACCENT, LV_PART_SCROLLBAR);
        lv_obj_set_style_bg_opa(scroll_cont, LV_OPA_COVER, LV_PART_SCROLLBAR);
    } else {
        lv_obj_clear_flag(scroll_cont, LV_OBJ_FLAG_SCROLLABLE);
    }

    for (int i = 0; i < (int)autons_.size(); i++) {
        int col = i % BTN_COLS;
        int row = i / BTN_COLS;
        int x   = BTN_PAD + col * (btn_w + BTN_PAD);
        int y   = BTN_PAD + row * (BTN_H  + BTN_PAD);

        lv_obj_t* btn = lv_btn_create(scroll_cont);
        lv_obj_set_size(btn, btn_w, BTN_H);
        lv_obj_set_pos(btn, x, y);
        lv_obj_add_style(btn, &s_btn_idle, 0);
        lv_obj_add_style(btn, &s_btn_sel,  LV_STATE_CHECKED);
        lv_obj_set_user_data(btn, (void*)(intptr_t)i);
        lv_obj_add_event_cb(btn, btn_cb, LV_EVENT_CLICKED, NULL);

        if (autons_[i].banner) {
            int cw = btn_w - 4, ch = BTN_H - 4;
            lv_obj_t* clip = lv_obj_create(btn);
            lv_obj_set_size(clip, cw, ch);
            lv_obj_center(clip);
            lv_obj_clear_flag(clip, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_bg_opa(clip, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(clip, 0, 0);
            lv_obj_set_style_pad_all(clip, 0, 0);
            lv_obj_set_style_radius(clip, 0, 0);

            int iw = (int)autons_[i].banner->header.w;
            int ih = (int)autons_[i].banner->header.h;

            lv_obj_t* img = lv_img_create(clip);
            lv_img_set_src(img, autons_[i].banner);
            lv_obj_set_y(img, (ch - ih) / 2);
            lv_obj_set_x(img, -iw);

            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, img);
            lv_anim_set_exec_cb(&a, [](void* obj, int32_t v) {
                lv_obj_set_x((lv_obj_t*)obj, v);
            });
            lv_anim_set_values(&a, -iw, cw);
            lv_anim_set_time(&a, (iw + cw) * 7);
            lv_anim_set_delay(&a, i * 400);
            lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
            lv_anim_set_repeat_delay(&a, 800);
            lv_anim_start(&a);
        } else {
            lv_obj_t* lbl = lv_label_create(btn);
            lv_label_set_text(lbl, autons_[i].name.c_str());
            lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
            lv_obj_set_width(lbl, btn_w - 8);
            lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_center(lbl);
        }

        btn_objs_.push_back(btn);
    }

    lv_scr_load(screen_);
}

void AutonSelector::build_right_preview(lv_obj_t* parent) {
    img_obj_ = lv_img_create(parent);
    lv_img_set_src(img_obj_, &Balls);
    lv_img_set_angle(img_obj_, 2700);
    // zoom to fill preview panel (~182×196): scale height 196/136*256 ≈ 369
    lv_img_set_zoom(img_obj_, 244);
    lv_obj_align(img_obj_, LV_ALIGN_CENTER, 0, 0);
}

void AutonSelector::build_right_pid(lv_obj_t* parent) {
    int pw = PREVIEW_W - 8;  // panel width
    int ph = SCREEN_H - HEADER_H - 8;  // panel height

    // PID type tabs
    int tab_h = 22;
    int tab_w = pw / 4;
    for (int i = 0; i < 4; i++) {
        lv_obj_t* tb = lv_btn_create(parent);
        lv_obj_set_size(tb, tab_w - 2, tab_h - 2);
        lv_obj_set_pos(tb, i * tab_w + 1, 2);
        lv_obj_set_style_bg_color(tb, COL_BTN_IDLE, 0);
        lv_obj_set_style_bg_color(tb, COL_ACCENT, LV_STATE_CHECKED);
        lv_obj_set_style_bg_opa(tb, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(tb, COL_BORDER, 0);
        lv_obj_set_style_border_width(tb, 1, 0);
        lv_obj_set_style_radius(tb, 3, 0);
        lv_obj_set_user_data(tb, (void*)(intptr_t)i);
        lv_obj_add_event_cb(tb, pid_tab_cb, LV_EVENT_CLICKED, this);
        pid_tab_btns_[i] = tb;

        lv_obj_t* lbl = lv_label_create(tb);
        lv_label_set_text(lbl, PID_TAB_NAMES[i]);
        lv_obj_set_style_text_color(lbl, COL_TEXT_DIM, 0);
        lv_obj_set_style_text_color(lbl, COL_TEXT_SEL, LV_STATE_CHECKED);
        lv_obj_center(lbl);
    }

    // 4 constant rows
    int row_y_start = tab_h + 4;
    int apply_h     = 26;
    int row_h       = (ph - row_y_start - apply_h - 8) / 4;

    for (int s = 0; s < 4; s++) {
        int ry = row_y_start + s * row_h;

        // Slot name label
        lv_obj_t* name_l = lv_label_create(parent);
        lv_label_set_text(name_l, PID_SLOT_NAMES[s]);
        lv_obj_set_style_text_color(name_l, COL_TEXT_DIM, 0);
        lv_obj_set_pos(name_l, 4, ry + row_h / 2 - 7);

        // Dec button (iterates over all 4 pid types via active_pid_tab_)
        lv_obj_t* dec = lv_btn_create(parent);
        lv_obj_set_size(dec, 22, row_h - 4);
        lv_obj_set_pos(dec, 28, ry + 2);
        lv_obj_set_style_bg_color(dec, COL_BTN_IDLE, 0);
        lv_obj_set_style_bg_color(dec, COL_RED, LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(dec, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dec, 0, 0);
        lv_obj_set_style_radius(dec, 3, 0);
        lv_obj_set_user_data(dec, sel_pack(0, s, -1));  // pid filled in at runtime via active_pid_tab_
        lv_obj_add_event_cb(dec, pid_adj_cb, LV_EVENT_CLICKED, this);
        lv_obj_t* dl = lv_label_create(dec);
        lv_label_set_text(dl, LV_SYMBOL_MINUS);
        lv_obj_set_style_text_color(dl, COL_TEXT, 0);
        lv_obj_center(dl);

        // Value label (separate per pid type)
        for (int p = 0; p < 4; p++) {
            pid_val_labels_[p][s] = lv_label_create(parent);
            lv_obj_set_style_text_color(pid_val_labels_[p][s], COL_ACCENT, 0);
            lv_obj_set_pos(pid_val_labels_[p][s], 54, ry + row_h / 2 - 7);
            lv_label_set_text(pid_val_labels_[p][s], "0.000");
            // hide all except active tab
            if (p != 0) lv_obj_add_flag(pid_val_labels_[p][s], LV_OBJ_FLAG_HIDDEN);
        }

        // Inc button
        lv_obj_t* inc = lv_btn_create(parent);
        lv_obj_set_size(inc, 22, row_h - 4);
        lv_obj_set_pos(inc, pw - 26, ry + 2);
        lv_obj_set_style_bg_color(inc, COL_BTN_IDLE, 0);
        lv_obj_set_style_bg_color(inc, COL_ACCENT, LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(inc, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(inc, 0, 0);
        lv_obj_set_style_radius(inc, 3, 0);
        lv_obj_set_user_data(inc, sel_pack(0, s, +1));
        lv_obj_add_event_cb(inc, pid_adj_cb, LV_EVENT_CLICKED, this);
        lv_obj_t* il = lv_label_create(inc);
        lv_label_set_text(il, LV_SYMBOL_PLUS);
        lv_obj_set_style_text_color(il, COL_TEXT, 0);
        lv_obj_center(il);
    }

    // Apply button
    lv_obj_t* apply = lv_btn_create(parent);
    lv_obj_set_size(apply, pw - 8, apply_h);
    lv_obj_set_pos(apply, 4, ph - apply_h - 4);
    lv_obj_set_style_bg_color(apply, COL_ACCENT, 0);
    lv_obj_set_style_bg_grad_color(apply, COL_ACCENT2, 0);
    lv_obj_set_style_bg_grad_dir(apply, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(apply, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(apply, COL_BTN_IDLE, LV_STATE_PRESSED);
    lv_obj_set_style_border_width(apply, 0, 0);
    lv_obj_set_style_radius(apply, 4, 0);
    lv_obj_add_event_cb(apply, pid_apply_cb, LV_EVENT_CLICKED, this);
    lv_obj_t* al = lv_label_create(apply);
    lv_label_set_text(al, LV_SYMBOL_OK "  Apply to EZ");
    lv_obj_set_style_text_color(al, COL_TEXT_SEL, 0);
    lv_obj_center(al);

    // Activate first tab visually
    select_pid_tab(0);
}

// ─── Panel switching ─────────────────────────────────────────────────────────

void AutonSelector::switch_panel(bool to_pid) {
    show_pid_ = to_pid;
    if (to_pid) {
        init_pid_vals();
        refresh_pid_labels();
        lv_obj_clear_flag(pid_cont_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(preview_cont_, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(toggle_lbl_, LV_SYMBOL_IMAGE "  Preview");
    } else {
        lv_obj_clear_flag(preview_cont_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(pid_cont_, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(toggle_lbl_, LV_SYMBOL_SETTINGS " PID Tune");
    }
}

void AutonSelector::select_pid_tab(int idx) {
    active_pid_tab_ = idx;
    for (int i = 0; i < 4; i++) {
        if (i == idx) lv_obj_add_state(pid_tab_btns_[i],   LV_STATE_CHECKED);
        else          lv_obj_clear_state(pid_tab_btns_[i], LV_STATE_CHECKED);
        for (int s = 0; s < 4; s++) {
            if (pid_val_labels_[i][s]) {
                if (i == idx) lv_obj_clear_flag(pid_val_labels_[i][s], LV_OBJ_FLAG_HIDDEN);
                else          lv_obj_add_flag(pid_val_labels_[i][s],   LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

void AutonSelector::refresh_pid_labels() {
    for (int p = 0; p < 4; p++) {
        for (int s = 0; s < 4; s++) {
            if (!pid_val_labels_[p][s]) continue;
            char buf[12];
            snprintf(buf, sizeof(buf), "%.3f", pid_vals_[p][s]);
            lv_label_set_text(pid_val_labels_[p][s], buf);
        }
    }
}

void AutonSelector::init_pid_vals() {
    auto* drive = pid_tuner.get_drive();
    if (!drive) return;
    auto load = [this](int t, ez::PID::Constants c) {
        pid_vals_[t][0] = c.kp;
        pid_vals_[t][1] = c.ki;
        pid_vals_[t][2] = c.kd;
        pid_vals_[t][3] = c.start_i;
    };
    load(0, drive->pid_drive_constants_get());
    load(1, drive->pid_turn_constants_get());
    load(2, drive->pid_swing_constants_get());
    load(3, drive->pid_heading_constants_get());
}

void AutonSelector::apply_pid_vals() {
    auto* drive = pid_tuner.get_drive();
    if (!drive) return;
    auto& v = pid_vals_[active_pid_tab_];
    switch (active_pid_tab_) {
        case 0: drive->pid_drive_constants_set(v[0], v[1], v[2], v[3]);   break;
        case 1: drive->pid_turn_constants_set(v[0], v[1], v[2], v[3]);    break;
        case 2: drive->pid_swing_constants_set(v[0], v[1], v[2], v[3]);   break;
        case 3: drive->pid_heading_constants_set(v[0], v[1], v[2], v[3]); break;
        default: break;
    }
}

// ─── Selection ───────────────────────────────────────────────────────────────

void AutonSelector::select(int idx) {
    if (idx < 0 || idx >= (int)autons_.size()) return;
    for (int i = 0; i < (int)btn_objs_.size(); i++) {
        if (i == idx) lv_obj_add_state(btn_objs_[i],   LV_STATE_CHECKED);
        else          lv_obj_clear_state(btn_objs_[i], LV_STATE_CHECKED);
    }
    selected_idx_ = idx;
}

// ─── Run screen (shown during + after autonomous) ────────────────────────────

void AutonSelector::build_run_screen() {
    run_screen_ = lv_obj_create(NULL);
    lv_obj_set_size(run_screen_, SCREEN_W, SCREEN_H);
    lv_obj_set_style_bg_color(run_screen_, COL_BG, 0);
    lv_obj_set_style_bg_opa(run_screen_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(run_screen_, 0, 0);
    lv_obj_set_style_pad_all(run_screen_, 0, 0);

    // ── Image view ───────────────────────────────────────────────────────────
    run_img_cont_ = lv_obj_create(run_screen_);
    lv_obj_set_size(run_img_cont_, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(run_img_cont_, 0, 0);
    lv_obj_set_style_bg_opa(run_img_cont_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(run_img_cont_, 0, 0);
    lv_obj_set_style_pad_all(run_img_cont_, 0, 0);

    run_img_ = lv_img_create(run_img_cont_);
    lv_img_set_src(run_img_, &Balls);
    lv_img_set_angle(run_img_, 2700);
    lv_img_set_zoom(run_img_, 244);
    lv_obj_align(run_img_, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_translate_x(run_img_, 143, 0);
    lv_obj_set_style_translate_y(run_img_, 18,  0);

    // ── Info view ────────────────────────────────────────────────────────────
    run_info_cont_ = lv_obj_create(run_screen_);
    lv_obj_set_size(run_info_cont_, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(run_info_cont_, 0, 0);
    lv_obj_set_style_bg_opa(run_info_cont_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(run_info_cont_, 0, 0);
    lv_obj_set_style_pad_all(run_info_cont_, 0, 0);

    const char* auton_name = (selected_idx_ >= 0 && selected_idx_ < (int)autons_.size())
                             ? autons_[selected_idx_].name.c_str() : "—";
    const char* auton_desc = (selected_idx_ >= 0 && selected_idx_ < (int)autons_.size())
                             ? autons_[selected_idx_].description.c_str() : "";

    lv_obj_t* name_lbl = lv_label_create(run_info_cont_);
    lv_label_set_text(name_lbl, auton_name);
    lv_obj_set_style_text_color(name_lbl, COL_ACCENT, 0);
    lv_obj_align(name_lbl, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t* desc_lbl = lv_label_create(run_info_cont_);
    lv_label_set_text(desc_lbl, auton_desc);
    lv_obj_set_style_text_color(desc_lbl, COL_TEXT_DIM, 0);
    lv_label_set_long_mode(desc_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(desc_lbl, SCREEN_W - 32);
    lv_obj_set_style_text_align(desc_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(desc_lbl, LV_ALIGN_CENTER, 0, 20);
    lv_obj_add_flag(run_info_cont_, LV_OBJ_FLAG_HIDDEN);

    // ── Side strips ──────────────────────────────────────────────────────────
    // Image at zoom 452 is 337px wide → dead space = (480-337)/2 = 71px each side
    static constexpr int DEAD_W = 71;

    // Strip button — dark background with inner accent border, no children
    auto make_strip_btn = [&](int x, lv_event_cb_t cb, bool border_right) {
        lv_obj_t* btn = lv_btn_create(run_screen_);
        lv_obj_set_size(btn, DEAD_W, SCREEN_H);
        lv_obj_set_pos(btn, x, 0);
        lv_obj_set_style_bg_color(btn, lv_color_make(0x50, 0x35, 0x70), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(btn, COL_ACCENT, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_side(btn,
            border_right ? LV_BORDER_SIDE_RIGHT : LV_BORDER_SIDE_LEFT, 0);
        lv_obj_set_style_radius(btn, 0, 0);
        lv_obj_set_style_pad_all(btn, 0, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, this);

        // Scrolling radiant banner inside the strip
        int iw = (int)radiant_scroll_banner.header.w;
        int ih = (int)radiant_scroll_banner.header.h;
        lv_obj_t* clip = lv_obj_create(btn);
        lv_obj_set_size(clip, DEAD_W, SCREEN_H);
        lv_obj_set_pos(clip, 0, 0);
        lv_obj_clear_flag(clip, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(clip, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_opa(clip, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(clip, 0, 0);
        lv_obj_set_style_pad_all(clip, 0, 0);
        lv_obj_set_style_radius(clip, 0, 0);
        // Scale so banner height fills the strip width
        int zoom   = (DEAD_W * 256) / ih;
        int eff_iw = iw * zoom / 256;   // scaled scroll length
        int x_img  = DEAD_W / 2 - iw / 2;   // center pivot in strip
        // start_y: image bottom (pivot + eff_iw/2) sits at clip bottom (SCREEN_H)
        // end_y = start_y + eff_iw: image top (pivot - eff_iw/2) reaches clip bottom → exits
        // Travel = exactly eff_iw → the two copies hand off seamlessly with no gap.
        int start_y = SCREEN_H - (eff_iw / 2 + ih / 2);

        // Two images offset by eff_iw — one always covers the strip while the
        // other resets off-screen, producing a seamless endless loop.
        auto add_scroll_img = [&](int y_start) {
            lv_obj_t* img = lv_img_create(clip);
            lv_img_set_src(img, &radiant_scroll_banner);
            lv_img_set_angle(img, 2700);
            lv_img_set_pivot(img, iw / 2, ih / 2);
            lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
            lv_img_set_zoom(img, zoom);
            lv_obj_set_x(img, x_img);
            lv_obj_set_y(img, y_start);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, img);
            lv_anim_set_exec_cb(&a, [](void* obj, int32_t v) {
                lv_obj_set_y((lv_obj_t*)obj, v);
            });
            lv_anim_set_values(&a, y_start, y_start + eff_iw);
            lv_anim_set_time(&a, eff_iw * 7);
            lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
            lv_anim_set_repeat_delay(&a, 0);
            lv_anim_start(&a);
        };

        add_scroll_img(start_y);           // visible at start, scrolls through strip
        add_scroll_img(start_y - eff_iw);  // one banner-width behind, fills the gap
    };

    // Strip label — child of lv_layer_top() (never clipped by run_screen_).
    // Unrotated size: 60×18. After 270° rotation it appears as 18×60 (fits in 71px strip).
    // Both strips have their unrotated rect fully within 0…480 bounds:
    //   Left  strip centre x=35  → rect x=5,  width=60 → 5…65   ✓
    //   Right strip centre x=444 → rect x=414, width=60 → 414…474 ✓
    auto make_strip_lbl = [&](int strip_x, const char* text) -> lv_obj_t* {
        static constexpr int LW = 60, LH = 18;
        int cx = strip_x + DEAD_W / 2;
        lv_obj_t* lbl = lv_label_create(lv_layer_top());
        lv_label_set_text(lbl, text);
        lv_obj_set_size(lbl, LW, LH);
        lv_obj_set_pos(lbl, cx - LW / 2, SCREEN_H / 2 - LH / 2);
        lv_obj_set_style_text_color(lbl, COL_ACCENT, 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_bg_opa(lbl, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(lbl, 0, 0);
        lv_obj_set_style_pad_all(lbl, 0, 0);
        lv_obj_set_style_transform_angle(lbl, 2700, 0);
        lv_obj_set_style_transform_pivot_x(lbl, LW / 2, 0);
        lv_obj_set_style_transform_pivot_y(lbl, LH / 2, 0);
        lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
        return lbl;
    };

    make_strip_btn(0, run_back_cb, true);
    run_back_lbl_   = make_strip_lbl(0, "BACK");

    make_strip_btn(SCREEN_W - DEAD_W, run_toggle_cb, false);
    run_toggle_lbl_ = make_strip_lbl(SCREEN_W - DEAD_W, "INFO");

    run_show_img_ = true;
}

void AutonSelector::start_run_anim() {
    // Reset to start position
    lv_img_set_zoom(run_img_, 244);
    lv_obj_set_style_translate_x(run_img_, 143, 0);
    lv_obj_set_style_translate_y(run_img_, 18,  0);

    lv_anim_t az;
    lv_anim_init(&az);
    lv_anim_set_var(&az, run_img_);
    lv_anim_set_exec_cb(&az, [](void* obj, int32_t v) {
        lv_img_set_zoom((lv_obj_t*)obj, (uint16_t)v);
    });
    lv_anim_set_values(&az, 244, 452);
    lv_anim_set_time(&az, 550);
    lv_anim_set_path_cb(&az, lv_anim_path_ease_out);
    lv_anim_start(&az);

    lv_anim_t ax;
    lv_anim_init(&ax);
    lv_anim_set_var(&ax, run_img_);
    lv_anim_set_exec_cb(&ax, [](void* obj, int32_t v) {
        lv_obj_set_style_translate_x((lv_obj_t*)obj, v, 0);
    });
    lv_anim_set_values(&ax, 143, 0);
    lv_anim_set_time(&ax, 550);
    lv_anim_set_path_cb(&ax, lv_anim_path_ease_out);
    lv_anim_start(&ax);

    lv_anim_t ay;
    lv_anim_init(&ay);
    lv_anim_set_var(&ay, run_img_);
    lv_anim_set_exec_cb(&ay, [](void* obj, int32_t v) {
        lv_obj_set_style_translate_y((lv_obj_t*)obj, v, 0);
    });
    lv_anim_set_values(&ay, 18, 0);
    lv_anim_set_time(&ay, 550);
    lv_anim_set_path_cb(&ay, lv_anim_path_ease_out);
    lv_anim_start(&ay);
}

void AutonSelector::switch_run_view(bool show_img) {
    run_show_img_ = show_img;
    if (show_img) {
        lv_obj_clear_flag(run_img_cont_,  LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(run_info_cont_,   LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(run_toggle_lbl_, "INFO");
    } else {
        lv_obj_add_flag(run_img_cont_,    LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(run_info_cont_, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(run_toggle_lbl_, "IMG");
    }
}

void AutonSelector::run_toggle_cb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    self->switch_run_view(!self->run_show_img_);
}

void AutonSelector::run_back_cb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    lv_obj_add_flag(self->run_back_lbl_,   LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(self->run_toggle_lbl_, LV_OBJ_FLAG_HIDDEN);

    // Ready callback: switch to selector screen after the last anim finishes
    auto on_done = [](lv_anim_t* a) {
        auto* s = static_cast<AutonSelector*>(a->user_data);
        s->show();
    };

    // Reverse zoom: 452 → 244
    lv_anim_t az;
    lv_anim_init(&az);
    lv_anim_set_var(&az, self->run_img_);
    lv_anim_set_exec_cb(&az, [](void* obj, int32_t v) {
        lv_img_set_zoom((lv_obj_t*)obj, (uint16_t)v);
    });
    lv_anim_set_values(&az, 452, 244);
    lv_anim_set_time(&az, 450);
    lv_anim_set_path_cb(&az, lv_anim_path_ease_in);
    lv_anim_start(&az);

    // Reverse translate X: 0 → 143
    lv_anim_t ax;
    lv_anim_init(&ax);
    lv_anim_set_var(&ax, self->run_img_);
    lv_anim_set_exec_cb(&ax, [](void* obj, int32_t v) {
        lv_obj_set_style_translate_x((lv_obj_t*)obj, v, 0);
    });
    lv_anim_set_values(&ax, 0, 143);
    lv_anim_set_time(&ax, 450);
    lv_anim_set_path_cb(&ax, lv_anim_path_ease_in);
    lv_anim_start(&ax);

    // Reverse translate Y: 0 → 18 — switch screen when this one finishes
    lv_anim_t ay;
    lv_anim_init(&ay);
    lv_anim_set_var(&ay, self->run_img_);
    ay.user_data = self;
    lv_anim_set_exec_cb(&ay, [](void* obj, int32_t v) {
        lv_obj_set_style_translate_y((lv_obj_t*)obj, v, 0);
    });
    lv_anim_set_values(&ay, 0, 18);
    lv_anim_set_time(&ay, 450);
    lv_anim_set_path_cb(&ay, lv_anim_path_ease_in);
    lv_anim_set_ready_cb(&ay, on_done);
    lv_anim_start(&ay);
}

// ─── Callbacks ───────────────────────────────────────────────────────────────

void AutonSelector::btn_cb(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);
    int idx = (int)(intptr_t)lv_obj_get_user_data(btn);
    auton_selector.select(idx);

    // Show run screen preview with animation when auton is selected
    auto& self = auton_selector;
    if (!self.run_screen_) self.build_run_screen();
    // Always start on image view when selecting an auton
    self.switch_run_view(true);
    lv_scr_load(self.run_screen_);
    lv_obj_clear_flag(self.run_back_lbl_,   LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(self.run_toggle_lbl_, LV_OBJ_FLAG_HIDDEN);
    self.start_run_anim();
}

void AutonSelector::toggle_cb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    self->switch_panel(!self->show_pid_);
}

void AutonSelector::pid_tab_cb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    int idx = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    self->select_pid_tab(idx);
}

void AutonSelector::pid_adj_cb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    int pid_unused, slot, sign;
    sel_unpack(lv_obj_get_user_data(lv_event_get_target(e)), pid_unused, slot, sign);
    int pid = self->active_pid_tab_;
    double& v = self->pid_vals_[pid][slot];
    v += sign * PID_STEP[slot];
    if (v < 0.0) v = 0.0;
    char buf[12];
    snprintf(buf, sizeof(buf), "%.3f", v);
    lv_label_set_text(self->pid_val_labels_[pid][slot], buf);
}

void AutonSelector::pid_apply_cb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    self->apply_pid_vals();
}

} // namespace light
