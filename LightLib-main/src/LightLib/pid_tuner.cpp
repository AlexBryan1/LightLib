#include "pid_tuner.hpp"
#include "auton_selector.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include <cmath>

static constexpr int SW        = 480;
static constexpr int SH        = 240;
static constexpr int HDR_H     = 32;
static constexpr int TAB_H     = 26;
static constexpr int BODY_Y    = HDR_H + TAB_H;
static constexpr int BODY_H    = SH - BODY_Y;
static constexpr int GRAPH_W   = 266;
static constexpr int CTRL_X    = GRAPH_W + 6;
static constexpr int CTRL_W    = SW - CTRL_X - 4;
static constexpr int GRAPH_PTS = 100;

// ─── Noisy Boy palette ────────────────────────────────────────────────────────
#define C_BG       lv_color_make(0x0A, 0x06, 0x14)
#define C_PANEL    lv_color_make(0x14, 0x0C, 0x26)
#define C_YELLOW   lv_color_make(0xFF, 0xD0, 0x00)
#define C_YELLOW2  lv_color_make(0xCC, 0xA0, 0x00)
#define C_PURPLE   lv_color_make(0x5A, 0x00, 0xAA)
#define C_PURPLE2  lv_color_make(0x3A, 0x00, 0x7A)
#define C_IDLE     lv_color_make(0x1E, 0x10, 0x36)
#define C_BORDER   lv_color_make(0x44, 0x20, 0x66)
#define C_TEXT     lv_color_make(0xF0, 0xE8, 0xFF)
#define C_DIM      lv_color_make(0x88, 0x66, 0xAA)
#define C_DARK     lv_color_make(0x0A, 0x06, 0x14)
#define C_RED      lv_color_make(0xFF, 0x44, 0x44)
#define C_CYAN     lv_color_make(0x00, 0xCC, 0xFF)
#define C_ORANGE   lv_color_make(0xFF, 0x88, 0x00)

static constexpr double STEP[4]             = { 0.1, 0.001, 0.01, 0.5 };
static constexpr const char* SLOT_NAMES[4] = { "kP", "kI", "kD", "start_i" };
static constexpr const char* PID_NAMES[4]  = { "Drive", "Turn", "Swing", "Heading" };

static inline void* pack_ud(int slot, int sign) {
    return (void*)(intptr_t)(((slot & 0xF) << 1) | (sign > 0 ? 1 : 0));
}
static inline void unpack_ud(void* ud, int& slot, int& sign) {
    intptr_t v = (intptr_t)ud;
    slot = (v >> 1) & 0xF;
    sign = (v & 1) ? +1 : -1;
}

namespace light {

PidTuner pid_tuner;

void PidTuner::set_drive(ez::Drive* drive) {
    drive_ = drive;
    if (!drive_) return;
    auto load = [&](PidType t, ez::PID::Constants c) {
        constants_[t].val[KP]      = c.kp;
        constants_[t].val[KI]      = c.ki;
        constants_[t].val[KD]      = c.kd;
        constants_[t].val[START_I] = c.start_i;
    };
    // pid_drive_constants_get() / pid_swing_constants_get() return 0 when the
    // combined setter was used (they read forward_*PID which is zeroed by it).
    // Read directly from fwd_rev_*PID where the combined setter actually stores.
    load(PID_DRIVE,   drive_->fwd_rev_drivePID.constants_get());
    load(PID_TURN,    drive_->pid_turn_constants_get());
    load(PID_SWING,   drive_->fwd_rev_swingPID.constants_get());
    load(PID_HEADING, drive_->pid_heading_constants_get());
}

void PidTuner::start_task() {
    if (sample_task_) return;
    sample_task_ = new pros::Task(sample_task_fn, this,
                                  TASK_PRIORITY_DEFAULT - 1,
                                  TASK_STACK_DEPTH_DEFAULT, "pid_sampler");
}

void PidTuner::open() {
    build_ui();
    select_pid(PID_DRIVE);
    lv_scr_load(screen_);
}

void PidTuner::close() {
    recording_ = false;
    auton_selector.show();
}

void PidTuner::build_ui() {
    if (screen_) { lv_obj_del(screen_); screen_ = nullptr; }

    screen_ = lv_obj_create(NULL);
    lv_obj_set_size(screen_, SW, SH);
    lv_obj_set_style_bg_color(screen_, C_BG, 0);
    lv_obj_set_style_bg_opa(screen_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screen_, 0, 0);
    lv_obj_set_style_pad_all(screen_, 0, 0);

    // Header — deep purple gradient
    lv_obj_t* hdr = lv_obj_create(screen_);
    lv_obj_set_size(hdr, SW, HDR_H);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, C_PURPLE, 0);
    lv_obj_set_style_bg_grad_color(hdr, C_PURPLE2, 0);
    lv_obj_set_style_bg_grad_dir(hdr, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);

    // Back button
    lv_obj_t* back = lv_btn_create(hdr);
    lv_obj_set_size(back, 62, HDR_H - 8);
    lv_obj_align(back, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_set_style_bg_color(back, C_IDLE, 0);
    lv_obj_set_style_bg_color(back, C_RED, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(back, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(back, 4, 0);
    lv_obj_set_style_border_width(back, 0, 0);
    lv_obj_add_event_cb(back, back_cb, LV_EVENT_CLICKED, this);
    lv_obj_t* back_lbl = lv_label_create(back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(back_lbl, C_TEXT, 0);
    lv_obj_center(back_lbl);

    // Title
    lv_obj_t* title = lv_label_create(hdr);
    lv_label_set_text(title, "PID Tuner");
    lv_obj_set_style_text_color(title, C_YELLOW, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);

    // Record button
    lv_obj_t* rec = lv_btn_create(hdr);
    lv_obj_set_size(rec, 80, HDR_H - 8);
    lv_obj_align(rec, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_set_style_bg_color(rec, C_IDLE, 0);
    lv_obj_set_style_bg_color(rec, C_YELLOW, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(rec, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(rec, 4, 0);
    lv_obj_set_style_border_color(rec, C_YELLOW, 0);
    lv_obj_set_style_border_width(rec, 1, 0);
    lv_obj_add_event_cb(rec, record_cb, LV_EVENT_CLICKED, this);
    lv_obj_t* rec_lbl = lv_label_create(rec);
    lv_label_set_text(rec_lbl, LV_SYMBOL_PLAY " Record");
    lv_obj_set_style_text_color(rec_lbl, C_YELLOW, 0);
    lv_obj_center(rec_lbl);

    // PID type tabs
    int tab_w = (SW - 8) / PID_COUNT;
    for (int i = 0; i < PID_COUNT; i++) {
        lv_obj_t* tb = lv_btn_create(screen_);
        lv_obj_set_size(tb, tab_w - 2, TAB_H - 2);
        lv_obj_set_pos(tb, 4 + i * tab_w, HDR_H + 1);
        lv_obj_set_style_bg_color(tb, C_IDLE, 0);
        lv_obj_set_style_bg_color(tb, C_YELLOW, LV_STATE_CHECKED);
        lv_obj_set_style_bg_opa(tb, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(tb, C_BORDER, 0);
        lv_obj_set_style_border_width(tb, 1, 0);
        lv_obj_set_style_radius(tb, 3, 0);
        lv_obj_set_user_data(tb, (void*)(intptr_t)i);
        lv_obj_add_event_cb(tb, tab_cb, LV_EVENT_CLICKED, this);
        tab_btns_[i] = tb;

        lv_obj_t* lbl = lv_label_create(tb);
        lv_label_set_text(lbl, PID_NAMES[i]);
        lv_obj_set_style_text_color(lbl, C_DIM, 0);
        lv_obj_set_style_text_color(lbl, C_DARK, LV_STATE_CHECKED);
        lv_obj_center(lbl);
    }

    // Graph panel
    lv_obj_t* graph_panel = lv_obj_create(screen_);
    lv_obj_set_size(graph_panel, GRAPH_W, BODY_H - 8);
    lv_obj_set_pos(graph_panel, 4, BODY_Y + 4);
    lv_obj_set_style_bg_color(graph_panel, C_PANEL, 0);
    lv_obj_set_style_bg_opa(graph_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(graph_panel, C_BORDER, 0);
    lv_obj_set_style_border_width(graph_panel, 1, 0);
    lv_obj_set_style_radius(graph_panel, 4, 0);
    lv_obj_set_style_pad_all(graph_panel, 4, 0);

    auto make_legend = [&](lv_obj_t* parent, const char* txt, lv_color_t col, int x) {
        lv_obj_t* l = lv_label_create(parent);
        lv_label_set_text(l, txt);
        lv_obj_set_style_text_color(l, col, 0);
        lv_obj_set_pos(l, x, 2);
    };
    make_legend(graph_panel, "● L vel", C_YELLOW, 2);
    make_legend(graph_panel, "● R vel", C_CYAN,   60);
    make_legend(graph_panel, "● Error", C_ORANGE, 118);

    chart_ = lv_chart_create(graph_panel);
    lv_obj_set_size(chart_, GRAPH_W - 12, BODY_H - 40);
    lv_obj_set_pos(chart_, 2, 18);
    lv_obj_set_style_bg_color(chart_, lv_color_make(0x08, 0x04, 0x10), 0);
    lv_obj_set_style_bg_opa(chart_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(chart_, C_BORDER, 0);
    lv_obj_set_style_border_width(chart_, 1, 0);
    lv_obj_set_style_radius(chart_, 3, 0);
    lv_chart_set_type(chart_, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart_, GRAPH_PTS);
    lv_chart_set_range(chart_, LV_CHART_AXIS_PRIMARY_Y, -200, 200);
    lv_chart_set_div_line_count(chart_, 4, 5);
    lv_obj_set_style_line_width(chart_, 0, LV_PART_ITEMS);

    ser_left_  = lv_chart_add_series(chart_, C_YELLOW, LV_CHART_AXIS_PRIMARY_Y);
    ser_right_ = lv_chart_add_series(chart_, C_CYAN,   LV_CHART_AXIS_PRIMARY_Y);
    ser_err_   = lv_chart_add_series(chart_, C_ORANGE, LV_CHART_AXIS_PRIMARY_Y);

    // Control panel
    lv_obj_t* ctrl = lv_obj_create(screen_);
    lv_obj_set_size(ctrl, CTRL_W, BODY_H - 8);
    lv_obj_set_pos(ctrl, CTRL_X, BODY_Y + 4);
    lv_obj_set_style_bg_color(ctrl, C_PANEL, 0);
    lv_obj_set_style_bg_opa(ctrl, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(ctrl, C_YELLOW, 0);
    lv_obj_set_style_border_width(ctrl, 1, 0);
    lv_obj_set_style_radius(ctrl, 4, 0);
    lv_obj_set_style_pad_all(ctrl, 4, 0);

    int row_h = (BODY_H - 8 - 38) / CONST_COUNT;

    for (int i = 0; i < CONST_COUNT; i++) {
        int ry = 4 + i * row_h;

        lv_obj_t* name_l = lv_label_create(ctrl);
        lv_label_set_text(name_l, SLOT_NAMES[i]);
        lv_obj_set_style_text_color(name_l, C_DIM, 0);
        lv_obj_set_pos(name_l, 4, ry + (row_h / 2) - 7);

        lv_obj_t* dec = lv_btn_create(ctrl);
        lv_obj_set_size(dec, 26, row_h - 4);
        lv_obj_set_pos(dec, 38, ry + 2);
        lv_obj_set_style_bg_color(dec, C_IDLE, 0);
        lv_obj_set_style_bg_color(dec, C_RED, LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(dec, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(dec, 0, 0);
        lv_obj_set_style_radius(dec, 3, 0);
        lv_obj_set_user_data(dec, pack_ud(i, -1));
        lv_obj_add_event_cb(dec, dec_cb, LV_EVENT_CLICKED, this);
        lv_obj_t* dl = lv_label_create(dec);
        lv_label_set_text(dl, LV_SYMBOL_MINUS);
        lv_obj_set_style_text_color(dl, C_TEXT, 0);
        lv_obj_center(dl);

        val_labels_[i] = lv_label_create(ctrl);
        lv_obj_set_style_text_color(val_labels_[i], C_YELLOW, 0);
        lv_obj_set_pos(val_labels_[i], 68, ry + (row_h / 2) - 7);
        lv_label_set_text(val_labels_[i], "0.000");

        lv_obj_t* inc = lv_btn_create(ctrl);
        lv_obj_set_size(inc, 26, row_h - 4);
        lv_obj_set_pos(inc, CTRL_W - 34, ry + 2);
        lv_obj_set_style_bg_color(inc, C_IDLE, 0);
        lv_obj_set_style_bg_color(inc, C_YELLOW, LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(inc, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(inc, 0, 0);
        lv_obj_set_style_radius(inc, 3, 0);
        lv_obj_set_user_data(inc, pack_ud(i, +1));
        lv_obj_add_event_cb(inc, inc_cb, LV_EVENT_CLICKED, this);
        lv_obj_t* il = lv_label_create(inc);
        lv_label_set_text(il, LV_SYMBOL_PLUS);
        lv_obj_set_style_text_color(il, C_TEXT, 0);
        lv_obj_center(il);
    }

    // Apply button
    lv_obj_t* apply = lv_btn_create(ctrl);
    lv_obj_set_size(apply, CTRL_W - 10, 28);
    lv_obj_set_pos(apply, 4, BODY_H - 8 - 32);
    lv_obj_set_style_bg_color(apply, C_YELLOW, 0);
    lv_obj_set_style_bg_grad_color(apply, C_YELLOW2, 0);
    lv_obj_set_style_bg_grad_dir(apply, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(apply, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(apply, C_IDLE, LV_STATE_PRESSED);
    lv_obj_set_style_border_width(apply, 0, 0);
    lv_obj_set_style_radius(apply, 4, 0);
    lv_obj_add_event_cb(apply, apply_cb, LV_EVENT_CLICKED, this);
    lv_obj_t* al = lv_label_create(apply);
    lv_label_set_text(al, LV_SYMBOL_OK "  Apply to EZ");
    lv_obj_set_style_text_color(al, C_DARK, 0);
    lv_obj_center(al);
}

void PidTuner::select_pid(PidType t) {
    active_pid_ = t;
    for (int i = 0; i < PID_COUNT; i++) {
        if (i == (int)t) lv_obj_add_state(tab_btns_[i],   LV_STATE_CHECKED);
        else             lv_obj_clear_state(tab_btns_[i], LV_STATE_CHECKED);
    }
    refresh_value_labels();
}

void PidTuner::refresh_value_labels() {
    for (int i = 0; i < CONST_COUNT; i++) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.3f", constants_[active_pid_].val[i]);
        lv_label_set_text(val_labels_[i], buf);
    }
}

void PidTuner::apply_constants() {
    if (!drive_) return;
    auto& v = constants_[active_pid_].val;
    // EZ-Template setters: pid_X_constants_set(kP, kI, kD, start_i)
    switch (active_pid_) {
        case PID_DRIVE:
            // Apply to both forward and backward drive PIDs
            drive_->pid_drive_constants_set(v[KP], v[KI], v[KD], v[START_I]);
            break;
        case PID_TURN:
            drive_->pid_turn_constants_set(v[KP], v[KI], v[KD], v[START_I]);
            break;
        case PID_SWING:
            drive_->pid_swing_constants_set(v[KP], v[KI], v[KD], v[START_I]);
            break;
        case PID_HEADING:
            drive_->pid_heading_constants_set(v[KP], v[KI], v[KD], v[START_I]);
            break;
        default: break;
    }
}

void PidTuner::sample_task_fn(void* param) {
    auto* self = static_cast<PidTuner*>(param);
    while (true) {
        if (self->recording_ && self->is_open() && self->drive_)
            self->push_sample();
        pros::delay(40);
    }
}

void PidTuner::push_sample() {
    auto clamp = [](double v) -> lv_coord_t {
        return (lv_coord_t)(v < -200 ? -200 : v > 200 ? 200 : v);
    };
    double lv = drive_->drive_sensor_left()  * 0.5;
    double rv = drive_->drive_sensor_right() * 0.5;
    double ev = (lv - rv) * 0.5;

    lv_chart_set_next_value(chart_, ser_left_,  clamp(lv));
    lv_chart_set_next_value(chart_, ser_right_, clamp(rv));
    lv_chart_set_next_value(chart_, ser_err_,   clamp(ev));
    lv_obj_invalidate(chart_);
}

void PidTuner::tab_cb(lv_event_t* e) {
    auto* self = static_cast<PidTuner*>(lv_event_get_user_data(e));
    int idx = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    self->select_pid((PidType)idx);
}

void PidTuner::inc_cb(lv_event_t* e) {
    auto* self = static_cast<PidTuner*>(lv_event_get_user_data(e));
    int slot, sign;
    unpack_ud(lv_obj_get_user_data(lv_event_get_target(e)), slot, sign);
    self->constants_[self->active_pid_].val[slot] += STEP[slot];
    self->refresh_value_labels();
}

void PidTuner::dec_cb(lv_event_t* e) {
    auto* self = static_cast<PidTuner*>(lv_event_get_user_data(e));
    int slot, sign;
    unpack_ud(lv_obj_get_user_data(lv_event_get_target(e)), slot, sign);
    double& v = self->constants_[self->active_pid_].val[slot];
    v -= STEP[slot];
    if (v < 0.0) v = 0.0;
    self->refresh_value_labels();
}

void PidTuner::apply_cb(lv_event_t* e) {
    auto* self = static_cast<PidTuner*>(lv_event_get_user_data(e));
    self->apply_constants();
}

void PidTuner::back_cb(lv_event_t* e) {
    auto* self = static_cast<PidTuner*>(lv_event_get_user_data(e));
    self->close();
}

void PidTuner::record_cb(lv_event_t* e) {
    auto* self = static_cast<PidTuner*>(lv_event_get_user_data(e));
    self->recording_ = !self->recording_;
    lv_obj_t* btn = lv_event_get_target(e);
    lv_obj_t* lbl = lv_obj_get_child(btn, 0);
    if (self->recording_) {
        lv_label_set_text(lbl, LV_SYMBOL_STOP " Stop");
        lv_obj_set_style_bg_color(btn, C_RED, 0);
        lv_obj_set_style_border_color(btn, C_RED, 0);
    } else {
        lv_label_set_text(lbl, LV_SYMBOL_PLAY " Record");
        lv_obj_set_style_bg_color(btn, C_IDLE, 0);
        lv_obj_set_style_border_color(btn, C_YELLOW, 0);
        lv_chart_set_all_value(self->chart_, self->ser_left_,  0);
        lv_chart_set_all_value(self->chart_, self->ser_right_, 0);
        lv_chart_set_all_value(self->chart_, self->ser_err_,   0);
        lv_obj_invalidate(self->chart_);
    }
}

} // namespace light
