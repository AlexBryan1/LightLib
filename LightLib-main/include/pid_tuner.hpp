#pragma once
#include "liblvgl/lvgl.h"
#include "EZ-Template/api.hpp"
#include <array>

namespace light {

enum PidType   { PID_DRIVE = 0, PID_TURN, PID_SWING, PID_HEADING, PID_COUNT };
enum ConstSlot { KP = 0, KI, KD, START_I, CONST_COUNT };

struct PidConstants { double val[CONST_COUNT] = {0, 0, 0, 0}; };

class PidTuner {
public:
    void set_drive(ez::Drive* drive);
    void start_task();
    void open();
    void close();
    bool is_open() const { return screen_ != nullptr && lv_scr_act() == screen_; }
    ez::Drive* get_drive() const { return drive_; }

private:
    ez::Drive*          drive_        = nullptr;
    lv_obj_t*           screen_       = nullptr;
    lv_obj_t*           chart_        = nullptr;
    lv_chart_series_t*  ser_left_     = nullptr;
    lv_chart_series_t*  ser_right_    = nullptr;
    lv_chart_series_t*  ser_err_      = nullptr;
    lv_obj_t*           tab_btns_[PID_COUNT]     = {};
    lv_obj_t*           val_labels_[CONST_COUNT] = {};
    PidType             active_pid_   = PID_DRIVE;
    PidConstants        constants_[PID_COUNT];
    bool                recording_    = false;
    pros::Task*         sample_task_  = nullptr;

    void build_ui();
    void select_pid(PidType t);
    void refresh_value_labels();
    void apply_constants();
    void push_sample();

    static void sample_task_fn(void* param);
    static void tab_cb(lv_event_t* e);
    static void inc_cb(lv_event_t* e);
    static void dec_cb(lv_event_t* e);
    static void apply_cb(lv_event_t* e);
    static void back_cb(lv_event_t* e);
    static void record_cb(lv_event_t* e);
};

extern PidTuner pid_tuner;

} // namespace light
