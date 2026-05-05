#pragma once
#include <array>

#include "LightLib/lib.hpp"
#include "light/lvgl.h"

/**
 * \file pid_tuner.hpp
 *
 * Live on-brain PID tuner. Renders an LVGL screen with tabs for each PID
 * (drive, turn, swing, heading, EKF), live-editable gain rows, and a chart
 * of left/right wheel output and error during recording.
 */

namespace light {

/** PID controller categories selectable in the tuner. */
enum PidType { PID_DRIVE = 0, ///< Linear drive PID.
               PID_TURN,      ///< Heading/turn-in-place PID.
               PID_SWING,     ///< Single-side swing PID.
               PID_HEADING,   ///< Heading correction during drive moves.
               PID_EKF,       ///< EKF heading-fusion PID.
               PID_COUNT };   ///< Sentinel — number of PID slots.

/** Index into a PidConstants slot. */
enum ConstSlot { KP = 0,      ///< Proportional gain.
                 KI,          ///< Integral gain.
                 KD,          ///< Derivative gain.
                 START_I,     ///< Integral activation threshold.
                 CONST_COUNT };///< Sentinel — number of editable constants.

/** Container for one PID's four editable gain values. */
struct PidConstants {
  double val[CONST_COUNT] = {0, 0, 0, 0};
};

/**
 * Live PID tuner UI driven by LVGL.
 *
 * Bind a Drive instance via set_drive(), then start the background sampling
 * task with start_task(). open() renders the screen; close() returns to the
 * previous one. While open, gain rows live-edit the bound chassis's PIDs
 * and the chart plots wheel output / error.
 */
class PidTuner {
 public:
  /** Bind the Drive whose PIDs this tuner will edit. */
  void set_drive(light::Drive* drive);

  /** Spawn the background sampling task that feeds the chart. Idempotent. */
  void start_task();

  /** Render and activate the tuner screen. */
  void open();

  /** Close the tuner screen and restore the previous LVGL screen. */
  void close();

  /** \return true if the tuner screen is the currently active LVGL screen. */
  bool is_open() const { return screen_ != nullptr && lv_scr_act() == screen_; }

  /** \return The bound Drive instance, or nullptr if none. */
  light::Drive* get_drive() const { return drive_; }

 private:
  light::Drive* drive_ = nullptr;
  lv_obj_t* screen_ = nullptr;
  lv_obj_t* chart_ = nullptr;
  lv_chart_series_t* ser_left_ = nullptr;
  lv_chart_series_t* ser_right_ = nullptr;
  lv_chart_series_t* ser_err_ = nullptr;
  lv_obj_t* tab_btns_[PID_COUNT] = {};
  lv_obj_t* val_labels_[CONST_COUNT] = {};
  lv_obj_t* name_labels_[CONST_COUNT] = {};
  PidType active_pid_ = PID_DRIVE;
  PidConstants constants_[PID_COUNT];
  bool recording_ = false;
  pros::Task* sample_task_ = nullptr;
  double chart_y_max_ = 1.0;
  double chart_y_min_ = -1.0;
  // Last bounds we actually pushed to LVGL — guards against per-sample
  // lv_chart_set_range when nothing changed.
  double last_chart_y_max_ = 0.0;
  double last_chart_y_min_ = 0.0;

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

/** Process-wide singleton tuner instance. */
extern PidTuner pid_tuner;

}  // namespace light
