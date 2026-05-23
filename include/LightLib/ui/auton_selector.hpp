#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "light/lvgl.h"

/**
 * \file auton_selector.hpp
 *
 * On-brain auton picker with PID and odom side panels.
 *
 * `AutonSelector` is the default LightLib brain-screen UI. Register routines
 * with add(), show the screen via show(), and call run() from the
 * autonomous task to invoke the selected routine.
 */

namespace light {

/** One registered autonomous routine. */
struct Auton {
  std::string name;                       ///< Display name on the picker.
  std::string description;                ///< Long-form description shown in the side panel.
  std::function<void()> fn;               ///< Routine to invoke when selected.
  const lv_img_dsc_t* banner = nullptr;   ///< Optional scrolling image strip.
};

/**
 * On-brain auton picker UI.
 *
 * Singleton-style — use the global `auton_selector` instance. Holds the list
 * of registered autons, manages the LVGL screen lifecycle, and exposes a
 * compact PID-tuner side panel and live odom display.
 */
class AutonSelector {
 public:
  ~AutonSelector() { teardown_ui(); }

  /**
   * Register an autonomous routine.
   *
   * \param name
   *        display name on the picker button
   * \param desc
   *        description shown in the side panel
   * \param fn
   *        function invoked when the routine is selected
   */
  void add(const std::string& name, const std::string& desc, std::function<void()> fn);

  /** add() variant with a scrolling banner image shown inside the button. */
  void add(const std::string& name, const std::string& desc, std::function<void()> fn,
           const lv_img_dsc_t* banner);

  /** Build the LVGL UI. Call once in `initialize()`. */
  void init();

  /** Invoke the currently selected routine (call from `autonomous()`). */
  void run();

  /** Activate the picker screen. */
  void show();

  /** \return Index of the currently selected auton. */
  int selected() const { return selected_idx_; }

 private:
  std::vector<Auton> autons_;
  int selected_idx_ = 0;
  lv_obj_t* screen_ = nullptr;
  lv_obj_t* img_obj_ = nullptr;
  lv_obj_t* preview_cont_ = nullptr;
  lv_obj_t* pid_cont_ = nullptr;
  lv_obj_t* odom_cont_ = nullptr;
  lv_obj_t* odom_x_lbl_ = nullptr;
  lv_obj_t* odom_y_lbl_ = nullptr;
  lv_obj_t* odom_theta_lbl_ = nullptr;
  lv_timer_t* odom_timer_ = nullptr;
  // Last-rendered text per label — skip lv_label_set_text (and its
  // invalidation) when the formatted value is unchanged.
  char odom_x_last_[16] = {};
  char odom_y_last_[16] = {};
  char odom_th_last_[16] = {};
  lv_obj_t* toggle_lbl_ = nullptr;
  int right_panel_mode_ = 0;  // 0=preview, 1=pid, 2=odom
  std::vector<lv_obj_t*> btn_objs_;
  // Targets of LV_ANIM_REPEAT_INFINITE animations — must be lv_anim_del'd
  // before their parents are freed, otherwise the anim timer keeps firing
  // exec_cb on dangling memory and faults inside lv_obj_set_x/y.
  std::vector<lv_obj_t*> infinite_anim_targets_;

  // compact PID panel state
  lv_obj_t* pid_tab_btns_[4] = {};
  lv_obj_t* pid_val_labels_[4][4] = {};
  int active_pid_tab_ = 0;
  double pid_vals_[4][4] = {};

  void build_ui();
  void teardown_ui();
  void build_right_preview(lv_obj_t* parent);
  void build_right_pid(lv_obj_t* parent);
  void build_right_odom(lv_obj_t* parent);
  void switch_panel(int mode);
  void select_pid_tab(int idx);
  void refresh_pid_labels();
  void init_pid_vals();
  void apply_pid_vals();
  void select(int idx);

  // persistent full-screen view shown during/after autonomous
  lv_obj_t* run_screen_ = nullptr;
  lv_obj_t* run_img_cont_ = nullptr;
  lv_obj_t* run_info_cont_ = nullptr;
  lv_obj_t* run_toggle_lbl_ = nullptr;
  lv_obj_t* run_back_lbl_ = nullptr;
  lv_obj_t* run_img_ = nullptr;
  bool run_show_img_ = false;
  // Signals run() (auton task) that the deferred LVGL setup in
  // run_screen_load_async has finished running on the LVGL task.
  std::atomic<bool> run_setup_done_{false};
  void build_run_screen();
  void activate_run_screen();
  void start_run_anim();
  void switch_run_view(bool show_img);
  static void run_screen_load_async(void* p);
  static void run_toggle_cb(lv_event_t* e);
  static void run_back_cb(lv_event_t* e);

  static void btn_cb(lv_event_t* e);
  static void toggle_cb(lv_event_t* e);
  static void pid_tab_cb(lv_event_t* e);
  static void pid_adj_cb(lv_event_t* e);
  static void pid_apply_cb(lv_event_t* e);
  static void odom_timer_cb(lv_timer_t* timer);
};

/** Process-wide singleton selector instance. */
extern AutonSelector auton_selector;

}  // namespace light

/**
 * Defined in `src/auton_config.cpp`. Called once at the top of
 * `initialize()` to populate the on-brain selector with the user's
 * autonomous routines.
 */
void register_autons();
