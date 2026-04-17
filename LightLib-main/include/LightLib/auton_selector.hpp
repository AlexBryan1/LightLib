#pragma once
#include "liblvgl/lvgl.h"
#include <string>
#include <vector>
#include <functional>

namespace light {

struct Auton {
    std::string name;
    std::string description;
    std::function<void()> fn;
    const lv_img_dsc_t*  banner = nullptr;  // optional scrolling image strip
};

class AutonSelector {
public:
    void add(const std::string& name, const std::string& desc, std::function<void()> fn);
    // add() with a scrolling banner image shown inside the button
    void add(const std::string& name, const std::string& desc, std::function<void()> fn,
             const lv_img_dsc_t* banner);
    void init();
    void run();
    void show();
    int  selected() const { return selected_idx_; }

private:
    std::vector<Auton>     autons_;
    int                    selected_idx_  = 0;
    lv_obj_t*              screen_        = nullptr;
    lv_obj_t*              img_obj_       = nullptr;
    lv_obj_t*              preview_cont_  = nullptr;
    lv_obj_t*              pid_cont_      = nullptr;
    lv_obj_t*              odom_cont_     = nullptr;
    lv_obj_t*              odom_x_lbl_    = nullptr;
    lv_obj_t*              odom_y_lbl_    = nullptr;
    lv_obj_t*              odom_theta_lbl_ = nullptr;
    lv_timer_t*            odom_timer_    = nullptr;
    lv_obj_t*              toggle_lbl_    = nullptr;
    int                    right_panel_mode_ = 0;  // 0=preview, 1=pid, 2=odom
    std::vector<lv_obj_t*> btn_objs_;

    // compact PID panel state
    lv_obj_t*  pid_tab_btns_[4]      = {};
    lv_obj_t*  pid_val_labels_[4][4] = {};
    int        active_pid_tab_        = 0;
    double     pid_vals_[4][4]        = {};

    void build_ui();
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
    lv_obj_t*  run_screen_    = nullptr;
    lv_obj_t*  run_img_cont_  = nullptr;
    lv_obj_t*  run_info_cont_ = nullptr;
    lv_obj_t*  run_toggle_lbl_= nullptr;
    lv_obj_t*  run_back_lbl_  = nullptr;
    lv_obj_t*  run_img_       = nullptr;
    bool       run_show_img_  = false;
    void build_run_screen();
    void start_run_anim();
    void switch_run_view(bool show_img);
    static void run_toggle_cb(lv_event_t* e);
    static void run_back_cb(lv_event_t* e);

    static void btn_cb(lv_event_t* e);
    static void toggle_cb(lv_event_t* e);
    static void pid_tab_cb(lv_event_t* e);
    static void pid_adj_cb(lv_event_t* e);
    static void pid_apply_cb(lv_event_t* e);
    static void odom_timer_cb(lv_timer_t* timer);
};

extern AutonSelector auton_selector;

} // namespace light
