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
};

class AutonSelector {
public:
    void add(const std::string& name, const std::string& desc, std::function<void()> fn);
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
    lv_obj_t*              toggle_lbl_    = nullptr;
    bool                   show_pid_      = false;
    std::vector<lv_obj_t*> btn_objs_;

    // compact PID panel state
    lv_obj_t*  pid_tab_btns_[4]      = {};
    lv_obj_t*  pid_val_labels_[4][4] = {};
    int        active_pid_tab_        = 0;
    double     pid_vals_[4][4]        = {};

    void build_ui();
    void build_right_preview(lv_obj_t* parent);
    void build_right_pid(lv_obj_t* parent);
    void switch_panel(bool to_pid);
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
    bool       run_show_img_  = true;
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
};

extern AutonSelector auton_selector;

} // namespace light
