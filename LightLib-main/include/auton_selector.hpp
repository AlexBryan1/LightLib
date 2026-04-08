#pragma once
#include "liblvgl/lvgl.h"
#include <string>
#include <vector>
#include <functional>

LV_IMG_DECLARE(Balls);

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
    int                    selected_idx_ = 0;
    lv_obj_t*              screen_       = nullptr;
    lv_obj_t*              img_obj_      = nullptr;
    lv_obj_t*              desc_label_   = nullptr;
    std::vector<lv_obj_t*> btn_objs_;

    void build_ui();
    void select(int idx);
    static void btn_cb(lv_event_t* e);
};

extern AutonSelector auton_selector;

} // namespace light
