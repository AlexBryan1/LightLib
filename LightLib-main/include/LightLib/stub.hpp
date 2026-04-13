#pragma once
#include "EZ-Template/auton_selector.hpp"
#include "pros/adi.hpp"

inline ez::AutonSelector::AutonSelector() {}

inline void ez::AutonSelector::selected_auton_print() {}

namespace ez::as {
    inline void limit_switch_lcd_initialize(pros::adi::DigitalIn*, pros::adi::DigitalIn*) {}
    inline bool page_blank_is_on(int)  { return false; }
    inline int  page_blank_amount()    { return 0; }
    inline void page_blank_remove_all() {}
}