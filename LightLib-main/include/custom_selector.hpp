#pragma once
#include "main.h"
#include <string>
#include <functional>
#include <vector>

struct CustomAuton {
  std::string           name;
  std::function<void()> fn;
};
void display_splash(const char* mode);
void custom_selector_add(const std::string& name, std::function<void()> fn);
void custom_selector_init();
int  custom_selector_get();
void custom_selector_run();