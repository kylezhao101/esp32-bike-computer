#pragma once

#include <lvgl.h>
#include "gps.hpp"

namespace ui::map_screen {
// All calls run on the LVGL/UI thread.
void create(lv_obj_t* parent);
void update(const gps::State& position);
}
