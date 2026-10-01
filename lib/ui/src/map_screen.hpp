#pragma once

#include <lvgl.h>
#include "gps.hpp"

namespace ui::map_screen {
// All calls run on the LVGL/UI thread. update() also services the background
// loader while the map is hidden, keeping the cached viewport ready to open.
void create(lv_obj_t* parent);
void pause_for_wifi();
void update(const gps::State& position);
}
