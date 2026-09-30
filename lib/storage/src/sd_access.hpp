#pragma once

#include "freertos/FreeRTOS.h"

namespace sd_access {
bool init();
bool lock(TickType_t timeout = portMAX_DELAY);
void unlock();
}
