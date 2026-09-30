#include "sd_access.hpp"
#include "freertos/semphr.h"

namespace sd_access {
namespace { SemaphoreHandle_t mutex = nullptr; }

bool init()
{
    if (!mutex) mutex = xSemaphoreCreateMutex();
    return mutex != nullptr;
}

bool lock(TickType_t timeout)
{
    return mutex && xSemaphoreTake(mutex, timeout) == pdTRUE;
}

void unlock()
{
    if (mutex) xSemaphoreGive(mutex);
}
}
