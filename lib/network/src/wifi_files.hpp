#pragma once

#include <cstddef>

namespace wifi_files {
bool init();
bool start();
void display_status(char* buffer, size_t size);
}
