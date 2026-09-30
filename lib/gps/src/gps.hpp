#pragma once

#include <cstdint>

namespace gps {

struct State {
    bool receiving = false;
    bool has_fix = false;
    int satellites = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    float speed_kmh = 0.0f;
    float course_deg = 0.0f;
    // POSIX UTC seconds, updated only from a valid RMC date/time.
    int64_t utc_seconds = 0;
    int64_t utc_updated_us = 0;
    int64_t fix_updated_us = 0;
    int64_t speed_updated_us = 0;
    bool has_utc_time = false;
    bool has_speed = false;
};

void init();
State get_state();

} // namespace gps
