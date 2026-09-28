#pragma once

namespace gps {

struct State {
    bool receiving = false;
    bool has_fix = false;
    int satellites = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    float speed_kmh = 0.0f;
    float course_deg = 0.0f;
};

void init();
State get_state();

} // namespace gps