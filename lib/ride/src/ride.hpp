#pragma once   
#include <cstdint>

namespace ride
{
    struct RideData
    {
        bool active;

        float distance_km;

        uint32_t elapsed_seconds;
        uint32_t moving_seconds;

        float average_speed_kmh;
        float max_speed_kmh;

        float average_cadence_rpm;
    };

    void start();
    void stop();

    // One coherent sensor snapshot. Expiry uses esp_timer_get_time().
    struct Sample
    {
        float speed_kmh = 0;
        float cadence_rpm = 0;
        uint32_t wheel_revs = 0;
        uint32_t wheel_generation = 0;
        bool wheel_valid = false;
        int64_t speed_valid_until_us = 0;
        int64_t cadence_valid_until_us = 0;
    };

    void update(const Sample& sample);

    RideData get_ride_data();
}