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

    // Latest once-per-second snapshot for the future FIT writer. Availability
    // flags distinguish missing/stale sensors from real zero readings.
    struct RecordingSample
    {
        int64_t sampled_at_us = 0;
        int64_t utc_seconds = 0;
        bool utc_valid = false;
        bool gps_fix_valid = false;
        double latitude = 0.0;
        double longitude = 0.0;
        float speed_kmh = 0.0f;
        bool speed_valid = false;
        float cadence_rpm = 0.0f;
        bool cadence_valid = false;
        float distance_km = 0.0f;
        uint32_t elapsed_seconds = 0;
    };

    void update(const Sample& sample);

    RideData get_ride_data();

    // The sampler publishes one coherent-by-source snapshot each second. The
    // FIT writer can copy it safely from its own task when added.
    void publish_recording_sample(const RecordingSample& sample);
    RecordingSample get_recording_sample();
}
