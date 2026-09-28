#pragma once
#include <cstdint>

namespace bluetooth
{
    enum class ConnectionState { Idle, Searching, Connecting, Connected };

    struct SensorData
    {
        bool connected;
        ConnectionState connection_state;
        bool battery_valid;
        float speed_kmh;
        float cadence_rpm;
        std::uint32_t wheel_revs;
        std::uint8_t battery_percentage;
        bool wheel_valid;
        std::uint32_t wheel_generation;
        std::int64_t speed_valid_until_us;
        std::int64_t cadence_valid_until_us;
    };

    void init();
    void search();
    void cancel_search();

    SensorData get_sensor_data();
}