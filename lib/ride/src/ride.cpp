#include "ride.hpp"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>

namespace ride
{
static RideData data{};
static Sample previous{};
static int64_t start_us = 0;
static int64_t last_us = 0;
static int64_t moving_us = 0;
static int64_t pedaling_us = 0;
static double cadence_rpm_us = 0;
static constexpr double WHEEL_KM = 2.105 / 1000.0;

// Apply the previous reading only to the interval for which it was valid.
static void advance(int64_t now)
{
    if (!data.active) return;
    if (previous.speed_kmh > 0) {
        moving_us += std::max<int64_t>(0,
            std::min(now, previous.speed_valid_until_us) - last_us);
    }
    if (previous.cadence_rpm > 0) {
        const int64_t dt = std::max<int64_t>(0,
            std::min(now, previous.cadence_valid_until_us) - last_us);
        pedaling_us += dt;
        cadence_rpm_us += previous.cadence_rpm * static_cast<double>(dt);
    }
    last_us = now;
    data.elapsed_seconds = (now - start_us) / 1000000;
    data.moving_seconds = moving_us / 1000000;
    data.average_speed_kmh = moving_us > 0
        ? data.distance_km * 3600000000.0 / moving_us : 0;
    data.average_cadence_rpm = pedaling_us > 0
        ? cadence_rpm_us / pedaling_us : 0;
}

void start()
{
    data = {};
    data.active = true;
    previous = {};
    moving_us = pedaling_us = 0;
    cadence_rpm_us = 0;
    start_us = last_us = esp_timer_get_time();
}

void stop()
{
    advance(esp_timer_get_time());
    data.active = false;
}

void update(const Sample& sample)
{
    if (!data.active) return;
    const int64_t now = esp_timer_get_time();
    advance(now);
    if (sample.wheel_valid && previous.wheel_valid &&
        sample.wheel_generation == previous.wheel_generation) {
        // Unsigned subtraction also handles a genuine counter wrap.
        const uint32_t delta = sample.wheel_revs - previous.wheel_revs;
        data.distance_km += delta * WHEEL_KM;
    }
    previous = sample;
    if (!std::isfinite(previous.speed_kmh) || previous.speed_kmh < 0 ||
        previous.speed_valid_until_us <= now) previous.speed_kmh = 0;
    if (!std::isfinite(previous.cadence_rpm) || previous.cadence_rpm < 0 ||
        previous.cadence_valid_until_us <= now) previous.cadence_rpm = 0;
    data.max_speed_kmh = std::max(data.max_speed_kmh, previous.speed_kmh);
    advance(now); // Refresh derived metrics after adding distance.
}

RideData get_ride_data()
{
    advance(esp_timer_get_time());
    return data;
}
} // namespace ride
