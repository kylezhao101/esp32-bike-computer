#include "gps.hpp"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace gps {
namespace {

constexpr uart_port_t GPS_UART = UART_NUM_2;
constexpr int GPS_RX_PIN = 35;
constexpr int64_t DATA_TIMEOUT_US = 3'000'000;

SemaphoreHandle_t state_mutex = nullptr;
State state;
int64_t last_message_us = 0;

double parse_coordinate(const char* value, char hemisphere) {
    const double raw = std::strtod(value, nullptr);
    const int degrees = static_cast<int>(raw / 100.0);
    const double minutes = raw - degrees * 100.0;
    double result = degrees + minutes / 60.0;

    if (hemisphere == 'S' || hemisphere == 'W') {
        result = -result;
    }
    return result;
}

bool digits(const char* value, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (value[i] < '0' || value[i] > '9') return false;
    }
    return true;
}

bool parse_rmc_utc(const char* time_field, const char* date_field, int64_t& seconds)
{
    if (!time_field || !date_field || std::strlen(time_field) < 6 ||
        std::strlen(date_field) != 6 || !digits(time_field, 6) || !digits(date_field, 6))
        return false;

    const int hour = (time_field[0] - '0') * 10 + time_field[1] - '0';
    const int minute = (time_field[2] - '0') * 10 + time_field[3] - '0';
    const int second = (time_field[4] - '0') * 10 + time_field[5] - '0';
    const int day = (date_field[0] - '0') * 10 + date_field[1] - '0';
    const int month = (date_field[2] - '0') * 10 + date_field[3] - '0';
    const int short_year = (date_field[4] - '0') * 10 + date_field[5] - '0';
    const int year = short_year >= 80 ? 1900 + short_year : 2000 + short_year;
    if (hour > 23 || minute > 59 || second > 60 || month < 1 || month > 12)
        return false;

    constexpr int month_lengths[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    const bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    const int days_in_month = month_lengths[month - 1] + (month == 2 && leap ? 1 : 0);
    if (day < 1 || day > days_in_month) return false;

    // Gregorian civil date to days since 1970-01-01 (UTC).
    const int adjusted_year = year - (month <= 2);
    const int era = (adjusted_year >= 0 ? adjusted_year : adjusted_year - 399) / 400;
    const unsigned year_of_era = static_cast<unsigned>(adjusted_year - era * 400);
    const unsigned adjusted_month = static_cast<unsigned>(month + (month > 2 ? -3 : 9));
    const unsigned day_of_year = (153 * adjusted_month + 2) / 5 + day - 1;
    const unsigned day_of_era = year_of_era * 365 + year_of_era / 4 -
                                year_of_era / 100 + day_of_year;
    const int64_t days = int64_t(era) * 146097 + day_of_era - 719468;
    seconds = days * 86400 + hour * 3600 + minute * 60 + second;
    return true;
}

bool valid_position(const char* lat, const char* ns, const char* lon, const char* ew,
                    double& latitude, double& longitude)
{
    if (!lat[0] || !ns[0] || !lon[0] || !ew[0] ||
        (ns[0] != 'N' && ns[0] != 'S') || (ew[0] != 'E' && ew[0] != 'W'))
        return false;
    latitude = parse_coordinate(lat, ns[0]);
    longitude = parse_coordinate(lon, ew[0]);
    return std::isfinite(latitude) && std::isfinite(longitude) &&
           latitude >= -90.0 && latitude <= 90.0 &&
           longitude >= -180.0 && longitude <= 180.0;
}

bool valid_checksum(char* sentence) {
    char* star = std::strchr(sentence, '*');
    if (!star || !star[1] || !star[2] || star[3] != '\0') {
        return false;
    }

    unsigned int expected = 0;
    if (std::sscanf(star + 1, "%2x", &expected) != 1) {
        return false;
    }

    unsigned char actual = 0;
    for (char* p = sentence + 1; p < star; ++p) {
        actual ^= static_cast<unsigned char>(*p);
    }

    if (actual != expected) return false;
    *star = '\0'; // Remove checksum before splitting fields
    return true;
}

void handle_sentence(char* sentence) {
    if (sentence[0] != '$' || !valid_checksum(sentence)) return;

    char* fields[20] = {};
    int count = 0;
    fields[count++] = sentence;

    for (char* p = sentence; *p && count < 20; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }

    if (std::strlen(fields[0]) < 6) return;

    const char* type = fields[0] + 3; // GGA/RMC, regardless of GN or GP
    if (std::strcmp(type, "GGA") != 0 &&
        std::strcmp(type, "RMC") != 0) {
        return;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.receiving = true;
    last_message_us = esp_timer_get_time();

    if (std::strcmp(type, "GGA") == 0 && count > 7) {
        const int quality = std::atoi(fields[6]);
        state.satellites = std::atoi(fields[7]);
        double latitude = 0.0;
        double longitude = 0.0;
        state.has_fix = quality > 0 && count > 5 &&
                        valid_position(fields[2], fields[3], fields[4], fields[5],
                                       latitude, longitude);

        if (state.has_fix) {
            state.latitude = latitude;
            state.longitude = longitude;
            state.fix_updated_us = esp_timer_get_time();
        } else {
            state.latitude = 0.0;
            state.longitude = 0.0;
        }
    }

    if (std::strcmp(type, "RMC") == 0 && count > 9) {
        if (fields[2][0] == 'A') {
            int64_t utc_seconds = 0;
            if (parse_rmc_utc(fields[1], fields[9], utc_seconds)) {
                state.utc_seconds = utc_seconds;
                state.utc_updated_us = last_message_us;
                state.has_utc_time = true;
            }
            if (fields[7][0]) {
                const float speed = static_cast<float>(std::atof(fields[7]) * 1.852);
                if (std::isfinite(speed) && speed >= 0.0f) {
                    state.speed_kmh = speed;
                    state.speed_updated_us = last_message_us;
                    state.has_speed = true;
                } else {
                    state.has_speed = false;
                }
            } else {
                state.has_speed = false;
                state.speed_kmh = 0.0f;
            }
            if (fields[8][0])
                state.course_deg = static_cast<float>(std::atof(fields[8]));
        } else {
            state.has_fix = false;
            state.has_utc_time = false;
            state.has_speed = false;
            state.speed_kmh = 0.0f;
        }
    }

    xSemaphoreGive(state_mutex);
}

void gps_task(void*) {
    uint8_t data[128];
    char sentence[128];
    size_t length = 0;

    while (true) {
        const int received = uart_read_bytes(
            GPS_UART, data, sizeof(data), pdMS_TO_TICKS(1000)
        );

        for (int i = 0; i < received; ++i) {
            const char c = static_cast<char>(data[i]);

            if (c == '$') {
                length = 0;
                sentence[length++] = c;
            } else if (length == 0) {
                continue;
            } else if (c == '\r') {
                continue;
            } else if (c == '\n') {
                sentence[length] = '\0';
                handle_sentence(sentence);
                length = 0;
            } else if (length < sizeof(sentence) - 1) {
                sentence[length++] = c;
            } else {
                length = 0;
            }
        }
    }
}

} // namespace

void init() {
    state_mutex = xSemaphoreCreateMutex();
    assert(state_mutex != nullptr);

    uart_config_t config{};
    config.baud_rate = 9600;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(
        GPS_UART, UART_PIN_NO_CHANGE, GPS_RX_PIN,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE
    ));
    ESP_ERROR_CHECK(uart_driver_install(
        GPS_UART, 2048, 0, 0, nullptr, 0
    ));

    assert(xTaskCreate(
        gps_task, "gps_task", 4096, nullptr, 4, nullptr
    ) == pdPASS);
}

State get_state() {
    if (!state_mutex) return {};

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    State snapshot = state;
    const int64_t last_update = last_message_us;
    xSemaphoreGive(state_mutex);

    if (esp_timer_get_time() - last_update > DATA_TIMEOUT_US) {
        snapshot.receiving = false;
        snapshot.has_fix = false;
        snapshot.satellites = 0;
        snapshot.speed_kmh = 0.0f;
    }
    const int64_t now = esp_timer_get_time();
    if (snapshot.has_fix && now - snapshot.fix_updated_us > DATA_TIMEOUT_US)
        snapshot.has_fix = false;
    if (snapshot.has_utc_time && now - snapshot.utc_updated_us > DATA_TIMEOUT_US)
        snapshot.has_utc_time = false;
    if (snapshot.has_speed && now - snapshot.speed_updated_us > DATA_TIMEOUT_US)
        snapshot.has_speed = false;
    return snapshot;
}

} // namespace gps
