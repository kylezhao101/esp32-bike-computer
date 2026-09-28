#include "gps.hpp"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
        state.has_fix = quality > 0;

        if (state.has_fix &&
            fields[2][0] && fields[3][0] &&
            fields[4][0] && fields[5][0]) {
            state.latitude =
                parse_coordinate(fields[2], fields[3][0]);
            state.longitude =
                parse_coordinate(fields[4], fields[5][0]);
        } else {
            state.latitude = 0.0;
            state.longitude = 0.0;
            state.speed_kmh = 0.0f;
        }
    }

    if (std::strcmp(type, "RMC") == 0 && count > 8) {
        if (fields[2][0] == 'A') {
            state.speed_kmh =
                static_cast<float>(std::atof(fields[7]) * 1.852);
            state.course_deg =
                static_cast<float>(std::atof(fields[8]));
        } else {
            state.has_fix = false;
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
    return snapshot;
}

} // namespace gps