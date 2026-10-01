#include "ui.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "display.hpp"
#include <lvgl.h>

#include "bluetooth.hpp"
#include "ride.hpp"
#include "fit_recorder.hpp"
#include "gps.hpp"
#include "wifi_files.hpp"

#include "map_screen.hpp"
#include <cstdio>
#include <cstring>

namespace ui
{

static LGFX display;

// LVGL renders into one strip while SPI DMA transmits the other.
DMA_ATTR static uint16_t draw_buffer[320 * 20];
DMA_ATTR static uint16_t draw_buffer_next[320 * 20];
static bool flush_pending = false;
static int64_t dma_wait_us = 0;
static uint32_t refresh_pixels = 0;

static void finish_display_transfer()
{
    if (!flush_pending) return;
    const int64_t start = esp_timer_get_time();
    display.waitDMA();
    display.endWrite();
    dma_wait_us += esp_timer_get_time() - start;
    flush_pending = false;
}

static uint32_t ui_tick_ms()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

static void display_refresh_event(lv_event_t* event)
{
    static int64_t started = 0;
    static int64_t last_report = 0;
    static int64_t total_us = 0;
    static int64_t max_us = 0;
    static int64_t total_wait_us = 0;
    static uint32_t frames = 0;
    static uint32_t pixels = 0;
    if (lv_event_get_code(event) == LV_EVENT_REFR_START) {
        finish_display_transfer();
        started = esp_timer_get_time();
        dma_wait_us = 0;
        refresh_pixels = 0;
        return;
    }
    finish_display_transfer();
    if (!refresh_pixels) return;
    const int64_t now = esp_timer_get_time();
    const int64_t elapsed = now - started;
    total_us += elapsed;
    if (elapsed > max_us) max_us = elapsed;
    total_wait_us += dma_wait_us;
    pixels += refresh_pixels;
    ++frames;
    if (now - last_report < 5000000) return;
    ESP_LOGI("ui_perf", "refresh avg/max %lld/%lld ms, DMA wait avg %lld ms, "
             "avg %lu pixels, free/largest heap %u/%u bytes",
             total_us / frames / 1000, max_us / 1000,
             total_wait_us / frames / 1000, static_cast<unsigned long>(pixels / frames),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    last_report = now;
    total_us = max_us = total_wait_us = 0;
    frames = pixels = 0;
}

static lv_obj_t* left_panel = nullptr;
static lv_obj_t* right_panel = nullptr;
static lv_obj_t* dashboard_screen = nullptr;
static lv_obj_t* sensor_screen = nullptr;

static lv_obj_t* speed_title = nullptr;
static lv_obj_t* speed_label = nullptr;
static lv_obj_t* speed_unit = nullptr;

static lv_obj_t* cadence_title = nullptr;
static lv_obj_t* cadence_label = nullptr;
static lv_obj_t* cadence_unit = nullptr;

static lv_obj_t* distance_label = nullptr;
static lv_obj_t* elapsed_label = nullptr;

static void update_ride()
{
    const auto sensor = bluetooth::get_sensor_data();
    ride::Sample sample{};
    sample.speed_kmh = sensor.speed_kmh;
    sample.cadence_rpm = sensor.cadence_rpm;
    sample.wheel_revs = sensor.wheel_revs;
    sample.wheel_generation = sensor.wheel_generation;
    sample.wheel_valid = sensor.wheel_valid;
    sample.speed_valid_until_us = sensor.speed_valid_until_us;
    sample.cadence_valid_until_us = sensor.cadence_valid_until_us;
    ride::update(sample);
}

static ride::RecordingSample sample_ride_for_recording()
{
    const int64_t now = esp_timer_get_time();
    const bluetooth::SensorData sensor = bluetooth::get_sensor_data();
    const gps::State location = gps::get_state();
    const ride::RideData totals = ride::get_ride_data();

    ride::RecordingSample sample{};
    sample.sampled_at_us = now;
    sample.utc_seconds = location.utc_seconds;
    sample.utc_valid = location.has_utc_time;
    sample.gps_fix_valid = location.receiving && location.has_fix;
    if (sample.gps_fix_valid) {
        sample.latitude = location.latitude;
        sample.longitude = location.longitude;
    }
    if (sensor.wheel_supported) {
        sample.speed_kmh = now < sensor.speed_valid_until_us
            ? sensor.speed_kmh : 0.0f;
        sample.speed_valid = true;
    } else if (location.has_speed) {
        sample.speed_kmh = location.speed_kmh;
        sample.speed_valid = true;
    }
    if (sensor.crank_supported) {
        sample.cadence_rpm = now < sensor.cadence_valid_until_us
            ? sensor.cadence_rpm : 0.0f;
        sample.cadence_valid = true;
    }
    sample.distance_km = totals.distance_km;
    sample.elapsed_seconds = totals.elapsed_seconds;
    ride::publish_recording_sample(sample);
    return sample;
}


static lv_obj_t* ride_button = nullptr;
static lv_obj_t* ride_button_label = nullptr;

static lv_obj_t* sensor_info_label = nullptr;
static lv_obj_t* gps_status_label = nullptr;
static lv_obj_t* wifi_status_label = nullptr;
static lv_obj_t* wifi_start_button = nullptr;
static lv_obj_t* sensor_button = nullptr;
static lv_obj_t* sensor_button_label = nullptr;
static lv_obj_t* fit_error_dialog = nullptr;
static int64_t next_record_sample_us = 0;

static void show_sensor_page_event_cb(lv_event_t* event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED && sensor_screen)
        lv_screen_load(sensor_screen);
}

static void show_dashboard_page_event_cb(lv_event_t* event)
{
    if (lv_event_get_code(event) == LV_EVENT_CLICKED && dashboard_screen)
        lv_screen_load(dashboard_screen);
}

static void sensor_button_event_cb(lv_event_t* event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    const auto state = bluetooth::get_sensor_data().connection_state;
    if (state == bluetooth::ConnectionState::Searching ||
        state == bluetooth::ConnectionState::Connecting)
        bluetooth::cancel_search();
    else if (state == bluetooth::ConnectionState::Idle)
        bluetooth::search();
}


// -------------------------
// Touch calibration
// -------------------------

static int32_t map_touch_x(int32_t x)
{
    x = 292 - x;
    x = (x * 319) / 292;

    if (x < 0)
        x = 0;

    if (x > 319)
        x = 319;

    return x;
}


static int32_t map_touch_y(int32_t y)
{
    y -= 18;
    y = (y * 239) / (221 - 18);

    if (y < 0)
        y = 0;

    if (y > 239)
        y = 239;

    return y;
}


// -------------------------
// LVGL touch input
// -------------------------

static void lvgl_touch_cb(
    lv_indev_t* indev,
    lv_indev_data_t* data
)
{
    uint16_t raw_x;
    uint16_t raw_y;

    finish_display_transfer();

    if (display.getTouch(
            &raw_x,
            &raw_y
        ))
    {
        data->point.x =
            map_touch_x(raw_x);

        data->point.y =
            map_touch_y(raw_y);

        data->state =
            LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state =
            LV_INDEV_STATE_RELEASED;
    }
}


// -------------------------
// LVGL display flush
// -------------------------

static void lvgl_flush_cb(
    lv_display_t*,
    const lv_area_t* area,
    uint8_t* px_map
)
{
    int32_t width =
        area->x2 - area->x1 + 1;

    int32_t height =
        area->y2 - area->y1 + 1;
    refresh_pixels += width * height;

    finish_display_transfer();
    // Send native SPI byte order directly, avoiding LovyanGFX's conversion
    // buffers. LVGL redraws this partial buffer before reusing it.
    lv_draw_rgb565_swap(px_map, width * height);
    display.startWrite();

    display.setAddrWindow(
        area->x1,
        area->y1,
        width,
        height
    );

    display.writePixelsDMA(
        reinterpret_cast<uint16_t*>(px_map),
        width * height,
        false
    );
    flush_pending = true;
    // The wait callback ends the transaction before LVGL reuses this buffer.
}

static void wifi_start_event_cb(lv_event_t* event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    bluetooth::pause_discovery_for_wifi();
    map_screen::pause_for_wifi();
    if (wifi_files::start()) {
        lv_obj_add_state(wifi_start_button, LV_STATE_DISABLED);
        lv_label_set_text(lv_obj_get_child(wifi_start_button, 0), "Starting...");
    }
}

static void close_fit_error_event_cb(lv_event_t* event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    lv_obj_t* dialog = static_cast<lv_obj_t*>(lv_event_get_user_data(event));
    if (fit_error_dialog == dialog) fit_error_dialog = nullptr;
    lv_msgbox_close(dialog);
}

static void show_fit_error(const char* message)
{
    if (fit_error_dialog) return;
    fit_error_dialog = lv_msgbox_create(nullptr);
    lv_msgbox_add_title(fit_error_dialog, "Ride recording error");
    lv_msgbox_add_text(fit_error_dialog, message);
    lv_obj_t* ok = lv_msgbox_add_footer_button(fit_error_dialog, "OK");
    lv_obj_add_event_cb(ok, close_fit_error_event_cb, LV_EVENT_CLICKED,
                        fit_error_dialog);
}

static void lvgl_flush_wait_cb(lv_display_t*)
{
    finish_display_transfer();
}


// -------------------------
// Ride button
// -------------------------

static void finish_ride()
{
    update_ride();
    ride::stop();
    const auto final_sample = sample_ride_for_recording();
    const bool sample_queued = fit_recorder::submit(final_sample);
    const bool stop_queued = fit_recorder::stop(ride::get_ride_data());
    lv_label_set_text(ride_button_label, "START");

    if (!sample_queued || !stop_queued)
        show_fit_error("Could not queue all ride data for saving. The FIT file may be incomplete.");
}

static void end_ride_confirmation_event_cb(lv_event_t* event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    lv_obj_t* dialog = static_cast<lv_obj_t*>(lv_event_get_user_data(event));
    lv_obj_t* button = lv_event_get_target_obj(event);
    lv_obj_t* label = lv_obj_get_child(button, 0);
    const bool confirmed = label &&
        std::strcmp(lv_label_get_text(label), "End Ride") == 0;
    lv_msgbox_close(dialog);
    if (confirmed) finish_ride();
}

static void ride_button_event_cb(
    lv_event_t* event
)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED)
        return;

    ride::RideData ride_data =
        ride::get_ride_data();

    if (ride_data.active)
    {
        lv_obj_t* dialog = lv_msgbox_create(nullptr);
        lv_msgbox_add_title(dialog, "End this ride?");
        lv_msgbox_add_text(dialog, "Save the ride and stop recording?");
        lv_obj_t* cancel = lv_msgbox_add_footer_button(dialog, "Cancel");
        lv_obj_add_event_cb(cancel, end_ride_confirmation_event_cb,
                            LV_EVENT_CLICKED, dialog);
        lv_obj_t* end = lv_msgbox_add_footer_button(dialog, "End Ride");
        lv_obj_add_event_cb(end, end_ride_confirmation_event_cb,
                            LV_EVENT_CLICKED, dialog);
    }
    else
    {
        if (!fit_recorder::start()) {
            show_fit_error("FIT recorder is unavailable. Check the SD card and restart the device.");
            return;
        }
        ride::start();
        update_ride();
        fit_recorder::submit(sample_ride_for_recording());
        next_record_sample_us = esp_timer_get_time() + 1000000;

        lv_label_set_text(
            ride_button_label,
            "END"
        );
    }
}


// -------------------------
// UI task
// -------------------------

static void ui_task(void* param)
{
    char speed_text[32];
    char cadence_text[32];
    char sensor_text[80];
    char distance_text[32];
    char elapsed_text[32];

    uint32_t last_ui_update = ui_tick_ms();
    auto previous_fit_status = fit_recorder::status();
    while (true)
    {
        lv_timer_handler();
        finish_display_transfer();

        const uint32_t now = ui_tick_ms();
        if (ride::get_ride_data().active) {
            const int64_t now_us = esp_timer_get_time();
            if (now_us >= next_record_sample_us) {
                // Do not fabricate catch-up records after a delayed UI pass.
                // The monotonic timestamp records when this sample was taken.
                update_ride();
                fit_recorder::submit(sample_ride_for_recording());
                next_record_sample_us = now_us + 1000000;
            }
        }
        if (now - last_ui_update >= 100)
        {
            last_ui_update = now;

            bluetooth::SensorData sensor_data =
                bluetooth::get_sensor_data();

            ride::RideData ride_data =
                ride::get_ride_data();

            gps::State gps_data = gps::get_state();
            map_screen::update(gps_data);

            char gps_text[48];
            if (!gps_data.receiving) {
                snprintf(gps_text, sizeof(gps_text), "GPS disconnected");
            } else if (!gps_data.has_fix) {
                snprintf(gps_text, sizeof(gps_text),
                        "GPS searching | %d sats", gps_data.satellites);
            } else {
                snprintf(gps_text, sizeof(gps_text),
                        "GPS fix | %d sats", gps_data.satellites);
            }

            lv_label_set_text(gps_status_label, gps_text);



            // -------------------------
            // Speed
            // -------------------------

            snprintf(
                speed_text,
                sizeof(speed_text),
                "%.1f",
                sensor_data.speed_kmh
            );

            lv_label_set_text(
                speed_label,
                sensor_data.connected ? speed_text : "--"
            );


            // -------------------------
            // Cadence
            // -------------------------

            snprintf(
                cadence_text,
                sizeof(cadence_text),
                "%.0f",
                sensor_data.cadence_rpm
            );

            lv_label_set_text(
                cadence_label,
                sensor_data.connected ? cadence_text : "--"
            );


            // -------------------------
            // Sensor status
            // -------------------------

            const auto state = sensor_data.connection_state;
            switch (state)
            {
                case bluetooth::ConnectionState::Connected:
                    if (sensor_data.battery_valid)
                        snprintf(sensor_text, sizeof(sensor_text),
                                 "Sensor connected | %u%%", sensor_data.battery_percentage);
                    else
                        snprintf(sensor_text, sizeof(sensor_text), "Sensor connected");
                    break;
                case bluetooth::ConnectionState::Searching:
                    snprintf(sensor_text, sizeof(sensor_text), "Searching for sensor...");
                    break;
                case bluetooth::ConnectionState::Connecting:
                    snprintf(sensor_text, sizeof(sensor_text), "Connecting to sensor...");
                    break;
                default:
                    snprintf(sensor_text, sizeof(sensor_text),
                             "Sensor not connected\nWake sensor, then tap Search");
                    break;
            }
            lv_label_set_text(sensor_info_label, sensor_text);
            char wifi_text[100];
            wifi_files::display_status(wifi_text, sizeof(wifi_text));
            if (wifi_status_label && std::strcmp(lv_label_get_text(wifi_status_label), wifi_text) != 0)
                lv_label_set_text(wifi_status_label, wifi_text);
            const auto fit_state = fit_recorder::status();
            if (fit_state == fit_recorder::Status::Error &&
                previous_fit_status != fit_recorder::Status::Error) {
                show_fit_error("The FIT ride file could not be saved completely. Check the SD card.");
            }
            previous_fit_status = fit_state;
            if (fit_state == fit_recorder::Status::Unavailable)
                lv_label_set_text(sensor_info_label, "FIT recorder unavailable");
            else if (ride_data.active && fit_state == fit_recorder::Status::WaitingForTime)
                lv_label_set_text(sensor_info_label, "Recording | waiting for GPS time");
            else if (fit_state == fit_recorder::Status::Error)
                lv_label_set_text(sensor_info_label, "FIT write error | ride data at risk");
            if (state == bluetooth::ConnectionState::Connected) {
                lv_obj_set_hidden(sensor_button, true);
            } else {
                lv_obj_set_hidden(sensor_button, false);
                lv_label_set_text(sensor_button_label,
                    state == bluetooth::ConnectionState::Idle ? "Search" : "Cancel");
            }


            // -------------------------
            // Ride distance
            // -------------------------

            snprintf(
                distance_text,
                sizeof(distance_text),
                "%.2f km",
                ride_data.distance_km
            );

            lv_label_set_text(
                distance_label,
                distance_text
            );


            // -------------------------
            // Ride elapsed time
            // -------------------------

            uint32_t hours =
                ride_data.elapsed_seconds / 3600;

            uint32_t minutes =
                (ride_data.elapsed_seconds % 3600) / 60;

            uint32_t seconds =
                ride_data.elapsed_seconds % 60;

            if (hours > 0)
            {
                snprintf(
                    elapsed_text,
                    sizeof(elapsed_text),
                    "%lu:%02lu:%02lu",
                    static_cast<unsigned long>(hours),
                    static_cast<unsigned long>(minutes),
                    static_cast<unsigned long>(seconds)
                );
            }
            else
            {
                snprintf(
                    elapsed_text,
                    sizeof(elapsed_text),
                    "%02lu:%02lu",
                    static_cast<unsigned long>(minutes),
                    static_cast<unsigned long>(seconds)
                );
            }

            lv_label_set_text(
                elapsed_label,
                elapsed_text
            );
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void init()
{
    fit_recorder::init();
    display.init();
    display.initDMA();
    display.setSwapBytes(true);
    display.setRotation(1);
    display.setBrightness(255);

    lv_init();
    lv_tick_set_cb(ui_tick_ms);


    // -------------------------
    // LVGL display
    // -------------------------

    lv_display_t* lv_display =
        lv_display_create(320, 240);

    lv_display_set_color_format(lv_display, LV_COLOR_FORMAT_RGB565);

    lv_display_set_flush_cb(
        lv_display,
        lvgl_flush_cb
    );
    lv_display_set_flush_wait_cb(lv_display, lvgl_flush_wait_cb);
    lv_display_add_event_cb(lv_display, display_refresh_event, LV_EVENT_REFR_START, nullptr);
    lv_display_add_event_cb(lv_display, display_refresh_event, LV_EVENT_REFR_READY, nullptr);

    lv_display_set_buffers(
        lv_display,
        draw_buffer,
        draw_buffer_next,
        sizeof(draw_buffer),
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );


    // -------------------------
    // LVGL touch input
    // -------------------------

    lv_indev_t* touch_indev =
        lv_indev_create();

    lv_indev_set_type(
        touch_indev,
        LV_INDEV_TYPE_POINTER
    );

    lv_indev_set_read_cb(
        touch_indev,
        lvgl_touch_cb
    );

    lv_indev_set_display(
        touch_indev,
        lv_display
    );


    lv_obj_t* screen = lv_screen_active();
    dashboard_screen = screen;
    lv_obj_set_scrollable(dashboard_screen, false);


    // -------------------------
    // Left: Speed
    // -------------------------

    left_panel =
        lv_obj_create(screen);

    lv_obj_set_size(
        left_panel,
        160,
        165
    );
    lv_obj_set_scrollable(left_panel, false);

    lv_obj_align(
        left_panel,
        LV_ALIGN_TOP_LEFT,
        0,
        0
    );

    lv_obj_set_style_bg_opa(
        left_panel,
        LV_OPA_TRANSP,
        0
    );

    lv_obj_set_style_border_width(
        left_panel,
        0,
        0
    );

    speed_title =
        lv_label_create(left_panel);

    lv_label_set_text(
        speed_title,
        "SPEED"
    );

    lv_obj_align(
        speed_title,
        LV_ALIGN_TOP_LEFT,
        10,
        8
    );

    speed_label =
        lv_label_create(left_panel);

    lv_label_set_text(
        speed_label,
        "0.0"
    );

    lv_obj_set_style_text_font(
        speed_label,
        &lv_font_montserrat_48,
        0
    );

    lv_obj_align(
        speed_label,
        LV_ALIGN_TOP_LEFT,
        10,
        31
    );

    speed_unit =
        lv_label_create(left_panel);

    lv_label_set_text(
        speed_unit,
        "km/h"
    );

    lv_obj_align(
        speed_unit,
        LV_ALIGN_TOP_LEFT,
        10,
        88
    );


    // -------------------------
    // Right: Cadence
    // -------------------------

    right_panel =
        lv_obj_create(screen);

    lv_obj_set_size(
        right_panel,
        160,
        165
    );
    lv_obj_set_scrollable(right_panel, false);

    lv_obj_align(
        right_panel,
        LV_ALIGN_TOP_RIGHT,
        0,
        0
    );

    lv_obj_set_style_bg_opa(
        right_panel,
        LV_OPA_TRANSP,
        0
    );

    lv_obj_set_style_border_width(
        right_panel,
        0,
        0
    );

    cadence_title =
        lv_label_create(right_panel);

    lv_label_set_text(
        cadence_title,
        "CADENCE"
    );

    lv_obj_align(
        cadence_title,
        LV_ALIGN_TOP_LEFT,
        10,
        8
    );

    cadence_label =
        lv_label_create(right_panel);

    lv_label_set_text(
        cadence_label,
        "0"
    );

    lv_obj_set_style_text_font(
        cadence_label,
        &lv_font_montserrat_48,
        0
    );

    lv_obj_align(
        cadence_label,
        LV_ALIGN_TOP_LEFT,
        10,
        31
    );

    cadence_unit =
        lv_label_create(right_panel);

    lv_label_set_text(
        cadence_unit,
        "rpm"
    );

    lv_obj_align(
        cadence_unit,
        LV_ALIGN_TOP_LEFT,
        10,
        88
    );


    // -------------------------
    // Ride stats
    // -------------------------

    distance_label =
        lv_label_create(left_panel);

    lv_label_set_text(
        distance_label,
        "0.00 km"
    );
    lv_obj_set_style_text_font(distance_label, &lv_font_montserrat_24, 0);

    lv_obj_align(distance_label, LV_ALIGN_TOP_LEFT, 10, 122);


    elapsed_label =
        lv_label_create(right_panel);

    lv_label_set_text(
        elapsed_label,
        "00:00"
    );
    lv_obj_set_style_text_font(elapsed_label, &lv_font_montserrat_24, 0);

    lv_obj_align(elapsed_label, LV_ALIGN_TOP_LEFT, 10, 122);


    // -------------------------
    // Ride button
    // -------------------------

    ride_button =
        lv_button_create(screen);

    lv_obj_set_size(
        ride_button,
        90,
        34
    );

    lv_obj_align(
        ride_button,
        LV_ALIGN_BOTTOM_LEFT,
        8,
        -3
    );

    ride_button_label =
        lv_label_create(ride_button);

    lv_label_set_text(
        ride_button_label,
        "START"
    );

    lv_obj_center(
        ride_button_label
    );

    lv_obj_add_event_cb(
        ride_button,
        ride_button_event_cb,
        LV_EVENT_CLICKED,
        nullptr
    );


    // -------------------------
    // Sensors page navigation
    // -------------------------

    lv_obj_t* sensors_page_button = lv_button_create(screen);
    lv_obj_set_size(sensors_page_button, 80, 34);
    lv_obj_align(sensors_page_button, LV_ALIGN_BOTTOM_MID, 0, -3);
    lv_obj_t* sensors_page_label = lv_label_create(sensors_page_button);
    lv_label_set_text(sensors_page_label, "Sensors");
    lv_obj_center(sensors_page_label);
    lv_obj_add_event_cb(sensors_page_button, show_sensor_page_event_cb,
                       LV_EVENT_CLICKED, nullptr);

    // -------------------------
    // Sensors page
    // -------------------------

    sensor_screen = lv_obj_create(nullptr);
    lv_obj_set_scrollable(sensor_screen, false);

    lv_obj_t* sensor_page_title = lv_label_create(sensor_screen);
    lv_label_set_text(sensor_page_title, "SENSORS");
    lv_obj_align(sensor_page_title, LV_ALIGN_TOP_LEFT, 16, 18);

    wifi_status_label = lv_label_create(sensor_screen);
    lv_obj_set_width(wifi_status_label, 288);
    lv_obj_set_height(wifi_status_label, 42);
    lv_label_set_long_mode(wifi_status_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(wifi_status_label, "Starting Wi-Fi...");
    lv_obj_align(wifi_status_label, LV_ALIGN_TOP_LEFT, 16, 42);

    gps_status_label = lv_label_create(sensor_screen);
    lv_obj_set_width(gps_status_label, 288);
    lv_label_set_text(gps_status_label, "GPS disconnected");
    lv_obj_align(gps_status_label, LV_ALIGN_TOP_LEFT, 16, 88);

    sensor_info_label = lv_label_create(sensor_screen);
    lv_obj_set_width(sensor_info_label, 288);
    lv_obj_set_height(sensor_info_label, 48);
    lv_label_set_long_mode(sensor_info_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(sensor_info_label, "Searching for sensor...");
    lv_obj_align(sensor_info_label, LV_ALIGN_TOP_LEFT, 16, 110);

    sensor_button = lv_button_create(sensor_screen);
    lv_obj_set_size(sensor_button, 90, 36);
    lv_obj_align(sensor_button, LV_ALIGN_TOP_LEFT, 16, 164);
    sensor_button_label = lv_label_create(sensor_button);
    lv_label_set_text(sensor_button_label, "Cancel");
    lv_obj_center(sensor_button_label);
    lv_obj_add_event_cb(sensor_button, sensor_button_event_cb,
                       LV_EVENT_CLICKED, nullptr);

    wifi_start_button = lv_button_create(sensor_screen);
    lv_obj_set_size(wifi_start_button, 112, 34);
    lv_obj_align(wifi_start_button, LV_ALIGN_BOTTOM_RIGHT, -8, -3);
    lv_obj_t* wifi_start_label = lv_label_create(wifi_start_button);
    lv_label_set_text(wifi_start_label, "Start Wi-Fi");
    lv_obj_center(wifi_start_label);
    lv_obj_add_event_cb(wifi_start_button, wifi_start_event_cb,
                       LV_EVENT_CLICKED, nullptr);

    lv_obj_t* back_button = lv_button_create(sensor_screen);
    lv_obj_set_size(back_button, 76, 34);
    lv_obj_align(back_button, LV_ALIGN_BOTTOM_LEFT, 8, -3);
    lv_obj_t* back_label = lv_label_create(back_button);
    lv_label_set_text(back_label, "Back");
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back_button, show_dashboard_page_event_cb,
                       LV_EVENT_CLICKED, nullptr);


    map_screen::create(screen);


    // -------------------------
    // Start UI task
    // -------------------------

    xTaskCreate(
        ui_task,
        "ui_task",
        8192,
        nullptr,
        5,
        nullptr
    );
}

} // namespace ui
