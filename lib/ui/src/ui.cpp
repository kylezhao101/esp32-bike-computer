#include "ui.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.hpp"
#include <lvgl.h>

#include "bluetooth.hpp"
#include "ride.hpp"
#include "gps.hpp"

namespace ui
{

static LGFX display;

static uint16_t draw_buffer[320 * 20];

static lv_obj_t* left_panel = nullptr;
static lv_obj_t* right_panel = nullptr;

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


static lv_obj_t* ride_button = nullptr;
static lv_obj_t* ride_button_label = nullptr;

static lv_obj_t* sensor_info_label = nullptr;
static lv_obj_t* gps_status_label = nullptr;
static lv_obj_t* sensor_button = nullptr;
static lv_obj_t* sensor_button_label = nullptr;

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
    lv_display_t* lv_display,
    const lv_area_t* area,
    uint8_t* px_map
)
{
    int32_t width =
        area->x2 - area->x1 + 1;

    int32_t height =
        area->y2 - area->y1 + 1;

    display.startWrite();

    display.setAddrWindow(
        area->x1,
        area->y1,
        width,
        height
    );

    display.pushPixels(
        reinterpret_cast<uint16_t*>(px_map),
        width * height
    );

    display.endWrite();

    lv_display_flush_ready(lv_display);
}


// -------------------------
// Ride button
// -------------------------

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
        update_ride();
        ride::stop();

        lv_label_set_text(
            ride_button_label,
            "START"
        );
    }
    else
    {
        ride::start();
        update_ride();

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

    int ui_update_counter = 0;

    while (true)
    {
        lv_tick_inc(10);
        lv_timer_handler();

        ui_update_counter += 10;

        if (ui_update_counter >= 100)
        {
            ui_update_counter = 0;

            bluetooth::SensorData sensor_data =
                bluetooth::get_sensor_data();

            update_ride();

            ride::RideData ride_data =
                ride::get_ride_data();

            gps::State gps_data = gps::get_state();

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
            if (state == bluetooth::ConnectionState::Connected) {
                lv_obj_add_flag(sensor_button, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_remove_flag(sensor_button, LV_OBJ_FLAG_HIDDEN);
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
    display.init();
    display.setSwapBytes(true);
    display.setRotation(1);
    display.setBrightness(255);

    lv_init();


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

    lv_display_set_buffers(
        lv_display,
        draw_buffer,
        nullptr,
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


    lv_obj_t* screen =
        lv_screen_active();


    // -------------------------
    // Left: Speed
    // -------------------------

    left_panel =
        lv_obj_create(screen);

    lv_obj_set_size(
        left_panel,
        190,
        150
    );

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
        LV_ALIGN_LEFT_MID,
        10,
        -8
    );

    speed_unit =
        lv_label_create(left_panel);

    lv_label_set_text(
        speed_unit,
        "km/h"
    );

    lv_obj_set_style_text_font(
        speed_unit,
        &lv_font_montserrat_24,
        0
    );

    lv_obj_align(
        speed_unit,
        LV_ALIGN_BOTTOM_LEFT,
        10,
        -10
    );


    // -------------------------
    // Right: Cadence
    // -------------------------

    right_panel =
        lv_obj_create(screen);

    lv_obj_set_size(
        right_panel,
        130,
        150
    );

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
        5,
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
        LV_ALIGN_LEFT_MID,
        5,
        -8
    );

    cadence_unit =
        lv_label_create(right_panel);

    lv_label_set_text(
        cadence_unit,
        "rpm"
    );

    lv_obj_set_style_text_font(
        cadence_unit,
        &lv_font_montserrat_24,
        0
    );

    lv_obj_align(
        cadence_unit,
        LV_ALIGN_BOTTOM_LEFT,
        5,
        -10
    );


    // -------------------------
    // Ride stats
    // -------------------------

    distance_label =
        lv_label_create(screen);

    lv_label_set_text(
        distance_label,
        "0.00 km"
    );

    lv_obj_align(
        distance_label,
        LV_ALIGN_BOTTOM_MID,
        -45,
        -42
    );


    elapsed_label =
        lv_label_create(screen);

    lv_label_set_text(
        elapsed_label,
        "00:00"
    );

    lv_obj_align(
        elapsed_label,
        LV_ALIGN_BOTTOM_MID,
        55,
        -42
    );


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
    // Sensor info
    // -------------------------

    sensor_info_label = lv_label_create(screen);
    lv_obj_set_width(sensor_info_label, 220);
    lv_label_set_text(sensor_info_label, "Searching for sensor...");
    lv_obj_align(sensor_info_label, LV_ALIGN_TOP_LEFT, 8, 151);

    gps_status_label = lv_label_create(screen);
    lv_label_set_text(gps_status_label, "GPS disconnected");
    lv_obj_align(gps_status_label, LV_ALIGN_TOP_LEFT, 8, 184);

    sensor_button = lv_button_create(screen);
    lv_obj_set_size(sensor_button, 76, 34);
    lv_obj_align(sensor_button, LV_ALIGN_TOP_RIGHT, -8, 150);
    sensor_button_label = lv_label_create(sensor_button);
    lv_label_set_text(sensor_button_label, "Cancel");
    lv_obj_center(sensor_button_label);
    lv_obj_add_event_cb(sensor_button, sensor_button_event_cb,
                       LV_EVENT_CLICKED, nullptr);


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
