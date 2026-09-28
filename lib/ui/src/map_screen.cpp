#include "map_screen.hpp"
#include "map_frame.hpp"
#include "map_geo.h"
#include "esp_timer.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ui::map_screen {
namespace {
constexpr int width = map_frame::width;
constexpr int height = map_frame::height;
constexpr int zoom = map_frame::zoom;
// Keep the map still for small GPS movements; move the marker instead.
constexpr int recenter_distance_px = 12;

lv_obj_t* images[map_frame::max_pieces]{};
lv_obj_t* viewport = nullptr;
lv_obj_t* marker = nullptr;
lv_obj_t* status = nullptr;
bool cache_ready = false;
bool requested = false;
bool load_failed = false;
int active_frame = -1;
int loading_frame = -1;
int requested_left = 0;
int requested_top = 0;
bool had_fix = false;
bool marker_has_fix = false;
int fix_x = 0;
int fix_y = 0;
int64_t last_update_us = 0;

void set_status(const char* text)
{
    if (std::strcmp(lv_label_get_text(status), text) != 0)
        lv_label_set_text(status, text);
}

void accept_frame(int index)
{
    const auto& frame = map_frame::get(index);
    load_failed = frame.failed;
    if (frame.failed) return; // Keep the previously completed viewport.
    for (int i = 0; i < map_frame::max_pieces; ++i) {
        if (i < frame.count) {
            const auto& piece = frame.pieces[i];
            lv_image_set_src(images[i], &piece.image);
            lv_obj_set_pos(images[i], piece.x, piece.y);
            lv_obj_set_hidden(images[i], false);
        } else {
            lv_obj_set_hidden(images[i], true);
            lv_image_set_src(images[i], nullptr);
        }
    }
    // No image object references the previous buffer now. With RAM_LOAD=0,
    // indexed images are row-decoded without caching an expanded image.
    active_frame = index;
}

void request_frame(int left, int top)
{
    if (loading_frame >= 0) return;
    if (requested && std::abs(left - requested_left) < recenter_distance_px &&
        std::abs(top - requested_top) < recenter_distance_px) return;
    const int next = active_frame == 0 ? 1 : 0;
    if (!map_frame::request(next, left, top)) return;
    loading_frame = next;
    requested_left = left;
    requested_top = top;
    requested = true;
}

void toggle(lv_event_t* event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    if (lv_obj_is_hidden(viewport)) {
        lv_obj_set_hidden(viewport, false);
        last_update_us = 0;
        update(gps::get_state());
    } else {
        lv_obj_set_hidden(viewport, true);
    }
}

void button(lv_obj_t* parent, const char* text, lv_align_t align)
{
    lv_obj_t* obj = lv_button_create(parent);
    lv_obj_set_size(obj, 64, 32);
    lv_obj_align(obj, align, -6, -4);
    lv_obj_t* label = lv_label_create(obj);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    lv_obj_add_event_cb(obj, toggle, LV_EVENT_CLICKED, nullptr);
}
} // namespace

void create(lv_obj_t* parent)
{
    button(parent, "Map", LV_ALIGN_BOTTOM_RIGHT);
    viewport = lv_obj_create(parent);
    lv_obj_remove_style_all(viewport);
    lv_obj_set_size(viewport, width, height);
    lv_obj_set_pos(viewport, 0, 0);
    lv_obj_set_scrollable(viewport, false);
    lv_obj_set_style_bg_color(viewport, lv_color_hex(0x30343b), 0);
    lv_obj_set_style_bg_opa(viewport, LV_OPA_COVER, 0);

    for (auto& image : images) {
        image = lv_image_create(viewport);
        lv_obj_set_hidden(image, true);
        lv_obj_set_clickable(image, false);
    }
    marker = lv_obj_create(viewport);
    lv_obj_remove_style_all(marker);
    lv_obj_set_size(marker, 12, 12);
    lv_obj_set_pos(marker, width / 2 - 6, height / 2 - 6);
    lv_obj_set_style_radius(marker, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(marker, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(marker, lv_color_hex(0x168bff), 0);
    lv_obj_set_style_border_color(marker, lv_color_white(), 0);
    lv_obj_set_style_border_width(marker, 2, 0);
    lv_obj_set_hidden(marker, true);
    lv_obj_set_clickable(marker, false);
    lv_obj_set_scrollable(marker, false);

    status = lv_label_create(viewport);
    lv_obj_set_style_bg_color(status, lv_color_hex(0x20242b), 0);
    lv_obj_set_style_bg_opa(status, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(status, lv_color_white(), 0);
    lv_obj_set_style_pad_all(status, 4, 0);
    lv_obj_align(status, LV_ALIGN_TOP_LEFT, 4, 4);
    lv_label_set_text(status, "Waiting for GPS");
    button(viewport, "Back", LV_ALIGN_BOTTOM_RIGHT);

    // Start after creating the UI so the cache's heap check includes its cost.
    cache_ready = map_frame::init();
    update(gps::get_state());
}

void update(const gps::State& position)
{
    if (!viewport) return;
    // Consume completions even while the map is hidden. Returning to it uses
    // the prepared RAM image instead of reopening tile files on the UI thread.
    const int completed = map_frame::poll();
    if (completed >= 0) {
        loading_frame = -1;
        accept_frame(completed);
    }
    const int64_t now = esp_timer_get_time();
    if (completed < 0 && last_update_us && now - last_update_us < 250000) return;
    last_update_us = now;
    if (!cache_ready) {
        set_status("Not enough memory for map");
        return;
    }
    const bool valid = position.receiving && position.has_fix &&
                       std::isfinite(position.latitude) && std::isfinite(position.longitude) &&
                       position.latitude >= -90 && position.latitude <= 90 &&
                       position.longitude >= -180 && position.longitude <= 180;
    if (valid) {
        fix_x = static_cast<int>(std::lround(map_geo_lon_to_world_px(position.longitude, zoom)));
        fix_y = static_cast<int>(std::lround(map_geo_lat_to_world_py(position.latitude, zoom)));
        had_fix = true;
    }
    // Before a fix, retain the explicitly labelled preview of the test area.
    const int center_x = had_fix ? fix_x : static_cast<int>(std::lround(
        map_geo_lon_to_world_px(-123.12683, zoom)));
    const int center_y = had_fix ? fix_y : static_cast<int>(std::lround(
        map_geo_lat_to_world_py(49.21486, zoom)));
    request_frame(center_x - width / 2, center_y - height / 2);

    int missing_tiles = 0;
    if (active_frame >= 0) {
        const auto& frame = map_frame::get(active_frame);
        missing_tiles = frame.missing;
        const int x = fix_x - frame.left;
        const int y = fix_y - frame.top;
        // Do not pin a stale/far-away fix to the center while a new map loads.
        const bool visible = had_fix && x >= 0 && x < width && y >= 0 && y < height;
        if (visible) lv_obj_set_pos(marker, x - 6, y - 6);
        lv_obj_set_hidden(marker, !visible);
    }
    if (valid) {
        if (!marker_has_fix)
            lv_obj_set_style_bg_color(marker, lv_color_hex(0x168bff), 0);
    } else if (marker_has_fix) {
        lv_obj_set_style_bg_color(marker, lv_color_hex(0x999999), 0);
    }
    marker_has_fix = valid;
    char text[96];
    if (load_failed) {
        std::snprintf(text, sizeof(text), "Map update failed");
    } else if (active_frame < 0) {
        std::snprintf(text, sizeof(text), "Loading map | %s", had_fix ? "GPS" : "Preview");
    } else if (valid) {
        if (missing_tiles)
            std::snprintf(text, sizeof(text), "GPS | z%d | %d tiles missing", zoom, missing_tiles);
        else
            std::snprintf(text, sizeof(text), "GPS | %d sats | z%d", position.satellites, zoom);
    } else {
        std::snprintf(text, sizeof(text), "%s | %s",
                      position.receiving ? "Waiting for fix" : "GPS disconnected",
                      had_fix ? "Last position" : "Preview");
    }
    set_status(text);
}
} // namespace ui::map_screen
