#include "map_screen.hpp"
#include "map_tiles.h"
#include "map_geo.h"
#include "esp_timer.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ui::map_screen {
namespace {
constexpr int width = 320;
constexpr int height = 240;
constexpr int tile_size = MAP_TILES_TILE_SIZE;
constexpr int zoom = 16;
constexpr int tile_count = 1 << zoom;
constexpr int columns = 3;
constexpr int rows = 2;

struct Tile {
    lv_obj_t* image = nullptr;
    int x = -1;
    int y = -1;
    bool available = false;
};

Tile tiles[columns * rows];
map_tiles_handle_t map = nullptr;
lv_obj_t* viewport = nullptr;
lv_obj_t* marker = nullptr;
lv_obj_t* status = nullptr;
bool positioned = false;
bool had_fix = false;
int previous_left = 0;
int previous_top = 0;
int missing_tiles = 0;
int64_t last_update_us = 0;

int wrap(int value, int modulus)
{
    return (value % modulus + modulus) % modulus;
}

void set_status(const char* text)
{
    if (std::strcmp(lv_label_get_text(status), text) != 0)
        lv_label_set_text(status, text);
}

void position_tiles(double latitude, double longitude)
{
    // Round the shared viewport origin once to keep all tile edges adjacent.
    const int left = static_cast<int>(std::lround(
        map_geo_lon_to_world_px(longitude, zoom))) - width / 2;
    const int top = static_cast<int>(std::lround(
        map_geo_lat_to_world_py(latitude, zoom))) - height / 2;
    if (positioned && left == previous_left && top == previous_top) return;
    previous_left = left;
    previous_top = top;
    positioned = true;

    const int first_x = static_cast<int>(std::floor(double(left) / tile_size));
    const int last_x = static_cast<int>(std::floor(double(left + width - 1) / tile_size));
    const int first_y = static_cast<int>(std::floor(double(top) / tile_size));
    const int last_y = static_cast<int>(std::floor(double(top + height - 1) / tile_size));
    bool visible[columns * rows]{};
    missing_tiles = 0;

    for (int y = first_y; y <= last_y; ++y) {
        if (y < 0 || y >= tile_count) continue; // No vertical wrapping at the poles.
        for (int x = first_x; x <= last_x; ++x) {
            // Stable slots keep overlapping tiles when crossing a tile boundary.
            const int slot = wrap(y, rows) * columns + wrap(x, columns);
            Tile& tile = tiles[slot];
            const int wrapped_x = wrap(x, tile_count);
            if (tile.x != wrapped_x || tile.y != y) {
                tile.x = wrapped_x;
                tile.y = y;
                char source[260];
                tile.available = map_tiles_get_file_source(map, wrapped_x, y,
                                                           source, sizeof(source));
                if (tile.available) lv_image_set_src(tile.image, source);
            }
            visible[slot] = tile.available;
            if (tile.available) {
                lv_obj_set_pos(tile.image, x * tile_size - left, y * tile_size - top);
            } else {
                ++missing_tiles;
            }
        }
    }
    for (int i = 0; i < columns * rows; ++i) {
        lv_obj_set_hidden(tiles[i].image, !visible[i]);
    }
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
    map_tiles_config_t config{};
    config.base_path = "/sdcard";
    config.tile_folders[0] = "tiles1";
    config.tile_type_count = 1;
    config.default_zoom = zoom;
    config.grid_cols = 1;
    config.grid_rows = 1;
    map = map_tiles_init(&config); // Coordinate/file metadata only, no tile buffers.

    button(parent, "Map", LV_ALIGN_BOTTOM_RIGHT);
    viewport = lv_obj_create(parent);
    lv_obj_remove_style_all(viewport);
    lv_obj_set_size(viewport, width, height);
    lv_obj_set_pos(viewport, 0, 0);
    lv_obj_set_scrollable(viewport, false);
    lv_obj_set_style_bg_color(viewport, lv_color_hex(0x30343b), 0);
    lv_obj_set_style_bg_opa(viewport, LV_OPA_COVER, 0);

    for (Tile& tile : tiles) {
        tile.image = lv_image_create(viewport);
        lv_obj_set_hidden(tile.image, true);
        lv_obj_set_clickable(tile.image, false);
    }
    marker = lv_obj_create(viewport);
    lv_obj_remove_style_all(marker);
    lv_obj_set_size(marker, 12, 12);
    lv_obj_center(marker);
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

    // Preserve the previous test area as an explicitly labelled preview until
    // GPS acquires a fix. Never show a live-position marker at this fallback.
    if (map) position_tiles(49.21486, -123.12683);
    update(gps::get_state());
}

void update(const gps::State& position)
{
    if (!viewport || lv_obj_is_hidden(viewport)) return;
    const int64_t now = esp_timer_get_time();
    if (last_update_us && now - last_update_us < 250000) return;
    last_update_us = now;
    if (!map) {
        set_status("Map unavailable");
        return;
    }
    const bool valid = position.receiving && position.has_fix &&
                       std::isfinite(position.latitude) && std::isfinite(position.longitude) &&
                       position.latitude >= -90 && position.latitude <= 90 &&
                       position.longitude >= -180 && position.longitude <= 180;
    if (valid) {
        position_tiles(position.latitude, position.longitude);
        had_fix = true;
        lv_obj_set_hidden(marker, false);
        lv_obj_set_style_bg_color(marker, lv_color_hex(0x168bff), 0);
    } else if (had_fix) {
        lv_obj_set_style_bg_color(marker, lv_color_hex(0x999999), 0);
    }
    char text[96];
    if (valid) {
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
