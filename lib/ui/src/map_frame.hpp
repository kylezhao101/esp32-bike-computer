#pragma once

#include <lvgl.h>
#include <cstddef>

namespace ui::map_frame {
constexpr int width = 320;
constexpr int height = 240;
constexpr int zoom = 16;
constexpr int max_pieces = 6;
constexpr size_t storage_bytes = 40 * 1024;

struct Piece {
    lv_image_dsc_t image{};
    int x = 0;
    int y = 0;
};

struct Frame {
    Piece pieces[max_pieces]{};
    uint8_t* storage = nullptr;
    int left = 0;
    int top = 0;
    int count = 0;
    int missing = 0;
    int legacy_tiles = 0;
    bool failed = false;
};

// UI-thread API. The worker owns a requested buffer until poll() returns its
// index; it never calls LVGL. Callers must detach every image using a buffer
// before requesting it again or releasing the cache.
bool init();
void release();
bool request(int index, int left, int top);
int poll(); // -1 while idle/loading
const Frame& get(int index);
} // namespace ui::map_frame
