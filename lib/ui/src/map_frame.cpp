#include "map_frame.hpp"
#include "sd_access.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

#if LV_BIN_DECODER_RAM_LOAD
#error "Map I4 frames require row decoding; whole-image ARGB expansion exceeds the RAM budget"
#endif

namespace ui::map_frame {
namespace {
constexpr int tile_size = 256;
constexpr int tile_count = 1 << zoom;
constexpr size_t palette_bytes = 16 * 4;
// Opaque map-oriented fallback for existing RGB565 files. Converted I4 files
// carry their own adaptive palette instead and avoid this quantization step.
constexpr uint32_t legacy_palette[16] = {
    0xffffffff, 0xfff2efe9, 0xffe8e4dd, 0xffd6d3cc,
    0xffb8b6b0, 0xff808080, 0xff404040, 0xff202020,
    0xffcdebb0, 0xffaad19e, 0xff6da66b, 0xffaad3df,
    0xff69a8cf, 0xfff7d6a0, 0xfff6f0c5, 0xffe8b4b4,
};
struct Request { int index; int left; int top; };
Frame frames[2];
QueueHandle_t requests = nullptr;
QueueHandle_t completed = nullptr;
bool busy = false; // UI-thread only; queues transfer frame ownership.
TaskHandle_t loader_task = nullptr;
constexpr size_t read_buffer_bytes = 4096;
constexpr size_t legacy_lookup_bytes = 4096;
// Allocate worker scratch from the heap: ESP32's linker-accessible static
// DRAM region is smaller than the total RAM available to the heap allocator.
uint8_t* read_buffer = nullptr;
uint8_t* legacy_lookup = nullptr;

int floor_tile(int pixel)
{
    int result = pixel / tile_size;
    if (pixel < 0 && pixel % tile_size) --result;
    return result;
}

bool make_legacy_lookup()
{
    if (legacy_lookup) return true;
    legacy_lookup = static_cast<uint8_t*>(heap_caps_malloc(legacy_lookup_bytes, MALLOC_CAP_8BIT));
    if (!legacy_lookup) return false;
    for (int key = 0; key < 4096; ++key) {
        const int r = ((key >> 8) & 15) * 17;
        const int g = ((key >> 4) & 15) * 17;
        const int b = (key & 15) * 17;
        int best = 0;
        int distance = 0x7fffffff;
        for (int i = 0; i < 16; ++i) {
            const int dr = r - int((legacy_palette[i] >> 16) & 255);
            const int dg = g - int((legacy_palette[i] >> 8) & 255);
            const int db = b - int(legacy_palette[i] & 255);
            const int d = dr * dr + dg * dg + db * db;
            if (d < distance) { distance = d; best = i; }
        }
        legacy_lookup[key] = best;
    }
    return true;
}

bool read_piece(Piece& piece, uint8_t* dest, int tile_x, int tile_y,
                int crop_x, int crop_y, int w, int h, bool& legacy)
{
    char path[96];
    const int wrapped_x = (tile_x % tile_count + tile_count) % tile_count;
    std::snprintf(path, sizeof(path), "/sdcard/tiles1/%d/%d/%d.bin", zoom, wrapped_x, tile_y);
    if (!sd_access::lock(portMAX_DELAY)) return false;
    FILE* file = std::fopen(path, "rb");
    if (!file) {
        sd_access::unlock();
        ESP_LOGW("map_frame", "Missing tile %s", path);
        return false;
    }
    lv_image_header_t header{};
    static_assert(sizeof(header) == 12, "Tile format requires a 12-byte header");
    bool ok = std::fread(&header, 1, sizeof(header), file) == sizeof(header);
    const bool indexed = header.cf == LV_COLOR_FORMAT_I4;
    const int source_stride = indexed ? 128 : 512;
    const long pixels_offset = sizeof(header) + (indexed ? palette_bytes : 0);
    const long expected_size = pixels_offset + source_stride * tile_size;
    ok = ok && header.magic == LV_IMAGE_HEADER_MAGIC && header.flags == 0 &&
         header.reserved_2 == 0 && header.w == tile_size && header.h == tile_size &&
         (indexed || header.cf == LV_COLOR_FORMAT_RGB565) && header.stride == source_stride;
    ok = ok && std::fseek(file, 0, SEEK_END) == 0 && std::ftell(file) == expected_size;
    if (ok && !indexed && !make_legacy_lookup()) {
        ESP_LOGE("map_frame", "Could not allocate 4 KiB RGB565 lookup table");
        ok = false;
    }
    if (ok && indexed) {
        ok = std::fseek(file, sizeof(header), SEEK_SET) == 0 &&
             std::fread(dest, 1, palette_bytes, file) == palette_bytes;
        for (int i = 0; ok && i < 16; ++i) ok = dest[i * 4 + 3] == 255;
    } else if (ok) {
        std::memcpy(dest, legacy_palette, palette_bytes);
    }
    const int stride = (w + 1) / 2;
    if (ok) {
        std::memset(dest + palette_bytes, 0, stride * h);
        ok = std::fseek(file, pixels_offset + crop_y * source_stride, SEEK_SET) == 0;
    }
    for (int row = 0; ok && row < h;) {
        const int rows = std::min(h - row, int(read_buffer_bytes) / source_stride);
        const size_t bytes = rows * source_stride;
        ok = std::fread(read_buffer, 1, bytes, file) == bytes;
        if (!ok) break;
        for (int y = 0; y < rows; ++y) {
            const uint8_t* src = read_buffer + y * source_stride;
            uint8_t* out = dest + palette_bytes + (row + y) * stride;
            for (int x = 0; x < w; ++x) {
                const int sx = crop_x + x;
                uint8_t index;
                if (indexed) {
                    index = (src[sx / 2] >> ((sx & 1) ? 0 : 4)) & 15;
                } else {
                    const uint16_t color = src[sx * 2] | (uint16_t(src[sx * 2 + 1]) << 8);
                    const int key = ((color >> 4) & 0xf00) | ((color >> 3) & 0xf0) | ((color >> 1) & 15);
                    index = legacy_lookup[key];
                }
                out[x / 2] |= index << ((x & 1) ? 0 : 4);
            }
        }
        row += rows;
    }
    std::fclose(file);
    sd_access::unlock();
    if (!ok) {
        ESP_LOGW("map_frame", "Invalid/unreadable tile %s (expected opaque I4 or RGB565)", path);
        return false;
    }
    piece.image = {};
    piece.image.header.magic = LV_IMAGE_HEADER_MAGIC;
    piece.image.header.cf = LV_COLOR_FORMAT_I4;
    piece.image.header.w = w;
    piece.image.header.h = h;
    piece.image.header.stride = stride;
    piece.image.data = dest;
    piece.image.data_size = palette_bytes + stride * h;
    legacy = !indexed;
    return true;
}

void load(Frame& frame, const Request& request)
{
    frame.left = request.left;
    frame.top = request.top;
    frame.count = frame.missing = frame.legacy_tiles = 0;
    frame.failed = false;
    size_t used = 0;
    const int right = frame.left + width;
    const int bottom = frame.top + height;
    for (int ty = floor_tile(frame.top); ty <= floor_tile(bottom - 1); ++ty) {
        for (int tx = floor_tile(frame.left); tx <= floor_tile(right - 1); ++tx) {
            if (ty < 0 || ty >= tile_count) { ++frame.missing; continue; }
            const int x = std::max(frame.left, tx * tile_size);
            const int y = std::max(frame.top, ty * tile_size);
            const int w = std::min(right, (tx + 1) * tile_size) - x;
            const int h = std::min(bottom, (ty + 1) * tile_size) - y;
            used = (used + 3) & ~size_t(3); // Align each BGRA palette.
            const size_t bytes = palette_bytes + ((w + 1) / 2) * h;
            if (frame.count == max_pieces || used + bytes > storage_bytes) {
                frame.failed = true;
                return;
            }
            Piece& piece = frame.pieces[frame.count];
            bool legacy = false;
            if (read_piece(piece, frame.storage + used, tx, ty,
                           x - tx * tile_size, y - ty * tile_size, w, h, legacy)) {
                piece.x = x - frame.left;
                piece.y = y - frame.top;
                used += bytes;
                ++frame.count;
                if (legacy) ++frame.legacy_tiles;
            } else {
                ++frame.missing;
            }
        }
    }
}

void loader(void*)
{
    Request request;
    for (;;) {
        if (xQueueReceive(requests, &request, portMAX_DELAY) != pdTRUE) continue;
        if (request.index < 0) {
            xQueueSend(completed, &request.index, portMAX_DELAY);
            loader_task = nullptr;
            vTaskDelete(nullptr);
        }
        const int64_t start = esp_timer_get_time();
        Frame& frame = frames[request.index];
        load(frame, request);
        ESP_LOGI("map_frame", "Prepared viewport in %lld ms: %d pieces, %d missing, %d RGB565 conversions",
                 (esp_timer_get_time() - start) / 1000, frame.count, frame.missing, frame.legacy_tiles);
        xQueueSend(completed, &request.index, portMAX_DELAY);
    }
}
} // namespace

bool init()
{
    if (requests) return true;
    ESP_LOGI("map_frame", "Before cache: free %u, largest %u bytes",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    for (Frame& frame : frames) {
        frame.storage = static_cast<uint8_t*>(heap_caps_malloc(storage_bytes, MALLOC_CAP_8BIT));
    }
    read_buffer = static_cast<uint8_t*>(heap_caps_malloc(read_buffer_bytes, MALLOC_CAP_8BIT));
    requests = xQueueCreate(1, sizeof(Request));
    completed = xQueueCreate(1, sizeof(int));
    // Retain headroom for task stacks, BLE, LVGL and filesystem allocations.
    const size_t remaining = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    const char* failure = nullptr;
    if (!frames[0].storage || !frames[1].storage) failure = "40 KiB viewport allocation";
    else if (!read_buffer) failure = "4 KiB read-buffer allocation";
    else if (!requests || !completed) failure = "queue allocation";
    else if (remaining < 24 * 1024) failure = "less than 24 KiB remaining headroom";
    else if (xTaskCreate(loader, "map_loader", 4096, nullptr, 3, &loader_task) != pdPASS)
        failure = "loader task allocation";
    if (!failure) {
        ESP_LOGI("map_frame", "Two 40 KiB I4 viewport buffers ready");
        return true;
    }
    ESP_LOGE("map_frame", "Cache failed: %s; free %u, largest %u bytes; "
             "viewport allocations %d/%d, read buffer %d, lazy RGB565 lookup %d",
             failure, static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
             frames[0].storage != nullptr, frames[1].storage != nullptr,
             read_buffer != nullptr, legacy_lookup != nullptr);
    for (Frame& frame : frames) { heap_caps_free(frame.storage); frame.storage = nullptr; }
    heap_caps_free(read_buffer);
    heap_caps_free(legacy_lookup);
    read_buffer = legacy_lookup = nullptr;
    if (requests) vQueueDelete(requests);
    if (completed) vQueueDelete(completed);
    requests = completed = nullptr;
    ESP_LOGE("map_frame", "Not enough RAM/resources for indexed map cache");
    return false;
}

void release()
{
    if (!requests) return;
    if (loader_task) {
        const Request stop{-1, 0, 0};
        xQueueSend(requests, &stop, portMAX_DELAY);
        int completed_index;
        do {
            xQueueReceive(completed, &completed_index, portMAX_DELAY);
        } while (completed_index != -1);
    }
    busy = false;
    for (Frame& frame : frames) {
        heap_caps_free(frame.storage);
        frame.storage = nullptr;
        frame.count = 0;
    }
    heap_caps_free(read_buffer);
    heap_caps_free(legacy_lookup);
    read_buffer = legacy_lookup = nullptr;
    vQueueDelete(requests);
    vQueueDelete(completed);
    requests = completed = nullptr;
}

bool request(int index, int left, int top)
{
    if (!requests || busy || index < 0 || index > 1) return false;
    const Request request{index, left, top};
    if (xQueueSend(requests, &request, 0) != pdTRUE) return false;
    busy = true;
    return true;
}

int poll()
{
    int index;
    if (!completed || xQueueReceive(completed, &index, 0) != pdTRUE) return -1;
    busy = false;
    return index;
}

const Frame& get(int index) { return frames[index]; }
} // namespace ui::map_frame
