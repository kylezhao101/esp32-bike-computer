# Cached map hardware check

Build/upload the esp32dev environment, then monitor at 115200 baud. Keep the SD
layout `/tiles1/16/<x>/<y>.bin` and include neighboring tiles along the test ride.
Both opaque I4 and the original RGB565 tile files are accepted, including a mix.
For best loading speed and color fidelity, generate I4 from the original PNGs:

```powershell
python components/map_tiles/script/lvgl_map_tile_converter.py --input ./map_tiles --output ./tiles1 --format i4 --force
```

Copy `tiles1` to the card. I4 preserves 256x256 pixels but uses 16 colors per
tile. Existing RGB565 files require no conversion to try the firmware; they use
a fixed 16-color map palette selected by the background task.

## Expected behavior

- `SD card mounted successfully` and `Two 40 KiB I4 viewport buffers ready`
  appear at boot. Failure to allocate the cache must show a memory error while
  leaving the dashboard and Back button usable.
- On first use, a loading message appears until the worker prepares a viewport.
  Before a GPS fix it uses the labelled Preview location (49.21486, -123.12683)
  with no position marker. After loading, drawing reads only RAM.
- Back/Map repeatedly at the same position: reopening displays the cached map
  without a new `Prepared viewport` log or SD read. Test responsiveness while
  Bluetooth is connected and during a pending load as well.
- Walk/ride: small movements move the blue dot; a displacement of 12 pixels on
  either axis requests another viewport. The old map remains visible while it
  loads. Newly completed fragments appear together in the next UI refresh, with
  no gray gaps at tile boundaries. SPI still sends the screen in strips; this
  removes SD pauses, not the panel's physical transfer time.
- Cross a tile edge and a corner, including odd pixel offsets: roads and labels
  should remain aligned and colors should not swap or show alternating pixels.
- Close the map, move, then reopen it: GPS updates continue to prepare the cache
  while hidden. If a load is pending, the most recently completed map is shown.
- Disconnect GPS after a fix: retain the last location and turn its marker gray.
  Reconnect: the marker turns blue and following resumes. A jump outside the
  cached viewport hides the marker until it can be drawn at the correct location.
- Repeat movement and screen toggles long enough to exercise both buffers many
  times. No progressive heap loss, corrupt fragments, reset or watchdog timeout
  should occur. The worker must never modify the displayed buffer.

## Timing and memory

`LV_USE_STDLIB_MALLOC=1` selects LVGL's C-library allocator, which uses the
ESP-IDF heap. This removes the private 64 KiB LVGL arena; the UI allocates only
what it needs from the same heap as the map. This does not create extra physical
RAM, but makes unused arena capacity available to the rest of the firmware.
Cache initialization logs free heap and the largest block before allocation.
Failures identify viewport/scratch/queue allocation, the 32 KiB headroom check,
or task creation, and report the remaining heap before cleanup.

`map_frame` logs background preparation time, piece count, missing tiles and
legacy RGB565 conversions. `ui_perf` logs actual UI refresh avg/max duration,
DMA wait time, pixel count and free/largest heap. Compare cold-load preparation
with cached refreshes separately. No on-device timing measurements have been
made as part of this change.

Two 40 KiB allocations hold clipped I4 fragments covering a 320x240 viewport;
up to six fragments each retain their own palette. The worker has an additional
heap-allocated 4 KiB SD read buffer and 4 KiB legacy palette lookup, and a
4 KiB task stack, plus
small descriptors, queues and filesystem overhead. No complete 128 KiB RGB565
tile or ARGB8888 map is allocated. The two 12.5 KiB DMA display buffers remain.

Keep `LV_BIN_DECODER_RAM_LOAD=0`: LVGL expands only the current image row from
cached I4 pixels, avoiding a full ARGB8888 image allocation. The UI performs all
LVGL operations; one worker performs all map-file access. Queue handoffs transfer
exclusive ownership of the inactive buffer. The UI detaches every old image
source before that buffer can be submitted for another load.

## Failure checks

Boot with a missing tile, a truncated file, an unsupported color format, and an
I4 palette containing a nonopaque entry. Each should log the rejected path and
leave that region gray while adjacent tiles render. With a valid fix, the status
shows the missing count. Stationary updates and reopening must not repeatedly
retry missing files; reboot after restoring files, or move enough to request a
new viewport. Remove the card during loading and confirm the UI remains usable.

Accepted files have a 12-byte LVGL v9 header, magic 0x19, zero flags/reserved
bits, and dimensions 256x256. RGB565: format 0x12, 512-byte stride, 131084 total
bytes. I4: format 0x09, 128-byte stride, 64-byte opaque BGRA palette, 32844 total
bytes. File length is checked before reading the visible rows.
