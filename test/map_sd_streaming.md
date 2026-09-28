# SD tile streaming hardware check

Build and upload the esp32dev environment, then monitor at 115200 baud.
Keep the existing SD card layout: `/tiles1/16/<x>/<y>.bin`. Include neighboring
tiles covering your test ride. The map follows valid GPS fixes at zoom 16.
Before the first fix it shows the original test location (49.21486, -123.12683),
labelled Preview with no position marker. Tile filenames are logged on load.

Expected results:

- `SD card mounted successfully` appears.
- `Streaming tile ...` appears for the visible tiles.
- The map fills the screen with correct colours and no allocation failure,
  reset, or watchdog timeout. Back returns to the ride dashboard; Map reopens it.
- Once GPS fixes outdoors, a blue dot appears at the center and the map moves
  underneath it. Walk/ride across a tile boundary and a corner: adjacent tiles
  should meet without gaps, and newly visible files should load automatically.
- Remaining stationary should not repeatedly validate/reload tile sources.
- Disconnect GPS after a fix: the last map position remains, the dot turns gray,
  and the status reports Last position. Reconnect and verify following resumes.
- Test long enough to cross several tile boundaries and confirm there is no
  progressive heap loss. Tiles retain file paths, never full pixel buffers.

The LVGL binary decoder reads one RGB565 row (at most 512 bytes) at a time.
The stdio filesystem has a 4096-byte read cache per open LVGL file, plus
stdio/FAT and decoder overhead. LVGL can briefly open a file twice while
inspecting and opening the image. The existing 12.5 KiB display buffer remains.
No complete tile pixel buffer is allocated by this UI path. Up to six image
objects cover the 320x240 viewport (at most three columns and two rows).
Updates are limited to about three to four per second and skip unchanged
rounded pixel positions. Heading rotation and manual panning are not implemented.

For failure checks, boot with one visible tile missing, then with a truncated
copy. Both should log an explicit error and leave that area gray while adjacent
tiles render. With a GPS fix, the status includes a missing tile count.
Restore the original tile afterwards. Accepted files match the supplied converter:
12-byte LVGL v9 header, 256x256 uncompressed RGB565, 512-byte stride, total
131084 bytes. Other formats are rejected before rendering.

Missing files are remembered while their tile slot is retained to avoid repeated
SD probes. After adding files to the card, reboot to retry them. The old full-tile
loader and navigation cache APIs remain available, but the UI does not call them.
SD redraw latency still needs measurement on hardware before adding animation
or a larger block cache.
