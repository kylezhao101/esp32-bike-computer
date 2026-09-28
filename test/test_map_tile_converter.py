"""Host tests for the tile binary contract; no firmware build required."""
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "tile_converter", ROOT / "components/map_tiles/script/lvgl_map_tile_converter.py")
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)


class TileConverterTests(unittest.TestCase):
    def convert(self, image, pixel_format):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "tile.png"
            target = Path(directory) / "tile.bin"
            image.save(source)
            converter.make_lvgl_bin(source, target, pixel_format)
            return target.read_bytes()

    def test_i4_header_size_and_opaque_palette(self):
        data = self.convert(Image.new("RGB", (256, 256), (12, 34, 56)), "i4")
        self.assertEqual(struct.unpack("<BBHHHHH", data[:12]),
                         (0x19, 0x09, 0, 256, 256, 128, 0))
        self.assertEqual(len(data), 32844)
        self.assertEqual(data[15:76:4], bytes([255] * 16))

    def test_i4_round_trip_odd_width_and_palette_byte_order(self):
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
                  (255, 255, 255), (0, 0, 0)]
        original = [colors[(x + y) % len(colors)] for y in range(3) for x in range(5)]
        image = Image.new("RGB", (5, 3))
        image.putdata(original)
        data = self.convert(image, "i4")
        self.assertEqual(len(data), 12 + 64 + 3 * 3)
        decoded = []
        for y in range(3):
            for x in range(5):
                byte = data[76 + y * 3 + x // 2]
                index = (byte >> (0 if x & 1 else 4)) & 15
                b, g, r, a = data[12 + index * 4:16 + index * 4]
                self.assertEqual(a, 255)
                decoded.append((r, g, b))
            self.assertEqual(data[76 + y * 3 + 2] & 15, 0)
        self.assertEqual(decoded, original)

    def test_rgb565_output_is_backward_compatible(self):
        image = Image.new("RGB", (3, 1))
        image.putdata([(255, 0, 0), (0, 255, 0), (0, 0, 255)])
        data = self.convert(image, "rgb565")
        self.assertEqual(struct.unpack("<BBHHHHH", data[:12]),
                         (0x19, 0x12, 0, 3, 1, 6, 0))
        self.assertEqual(data[12:], bytes.fromhex("00f8e0071f00"))

    def test_threaded_conversion_passes_format(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            tiles = root / "input/16/1"
            tiles.mkdir(parents=True)
            for y in range(2):
                Image.new("RGB", (256, 256), (20, 40, 60)).save(tiles / f"{y}.png")
            previous = converter.INPUT_ROOT, converter.OUTPUT_ROOT
            try:
                converter.INPUT_ROOT = str(root / "input")
                converter.OUTPUT_ROOT = str(root / "output")
                converter.convert_all_tiles(jobs=2, pixel_format="i4")
                for y in range(2):
                    self.assertEqual((root / f"output/16/1/{y}.bin").stat().st_size, 32844)
            finally:
                converter.INPUT_ROOT, converter.OUTPUT_ROOT = previous


if __name__ == "__main__":
    unittest.main()
