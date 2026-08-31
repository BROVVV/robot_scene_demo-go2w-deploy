from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from PIL import Image

from app.vision.crop_utils import expand_bbox, map_bbox_from_crop, save_candidate_crop


class CropUtilsTest(unittest.TestCase):
    def test_expand_bbox_clamps_to_image(self) -> None:
        self.assertEqual(expand_bbox([0.0, 0.0, 0.2, 0.2], 100, 80, 2.0), [0, 0, 30, 24])

    def test_crop_is_saved(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "source.jpg"
            output = Path(tmp) / "crop.jpg"
            Image.new("RGB", (100, 80), "white").save(source)
            saved, bounds = save_candidate_crop(source, [0.2, 0.2, 0.4, 0.5], output)
            self.assertTrue(saved.is_file())
            self.assertEqual(len(bounds), 4)

    def test_maps_crop_coordinates_to_original(self) -> None:
        mapped = map_bbox_from_crop([0.0, 0.0, 1.0, 1.0], [20, 10, 60, 50], 100, 100)
        self.assertEqual(mapped, [0.2, 0.1, 0.6, 0.5])


if __name__ == "__main__":
    unittest.main()
