"""Bounding-box expansion, crop export and coordinate mapping."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from PIL import Image


def expand_bbox(
    bbox: Sequence[float],
    image_width: int,
    image_height: int,
    ratio: float = 1.35,
    normalized: bool | None = None,
) -> list[int]:
    if len(bbox) != 4:
        raise ValueError("bbox must contain [x1, y1, x2, y2].")
    values = [float(value) for value in bbox]
    if normalized is None:
        normalized = max(values) <= 1.0
    if normalized:
        x1, y1, x2, y2 = [
            values[0] * image_width,
            values[1] * image_height,
            values[2] * image_width,
            values[3] * image_height,
        ]
    else:
        x1, y1, x2, y2 = values
    if x2 <= x1 or y2 <= y1:
        raise ValueError("bbox must have positive width and height.")
    ratio = max(1.0, float(ratio))
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    half_width = (x2 - x1) * ratio / 2.0
    half_height = (y2 - y1) * ratio / 2.0
    return [
        max(0, int(round(center_x - half_width))),
        max(0, int(round(center_y - half_height))),
        min(image_width, int(round(center_x + half_width))),
        min(image_height, int(round(center_y + half_height))),
    ]


def save_candidate_crop(
    image_path: str | Path,
    bbox: Sequence[float],
    output_path: str | Path,
    expand_ratio: float = 1.35,
) -> tuple[Path, list[int]]:
    source = Path(image_path)
    if not source.is_file():
        raise FileNotFoundError(f"Image file not found: {source}")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with Image.open(source) as image:
        rgb = image.convert("RGB")
        expanded = expand_bbox(bbox, rgb.width, rgb.height, expand_ratio)
        rgb.crop(tuple(expanded)).save(output, format="JPEG", quality=92)
    return output, expanded


def map_bbox_from_crop(
    crop_bbox: Sequence[float],
    crop_bounds: Sequence[int],
    image_width: int,
    image_height: int,
) -> list[float]:
    if len(crop_bbox) != 4 or len(crop_bounds) != 4:
        raise ValueError("Both boxes must contain four coordinates.")
    left, top, right, bottom = [float(value) for value in crop_bounds]
    crop_width = max(1.0, right - left)
    crop_height = max(1.0, bottom - top)
    x1, y1, x2, y2 = [float(value) for value in crop_bbox]
    if max(x1, y1, x2, y2) <= 1.0:
        x1, x2 = x1 * crop_width, x2 * crop_width
        y1, y2 = y1 * crop_height, y2 * crop_height
    return [
        max(0.0, min(1.0, (left + x1) / image_width)),
        max(0.0, min(1.0, (top + y1) / image_height)),
        max(0.0, min(1.0, (left + x2) / image_width)),
        max(0.0, min(1.0, (top + y2) / image_height)),
    ]
