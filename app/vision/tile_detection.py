"""Utilities for optional large-image tiled detection and NMS merging."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from app.vision.schema import CandidateObject


@dataclass(frozen=True)
class ImageTile:
    left: int
    top: int
    right: int
    bottom: int


def generate_tiles(
    width: int,
    height: int,
    tile_size: int = 960,
    overlap: int = 160,
) -> list[ImageTile]:
    if tile_size <= 0 or overlap < 0 or overlap >= tile_size:
        raise ValueError("Require tile_size > overlap >= 0.")
    step = tile_size - overlap
    xs = _origins(width, tile_size, step)
    ys = _origins(height, tile_size, step)
    return [
        ImageTile(x, y, min(width, x + tile_size), min(height, y + tile_size))
        for y in ys
        for x in xs
    ]


def map_tile_bbox(
    bbox: list[float],
    tile: ImageTile,
    image_width: int,
    image_height: int,
) -> list[float]:
    tile_width = max(1, tile.right - tile.left)
    tile_height = max(1, tile.bottom - tile.top)
    x1, y1, x2, y2 = bbox
    if max(bbox) <= 1.0:
        x1, x2 = x1 * tile_width, x2 * tile_width
        y1, y2 = y1 * tile_height, y2 * tile_height
    return [
        (tile.left + x1) / image_width,
        (tile.top + y1) / image_height,
        (tile.left + x2) / image_width,
        (tile.top + y2) / image_height,
    ]


def nms_candidates(
    candidates: Iterable[CandidateObject],
    iou_threshold: float = 0.5,
) -> list[CandidateObject]:
    ordered = sorted(
        candidates,
        key=lambda item: float(item.detector_score or item.score),
        reverse=True,
    )
    kept: list[CandidateObject] = []
    for candidate in ordered:
        if not candidate.bbox:
            kept.append(candidate)
            continue
        if any(
            _same_label(candidate, prior)
            and _iou(candidate.bbox, prior.bbox or []) >= iou_threshold
            for prior in kept
        ):
            continue
        kept.append(candidate)
    return kept


def _origins(length: int, tile_size: int, step: int) -> list[int]:
    if length <= tile_size:
        return [0]
    values = list(range(0, length - tile_size + 1, step))
    final = length - tile_size
    if values[-1] != final:
        values.append(final)
    return values


def _same_label(left: CandidateObject, right: CandidateObject) -> bool:
    return left.label.lower().strip() == right.label.lower().strip()


def _iou(left: list[float], right: list[float]) -> float:
    if len(left) != 4 or len(right) != 4:
        return 0.0
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0
