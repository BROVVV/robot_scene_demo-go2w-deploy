"""Frame-aware 2D occupancy-map quality metrics.

The functions operate on recorded occupancy arrays and never mutate or
beautify the navigation grid.  Comparisons are made in world coordinates so a
changing grid origin does not masquerade as map churn.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable

import numpy as np


@dataclass(frozen=True)
class Grid2D:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "map"
    map_epoch: str | None = None

    def __post_init__(self) -> None:
        if self.data.ndim != 2:
            raise ValueError("occupancy data must be a 2D array")
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("resolution must be finite and positive")


def occupancy_counts(grid: Grid2D) -> dict[str, int | float]:
    data = np.asarray(grid.data)
    unknown = int(np.count_nonzero(data < 0))
    free = int(np.count_nonzero((data >= 0) & (data < 50)))
    occupied = int(np.count_nonzero(data >= 50))
    observed = free + occupied
    return {
        "width": int(data.shape[1]),
        "height": int(data.shape[0]),
        "resolution_m": float(grid.resolution),
        "unknown_cells": unknown,
        "free_cells": free,
        "occupied_cells": occupied,
        "observed_cells": observed,
        "free_area_m2": free * grid.resolution * grid.resolution,
        "observed_area_m2": observed * grid.resolution * grid.resolution,
    }


def compare_world_aligned(
    before: Grid2D,
    after: Grid2D,
    *,
    roi: tuple[float, float, float, float] | None = None,
) -> dict[str, Any]:
    """Compare overlapping cells after resolving both origins in world space."""
    if before.frame_id != after.frame_id:
        raise ValueError("cannot compare grids from different frames")
    if before.map_epoch and after.map_epoch and before.map_epoch != after.map_epoch:
        raise ValueError("cannot compare grids from different map epochs")
    resolution = min(before.resolution, after.resolution)
    if not math.isclose(before.resolution, after.resolution, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("different resolutions require explicit resampling")
    bounds = _overlap_bounds(before, after)
    if roi is not None:
        bounds = (
            max(bounds[0], roi[0]), max(bounds[1], roi[1]),
            min(bounds[2], roi[2]), min(bounds[3], roi[3]),
        )
    if bounds[2] <= bounds[0] or bounds[3] <= bounds[1]:
        return {"comparable_cells": 0, "reason": "no_world_overlap"}
    # This runs in the ROS subscription executor.  The former nested Python
    # loop called ``_class_at`` once per cell and could occupy the executor for
    # longer than the localization-health timeout on a growing map.  Besides
    # wasting a core, that starved TF/health callbacks and made the independent
    # vision sidecar conclude that global pose had disappeared.  Build the
    # aligned index vectors once and let NumPy classify the full overlap.
    xs = np.arange(bounds[0] + resolution / 2.0, bounds[2] - 1e-12, resolution)
    ys = np.arange(bounds[1] + resolution / 2.0, bounds[3] - 1e-12, resolution)
    if xs.size == 0 or ys.size == 0:
        return {"comparable_cells": 0, "reason": "empty_roi"}

    before_cols = np.floor((xs - before.origin_x) / resolution).astype(np.intp)
    before_rows = np.floor((ys - before.origin_y) / resolution).astype(np.intp)
    after_cols = np.floor((xs - after.origin_x) / resolution).astype(np.intp)
    after_rows = np.floor((ys - after.origin_y) / resolution).astype(np.intp)
    # Bounds come from the geometric overlap.  Clipping only protects against
    # a one-ulp boundary round-off; it does not expand the compared region.
    before_cols = np.clip(before_cols, 0, before.data.shape[1] - 1)
    before_rows = np.clip(before_rows, 0, before.data.shape[0] - 1)
    after_cols = np.clip(after_cols, 0, after.data.shape[1] - 1)
    after_rows = np.clip(after_rows, 0, after.data.shape[0] - 1)

    before_values = before.data[np.ix_(before_rows, before_cols)]
    after_values = after.data[np.ix_(after_rows, after_cols)]
    before_classes = np.where(before_values < 0, -1,
                              np.where(before_values >= 50, 1, 0)).astype(np.int8)
    after_classes = np.where(after_values < 0, -1,
                             np.where(after_values >= 50, 1, 0)).astype(np.int8)
    known_both = (before_classes >= 0) & (after_classes >= 0)
    flips = known_both & (before_classes != after_classes)
    newly_observed = (before_classes < 0) & (after_classes >= 0)
    became_unknown = (before_classes >= 0) & (after_classes < 0)
    comparable_cells = int(before_classes.size)
    return {
        "comparable_cells": comparable_cells,
        "known_in_both": int(np.count_nonzero(known_both)),
        "known_flip_cells": int(np.count_nonzero(flips)),
        "known_flip_rate": (
            float(np.count_nonzero(flips) / np.count_nonzero(known_both))
            if np.count_nonzero(known_both) else None
        ),
        "newly_observed_cells": int(np.count_nonzero(newly_observed)),
        "became_unknown_cells": int(np.count_nonzero(became_unknown)),
        "roi_world": [float(v) for v in bounds],
    }


def sector_coverage(
    ranges: Iterable[float],
    *,
    range_min: float,
    range_max: float,
    sector_count: int = 12,
) -> dict[str, Any]:
    values = np.asarray(list(ranges), dtype=float)
    if sector_count <= 0:
        raise ValueError("sector_count must be positive")
    valid = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    sectors = np.array_split(valid, sector_count)
    ratios = [float(np.mean(part)) if len(part) else 0.0 for part in sectors]
    longest = _longest_false_run(valid.tolist())
    return {
        "valid_bins": int(np.count_nonzero(valid)),
        "total_bins": int(len(values)),
        "valid_ratio": float(np.mean(valid)) if len(values) else 0.0,
        "sector_valid_ratios": ratios,
        "longest_missing_bins": int(longest),
        "longest_missing_deg": (
            float(longest * 360.0 / len(values)) if len(values) else 360.0
        ),
    }


def health_dimensions(
    *, input_health: str, geometry_health: str, localization_health: str,
    mapping_quality: str, motion_authorized: bool,
) -> dict[str, Any]:
    return {
        "input_health": str(input_health),
        "geometry_health": str(geometry_health),
        "localization_health": str(localization_health),
        "mapping_quality": str(mapping_quality),
        "motion_authorized": bool(motion_authorized),
    }


def _overlap_bounds(left: Grid2D, right: Grid2D) -> tuple[float, float, float, float]:
    return (
        max(left.origin_x, right.origin_x),
        max(left.origin_y, right.origin_y),
        min(left.origin_x + left.data.shape[1] * left.resolution,
            right.origin_x + right.data.shape[1] * right.resolution),
        min(left.origin_y + left.data.shape[0] * left.resolution,
            right.origin_y + right.data.shape[0] * right.resolution),
    )


def _class_at(grid: Grid2D, x: float, y: float) -> int:
    col = int(math.floor((x - grid.origin_x) / grid.resolution))
    row = int(math.floor((y - grid.origin_y) / grid.resolution))
    value = float(grid.data[row, col])
    if value < 0:
        return -1
    return 1 if value >= 50 else 0


def _longest_false_run(values: list[bool]) -> int:
    if not values:
        return 0
    doubled = values + values
    longest = current = 0
    for value in doubled:
        current = 0 if value else current + 1
        longest = max(longest, current)
    return min(longest, len(values))
