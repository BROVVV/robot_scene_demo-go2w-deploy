"""Per-frame diagnostic analysis for the PandarXT-16 stream.

The Pandar raw stream is accepted only as diagnostic evidence until its
transform, self-occlusion and clock are validated.  This module turns one
PointCloud2 frame into a compact, provenance-carrying diagnostics payload that
records zero/near-zero returns, ring coverage, range statistics, timestamps
and an estimate of self-occlusion against the robot body/mount.

Nothing here authorises motion or a formal transform.
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Sequence

import numpy as np


DEFAULT_ZERO_RETURN_MAX_M = 0.05


def analyze_pandar_frame(
    *,
    xyz: Sequence[Sequence[float]],
    ring: Sequence[int] | None = None,
    point_timestamp: Sequence[float] | None = None,
    zero_return_max_m: float = DEFAULT_ZERO_RETURN_MAX_M,
    expected_rings: int = 16,
    frame_id: str = "pandarxt16_link_unvalidated",
    source_topic: str = "/hesai/pandarxt16/points_raw",
    candidate_transform_available: bool = True,
    transform_validated: bool = False,
    freshness_seconds: float | None = None,
) -> dict[str, Any]:
    """Analyze one Pandar frame into diagnostics with provenance."""
    values = np.asarray(xyz, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("xyz must have shape Nx3")
    points = values.shape[0]

    finite = np.isfinite(values).all(axis=1)
    ranges = np.linalg.norm(values, axis=1)
    finite_ranges = ranges[finite]
    zero_near_zero = (
        finite & (ranges <= zero_return_max_m)
        if finite_ranges.size
        else np.zeros(points, dtype=bool)
    )
    valid = finite & (ranges > zero_return_max_m)
    valid_ranges = ranges[valid]

    point_time_array = (
        np.asarray(point_timestamp, dtype=np.float64)
        if point_timestamp is not None
        else None
    )

    ring_counts: dict[str, int] = {}
    valid_rings: list[int] = []
    if ring is not None:
        rings = np.asarray(ring, dtype=np.int64)
        for index in range(expected_rings):
            ring_counts[str(index)] = int(np.count_nonzero(rings[valid] == index))
        valid_rings = sorted(
            {int(value) for value in rings[valid] if 0 <= int(value) < expected_rings}
        )

    return {
        "frame_id": frame_id,
        "source_topic": source_topic,
        "total_points": int(points),
        "finite_points": int(np.count_nonzero(finite)),
        "non_finite_points": int(np.count_nonzero(~finite)),
        "zero_or_near_zero_points": int(np.count_nonzero(zero_near_zero)),
        "zero_return_fraction": (
            round(float(np.count_nonzero(zero_near_zero)) / max(points, 1), 6)
        ),
        "valid_points": int(np.count_nonzero(valid)),
        "valid_return_fraction": (
            round(float(np.count_nonzero(valid)) / max(points, 1), 6)
        ),
        "range_min_m": (
            float(np.min(valid_ranges)) if valid_ranges.size else None
        ),
        "range_max_m": (
            float(np.max(valid_ranges)) if valid_ranges.size else None
        ),
        "range_median_m": (
            float(np.median(valid_ranges)) if valid_ranges.size else None
        ),
        "points_per_ring": ring_counts,
        "valid_rings": valid_rings,
        "rings_expected": int(expected_rings),
        "all_rings_have_valid_returns": len(valid_rings) == expected_rings,
        "point_timestamp_span_s": (
            float(np.ptp(point_time_array))
            if point_time_array is not None and point_time_array.size
            else None
        ),
        "point_timestamps_finite": bool(
            point_time_array is None or np.all(np.isfinite(point_time_array))
        ),
        "candidate_transform_available": bool(candidate_transform_available),
        "transform_validated": bool(transform_validated),
        "freshness_seconds": freshness_seconds,
        "fresh": bool(
            freshness_seconds is not None
            and math.isfinite(freshness_seconds)
            and freshness_seconds >= 0.0
        ),
        "diagnostic_only": True,
        "authorizes_motion": False,
        "authorizes_safety_integration": False,
    }


def estimate_self_occlusion_fraction(
    *,
    xyz: Sequence[Sequence[float]],
    body_region: dict[str, tuple[float, float, float, float, float, float]],
    zero_return_max_m: float = DEFAULT_ZERO_RETURN_MAX_M,
) -> dict[str, Any]:
    """Estimate how many returns lie inside the robot body/mount region.

    ``body_region`` is an AABB in base_link-ish coordinates:
    ``(x_min, x_max, y_min, y_max, z_min, z_max)``.  Points inside it are
    likely self/occlusion returns (the sensor sees the robot's own frame).
    """
    values = np.asarray(xyz, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("xyz must have shape Nx3")
    x_min, x_max, y_min, y_max, z_min, z_max = (
        float(body_region[key]) for key in (
            "x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
        )
    )
    ranges = np.linalg.norm(values, axis=1)
    finite = np.isfinite(values).all(axis=1)
    valid = finite & (ranges > zero_return_max_m)
    inside = (
        valid
        & (values[:, 0] >= x_min)
        & (values[:, 0] <= x_max)
        & (values[:, 1] >= y_min)
        & (values[:, 1] <= y_max)
        & (values[:, 2] >= z_min)
        & (values[:, 2] <= z_max)
    )
    valid_count = max(int(np.count_nonzero(valid)), 1)
    return {
        "valid_points": int(np.count_nonzero(valid)),
        "inside_body_region": int(np.count_nonzero(inside)),
        "self_occlusion_fraction": round(
            float(np.count_nonzero(inside)) / valid_count, 6
        ),
        "body_region": {
            "x_min": x_min, "x_max": x_max,
            "y_min": y_min, "y_max": y_max,
            "z_min": z_min, "z_max": z_max,
        },
    }


def azimuth_occupied_bins(
    *,
    xyz: Sequence[Sequence[float]],
    bearing_bin_count: int = 360,
    zero_return_max_m: float = DEFAULT_ZERO_RETURN_MAX_M,
    min_range_m: float = 0.3,
    max_range_m: float = 6.0,
) -> dict[str, Any]:
    """Occupied-azimuth histogram for the Pandar (360-degree sweep)."""
    if bearing_bin_count < 12:
        raise ValueError("bearing_bin_count must be at least 12")
    values = np.asarray(xyz, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("xyz must have shape Nx3")
    ranges = np.linalg.norm(values, axis=1)
    finite = np.isfinite(values).all(axis=1)
    valid = finite & (ranges > zero_return_max_m) & (ranges >= min_range_m) & (ranges <= max_range_m)
    angles = np.arctan2(values[valid, 1], values[valid, 0])
    bins = ((angles / (2.0 * math.pi)) % 1.0 * bearing_bin_count).astype(np.int64)
    occupied = np.bincount(bins, minlength=bearing_bin_count)
    return {
        "bearing_bin_count": int(bearing_bin_count),
        "occupied_bins": int(np.count_nonzero(occupied)),
        "occupied_fraction": round(
            float(np.count_nonzero(occupied)) / bearing_bin_count, 6
        ),
        "empty_bins": int(bearing_bin_count - np.count_nonzero(occupied)),
        "range_min_m": float(min_range_m),
        "range_max_m": float(max_range_m),
    }
