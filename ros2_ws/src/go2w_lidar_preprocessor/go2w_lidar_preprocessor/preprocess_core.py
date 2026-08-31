"""Geometry-safe LiDAR processing in REP-103 base_link coordinates."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PreprocessParameters:
    minimum_range: float
    maximum_range: float
    minimum_height: float
    maximum_height: float
    ground_height: float
    collision_maximum_height: float
    self_half_length: float
    self_half_width: float
    self_filter_margin: float
    front_corridor_half_width: float
    rotation_envelope_radius: float
    self_regions: tuple[tuple[float, float, float, float, float, float], ...] = ()


@dataclass(frozen=True)
class Clearance:
    front: float
    left: float
    right: float


def rotation_observability_report(
    p: PreprocessParameters, *, angular_samples: int = 720
) -> dict:
    """Audit whether collision-height LiDAR can observe the swept annulus.

    This is deliberately a worst-case geometry check.  A no-return ray is
    useful only outside the configured minimum range and outside every
    self-filter region that overlaps the collision-height band.  Any hidden
    part of the free annulus between the manufacturer footprint and the
    rotation envelope requires independent physical evidence before side
    clearance may be called valid.
    """

    if angular_samples < 8:
        raise ValueError("rotation observability requires at least 8 angular samples")
    angles = np.linspace(-math.pi, math.pi, angular_samples, endpoint=False)
    blind_lengths: list[float] = []
    blind_angles = 0
    minimum_range_angles = 0
    base_mask_angles = 0
    named_region_angles = 0
    axis_details: dict[str, dict] = {}
    axis_names = {
        0: "rear",
        angular_samples // 4: "right",
        angular_samples // 2: "front",
        3 * angular_samples // 4: "left",
    }

    for index, angle in enumerate(angles):
        cosine = float(math.cos(angle))
        sine = float(math.sin(angle))
        footprint_edge = _rectangle_ray_exit_radius(
            cosine, sine, p.self_half_length, p.self_half_width
        )
        envelope_edge = p.rotation_envelope_radius
        if footprint_edge >= envelope_edge:
            blind_lengths.append(0.0)
            continue

        intervals: list[tuple[float, float]] = []
        cause_flags = {"minimum_range": False, "base_mask": False, "named_region": False}

        def add_interval(start: float, end: float, cause: str) -> None:
            clipped_start = max(footprint_edge, float(start))
            clipped_end = min(envelope_edge, float(end))
            if clipped_end > clipped_start + 1e-9:
                intervals.append((clipped_start, clipped_end))
                cause_flags[cause] = True

        add_interval(0.0, p.minimum_range, "minimum_range")
        base_interval = _ray_box_interval(
            cosine,
            sine,
            -p.self_half_length - p.self_filter_margin,
            p.self_half_length + p.self_filter_margin,
            -p.self_half_width - p.self_filter_margin,
            p.self_half_width + p.self_filter_margin,
        )
        if base_interval is not None:
            add_interval(*base_interval, "base_mask")
        for x_min, x_max, y_min, y_max, z_min, z_max in p.self_regions:
            if z_max <= p.ground_height or z_min > p.collision_maximum_height:
                continue
            interval = _ray_box_interval(
                cosine, sine, x_min, x_max, y_min, y_max
            )
            if interval is not None:
                add_interval(*interval, "named_region")

        merged = _merge_intervals(intervals)
        blind_length = sum(end - start for start, end in merged)
        blind_lengths.append(blind_length)
        if blind_length > 1e-9:
            blind_angles += 1
        minimum_range_angles += int(cause_flags["minimum_range"])
        base_mask_angles += int(cause_flags["base_mask"])
        named_region_angles += int(cause_flags["named_region"])
        if index in axis_names:
            axis_details[axis_names[index]] = {
                "footprint_edge_m": round(footprint_edge, 6),
                "rotation_envelope_edge_m": round(envelope_edge, 6),
                "unobservable_intervals_m": [
                    [round(start, 6), round(end, 6)] for start, end in merged
                ],
                "unobservable_radial_length_m": round(blind_length, 6),
            }

    maximum_blind = max(blind_lengths, default=0.0)
    return {
        "angular_samples": angular_samples,
        "angles_with_unobservable_free_space": blind_angles,
        "unobservable_angle_fraction": round(blind_angles / angular_samples, 6),
        "maximum_unobservable_radial_length_m": round(maximum_blind, 6),
        "cause_angle_counts": {
            "minimum_range": minimum_range_angles,
            "base_self_mask": base_mask_angles,
            "named_self_regions": named_region_angles,
        },
        "axis_details": axis_details,
        "sensor_only_rotation_observability_complete": blind_angles == 0,
        "requires_independent_physical_validation": blind_angles > 0,
    }


def _rectangle_ray_exit_radius(
    cosine: float, sine: float, half_length: float, half_width: float
) -> float:
    x_exit = math.inf if abs(cosine) <= 1e-12 else half_length / abs(cosine)
    y_exit = math.inf if abs(sine) <= 1e-12 else half_width / abs(sine)
    return min(x_exit, y_exit)


def _ray_box_interval(
    cosine: float,
    sine: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> tuple[float, float] | None:
    low = 0.0
    high = math.inf
    for direction, lower, upper in (
        (cosine, x_min, x_max),
        (sine, y_min, y_max),
    ):
        if abs(direction) <= 1e-12:
            if lower > 0.0 or upper < 0.0:
                return None
            continue
        first, second = lower / direction, upper / direction
        low = max(low, min(first, second))
        high = min(high, max(first, second))
        if high <= low:
            return None
    return (low, high) if high > 0.0 else None


def _merge_intervals(
    intervals: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    merged: list[list[float]] = []
    for start, end in sorted(intervals):
        if not merged or start > merged[-1][1] + 1e-9:
            merged.append([start, end])
        else:
            merged[-1][1] = max(merged[-1][1], end)
    return [(start, end) for start, end in merged]


def transform_points(
    points: np.ndarray,
    translation_xyz,
    quaternion_xyzw,
) -> np.ndarray:
    """Apply a geometry_msgs Transform to Nx3 points without rewriting fields."""

    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    translation = np.asarray(translation_xyz, dtype=np.float64)
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if translation.shape != (3,) or quaternion.shape != (4,):
        raise ValueError("transform must contain XYZ translation and XYZW quaternion")
    if not np.isfinite(translation).all() or not np.isfinite(quaternion).all():
        raise ValueError("transform must be finite")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("transform quaternion has zero norm")
    x, y, z, w = quaternion / norm
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return values @ rotation.T + translation


def filter_points_base_link(points: np.ndarray, p: PreprocessParameters):
    """Filter Nx3 points after they have already been transformed to base_link."""
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    finite = np.isfinite(values).all(axis=1)
    ranges = np.linalg.norm(values, axis=1)
    bounded = (
        finite
        & (ranges >= p.minimum_range)
        & (ranges <= p.maximum_range)
        & (values[:, 2] >= p.minimum_height)
        & (values[:, 2] <= p.maximum_height)
    )
    self_points = (
        (np.abs(values[:, 0]) <= p.self_half_length + p.self_filter_margin)
        & (np.abs(values[:, 1]) <= p.self_half_width + p.self_filter_margin)
    )
    for (x_min, x_max, y_min, y_max, z_min, z_max) in p.self_regions:
        self_points |= (
            (values[:, 0] >= x_min)
            & (values[:, 0] <= x_max)
            & (values[:, 1] >= y_min)
            & (values[:, 1] <= y_max)
            & (values[:, 2] >= z_min)
            & (values[:, 2] <= z_max)
        )
    filtered = values[bounded & ~self_points]
    obstacles = filtered[filtered[:, 2] > p.ground_height]
    return filtered, obstacles


def collision_obstacles(obstacles: np.ndarray, p: PreprocessParameters) -> np.ndarray:
    """Keep only points intersecting the robot's vertical swept envelope.

    ``obstacles`` intentionally remains a full-height semantic/mapping cloud.
    Navigation scans and clearance estimates must not treat surfaces above the
    standing robot as collisions, so they pass through this separate filter.
    """

    values = np.asarray(obstacles, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("obstacles must have shape Nx3")
    return values[values[:, 2] <= p.collision_maximum_height]


def directional_clearance(obstacles: np.ndarray, p: PreprocessParameters) -> Clearance:
    values = np.asarray(obstacles, dtype=np.float64)
    if values.size == 0:
        return Clearance(math.inf, math.inf, math.inf)
    forward = values[:, 0]
    left_axis = values[:, 1]
    radial = np.hypot(forward, left_axis)
    front_mask = (forward > 0.0) & (np.abs(left_axis) <= p.front_corridor_half_width)
    left_mask = (left_axis > 0.0) & (radial <= p.rotation_envelope_radius)
    right_mask = (left_axis < 0.0) & (radial <= p.rotation_envelope_radius)

    def minimum(values_: np.ndarray) -> float:
        return float(np.min(values_)) if values_.size else math.inf

    return Clearance(
        front=minimum(forward[front_mask]),
        left=minimum(radial[left_mask]),
        right=minimum(radial[right_mask]),
    )


def laser_scan_ranges(
    obstacles: np.ndarray,
    *,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> np.ndarray:
    count = int(math.ceil((angle_max - angle_min) / angle_increment))
    ranges = np.full(count, np.inf, dtype=np.float32)
    if len(obstacles) == 0:
        return ranges
    angles = np.arctan2(obstacles[:, 1], obstacles[:, 0])
    distances = np.hypot(obstacles[:, 0], obstacles[:, 1])
    valid = (
        (angles >= angle_min)
        & (angles < angle_max)
        & (distances >= range_min)
        & (distances <= range_max)
    )
    bins = ((angles[valid] - angle_min) / angle_increment).astype(np.int64)
    for index, distance in zip(bins, distances[valid]):
        ranges[index] = min(ranges[index], float(distance))
    return ranges
