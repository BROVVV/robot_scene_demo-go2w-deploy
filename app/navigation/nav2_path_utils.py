"""Geometry helpers for Nav2 paths."""

from __future__ import annotations

import math
from typing import Any, Sequence

from .nav2_models import quaternion_to_yaw, yaw_to_quaternion


def _xy(p: Any) -> tuple[float, float]:
    return (float(p["x"]), float(p["y"])) if isinstance(p, dict) else (float(p.x), float(p.y))


def validate_finite_coordinates(poses: Sequence[Any]) -> None:
    for pose in poses:
        if not all(math.isfinite(v) for v in _xy(pose)):
            raise ValueError("路径包含 NaN/Inf")


def compute_cumulative_distances(poses: Sequence[Any]) -> list[float]:
    validate_finite_coordinates(poses)
    result = [0.0] if poses else []
    for previous, current in zip(poses, poses[1:]):
        result.append(result[-1] + math.dist(_xy(previous), _xy(current)))
    return result


def compute_path_length(poses: Sequence[Any]) -> float:
    values = compute_cumulative_distances(poses)
    return values[-1] if values else 0.0


def compute_segment_heading(start: Any, end: Any) -> float:
    x1, y1 = _xy(start)
    x2, y2 = _xy(end)
    return math.atan2(y2-y1, x2-x1)


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def compute_progress_ratio(initial_length: float, remaining: float) -> float:
    if initial_length <= 0:
        return 1.0 if remaining <= 0 else 0.0
    return max(0.0, min(1.0, 1.0 - remaining / initial_length))


def simplify_path_rdp(poses: Sequence[Any], epsilon: float) -> list[Any]:
    if epsilon < 0:
        raise ValueError("epsilon 不能为负")
    if len(poses) < 3:
        return list(poses)
    start, end = _xy(poses[0]), _xy(poses[-1])
    dx, dy = end[0]-start[0], end[1]-start[1]
    denominator = math.hypot(dx, dy)
    distances = []
    for p in poses[1:-1]:
        x, y = _xy(p)
        distance = math.hypot(x-start[0], y-start[1]) if denominator == 0 else abs(dy*x-dx*y+end[0]*start[1]-end[1]*start[0]) / denominator
        distances.append(distance)
    maximum = max(distances, default=0.0)
    index = distances.index(maximum) + 1 if distances else 0
    if maximum > epsilon:
        left = simplify_path_rdp(poses[:index+1], epsilon)
        right = simplify_path_rdp(poses[index:], epsilon)
        return left[:-1] + right
    return [poses[0], poses[-1]]


def map_to_pixel(x: float, y: float, origin_x: float, origin_y: float, resolution: float, image_height: int) -> tuple[float, float]:
    if resolution <= 0:
        raise ValueError("resolution 必须大于 0")
    return ((x-origin_x)/resolution, image_height-1-(y-origin_y)/resolution)


__all__ = ["compute_cumulative_distances", "compute_path_length", "compute_progress_ratio",
           "compute_segment_heading", "map_to_pixel", "normalize_angle", "quaternion_to_yaw",
           "simplify_path_rdp", "validate_finite_coordinates", "yaw_to_quaternion"]
