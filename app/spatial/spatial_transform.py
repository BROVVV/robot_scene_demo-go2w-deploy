"""Pure-Python spatial transforms for camera_xyz -> map_xyz.

The camera frame uses the D435 optical convention:

* camera x = image right
* camera y = image down
* camera z = outward / forward

The robot base frame uses ROS REP-103 convention:

* base x = forward
* base y = left
* base z = up

These helpers are intentionally dependency-free so the whole transform chain
can be unit-tested offline.
"""

from __future__ import annotations

import math
from typing import Any, Iterable

from app.spatial.models import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_METRIC_LIDAR,
    SPATIAL_QUALITY_METRIC_RGBD,
    SPATIAL_QUALITY_RELATIVE_RGBD,
    SpatialPose,
)

DEFAULT_NOMINAL_CAMERA_TRANSLATION_M: tuple[float, float, float] = (0.10, 0.0, 0.30)
"""Nominal camera position in base_link coordinates (x forward, y left, z up)."""


def camera_optical_to_base(
    xyz_camera: tuple[float, float, float] | Iterable[float],
    *,
    camera_translation_m: tuple[float, float, float] = DEFAULT_NOMINAL_CAMERA_TRANSLATION_M,
) -> tuple[float, float, float]:
    """Transform a D435 optical-frame point into the robot base_link frame.

    The default camera frame is the standard optical frame (z forward, x
    right, y down).  ``camera_translation_m`` is the camera origin expressed
    in base_link coordinates.
    """
    xc, yc, zc = (float(v) for v in xyz_camera)
    tx, ty, tz = (float(v) for v in camera_translation_m)
    x_base = zc + tx
    y_base = -xc + ty
    z_base = -yc + tz
    return (round(x_base, 4), round(y_base, 4), round(z_base, 4))


def base_to_world(
    xyz_base: tuple[float, float, float] | Iterable[float],
    pose: SpatialPose | dict[str, Any],
) -> tuple[float, float, float]:
    """Rotate+translate a base_link point by a 2D robot pose (x, y, yaw)."""
    bx, by, bz = (float(v) for v in xyz_base)
    if isinstance(pose, SpatialPose):
        x = float(pose.x)
        y = float(pose.y)
        yaw = float(pose.yaw)
    else:
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))
        yaw_value = pose.get("yaw_rad", pose.get("yaw"))
        if yaw_value is None and pose.get("yaw_deg") is not None:
            yaw_value = math.radians(float(pose["yaw_deg"]))
        yaw = float(yaw_value or 0.0)
    wx = x + math.cos(yaw) * bx - math.sin(yaw) * by
    wy = y + math.sin(yaw) * bx + math.cos(yaw) * by
    return (round(wx, 4), round(wy, 4), round(bz, 4))


def camera_point_to_map(
    xyz_camera: tuple[float, float, float] | Iterable[float],
    pose: SpatialPose | dict[str, Any],
    *,
    camera_translation_m: tuple[float, float, float] = DEFAULT_NOMINAL_CAMERA_TRANSLATION_M,
) -> tuple[float, float, float] | None:
    """Project a camera-optical 3D point into the robot's spatial map frame.

    Uses the nominal optical->base axis convention and the robot's planar
    pose.  Returns ``None`` when the pose is missing/unusable because a
    camera-local observation cannot be turned into a world coordinate without
    a robot pose.
    """
    if pose is None:
        return None
    try:
        xyz_base = camera_optical_to_base(
            xyz_camera, camera_translation_m=camera_translation_m
        )
        return base_to_world(xyz_base, pose)
    except (TypeError, ValueError):
        return None


def transform_quality(
    *,
    map_available: bool,
    pose_available: bool,
    transform_source: str,
    has_map_frame: bool = False,
) -> str:
    """Return the SPATIAL_QUALITY_* value for a transform chain.

    ``transform_source`` is ``tf2`` (real measured transform) or
    ``nominal_extrinsic``.
    """
    if (
        map_available
        and pose_available
        and transform_source == "tf2"
        and has_map_frame
    ):
        return SPATIAL_QUALITY_METRIC_RGBD
    if pose_available and transform_source in {"tf2", "nominal_extrinsic"}:
        return SPATIAL_QUALITY_RELATIVE_RGBD
    return SPATIAL_QUALITY_CAMERA_LOCAL


def circular_mean(angles_deg: Iterable[float]) -> float:
    """Mean of angles (degrees) that handles wraparound correctly."""
    values = [float(v) for v in angles_deg if v is not None]
    if not values:
        return 0.0
    x = sum(math.cos(math.radians(v)) for v in values)
    y = sum(math.sin(math.radians(v)) for v in values)
    result = math.degrees(math.atan2(y, x))
    return result % 360.0 if result >= 0 else 360.0 - ((-result) % 360.0)


def weighted_position_mean(
    positions: Iterable[tuple[float, float, float]],
    weights: Iterable[float],
) -> tuple[float, float, float]:
    """Weighted running-mean of 3D positions."""
    total = 0.0
    acc = [0.0, 0.0, 0.0]
    for position, weight in zip(positions, weights):
        w = max(0.0, float(weight))
        for i in range(3):
            acc[i] += float(position[i]) * w
        total += w
    if total <= 1e-9:
        return (0.0, 0.0, 0.0)
    return tuple(round(v / total, 4) for v in acc)  # type: ignore[return-value]


def position_variance(
    positions: Iterable[tuple[float, float, float]],
    mean: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Unbiased-ish variance per axis over a sequence of positions."""
    count = 0
    acc = [0.0, 0.0, 0.0]
    for position in positions:
        for i in range(3):
            d = float(position[i]) - float(mean[i])
            acc[i] += d * d
        count += 1
    if count <= 1:
        return (0.0, 0.0, 0.0)
    return tuple(round(v / (count - 1), 6) for v in acc)  # type: ignore[return-value]


def quality_weight(quality: str) -> float:
    """Weights used when fusing positions across observations."""
    return {
        SPATIAL_QUALITY_METRIC_RGBD: 1.0,
        SPATIAL_QUALITY_METRIC_LIDAR: 1.0,
        SPATIAL_QUALITY_RELATIVE_RGBD: 0.6,
        SPATIAL_QUALITY_CAMERA_LOCAL: 0.0,
        "RGB_ONLY": 0.0,
    }.get(quality, 0.0)


def dynamic_merge_distance_m(
    *,
    label: str,
    base: float = 0.35,
    depth_m: float | None = None,
    confidence: float = 0.5,
    object_size_hint: float | None = None,
) -> float:
    """Return a per-object merge distance.

    Large static objects (table, sofa, cabinet) are allowed a wider merge
    radius; small movable objects use a tighter radius.  If a size class is
    not known, the base threshold is widened by pose/depth uncertainty.
    """
    label_lower = str(label or "").lower()
    large_keywords = (
        "桌", "柜", "沙", "床", "门", "墙", "冰箱", "洗衣机",
        "table", "sofa", "cabinet", "bed", "door", "wall",
    )
    small_keywords = (
        "杯", "瓶", "书", "盒", "遥控", "手机", "剪刀", "垃圾",
        "cup", "bottle", "book", "box", "remote", "phone", "scissors",
    )
    if object_size_hint is not None:
        size = float(object_size_hint)
    elif any(keyword in label_lower for keyword in large_keywords):
        size = 0.7
    elif any(keyword in label_lower for keyword in small_keywords):
        size = 0.25
    else:
        size = 0.35
    uncertainty = 0.05 if depth_m is None else min(0.15, 0.08 + float(depth_m) * 0.01)
    confidence = max(0.1, min(1.0, float(confidence or 0.5)))
    scale = 1.0 + (1.0 - confidence) * 0.2
    return round((float(base) + size) * 0.5 + uncertainty * scale, 4)