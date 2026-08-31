"""Small, dependency-free helpers for the WebUI SLAM point-cloud bridge.

The ROS subscriber lives in a system-Python sidecar because the FastAPI
environment and the ROS 2 environment are intentionally isolated.  Keeping
the byte decoding and bounded voxel accumulation here makes that boundary
testable without importing :mod:`rclpy`.
"""

from __future__ import annotations

import math
import struct
from collections import OrderedDict
from typing import Any, Iterable


POINT_FIELD_FLOAT32 = 7


def extract_xyz_points(
    message: Any,
    *,
    max_input_points: int = 8_000,
    max_range_m: float = 80.0,
    min_z_m: float = -10.0,
    max_z_m: float = 10.0,
) -> list[tuple[float, float, float]]:
    """Decode finite XYZ FLOAT32 values from a PointCloud2-like object.

    Organized clouds and padded rows are supported.  Large scans are sampled
    uniformly before decoding so the display bridge cannot monopolize a CPU.
    """

    width = max(0, int(getattr(message, "width", 0) or 0))
    height = max(1, int(getattr(message, "height", 1) or 1))
    point_step = max(0, int(getattr(message, "point_step", 0) or 0))
    row_step = int(getattr(message, "row_step", 0) or 0)
    if width <= 0 or point_step <= 0:
        return []
    if row_step <= 0:
        row_step = width * point_step

    fields = {
        str(getattr(field, "name", "")): field
        for field in (getattr(message, "fields", None) or [])
    }
    required = [fields.get(name) for name in ("x", "y", "z")]
    if any(field is None for field in required):
        return []
    if any(int(getattr(field, "datatype", -1)) != POINT_FIELD_FLOAT32 for field in required):
        return []
    offsets = [int(getattr(field, "offset", -1)) for field in required]
    if min(offsets) < 0 or max(offsets) + 4 > point_step:
        return []

    raw = memoryview(getattr(message, "data", b""))
    total = width * height
    limit = max(1, int(max_input_points))
    stride = max(1, math.ceil(total / limit))
    endian = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
    unpack_float = struct.Struct(endian + "f").unpack_from
    range_sq = float(max_range_m) ** 2
    points: list[tuple[float, float, float]] = []

    for index in range(0, total, stride):
        row, column = divmod(index, width)
        base = row * row_step + column * point_step
        if base + point_step > len(raw):
            break
        x = float(unpack_float(raw, base + offsets[0])[0])
        y = float(unpack_float(raw, base + offsets[1])[0])
        z = float(unpack_float(raw, base + offsets[2])[0])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if x * x + y * y + z * z > range_sq:
            continue
        if z < min_z_m or z > max_z_m:
            continue
        points.append((x, y, z))
    return points


class BoundedVoxelCloud:
    """Insertion-ordered voxel map with a hard memory bound."""

    def __init__(self, *, voxel_size_m: float = 0.12, max_points: int = 40_000) -> None:
        self.voxel_size_m = max(0.01, float(voxel_size_m))
        self.max_points = max(100, int(max_points))
        self._voxels: OrderedDict[tuple[int, int, int], tuple[float, float, float]] = OrderedDict()

    def clear(self) -> None:
        self._voxels.clear()

    def update(self, points: Iterable[tuple[float, float, float]]) -> None:
        inv = 1.0 / self.voxel_size_m
        for point in points:
            x, y, z = point
            key = (round(x * inv), round(y * inv), round(z * inv))
            if key in self._voxels:
                self._voxels.pop(key)
            self._voxels[key] = (float(x), float(y), float(z))
        while len(self._voxels) > self.max_points:
            self._voxels.popitem(last=False)

    def sampled(self, max_points: int) -> list[tuple[float, float, float]]:
        values = list(self._voxels.values())
        limit = max(1, int(max_points))
        if len(values) <= limit:
            return values
        stride = len(values) / limit
        return [values[min(len(values) - 1, int(i * stride))] for i in range(limit)]

    def __len__(self) -> int:
        return len(self._voxels)
