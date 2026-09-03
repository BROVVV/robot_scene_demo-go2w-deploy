"""Small helpers for the WebUI SLAM point-cloud bridge.

The ROS subscriber lives in a system-Python sidecar because the FastAPI
environment and the ROS 2 environment are intentionally isolated.  Keeping
the byte decoding and the global voxel map here makes that boundary
testable without importing :mod:`rclpy`.
"""

from __future__ import annotations

import math
import struct
from typing import Any, Iterable

import numpy as np


POINT_FIELD_FLOAT32 = 7
# ±524288 个体素能塞进一个 int64 哈希；0.12m 体素时等于 ±62km，比任何室内地图都大。
VOXEL_KEY_SPAN = 1 << 20


def _xyz_layout(message: Any) -> dict[str, Any] | None:
    """Validate the PointCloud2 layout once for both decoders."""

    width = max(0, int(getattr(message, "width", 0) or 0))
    height = max(1, int(getattr(message, "height", 1) or 1))
    point_step = max(0, int(getattr(message, "point_step", 0) or 0))
    row_step = int(getattr(message, "row_step", 0) or 0)
    if width <= 0 or point_step <= 0:
        return None
    if row_step <= 0:
        row_step = width * point_step
    fields = {
        str(getattr(field, "name", "")): field
        for field in (getattr(message, "fields", None) or [])
    }
    required = [fields.get(name) for name in ("x", "y", "z")]
    if any(field is None for field in required):
        return None
    if any(int(getattr(field, "datatype", -1)) != POINT_FIELD_FLOAT32
           for field in required):
        return None
    offsets = [int(getattr(field, "offset", -1)) for field in required]
    if min(offsets) < 0 or max(offsets) + 4 > point_step:
        return None
    return {
        "width": width, "height": height, "point_step": point_step,
        "row_step": row_step, "offsets": offsets,
        "bigendian": bool(getattr(message, "is_bigendian", False)),
    }


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

    layout = _xyz_layout(message)
    if layout is None:
        return []
    width = layout["width"]
    point_step = layout["point_step"]
    row_step = layout["row_step"]
    offsets = layout["offsets"]

    raw = memoryview(getattr(message, "data", b""))
    total = width * layout["height"]
    limit = max(1, int(max_input_points))
    stride = max(1, math.ceil(total / limit))
    endian = ">" if layout["bigendian"] else "<"
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


def extract_xyz_array(
    message: Any,
    *,
    max_range_m: float = 200.0,
    min_z_m: float = -10.0,
    max_z_m: float = 10.0,
) -> np.ndarray:
    """Decode a whole PointCloud2 into an ``(N, 3)`` float32 array.

    全局地图有十几万点，纯 Python struct 解码要 150ms 以上；而地图必须整帧解码
    （抽稀会把远处房间抽没），所以这条路径走 numpy。
    """

    layout = _xyz_layout(message)
    empty = np.zeros((0, 3), dtype=np.float32)
    if layout is None:
        return empty
    width, height = layout["width"], layout["height"]
    point_step, row_step = layout["point_step"], layout["row_step"]
    raw = np.frombuffer(getattr(message, "data", b""), dtype=np.uint8)
    if row_step != width * point_step:
        rows = min(height, raw.size // row_step)
        raw = np.ascontiguousarray(
            raw[:rows * row_step].reshape(rows, row_step)[:, :width * point_step]
        ).reshape(-1)
    total = min(width * height, raw.size // point_step)
    if total <= 0:
        return empty
    order = ">f4" if layout["bigendian"] else "<f4"
    record = np.dtype({
        "names": ["x", "y", "z"], "formats": [order] * 3,
        "offsets": layout["offsets"], "itemsize": point_step,
    })
    values = np.ascontiguousarray(raw[:total * point_step]).view(record)
    xyz = np.stack([values["x"], values["y"], values["z"]], axis=1).astype(np.float32)
    keep = np.isfinite(xyz).all(axis=1)
    keep &= (xyz * xyz).sum(axis=1) <= float(max_range_m) ** 2
    keep &= (xyz[:, 2] >= float(min_z_m)) & (xyz[:, 2] <= float(max_z_m))
    return xyz[keep]


def voxel_downsample(points: np.ndarray, voxel_size_m: float) -> np.ndarray:
    """Keep one point per voxel, in first-seen order."""

    array = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if array.shape[0] == 0:
        return array
    size = max(0.01, float(voxel_size_m))
    keys = np.floor(array.astype(np.float64) / size).astype(np.int64)
    keys += VOXEL_KEY_SPAN >> 1
    np.clip(keys, 0, VOXEL_KEY_SPAN - 1, out=keys)
    flat = (keys[:, 0] * VOXEL_KEY_SPAN + keys[:, 1]) * VOXEL_KEY_SPAN + keys[:, 2]
    _, index = np.unique(flat, return_index=True)
    return array[np.sort(index)]


def fit_voxel_size(points: np.ndarray, voxel_size_m: float,
                   max_points: int) -> tuple[np.ndarray, float]:
    """Downsample at ``voxel_size_m``, coarsening globally until it fits.

    §9.3.1：容量不够时只能整张图一起变粗。按插入顺序淘汰体素会把先走过的
    房间从地图上抹掉，而那部分地图 LIO 里其实一直都在。
    """

    limit = max(1, int(max_points))
    size = max(0.01, float(voxel_size_m))
    reduced = voxel_downsample(points, size)
    if reduced.shape[0] <= limit:
        return reduced, size
    fine = size
    for _ in range(16):
        # 面状点云的体素数≈size^-2，所以一步就能跳到接近目标的分辨率。
        size *= max(1.1, math.sqrt(reduced.shape[0] / limit))
        reduced = voxel_downsample(points, size)
        if reduced.shape[0] <= limit:
            break
    # 在 fine/size 之间二分几次，否则体积型点云会只用掉一半的点数预算。
    coarse, best = size, reduced
    for _ in range(3):
        middle = math.sqrt(fine * coarse)
        candidate = voxel_downsample(points, middle)
        if candidate.shape[0] <= limit:
            coarse, best = middle, candidate
        else:
            fine = middle
    if best.shape[0] > limit:
        best = best[::math.ceil(best.shape[0] / limit)]
    return best, coarse


class GlobalVoxelCloud:
    """Web-side voxel cache of the fixed-world SLAM map.

    §9.3：SLAM 侧的权威地图可以一直长大；这一层只是网页缓存，只允许整体替换
    和整体降低分辨率，任何区域都不许因为"缓存满了"而消失。
    """

    def __init__(self, *, voxel_size_m: float = 0.12,
                 max_voxels: int = 300_000) -> None:
        self.voxel_size_m = max(0.01, float(voxel_size_m))
        self.max_voxels = max(1_000, int(max_voxels))
        self.effective_voxel_size_m = self.voxel_size_m
        self.source_point_count = 0
        self._points = np.zeros((0, 3), dtype=np.float32)

    @property
    def capacity_limited(self) -> bool:
        return self.effective_voxel_size_m > self.voxel_size_m + 1e-9

    def clear(self) -> None:
        self._points = np.zeros((0, 3), dtype=np.float32)
        self.effective_voxel_size_m = self.voxel_size_m
        self.source_point_count = 0

    def replace(self, points: Iterable[tuple[float, float, float]] | np.ndarray) -> None:
        """整张地图替换：权威地图每次发布的都是完整的优化后历史。"""
        array = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        self.source_point_count = int(array.shape[0])
        self._points, self.effective_voxel_size_m = fit_voxel_size(
            array, self.voxel_size_m, self.max_voxels
        )

    def sample(self, max_points: int) -> tuple[list[tuple[float, float, float]],
                                               dict[str, Any]]:
        """Spatially uniform subset for the browser; never insertion-order truncation."""
        limit = max(1, int(max_points))
        if self._points.shape[0] <= limit:
            chosen, size, mode = self._points, self.effective_voxel_size_m, "all_voxels"
        else:
            chosen, size = fit_voxel_size(self._points,
                                          self.effective_voxel_size_m, limit)
            mode = "uniform_voxel"
        info = {"mode": mode, "voxel_size_m": round(float(size), 4),
                "limit": limit, "truncated": mode != "all_voxels"}
        return [(float(x), float(y), float(z)) for x, y, z in chosen], info

    def bounds(self) -> dict[str, list[float]]:
        """Extent of the whole cache, not of the sampled subset."""
        if self._points.shape[0] == 0:
            return {"min": [0.0, 0.0, 0.0], "max": [0.0, 0.0, 0.0]}
        return {
            "min": [round(float(value), 3) for value in self._points.min(axis=0)],
            "max": [round(float(value), 3) for value in self._points.max(axis=0)],
        }

    def extent_m(self) -> list[float]:
        box = self.bounds()
        return [round(box["max"][axis] - box["min"][axis], 3) for axis in range(3)]

    def __len__(self) -> int:
        return int(self._points.shape[0])
