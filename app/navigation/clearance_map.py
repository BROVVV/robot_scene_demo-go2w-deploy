"""计划书 §7：障碍距离场（clearance map）。

导航不再只问 "前方那一格能不能走"，而是问 "这条路离最近的墙有多远"。
UNKNOWN 与 STALE 一律 fail-closed 返回 ``None``，调用方必须把 ``None``
当成不可通行，禁止当成空旷。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence, Tuple

from app.navigation.safety_models import (
    LocalSafetySnapshot,
    OBSERVABLE_STATES,
    SECTOR_WIDTH_DEG,
    STATE_OCCUPIED,
)

# UNKNOWN 不是 0 成本也不是 inf：它可以被规划器权衡，但永远比已知空旷更贵。
UNKNOWN_COST = 100.0

# ``from __future__ import annotations`` postpones annotations, but this is a
# runtime type-alias assignment.  Python 3.8 cannot subscript built-in tuple.
Point = Tuple[float, float]


def clearance_cost(
    distance_m: float | None,
    hard_clearance_m: float = 0.40,
    preferred_clearance_m: float = 0.80,
) -> float:
    """净空成本：硬边界内为 inf，达到偏好净空为 0，中间二次上升。"""
    if distance_m is None:
        return UNKNOWN_COST
    if distance_m < hard_clearance_m:
        return math.inf
    if distance_m >= preferred_clearance_m:
        return 0.0
    span = max(1e-6, preferred_clearance_m - hard_clearance_m)
    x = (preferred_clearance_m - distance_m) / span
    return x * x


@dataclass
class PathClearance:
    """一条路径的净空统计；``min_clearance_m is None`` 表示存在不可观测采样点。"""

    min_clearance_m: float | None = None
    mean_clearance_m: float | None = None
    samples: int = 0
    unknown_samples: int = 0
    length_m: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "min_clearance_m": _round(self.min_clearance_m),
            "mean_clearance_m": _round(self.mean_clearance_m),
            "samples": self.samples,
            "unknown_samples": self.unknown_samples,
            "length_m": round(self.length_m, 3),
        }


@dataclass
class ClearanceMap:
    """障碍点集合 + 已观测范围，查询任意位置到最近障碍的距离。

    ``local_only=True`` 时只有以 ``origin`` 为圆心、``max_range_m`` 为半径的
    区域算已观测，机器人身边的局部证据不会被外推成整张地图。
    """

    obstacles: tuple[Point, ...] = ()
    known_bounds: tuple[float, float, float, float] | None = None
    origin: Point = (0.0, 0.0)
    local_only: bool = False
    max_range_m: float = 6.0
    timestamp: float = 0.0
    max_age_sec: float = 3.0
    stale: bool = False
    sample_step_m: float = 0.05
    provenance: dict[str, Any] = field(default_factory=dict)

    # ---- queries ---------------------------------------------------------

    def is_fresh(self, now: float | None = None) -> bool:
        if self.stale:
            return False
        if now is None or self.max_age_sec <= 0.0:
            return True
        return (float(now) - self.timestamp) <= self.max_age_sec

    def observed(self, x: float, y: float) -> bool:
        if self.local_only:
            return math.hypot(x - self.origin[0], y - self.origin[1]) <= self.max_range_m
        if self.known_bounds is None:
            return True
        min_x, min_y, max_x, max_y = self.known_bounds
        return min_x <= x <= max_x and min_y <= y <= max_y

    def clearance_at(self, x: float, y: float, *, now: float | None = None) -> float | None:
        if not self.is_fresh(now):
            return None
        if not self.observed(x, y):
            return None
        if not self.obstacles:
            return self.max_range_m
        nearest = min(
            math.hypot(x - ox, y - oy) for ox, oy in self.obstacles
        )
        return min(nearest, self.max_range_m)

    def pose_clearance(
        self, pose: Sequence[float], footprint_radius_m: float = 0.0,
        *, now: float | None = None,
    ) -> float | None:
        distance = self.clearance_at(float(pose[0]), float(pose[1]), now=now)
        if distance is None:
            return None
        return max(0.0, distance - float(footprint_radius_m))

    def path_clearance(
        self,
        path: Sequence[Sequence[float]],
        footprint_radius_m: float = 0.0,
        *,
        now: float | None = None,
    ) -> PathClearance:
        points = [(float(item[0]), float(item[1])) for item in path]
        if not points:
            return PathClearance()
        samples = _sample_polyline(points, self.sample_step_m)
        values: list[float] = []
        unknown = 0
        for x, y in samples:
            distance = self.clearance_at(x, y, now=now)
            if distance is None:
                unknown += 1
                continue
            values.append(max(0.0, distance - float(footprint_radius_m)))
        length = sum(
            math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
            for i in range(len(points) - 1)
        )
        if unknown or not values:
            # fail-closed：任何不可观测采样点都让整条路径的最小净空变成未知。
            return PathClearance(None, None, len(samples), unknown, length)
        return PathClearance(
            min(values), sum(values) / len(values), len(samples), 0, length
        )

    def swept_clearance(
        self,
        pose_a: Sequence[float],
        pose_b: Sequence[float],
        footprint_radius_m: float = 0.0,
        *,
        now: float | None = None,
    ) -> PathClearance:
        return self.path_clearance([pose_a, pose_b], footprint_radius_m, now=now)

    def line_of_sight(
        self,
        origin: Sequence[float],
        target: Sequence[float],
        *,
        obstacle_clearance_m: float = 0.05,
        now: float | None = None,
    ) -> bool:
        """视线：中途任何采样点贴到障碍或不可观测都算被挡（fail-closed）。"""
        start = (float(origin[0]), float(origin[1]))
        end = (float(target[0]), float(target[1]))
        for x, y in _sample_polyline([start, end], self.sample_step_m)[1:-1] or []:
            distance = self.clearance_at(x, y, now=now)
            if distance is None or distance <= obstacle_clearance_m:
                return False
        return True

    def to_dict(self) -> dict[str, Any]:
        return {
            "obstacles": len(self.obstacles),
            "origin": [round(value, 3) for value in self.origin],
            "local_only": self.local_only,
            "max_range_m": self.max_range_m,
            "timestamp": self.timestamp,
            "stale": self.stale,
            "provenance": dict(self.provenance),
        }


def _sample_polyline(points: Sequence[Point], step_m: float) -> list[Point]:
    step = max(1e-3, float(step_m))
    samples: list[Point] = [points[0]]
    for index in range(len(points) - 1):
        ax, ay = points[index]
        bx, by = points[index + 1]
        span = math.hypot(bx - ax, by - ay)
        count = max(1, int(math.ceil(span / step)))
        for k in range(1, count + 1):
            t = k / count
            samples.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return samples


def clearance_map_from_points(
    obstacles: Iterable[Sequence[float]],
    *,
    known_bounds: tuple[float, float, float, float] | None = None,
    origin: Point = (0.0, 0.0),
    local_only: bool = False,
    max_range_m: float = 6.0,
    timestamp: float = 0.0,
    max_age_sec: float = 3.0,
    provenance: dict[str, Any] | None = None,
) -> ClearanceMap:
    return ClearanceMap(
        obstacles=tuple((float(item[0]), float(item[1])) for item in obstacles),
        known_bounds=known_bounds,
        origin=origin,
        local_only=local_only,
        max_range_m=max_range_m,
        timestamp=timestamp,
        max_age_sec=max_age_sec,
        provenance=dict(provenance or {}),
    )


def clearance_map_from_local_safety(
    snapshot: LocalSafetySnapshot | None,
    *,
    pose: Sequence[float] = (0.0, 0.0, 0.0),
    max_range_m: float = 6.0,
    points_per_sector: int = 3,
) -> ClearanceMap:
    """把 24 扇区局部快照转成 local-only 障碍点云。

    只有 CLEAR/OCCUPIED 扇区参与建图；其余扇区不产生障碍点，也不宣称空旷 ——
    因为 local-only 的已观测判定只看半径，所以这些方向仍会被后续采样当成
    未知（那些方向的 clearance 已在扇区里被置为 ``None``）。
    """
    if snapshot is None:
        return ClearanceMap(local_only=True, origin=(float(pose[0]), float(pose[1])),
                            max_range_m=max_range_m, stale=True)
    x0, y0 = float(pose[0]), float(pose[1])
    yaw_deg = math.degrees(float(pose[2])) if len(pose) > 2 else 0.0
    obstacles: list[Point] = []
    blind_sectors: list[float] = []
    for sector in snapshot.sectors:
        if sector.state not in OBSERVABLE_STATES or sector.clearance_m is None:
            blind_sectors.append(sector.bearing_deg)
            continue
        distance = float(sector.clearance_m)
        if sector.state != STATE_OCCUPIED and distance >= max_range_m:
            continue
        half = SECTOR_WIDTH_DEG / 2.0
        for index in range(max(1, points_per_sector)):
            share = 0.0 if points_per_sector <= 1 else (
                -half + index * (2.0 * half) / (points_per_sector - 1)
            )
            angle = math.radians(yaw_deg + sector.bearing_deg + share)
            obstacles.append((x0 + distance * math.cos(angle),
                              y0 + distance * math.sin(angle)))
    return ClearanceMap(
        obstacles=tuple(obstacles),
        origin=(x0, y0),
        local_only=True,
        max_range_m=max_range_m,
        timestamp=snapshot.timestamp,
        stale=not snapshot.lidar_fresh,
        provenance={"source": "local_safety_snapshot",
                    "blind_sectors_deg": blind_sectors},
    )


def _round(value: float | None) -> float | None:
    return None if value is None else round(value, 3)
