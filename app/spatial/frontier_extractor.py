"""Metric frontier extraction from a free/unknown occupancy map.

The frontier definition is strictly ``Free <-> Unknown`` boundary, never a
heading sector.  This is a lightweight 4-connected component extractor; it is
not a full Nav2 planner.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any

from app.spatial.models import FrontierCandidate, SpatialMapSnapshot, SpatialPose


class FrontierExtractor:
    def __init__(
        self,
        *,
        min_component_size: int = 10,
        max_candidates: int = 8,
        min_distance_m: float = 0.0,
        max_distance_m: float = 20.0,
        id_quantization_m: float = 0.25,
    ) -> None:
        self.min_component_size = max(1, int(min_component_size))
        self.max_candidates = max(1, int(max_candidates))
        self.min_distance_m = float(min_distance_m)
        self.max_distance_m = float(max_distance_m)
        # frontier_id 的位置量化格大小（米）。默认 0.25 m 是**实测**选出来的：
        # 质心每轮漂移 0.025~0.035 m，0.25 m 的格给 ±0.125 m 容忍窗，
        # 合成墙上实测 0.789~0.842 的跨轮保持率，且间距 >= 0.20 m 的两个
        # 前沿不会撞成同一个 id。取更大（0.5 m）保持率更高（0.895~0.947），
        # 但实测 0.15~0.45 m 间距的两个真实前沿会撞 id —— 而**撞 id 比丢 id
        # 危险**：撞了会把两个前沿的记忆混在一起，丢的只是退化成"没记忆"。
        self.id_quantization_m = max(1e-6, float(id_quantization_m))

    def _quantized_frontier_id(self, centroid: tuple[float, float]) -> str:
        """由质心量化格给出前沿身份。

        2026-10-14 之前这里生成的是
        ``f"frontier_{i:02d}_{int(cx*100)}_{int(cy*100)}"``，其中 ``i`` 是连通
        分量的**枚举序号**。两处都不稳定：
        * ``i`` 来自 ``set`` 的迭代序（``_connected_components``），任何一处
          小分量出现/消失都会让它之后的分量整体平移；
        * 质心被截断到 0.01 m，而地图分辨率是 0.05 m —— 前沿每长大一格就跨
          过 5 个量化桶，**每轮都换 id**（合成墙上实测 0/20 保持不变）。
        身份换成纯位置量化格之后，同一物理前沿只要质心漂移 < q/2 就还是同一个
        id，按 id 记账的访存记忆才可能跨轮累加。

        用 ``floor(x/q + 0.5)`` 而不是 ``int(x*100)``：后者向零截断，负坐标下
        0.5 的边界不对称，而且 0.01 m 的桶比地图分辨率还细。
        """
        q = self.id_quantization_m
        gx = math.floor(centroid[0] / q + 0.5)
        gy = math.floor(centroid[1] / q + 0.5)
        return f"frontier_{int(gx)}_{int(gy)}"

    def extract(
        self,
        map_snapshot: SpatialMapSnapshot | None,
        robot_pose: SpatialPose | None = None,
    ) -> list[FrontierCandidate]:
        if map_snapshot is None or map_snapshot.width <= 0 or map_snapshot.height <= 0:
            return []
        map_frame = map_snapshot.provenance.get("frame_id")
        if map_frame and robot_pose is not None and robot_pose.frame_id != map_frame:
            return []
        grid = self._build_grid(map_snapshot)
        frontier_cells = self._boundary_cells(grid, map_snapshot.width, map_snapshot.height)
        components = self._connected_components(frontier_cells)
        # 先按量化 id 分组，每组只留 cell 数最多的那个分量：噪声会把一条边界
        # 切成两段，两段可能落进同一个量化格。多出来的必须**显式**丢弃并把
        # 丢弃数写进 provenance —— 绝不给 id 加后缀来区分，那正是"枚举序号进
        # id"的病根（同一物理前沿每轮换名字，记忆永远是空条目）。
        groups: dict[str, list[tuple[int, tuple[float, float], tuple[float, float]]]] = {}
        for cells in components:
            if len(cells) < self.min_component_size:
                continue
            centroid = self._centroid(cells, map_snapshot)
            # A concave/ring boundary's centroid need not be free. Keep its
            # identity, but navigate to an actual observed boundary cell.
            representative = self._representative(cells, map_snapshot)
            groups.setdefault(self._quantized_frontier_id(centroid), []).append(
                (len(cells), centroid, representative)
            )
        candidates: list[FrontierCandidate] = []
        for frontier_id, members in groups.items():
            size_px, centroid, position = max(members, key=lambda member: (member[0], member[1]))
            if robot_pose is not None:
                dx = position[0] - robot_pose.x
                dy = position[1] - robot_pose.y
                dist = math.hypot(dx, dy)
                if dist < self.min_distance_m or dist > self.max_distance_m:
                    continue
                # Consumers compare bearing to world-frame yaw. Keep it in
                # the map frame, like CameraLocalSpatialProvider does.
                bearing = math.degrees(math.atan2(dy, dx))
                bearing = (bearing + 180.0) % 360.0 - 180.0
            else:
                dist = None
                bearing = None
            size_score = min(1.0, size_px / max(20.0, self.min_component_size * 4))
            candidates.append(
                FrontierCandidate(
                    frontier_id=frontier_id,
                    position=position,
                    frame=map_frame or (robot_pose.frame_id if robot_pose is not None else map_snapshot.source),
                    bearing_deg=round(bearing, 2) if bearing is not None else None,
                    distance_m=round(dist, 3) if dist is not None else None,
                    size_score=round(size_score, 4),
                    spatial_information_gain=round(size_score, 4),
                    reachable=True,
                    nearby_semantics=[],
                    provenance={
                        "source": "metric_frontier_extractor",
                        "cells": size_px,
                        "id_collisions_dropped": len(members) - 1,
                        "identity_centroid": centroid,
                        "goal_source": "observed_frontier_cell_center",
                        "reachability_checked": False,
                    },
                )
            )
        candidates.sort(key=lambda item: item.spatial_information_gain, reverse=True)
        return candidates[: self.max_candidates]

    @staticmethod
    def _build_grid(map_snapshot: SpatialMapSnapshot) -> list[list[int]]:
        grid = [[2] * map_snapshot.width for _ in range(map_snapshot.height)]
        for x, y in map_snapshot.free:
            if 0 <= x < map_snapshot.width and 0 <= y < map_snapshot.height:
                grid[y][x] = 0
        for x, y in map_snapshot.occupied:
            if 0 <= x < map_snapshot.width and 0 <= y < map_snapshot.height:
                grid[y][x] = 1
        return grid

    def _boundary_cells(
        self, grid: list[list[int]], width: int, height: int
    ) -> set[tuple[int, int]]:
        cells: set[tuple[int, int]] = set()
        for y in range(height):
            for x in range(width):
                if grid[y][x] != 0:
                    continue
                for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                    if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] == 2:
                        cells.add((x, y))
                        break
        return cells

    def _connected_components(
        self, cells: set[tuple[int, int]]
    ) -> list[list[tuple[int, int]]]:
        visited: set[tuple[int, int]] = set()
        components: list[list[tuple[int, int]]] = []
        for start in cells:
            if start in visited:
                continue
            queue = deque([start])
            visited.add(start)
            comp: list[tuple[int, int]] = []
            while queue:
                x, y = queue.popleft()
                comp.append((x, y))
                for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                    neighbor = (nx, ny)
                    if neighbor in cells and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            components.append(comp)
        return components

    def _centroid(
        self, cells: list[tuple[int, int]], map_snapshot: SpatialMapSnapshot
    ) -> tuple[float, float]:
        if not cells:
            return map_snapshot.origin
        ox, oy = map_snapshot.origin
        res = map_snapshot.resolution_m or 1.0
        sx = sum(x for x, _ in cells) / len(cells)
        sy = sum(y for _, y in cells) / len(cells)
        return (ox + sx * res, oy + sy * res)

    @staticmethod
    def _representative(
        cells: list[tuple[int, int]], map_snapshot: SpatialMapSnapshot
    ) -> tuple[float, float]:
        sx = sum(x for x, _ in cells) / len(cells)
        sy = sum(y for _, y in cells) / len(cells)
        x, y = min(cells, key=lambda p: ((p[0] - sx) ** 2 + (p[1] - sy) ** 2, p))
        ox, oy = map_snapshot.origin
        res = map_snapshot.resolution_m or 1.0
        return ox + (x + .5) * res, oy + (y + .5) * res
