# Copyright 2026 robot_scene_demo maintainers

"""Ray-traced occupancy projection validation (plan §18.3 / §9).

Python reference mirror of the C++ pointcloud_to_occupancy logic.  It is used
for offline acceptance of the *map semantics* the C++ node must reproduce:

  * synthetic corridor, robot at (0.5, 2.0), wall at x = 3.0
  * cells between robot and wall -> FREE
  * wall endpoint -> OCCUPIED
  * never scanned area  -> UNKNOWN
  * FrontierExtractor finds the free/unknown boundary (frontier)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from app.spatial.frontier_extractor import FrontierExtractor
from app.spatial.models import SpatialMapSnapshot, SpatialPose

RESOLUTION_M = 0.10
WIDTH = 80
HEIGHT = 80
ORIGIN = (-2.0, -2.0)
SENSOR = (0.5, 2.0)
WALL_X = 3.0


@dataclass
class Grid:
    width: int = WIDTH
    height: int = HEIGHT
    resolution: float = RESOLUTION_M
    origin: tuple[float, float] = ORIGIN
    log_odds: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.log_odds:
            self.log_odds = [0.0] * (self.width * self.height)

    def _index(self, x: int, y: int) -> int:
        return y * self.width + x

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def world_to_cell(self, wx: float, wy: float) -> tuple[int, int]:
        cx = int(math.floor((wx - self.origin[0]) / self.resolution))
        cy = int(math.floor((wy - self.origin[1]) / self.resolution))
        return cx, cy

    def add(self, x: int, y: int, delta: float) -> None:
        if not self.in_bounds(x, y):
            return
        lo = self.log_odds[self._index(x, y)] + delta
        self.log_odds[self._index(x, y)] = min(3.5, max(-2.0, lo))

    def raytrace_free(self, x0: int, y0: int, x1: int, y1: int) -> None:
        """Amanatides-Woo DDA mirroring the C++ implementation."""
        if not self.in_bounds(x0, y0):
            return
        dx = x1 - x0
        dy = y1 - y0
        step_x = 1 if dx > 0 else -1
        step_y = 1 if dy > 0 else -1
        t_delta_x = abs(1.0 / dx) if abs(dx) > 1e-12 else math.inf
        t_delta_y = abs(1.0 / dy) if abs(dy) > 1e-12 else math.inf
        if dx > 0:
            t_max_x = (math.floor(x0) + 1.0 - x0) * t_delta_x
        elif dx < 0:
            t_max_x = (x0 - math.floor(x0)) * t_delta_x
        else:
            t_max_x = t_delta_x
        if dy > 0:
            t_max_y = (math.floor(y0) + 1.0 - y0) * t_delta_y
        elif dy < 0:
            t_max_y = (y0 - math.floor(y0)) * t_delta_y
        else:
            t_max_y = t_delta_y
        vx, vy = x0, y0
        guard = (self.width + self.height) * 2 + 16
        for _ in range(guard):
            if t_max_x < t_max_y:
                vx += step_x
                t_max_x += t_delta_x
            else:
                vy += step_y
                t_max_y += t_delta_y
            if vx == x1 and vy == y1:
                break
            if not self.in_bounds(vx, vy):
                break
            if vx != x0 or vy != y0:
                self.add(vx, vy, -0.40)

    def cell_value(self, x: int, y: int) -> int:
        if not self.in_bounds(x, y):
            return -1
        lo = self.log_odds[self._index(x, y)]
        p = math.exp(lo) / (1.0 + math.exp(lo))
        if p >= 0.65:
            return 100
        if p <= 0.35:
            return 0
        return -1


def build_corridor_map(frames: int = 6) -> Grid:
    """Multi-frame projection: real 10 Hz scans accumulate log-odds over
    ~0.6 s before a single ray's miss crosses the free probability threshold."""
    grid = Grid()
    for _ in range(frames):
        sensor_cx, sensor_cy = grid.world_to_cell(*SENSOR)
        grid.add(sensor_cx, sensor_cy, -0.40)
        # Floor points + wall endpoints: enough rays to clear + hit the wall.
        for angle_deg in range(0, 360, 3):
            angle = math.radians(angle_deg)
            radius = 0.6
            px = SENSOR[0] + radius * math.cos(angle)
            py = SENSOR[1] + radius * math.sin(angle)
            cx, cy = grid.world_to_cell(px, py)
            grid.raytrace_free(sensor_cx, sensor_cy, cx, cy)
            grid.add(cx, cy, -0.40)
            # Wall endpoint on the corridor axis.
            wall_cx, wall_cy = grid.world_to_cell(WALL_X, 2.0)
            grid.raytrace_free(sensor_cx, sensor_cy, wall_cx, wall_cy)
            grid.add(wall_cx, wall_cy, 0.85)
    return grid


def grid_to_snapshot(grid: Grid) -> SpatialMapSnapshot:
    free: list[tuple[int, int]] = []
    occupied: list[tuple[int, int]] = []
    unknown: list[tuple[int, int]] = []
    for y in range(grid.height):
        for x in range(grid.width):
            value = grid.cell_value(x, y)
            if value == 0:
                free.append((x, y))
            elif value == 100:
                occupied.append((x, y))
            else:
                unknown.append((x, y))
    return SpatialMapSnapshot(
        revision=1,
        resolution_m=grid.resolution,
        origin=grid.origin,
        width=grid.width,
        height=grid.height,
        free=free,
        occupied=occupied,
        unknown=unknown,
        source="plain_slam_pandarxt16",
        provenance={"frame_id": "pslam_odom"},
    )


def test_corridor_free_occupied_unknown() -> None:
    grid = build_corridor_map()
    mid = grid.world_to_cell(1.5, 2.0)
    wall = grid.world_to_cell(WALL_X, 2.0)
    far = grid.world_to_cell(0.5, -1.5)
    assert grid.cell_value(*mid) == 0, "corridor between robot and wall must be free"
    assert grid.cell_value(*wall) == 100, "wall endpoint must be occupied"
    assert grid.cell_value(*far) == -1, "unseen area must stay unknown"


def test_frontier_extractor_finds_boundary() -> None:
    snapshot = grid_to_snapshot(build_corridor_map())
    pose = SpatialPose(x=0.5, y=2.0, yaw=0.0, frame_id="pslam_odom")
    extractor = FrontierExtractor(min_component_size=1)
    frontiers = extractor.extract(snapshot, pose)
    assert frontiers, "free/unknown boundary must yield at least one frontier"
    # The corridor-head frontier centroid lies close to the wall (x ~ 2.9).
    assert any(
        f.position is not None and f.position[0] > 2.0 for f in frontiers
    )


def test_unknown_preserved_outside_rays() -> None:
    grid = build_corridor_map()
    # A corner that neither the rays nor endpoints touched stays unknown.
    corner = grid.world_to_cell(-1.9, -1.9)
    assert grid.cell_value(*corner) == -1
