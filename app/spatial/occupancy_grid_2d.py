"""Pure geometry helpers for ROS OccupancyGrid map coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class OccupancyGrid2D:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]
    map_epoch: str | None = None

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0:
            raise ValueError("invalid occupancy grid geometry")
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy grid data length mismatch")
        if not all(math.isfinite(value) for value in (
            self.resolution, self.origin_x, self.origin_y, self.origin_yaw
        )):
            raise ValueError("nonfinite occupancy grid geometry")

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        """Convert a world point through the full rotated map origin."""
        dx, dy = float(x) - self.origin_x, float(y) - self.origin_y
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = math.floor(local_x / self.resolution)
        row = math.floor(local_y / self.resolution)
        if not (0 <= column < self.width and 0 <= row < self.height):
            return None
        return int(column), int(row)

    def value_at_world(self, x: float, y: float) -> int | None:
        cell = self.world_to_cell(x, y)
        if cell is None:
            return None
        column, row = cell
        return int(self.data[row * self.width + column])

    def line_evidence(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        *,
        occupied_threshold: int = 50,
        max_unknown_fraction: float = 0.35,
        endpoint_clearance_m: float = 0.25,
    ) -> tuple[bool, str]:
        """Return whether the map supports a direct local adjacency.

        Endpoint cells are omitted because an object can legitimately overlap
        an occupied footprint.  A known occupied cell in the interior proves a
        wall.  Mostly unknown or out of map paths are withheld rather than
        advertised as connected.
        """
        sx, sy = (float(value) for value in start)
        ex, ey = (float(value) for value in end)
        distance = math.hypot(ex - sx, ey - sy)
        if not math.isfinite(distance) or distance <= 1e-6:
            return False, "invalid_pair_geometry"
        clearance = min(endpoint_clearance_m, distance * 0.2)
        usable = max(0.0, distance - 2.0 * clearance)
        samples = max(2, int(math.ceil(usable / max(self.resolution * 0.5, 0.02))) + 1)
        unknown = 0
        for index in range(samples):
            along = clearance + usable * index / max(1, samples - 1)
            ratio = along / distance
            value = self.value_at_world(sx + (ex - sx) * ratio, sy + (ey - sy) * ratio)
            if value is None or value < 0:
                unknown += 1
            elif value >= occupied_threshold:
                return False, "occupied_cell_between_objects"
        if unknown / samples > max_unknown_fraction:
            return False, "insufficient_known_free_space"
        return True, "occupancy_grid_free_path"

