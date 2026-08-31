"""Lightweight depth BEV mapper (fallback, not SLAM).

This mapper deliberately does not claim metric SLAM.  It projects a sparse set
of depth pixels into the robot's current SE(2) frame to maintain a local
free/occupied grid for frontier extraction.  It is intended only for
experimental spatial exploration when RTAB-Map is unavailable.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from app.perception.rgbd_source import RGBDFrame
from app.spatial.models import (
    SPATIAL_QUALITY_RELATIVE_RGBD,
    SpatialMapSnapshot,
    SpatialPose,
)


class LightweightDepthBEVMapper:
    def __init__(
        self,
        *,
        resolution_m: float = 0.05,
        map_size_m: float = 8.0,
        min_depth_m: float = 0.15,
        max_depth_m: float = 6.0,
        row_step: int = 8,
        col_step: int = 8,
    ) -> None:
        self.resolution_m = float(resolution_m)
        self.map_size_m = float(map_size_m)
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.row_step = max(1, int(row_step))
        self.col_step = max(1, int(col_step))
        self.width = int(round(self.map_size_m / self.resolution_m))
        self.height = self.width
        self._free: set[tuple[int, int]] = set()
        self._occupied: set[tuple[int, int]] = set()
        self._origin = (0.0, 0.0)
        self._revision = 0

    def update(self, frame: RGBDFrame, pose: SpatialPose | None = None) -> SpatialMapSnapshot:
        depth = self._load_depth(frame)
        if depth is None:
            return self.get_map()
        # World origin follows the robot start; this is relative RGB-D.
        if pose is not None:
            self._origin = (pose.x, pose.y)
        yaw = pose.yaw if pose is not None else 0.0
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        h, w = depth.shape
        for v in range(h // 2, min(h, h // 2 + 12), self.row_step):
            for u in range(0, w, self.col_step):
                z = float(depth[v, u])
                if not (self.min_depth_m < z < self.max_depth_m):
                    continue
                x_cam = (u - frame.cx) / frame.fx * z if frame.fx > 0 else 0.0
                y_cam = (v - frame.cy) / frame.fy * z if frame.fy > 0 else 0.0
                # Camera -> robot/body: x forward, y left, z up (D435 optical z forward).
                x_robot = x_cam
                y_robot = -y_cam
                x_world = self._origin[0] + x_robot * cos_yaw - y_robot * sin_yaw
                y_world = self._origin[1] + x_robot * sin_yaw + y_robot * cos_yaw
                # Mark free cells up to the endpoint (coarse).
                steps = max(1, int(round(z / self.resolution_m)))
                for step in range(1, steps + 1):
                    t = step / steps
                    gx = int(round((self._origin[0] + (x_robot * t) * cos_yaw - (y_robot * t) * sin_yaw - (self._origin[0] - self.map_size_m / 2.0)) / self.resolution_m))
                    gy = int(round((self._origin[1] + (x_robot * t) * sin_yaw + (y_robot * t) * cos_yaw - (self._origin[1] - self.map_size_m / 2.0)) / self.resolution_m))
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        self._free.add((gx, gy))
                gx_end = int(round((x_world - (self._origin[0] - self.map_size_m / 2.0)) / self.resolution_m))
                gy_end = int(round((y_world - (self._origin[1] - self.map_size_m / 2.0)) / self.resolution_m))
                if 0 <= gx_end < self.width and 0 <= gy_end < self.height:
                    self._occupied.add((gx_end, gy_end))
                    self._free.discard((gx_end, gy_end))
        self._revision += 1
        return self.get_map()

    def get_map(self) -> SpatialMapSnapshot:
        return SpatialMapSnapshot(
            revision=self._revision,
            resolution_m=self.resolution_m,
            origin=(round(self._origin[0] - self.map_size_m / 2.0, 3),
                    round(self._origin[1] - self.map_size_m / 2.0, 3)),
            width=self.width,
            height=self.height,
            free=sorted(self._free),
            occupied=sorted(self._occupied),
            unknown=[],
            quality=SPATIAL_QUALITY_RELATIVE_RGBD,
            source="lightweight_depth_bev",
            provenance={"map_size_m": self.map_size_m},
        )

    @staticmethod
    def _load_depth(frame: RGBDFrame) -> np.ndarray | None:
        try:
            import cv2
        except ImportError:
            return None
        img = cv2.imread(frame.depth_ref, cv2.IMREAD_UNCHANGED)
        if img is None:
            return None
        return img.astype(np.float32) * float(frame.depth_unit_m or 0.001)
