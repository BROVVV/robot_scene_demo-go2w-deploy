"""Degraded spatial provider: camera-local RGB-D without a global map.

This is the fallback when RTAB-Map / RGB-D odometry is unavailable.  It emits
relative frontier candidates that represent *new spatial observation points*
(relocate distance + bearing), not just headings.
"""

from __future__ import annotations

import math
from typing import Any

from app.spatial.models import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    FrontierCandidate,
    SpatialMapSnapshot,
    SpatialPose,
)


class CameraLocalSpatialProvider:
    """Keeps a relative pose when the robot backend supplies one, but never
    claims a metric map."""

    def __init__(
        self,
        *,
        relocate_distance_m: float = 0.25,
        relative_bearings_deg: tuple[float, float, float] = (-30.0, 0.0, 30.0),
    ) -> None:
        self.relocate_distance_m = float(relocate_distance_m)
        self.relative_bearings_deg = tuple(float(v) for v in relative_bearings_deg)
        self._pose: SpatialPose | None = None

    def quality(self) -> str:
        return SPATIAL_QUALITY_CAMERA_LOCAL

    def set_pose(self, pose: SpatialPose | None) -> None:
        self._pose = pose

    def get_pose(self) -> SpatialPose | None:
        return self._pose

    def get_map(self) -> SpatialMapSnapshot | None:
        return None

    def get_frontiers(self) -> list[FrontierCandidate]:
        yaw_deg = math.degrees(self._pose.yaw) if self._pose is not None else 0.0
        candidates: list[FrontierCandidate] = []
        for i, delta_deg in enumerate(self.relative_bearings_deg):
            bearing = yaw_deg + delta_deg
            rad = math.radians(bearing)
            position = None
            if self._pose is not None:
                position = (
                    round(self._pose.x + self.relocate_distance_m * math.cos(rad), 4),
                    round(self._pose.y + self.relocate_distance_m * math.sin(rad), 4),
                )
            candidates.append(
                FrontierCandidate(
                    frontier_id=f"relative_f_{i}",
                    position=position,
                    frame=self._pose.frame_id if self._pose else "odom",
                    bearing_deg=round(bearing, 2),
                    distance_m=self.relocate_distance_m,
                    size_score=1.0,
                    spatial_information_gain=0.5,
                    reachable=True,
                    nearby_semantics=[],
                    provenance={"source": "camera_local_relative_frontier"},
                )
            )
        return candidates

    def camera_point_to_spatial(
        self,
        xyz_camera: tuple[float, float, float],
        pose: SpatialPose | None = None,
    ) -> tuple[float, float, float] | None:
        # No camera-to-base extrinsic / map transform is available.
        return None


def make_camera_local_provider(
    *,
    pose: SpatialPose | None = None,
    relocate_distance_m: float = 0.25,
) -> CameraLocalSpatialProvider:
    provider = CameraLocalSpatialProvider(relocate_distance_m=relocate_distance_m)
    provider.set_pose(pose)
    return provider
