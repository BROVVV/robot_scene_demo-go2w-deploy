"""SpatialProvider: unified spatial/map interface used by the exploration core.

This protocol keeps SemanticNavigation / PSG / LongTermGoalSelector decoupled from
RTAB-Map, ROS topics and camera-local implementation details.
"""

from __future__ import annotations

from typing import Protocol

from app.spatial.models import (
    FrontierCandidate,
    SpatialMapSnapshot,
    SpatialPose,
)


class SpatialProvider(Protocol):
    def quality(self) -> str:
        """Return one of SPATIAL_QUALITY_* values."""
        ...

    def get_pose(self) -> SpatialPose | None:
        """Current robot/camera pose in the provider's frame, if available."""
        ...

    def get_map(self) -> SpatialMapSnapshot | None:
        """Occupancy/free/unknown map snapshot, if available."""
        ...

    def get_frontiers(self) -> list[FrontierCandidate]:
        """Frontier candidates.  May be relative placeholders when no map."""
        ...

    def camera_point_to_spatial(
        self,
        xyz_camera: tuple[float, float, float],
        pose: SpatialPose | None = None,
    ) -> tuple[float, float, float] | None:
        """Transform a camera-frame point into the spatial map frame.

        Returns ``None`` when the transform is unknown (camera-local mode).
        """
        ...
