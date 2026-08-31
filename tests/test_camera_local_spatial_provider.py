"""Tests for CameraLocalSpatialProvider relative frontier generation."""

from __future__ import annotations

import math

from app.spatial.camera_local_spatial_provider import CameraLocalSpatialProvider
from app.spatial.models import SpatialPose


def test_frontiers_use_pose_yaw_radians():
    provider = CameraLocalSpatialProvider(relocate_distance_m=0.25)
    pose = SpatialPose(x=1.0, y=2.0, yaw=math.radians(90.0), frame_id="odom")
    provider.set_pose(pose)
    frontiers = provider.get_frontiers()
    assert len(frontiers) == 3
    # The 0° relative candidate should point in robot-forward direction (yaw 90°).
    forward = frontiers[1]
    assert forward.bearing_deg == 90.0
    assert forward.position is not None
    assert abs(forward.position[0] - 1.0) < 0.01
    assert abs(forward.position[1] - 2.25) < 0.01


def test_frontiers_without_pose_still_work():
    provider = CameraLocalSpatialProvider()
    frontiers = provider.get_frontiers()
    assert len(frontiers) == 3
    assert frontiers[0].bearing_deg == -30.0
