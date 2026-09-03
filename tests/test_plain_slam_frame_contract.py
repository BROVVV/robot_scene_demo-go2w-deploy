"""计划书 §17.9：plain_slam SpatialProvider 禁止混用 odom 与 pslam_odom。

不变量 3（坐标不能裸混）：wheel odom 数值严禁直接写入 pslam 世界坐标系。
"""

from __future__ import annotations

import pytest

from app.spatial.models import SpatialFrameMismatch, SpatialPose
from app.spatial.plain_slam_spatial_provider import PlainSlamSpatialProvider


def test_set_pose_rejects_wheel_odom_frame():
    provider = PlainSlamSpatialProvider()
    wheel = SpatialPose(
        x=1.0, y=2.0, yaw=0.5,
        frame_id="odom", quality="relative", source="go2w_wheel_odom",
    )
    with pytest.raises(SpatialFrameMismatch) as excinfo:
        provider.set_pose(wheel)
    message = str(excinfo.value)
    assert "pose_frame=odom" in message
    assert "map_frame=pslam_odom" in message
    # 混算被拒绝后内部 pose 仍为空（不写入错误 frame）。
    assert provider.get_pose() is None


def test_set_pose_accepts_pslam_odom_frame():
    provider = PlainSlamSpatialProvider()
    pslam = SpatialPose(
        x=1.0, y=2.0, yaw=0.5,
        frame_id="pslam_odom", quality="METRIC_LIDAR",
        source="plain_slam_pandarxt16_odom",
    )
    provider.set_pose(pslam)
    assert provider.get_pose() is pslam


def test_camera_point_to_spatial_rejects_mismatched_pose():
    provider = PlainSlamSpatialProvider()
    wheel = SpatialPose(
        x=0.0, y=0.0, yaw=0.0,
        frame_id="odom", quality="relative", source="go2w_wheel_odom",
    )
    with pytest.raises(SpatialFrameMismatch):
        provider.camera_point_to_spatial((0.0, 0.0, 1.0), pose=wheel)


def test_quality_constant_for_no_global_pose():
    from app.spatial.models import SPATIAL_QUALITY_NO_GLOBAL_POSE

    assert SPATIAL_QUALITY_NO_GLOBAL_POSE == "NO_GLOBAL_SPATIAL_POSE"
