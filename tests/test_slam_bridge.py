"""SLAMBridge 单元测试（20260908 计划书 §27.1 / §27.3）。"""

from __future__ import annotations

import math

import pytest

from app.slam.slam_bridge import SLAMBridge, compose


def test_compose_translation_and_rotation():
    assert compose((1.0, 2.0, 0.0), (0.5, 0.0, 0.0)) == pytest.approx((1.5, 2.0, 0.0))
    rotated = compose((0.0, 0.0, math.pi / 2.0), (1.0, 0.0, 0.0))
    assert rotated[0] == pytest.approx(0.0, abs=1e-9)
    assert rotated[1] == pytest.approx(1.0, abs=1e-9)


def test_robot_pose_composes_map_odom_and_odom_base():
    bridge = SLAMBridge()
    bridge.note_map_to_odom(100.0, (10.0, 0.0, 0.0))
    bridge.note_odom_to_base(100.0, (1.0, 2.0, 0.0))
    assert bridge.robot_pose_map(100.0) == pytest.approx((11.0, 2.0, 0.0))


def test_pose_lookup_uses_image_timestamp_not_latest():
    """§27.3：图片拍摄于 t0，检测在 t1 返回，必须用 t0 的位姿。"""
    bridge = SLAMBridge()
    bridge.note_map_to_odom(100.0, (0.0, 0.0, 0.0))
    bridge.note_odom_to_base(100.0, (0.0, 0.0, 0.0))
    bridge.note_odom_to_base(101.0, (1.0, 0.0, 0.0))

    at_image_time = bridge.robot_pose_map(100.0)
    latest = bridge.robot_pose_map()

    assert at_image_time[0] == pytest.approx(0.0)
    assert latest[0] == pytest.approx(1.0)


def test_camera_point_uses_image_time_pose():
    bridge = SLAMBridge()
    bridge.note_map_to_odom(100.0, (0.0, 0.0, 0.0))
    bridge.note_odom_to_base(100.0, (0.0, 0.0, 0.0))
    bridge.note_odom_to_base(101.0, (10.0, 0.0, 0.0))

    point = bridge.camera_point_to_map((0.0, 0.0, 2.0), 100.0)

    # camera(0,0,2) -> base(2.1, 0, 0.3) -> map(2.1, 0, 0.3)；
    # 若错误使用 t1 的位姿会得到 (12.1, 0, 0.3)。
    assert point[0] == pytest.approx(2.1, abs=1e-3)
    assert point[2] == pytest.approx(0.3, abs=1e-3)


def test_missing_pose_returns_none():
    bridge = SLAMBridge()
    assert bridge.robot_pose_map(100.0) is None
    assert bridge.camera_point_to_map((0.0, 0.0, 1.0), 100.0) is None


def test_out_of_order_and_invalid_stamps_ignored():
    bridge = SLAMBridge()
    bridge.note_odom_to_base(100.0, (0.0, 0.0, 0.0))
    bridge.note_odom_to_base(99.0, (5.0, 0.0, 0.0))
    bridge.note_odom_to_base(float("nan"), (7.0, 0.0, 0.0))
    bridge.note_odom_to_base(0.0, (7.0, 0.0, 0.0))
    assert bridge.status()["odom_to_base_samples"] == 1


def test_buffer_prunes_old_samples():
    bridge = SLAMBridge(buffer_seconds=1.0)
    bridge.note_odom_to_base(100.0, (0.0, 0.0, 0.0))
    bridge.note_odom_to_base(105.0, (1.0, 0.0, 0.0))
    assert bridge.status()["odom_to_base_samples"] == 1


def test_loop_reprojection_uses_latest_map_correction_and_historical_pose():
    bridge = SLAMBridge(camera_translation_m=(0.0, 0.0, 0.0))
    bridge.note_map_to_odom(100.0, (3.3, 0.0, 0.0))
    bridge.note_odom_to_base(100.0, (0.0, 0.0, 0.0))
    inserted = bridge.camera_point_to_map((0.0, 0.0, 1.0), 100.0)

    bridge.note_map_to_odom(110.0, (3.0, 0.0, 0.0))
    bridge.note_odom_to_base(110.0, (5.0, 0.0, 0.0))
    corrected = bridge.reproject_camera_point_to_map((0.0, 0.0, 1.0), 100.0)

    assert inserted[0] == pytest.approx(4.3)
    assert corrected[0] == pytest.approx(4.0)


def test_map_correction_revision_changes_only_when_transform_changes():
    bridge = SLAMBridge()
    bridge.note_map_to_odom(100.0, (0.0, 0.0, 0.0))
    assert bridge.map_correction_revision == 0

    bridge.note_map_to_odom(101.0, (0.0, 0.0, 0.0))
    assert bridge.map_correction_revision == 0

    bridge.note_map_to_odom(102.0, (0.2, 0.0, 0.0))
    assert bridge.map_correction_revision == 1


def test_out_of_order_map_transform_does_not_change_revision():
    bridge = SLAMBridge()
    bridge.note_map_to_odom(100.0, (0.0, 0.0, 0.0))
    bridge.note_map_to_odom(99.0, (5.0, 0.0, 0.0))

    assert bridge.map_correction_revision == 0
    assert bridge.status()["map_to_odom_samples"] == 1
