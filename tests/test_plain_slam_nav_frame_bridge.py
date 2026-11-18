"""计划书 §12.1：Nav2 坐标桥测试。

被测模块是 ROS 包里的脚本，但几何与闸门逻辑刻意不依赖 rclpy，所以宿主机的
测试环境（无 rclpy）可以直接 import 它。
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws/src/robot_scene_nav_bringup/scripts/plain_slam_nav_frame_bridge.py"
)


def _module():
    spec = importlib.util.spec_from_file_location("nav_frame_bridge_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    # 必须先登记进 sys.modules：dataclass 装饰器要靠 cls.__module__ 回查模块命名空间。
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


mod = _module()
Pose2D = mod.Pose2D


class _Header:
    def __init__(self, frame_id: str = "") -> None:
        self.frame_id = frame_id


class _Grid:
    """OccupancyGrid 的最小替身：只关心 header 与 info 有没有被改动。"""

    def __init__(self) -> None:
        self.header = _Header("pslam_odom")
        self.info = {"resolution": 0.10, "origin": (-40.0, -40.0, 0.0)}
        self.data = [0, 100, -1, 0]


def _grid_snapshot(grid: _Grid) -> dict:
    return {
        "resolution": grid.info["resolution"],
        "origin": grid.info["origin"],
        "data": list(grid.data),
        "width": 4,
    }


# ---------------------------------------------------------------------------
# §12.1 T_map_odom = T_map_base * inverse(T_odom_base)
# ---------------------------------------------------------------------------


def test_map_to_odom_matches_the_closed_form() -> None:
    # map 系下机体在 (3, 4, +30°)；odom_fused 系下机体在 (1, 2, +10°)。
    map_to_base = Pose2D(3.0, 4.0, math.radians(30.0))
    odom_to_base = Pose2D(1.0, 2.0, math.radians(10.0))
    result = mod.map_to_odom_2d(map_to_base, odom_to_base)

    # 手算：T_odom_base 的逆 = 平移 -R(-10°)·(1,2)，yaw -10°；
    # 再与 T_map_base 复合。
    cos_yaw, sin_yaw = math.cos(math.radians(10.0)), math.sin(math.radians(10.0))
    inverse_translation = (
        -(cos_yaw * 1.0 + sin_yaw * 2.0),
        -(-sin_yaw * 1.0 + cos_yaw * 2.0),
    )
    cos_map, sin_map = math.cos(math.radians(30.0)), math.sin(math.radians(30.0))
    expected_x = 3.0 + cos_map * inverse_translation[0] - sin_map * inverse_translation[1]
    expected_y = 4.0 + sin_map * inverse_translation[0] + cos_map * inverse_translation[1]

    assert result.x == pytest.approx(expected_x)
    assert result.y == pytest.approx(expected_y)
    assert result.yaw == pytest.approx(math.radians(20.0))


def test_map_to_odom_reproduces_map_to_base_when_composed_back() -> None:
    """复合回去必须还原 plain_slam 报的 map->base —— 这是公式成立的充要条件。"""
    map_to_base = Pose2D(1.7, -2.3, math.radians(-115.0))
    odom_to_base = Pose2D(-0.4, 0.9, math.radians(160.0))
    map_to_odom = mod.map_to_odom_2d(map_to_base, odom_to_base)
    assert mod.compose_pose2d(map_to_odom, odom_to_base).x == pytest.approx(map_to_base.x)
    assert mod.compose_pose2d(map_to_odom, odom_to_base).y == pytest.approx(map_to_base.y)
    assert mod.compose_pose2d(map_to_odom, odom_to_base).yaw == pytest.approx(
        map_to_base.yaw
    )


def test_identity_case_leaves_odom_frame_on_top_of_map() -> None:
    pose = Pose2D(2.0, -1.0, 0.5)
    result = mod.map_to_odom_2d(pose, pose)
    assert result.x == pytest.approx(0.0, abs=1e-12)
    assert result.y == pytest.approx(0.0, abs=1e-12)
    assert result.yaw == pytest.approx(0.0, abs=1e-12)


def test_yaw_round_trips_through_the_quaternion_helpers() -> None:
    for yaw in (0.0, 0.3, -1.9, math.pi - 1e-6):
        qx, qy, qz, qw = mod.quaternion_from_yaw(yaw)
        assert mod.yaw_from_quaternion(qx, qy, qz, qw) == pytest.approx(yaw)


# ---------------------------------------------------------------------------
# §12.1 /map 的 frame 改写
# ---------------------------------------------------------------------------


def test_map_is_republished_with_map_frame_and_untouched_content() -> None:
    grid = _Grid()
    before = _grid_snapshot(grid)
    returned = mod.retarget_map_frame(grid, "map")
    assert returned is grid
    assert grid.header.frame_id == "map"
    assert _grid_snapshot(grid) == before


# ---------------------------------------------------------------------------
# §12.1 localization_valid 的四类失败 + 正常
# ---------------------------------------------------------------------------


def _valid_kwargs(**overrides):
    kwargs = dict(
        map_received=True,
        plain_slam_pose=Pose2D(1.0, 2.0, 0.1),
        fused_pose=Pose2D(0.1, 0.2, 0.1),
        plain_slam_age_s=0.05,
        fused_age_s=0.05,
        frame_skew_s=0.01,
        tf_published=True,
    )
    kwargs.update(overrides)
    return kwargs


def test_synchronized_data_is_valid() -> None:
    verdict = mod.evaluate_localization(**_valid_kwargs())
    assert verdict.valid
    assert verdict.reason == mod.REASON_OK


def test_stale_odometry_invalidates() -> None:
    assert not mod.evaluate_localization(**_valid_kwargs(plain_slam_age_s=0.9)).valid
    assert not mod.evaluate_localization(**_valid_kwargs(fused_age_s=0.9)).valid
    # 恰好 0.5 s 还不算过期。
    assert mod.evaluate_localization(**_valid_kwargs(plain_slam_age_s=0.5)).valid


def test_large_frame_skew_invalidates() -> None:
    verdict = mod.evaluate_localization(**_valid_kwargs(frame_skew_s=0.4))
    assert not verdict.valid
    assert verdict.reason == mod.REASON_FRAME_SKEW


def test_nonfinite_pose_invalidates() -> None:
    assert not mod.evaluate_localization(
        **_valid_kwargs(plain_slam_pose=Pose2D(float("nan"), 0.0, 0.0))
    ).valid
    assert not mod.evaluate_localization(
        **_valid_kwargs(fused_pose=Pose2D(0.0, float("inf"), 0.0))
    ).valid


def test_missing_inputs_invalidate() -> None:
    assert not mod.evaluate_localization(**_valid_kwargs(map_received=False)).valid
    assert not mod.evaluate_localization(**_valid_kwargs(plain_slam_pose=None)).valid
    assert not mod.evaluate_localization(**_valid_kwargs(fused_pose=None)).valid


def test_tf_not_published_invalidates() -> None:
    verdict = mod.evaluate_localization(**_valid_kwargs(tf_published=False))
    assert not verdict.valid
    assert verdict.reason == mod.REASON_TF_NOT_PUBLISHED


def test_valid_is_never_carried_over() -> None:
    """闸门是纯函数：没有上一次的 true 可以沿用。"""
    assert mod.evaluate_localization(**_valid_kwargs()).valid
    assert not mod.evaluate_localization(**_valid_kwargs(fused_age_s=99.0)).valid


# ---------------------------------------------------------------------------
# §12.1 core：配对与发布内容
# ---------------------------------------------------------------------------


def _core():
    return mod.FrameBridgeCore()


def test_core_emits_both_transforms_when_synchronized() -> None:
    core = _core()
    core.note_map()
    core.note_plain_slam_odom(100.00, Pose2D(3.0, 4.0, 0.5))
    core.note_fused_odom(100.02, Pose2D(1.0, 2.0, 0.5))
    computation = core.compute(100.05)
    assert computation.ok
    assert computation.map_to_odom is not None
    # odom_fused -> base_link 直接用 fused odom 的位姿。
    assert computation.odom_to_base == Pose2D(1.0, 2.0, 0.5)
    assert computation.frame_skew_s == pytest.approx(-0.02)
    assert core.verdict(computation, tf_published=True).valid
    assert not core.verdict(computation, tf_published=False).valid


def test_core_without_map_never_emits_transforms() -> None:
    core = _core()
    core.note_plain_slam_odom(100.0, Pose2D(1.0, 1.0, 0.0))
    core.note_fused_odom(100.0, Pose2D(1.0, 1.0, 0.0))
    computation = core.compute(100.0)
    assert not computation.ok
    assert computation.reason == mod.REASON_NO_MAP
    assert computation.map_to_odom is None


def test_core_pairs_the_nearest_samples_not_the_latest_ones() -> None:
    """两路频率不同：配对应按时间最近取，否则 0.1 s 的 skew 门永远过不去。"""
    core = _core()
    core.note_map()
    # plain_slam 10 Hz，fused 50 Hz，两者相位差半格、最新时刻差 0.20 s。
    for index in range(2):
        core.note_plain_slam_odom(100.00 + index * 0.10, Pose2D(1.0, 0.0, 0.0))
    for index in range(6):
        core.note_fused_odom(100.05 + index * 0.05, Pose2D(0.0, 0.0, 0.0))
    computation = core.compute(100.35)
    assert computation.ok
    # 基准取滞后的那一路（100.10），两边各取最近样本 ⇒ 两侧都是 100.10，skew 为 0。
    # 若各取最新，skew 会是 100.10 - 100.30 = -0.20 s，直接越过 0.1 s 的门。
    assert computation.frame_skew_s == pytest.approx(0.0)
    # 新鲜度看的是各路自己的最新样本，不是配对帧。
    assert computation.plain_slam_age_s == pytest.approx(0.25)
    assert computation.fused_age_s == pytest.approx(0.05)


def test_core_goes_invalid_when_the_fused_odom_stops() -> None:
    core = _core()
    core.note_map()
    core.note_plain_slam_odom(200.0, Pose2D(1.0, 0.0, 0.0))
    core.note_fused_odom(200.0, Pose2D(0.0, 0.0, 0.0))
    assert core.compute(200.1).ok
    # plain_slam 继续有数据，只有 fused 断了。
    core.note_plain_slam_odom(201.0, Pose2D(1.0, 0.0, 0.0))
    stale = core.compute(201.0)
    assert not stale.ok
    assert stale.reason == mod.REASON_STALE_FUSED_ODOM
    assert stale.map_to_odom is None


def test_core_revives_after_the_stream_returns() -> None:
    core = _core()
    core.note_map()
    core.note_plain_slam_odom(300.0, Pose2D(1.0, 0.0, 0.0))
    core.note_fused_odom(300.0, Pose2D(0.0, 0.0, 0.0))
    assert not core.compute(400.0).ok
    core.note_plain_slam_odom(400.0, Pose2D(1.0, 0.0, 0.0))
    core.note_fused_odom(400.01, Pose2D(0.0, 0.0, 0.0))
    assert core.compute(400.02).ok
