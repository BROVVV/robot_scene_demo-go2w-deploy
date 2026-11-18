"""写端画幅（``focus_bounds``）的护栏：画幅不得超出当前地图。

回归背景（2026-09-17，用户第二次报同一句话）：WebUI 三维视图里"建的所有图只缩在
整个地图边边的一点，绝大部分都是空的"。真因是持久化轨迹被 odom 发散污染 —— 95 个点
里 24 个甩到 |xy|>50（最大 978.3 m），而 ``_focus_bounds`` 取的是**全部**点的 bbox：
画幅 span 被撑到 994.1 m，而点云只有 26 m（房间仅占画幅 2.6%），且画幅中心 (484.3,
81.9) 离数据中心 (-2.1, 2.0) 几百米 —— 地图被挤进画面角落的一个像素。

为什么两端的护栏都没拦住：写端的 ``MAX_TRAJECTORY_ABS_M`` 和前端 ``saneFrame`` 的
``FRAME_ABS_LIMIT_M`` 都是**绝对常量 1000 m**，而 09-15 的 313.7 / 996.8 m 与 09-17
的 978.3 m 三次发散全都刚好落在它下面。``app/spatial/lio_pose_quality.py`` 的模块
docstring 早就记下了这件事。本文件锁住改后的**相对判据**：画幅恒 ⊆ 地图 ± 边距。

被测函数住在 rclpy 节点类里（该模块把节点逻辑与这个纯函数放在一起），本文件用 stub
把 rclpy 顶掉后再 import —— 否则这个纯函数永远进不了 pytest。

**注入校准**：删掉 ``_focus_bounds`` 里那两行 ``np.maximum`` / ``np.minimum`` 后，
``test_diverged_trajectory_cannot_inflate_the_frame`` 与
``test_frame_never_exceeds_map_plus_margin`` 立刻失败（实测 span=994.10 m）；还原后全绿。
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import pytest

WRITER = (Path(__file__).resolve().parents[1]
          / "scripts" / "go2w" / "webui_map_snapshot_writer.py")

# 2026-09-17 真机快照 slam_map_3d.json 的 bounds 与 focus_margin_m 参数值。
MAP_LOW = np.array([-7.74, -11.02])
MAP_HIGH = np.array([3.57, 14.95])
MARGIN = 3.0
MAP_SPAN = max(MAP_HIGH[0] - MAP_LOW[0], MAP_HIGH[1] - MAP_LOW[1])

# 真机轨迹里落在地图内的那一小段（狗实际走过的地方）。
INSIDE = [(-0.28, 0.02), (0.49, 0.17), (-0.21, -0.12), (-0.55, -0.01), (0.24, 0.10),
          (-0.44, -0.68), (-0.38, -0.95), (-0.17, -0.81), (-0.55, -0.28), (0.03, -0.06),
          (-0.51, -0.41), (0.31, 0.09), (-0.33, -0.55), (-0.47, -0.83), (0.12, 0.05),
          (-0.25, -0.18), (0.41, 0.13), (-0.59, -0.36), (-0.19, -0.72), (0.07, -0.02)]
# 发散尾段：首点逐字取自 2026-09-17 实测轨迹，其余按实测的 ≈1.034 等比衰减补足。
DIVERGED = [(978.307, 164.836), (946.1, 159.4), (915.0, 154.2), (884.9, 149.1)]


class _Any:
    """rclpy 的最小替身：只为让模块 import 得进来，不模拟任何 ROS 行为。"""

    def __init__(self, *args, **kwargs):
        pass

    def __getattr__(self, name):
        return _Any()


@pytest.fixture(scope="module")
def focus_bounds():
    for name, attrs in {
        "rclpy": {},
        "rclpy.node": {"Node": _Any},
        "rclpy.qos": {"DurabilityPolicy": _Any, "HistoryPolicy": _Any,
                      "QoSProfile": _Any, "ReliabilityPolicy": _Any},
        "nav_msgs": {},
        "nav_msgs.msg": {"Odometry": _Any},
        "sensor_msgs": {},
        "sensor_msgs.msg": {"PointCloud2": _Any},
    }.items():
        module = types.ModuleType(name)
        for key, value in attrs.items():
            setattr(module, key, value)
        sys.modules[name] = module

    spec = importlib.util.spec_from_file_location("webui_map_snapshot_writer", WRITER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    writer = next(value for value in vars(module).values()
                  if isinstance(value, type) and hasattr(value, "_focus_bounds"))

    def call(trajectory, bounds=(MAP_LOW, MAP_HIGH)):
        node = object.__new__(writer)          # 绕过 __init__（那要一个活的 rclpy 节点）
        node._trajectory = list(trajectory)
        node._latest_bounds = bounds
        node._focus_margin = MARGIN
        node._min_z, node._max_z = 0.15, 2.80
        return node._focus_bounds()

    return call


def _span(frame) -> float:
    return max(frame["max"][0] - frame["min"][0], frame["max"][1] - frame["min"][1])


def test_diverged_trajectory_cannot_inflate_the_frame(focus_bounds):
    """本文件存在的理由：978.3 m 的发散点不得再把画幅撑到千米量级。"""
    frame = focus_bounds(INSIDE + DIVERGED)
    # 不夹的话这里是 994.10 m，前端 scale 相应缩小约 40 倍 —— 就是用户看到的那一版。
    assert _span(frame) < 3.0 * MAP_SPAN, _span(frame)
    assert max(abs(v) for v in frame["min"][:2]) < 100.0, frame
    assert max(abs(v) for v in frame["max"][:2]) < 100.0, frame


def test_frame_never_exceeds_map_plus_margin(focus_bounds):
    """相对判据的不变式：画幅恒 ⊆ 地图 ± 边距。"""
    for trajectory in (INSIDE, INSIDE + DIVERGED,
                       [(500.0, 600.0)] * 20):        # 含"与地图不同系"的极端
        frame = focus_bounds(trajectory)
        assert frame["min"][0] >= MAP_LOW[0] - MARGIN - 1e-9, frame
        assert frame["min"][1] >= MAP_LOW[1] - MARGIN - 1e-9, frame
        assert frame["max"][0] <= MAP_HIGH[0] + MARGIN + 1e-9, frame
        assert frame["max"][1] <= MAP_HIGH[1] + MARGIN + 1e-9, frame


def test_healthy_trajectory_still_tightens(focus_bounds):
    """地图内的轨迹仍要能收紧画幅 —— 夹取对它是空操作，别把功能改没了。"""
    frame = focus_bounds(INSIDE)
    assert _span(frame) == pytest.approx(MAP_SPAN, abs=1e-9) or _span(frame) < MAP_SPAN
    # 更硬的一条：画幅两端就是轨迹两端 ± 边距。
    low_x = min(x for x, _ in INSIDE)
    high_y = max(y for _, y in INSIDE)
    assert frame["min"][0] == pytest.approx(low_x - MARGIN, abs=1e-9)
    assert frame["max"][1] == pytest.approx(high_y + MARGIN, abs=1e-9)


def test_foreign_trajectory_falls_back_to_the_map(focus_bounds):
    """轨迹与地图不同系（SLAM 重启换了原点）：退回地图范围，不倒置、不塌成一点。"""
    frame = focus_bounds([(500.0, 600.0), (510.0, 610.0), (520.0, 620.0)] * 7)
    assert frame["min"][0] < frame["max"][0] and frame["min"][1] < frame["max"][1], frame
    assert _span(frame) == pytest.approx(MAP_SPAN + 2 * MARGIN, abs=1e-9), _span(frame)


def test_too_short_trajectory_returns_none(focus_bounds):
    """点太少时给 None，让前端退回 bounds（既有行为，别改）。"""
    assert focus_bounds([(0.0, 0.0)] * 19) is None
