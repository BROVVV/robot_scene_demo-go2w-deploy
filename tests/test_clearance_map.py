"""计划书 §7/§33：障碍距离场必须让 "走中间" 成为最省成本的选择。

同时验证 UNKNOWN / STALE 一律 fail-closed —— 未观测的空间永远不是空旷。
"""

from __future__ import annotations

import math

from app.navigation.clearance_map import (
    UNKNOWN_COST,
    clearance_cost,
    clearance_map_from_points,
)


def _corridor(half_width_m: float, *, length_m: float = 4.0, step_m: float = 0.05):
    """两面平行墙构成的走廊，沿 x 轴延伸，中线 y=0。"""
    points = []
    count = int(length_m / step_m) + 1
    for index in range(count):
        x = -length_m / 2.0 + index * step_m
        points.append((x, half_width_m))
        points.append((x, -half_width_m))
    return clearance_map_from_points(
        points, known_bounds=(-length_m / 2.0, -half_width_m, length_m / 2.0, half_width_m))


# 场景 A：2m 宽走廊，中线成本显著低于贴墙，并且会被选中。
def test_corridor_centre_line_is_cheapest() -> None:
    corridor = _corridor(1.0)
    centre = corridor.path_clearance([(-1.0, 0.0), (1.0, 0.0)], 0.20)
    near_wall = corridor.path_clearance([(-1.0, 0.7), (1.0, 0.7)], 0.20)
    assert centre.min_clearance_m is not None and near_wall.min_clearance_m is not None
    assert centre.min_clearance_m > near_wall.min_clearance_m
    centre_cost = clearance_cost(centre.min_clearance_m)
    wall_cost = clearance_cost(near_wall.min_clearance_m)
    assert centre_cost == 0.0
    assert wall_cost == math.inf
    best = min([(centre_cost, "centre"), (wall_cost, "wall")])[1]
    assert best == "centre"


# 中线成本随着偏离连续上升，不是阶跃。
def test_cost_rises_monotonically_towards_the_wall() -> None:
    corridor = _corridor(1.0)
    costs = []
    for offset in (0.0, 0.2, 0.35, 0.45):
        clearance = corridor.pose_clearance((0.0, offset), 0.20)
        costs.append(clearance_cost(clearance, 0.40, 0.80))
    assert costs == sorted(costs)
    assert costs[0] == 0.0
    assert costs[-1] > costs[1]


# 场景 B：窄通道只要 min clearance 仍高于硬边界就允许通过。
def test_narrow_but_legal_corridor_is_passable() -> None:
    corridor = _corridor(0.75)
    path = corridor.path_clearance([(-1.0, 0.0), (1.0, 0.0)], 0.20)
    assert path.min_clearance_m is not None
    assert path.min_clearance_m >= 0.40
    assert clearance_cost(path.min_clearance_m) < math.inf


# 场景 C：过窄通道 min clearance 低于硬边界 -> 必须拒绝。
def test_too_narrow_corridor_is_rejected() -> None:
    corridor = _corridor(0.5)
    path = corridor.path_clearance([(-1.0, 0.0), (1.0, 0.0)], 0.20)
    assert path.min_clearance_m is not None
    assert path.min_clearance_m < 0.40
    assert clearance_cost(path.min_clearance_m) == math.inf


# 未观测区域 fail-closed：整条路径的最小净空变未知，成本高于任何已知空旷。
def test_unknown_space_is_not_free() -> None:
    corridor = _corridor(1.0)
    beyond = corridor.path_clearance([(0.0, 0.0), (5.0, 0.0)], 0.20)
    assert beyond.min_clearance_m is None
    assert beyond.unknown_samples > 0
    assert clearance_cost(beyond.min_clearance_m) == UNKNOWN_COST
    assert corridor.clearance_at(5.0, 0.0) is None


# 过期地图不许再用。
def test_stale_map_returns_unknown() -> None:
    corridor = _corridor(1.0)
    corridor.timestamp = 100.0
    corridor.max_age_sec = 3.0
    assert corridor.clearance_at(0.0, 0.0, now=101.0) == 1.0
    assert corridor.clearance_at(0.0, 0.0, now=110.0) is None
    corridor.stale = True
    assert corridor.clearance_at(0.0, 0.0) is None


# 视线：穿墙不算可见，穿未观测区也不算可见。
def test_line_of_sight_is_fail_closed() -> None:
    corridor = _corridor(1.0)
    assert corridor.line_of_sight((-1.0, 0.0), (1.0, 0.0)) is True
    assert corridor.line_of_sight((0.0, -0.5), (0.0, 1.5)) is False
