"""计划书 §17.5/§17.6：local scan quota 与左右偏置回归测试。

不变量：``max_local_rotations`` 真正限制旋转次数；无额外信息时不能稳定输出
l30,l30,l30,...；候选必须左右对称。
"""

from __future__ import annotations

from app.navigation.local_scan import LocalScanState, select_local_scan_goal


def test_quota_exhausts_after_max_rotations():
    """§17.5：max_local_rotations=3 -> 同一个 Place 最多 3 次 local scan。"""
    state = LocalScanState()
    picked = 0
    for _ in range(6):
        goal_fields, state = select_local_scan_goal(
            current_yaw_deg=0.0,
            heading_coverage={},
            max_local_rotations=3,
            state=state,
        )
        if goal_fields is None:
            break
        picked += 1
    assert picked == 3
    assert state.steps == 3
    # 配额耗尽后即使 heading coverage 还是空的也必须退出 local scan。
    goal_fields, state = select_local_scan_goal(
        current_yaw_deg=30.0,
        heading_coverage={},
        max_local_rotations=3,
        state=state,
    )
    assert goal_fields is None


def test_quota_is_independent_of_heading_coverage():
    """§7.2：coverage 覆盖数不再充当旋转计数器。"""
    state = LocalScanState()
    # 即使 coverage 已覆盖很多 sector，只要 steps < quota 仍可继续。
    goal_fields, state = select_local_scan_goal(
        current_yaw_deg=0.0,
        heading_coverage={"0": 5, "1": 5, "2": 5, "3": 5},
        max_local_rotations=3,
        state=state,
    )
    assert goal_fields is not None
    # 但全部 sector 都覆盖时没有合法候选 -> None（落到其他规划分支）。
    full_coverage = {str(i): 1 for i in range(12)}
    goal_fields, state = select_local_scan_goal(
        current_yaw_deg=0.0,
        heading_coverage=full_coverage,
        max_local_rotations=3,
        state=state,
    )
    assert goal_fields is None


def test_not_fixed_left_first():
    """§17.6/§7.3：候选左右对称；连续规划多轮不能稳定输出 l30,l30,l30,...。"""
    # 同一 Place 连续 6 次选点（每轮用上一次的状态）——必须左右均有机会。
    state = LocalScanState()
    picked: list[float] = []
    for _ in range(6):
        goal_fields, state = select_local_scan_goal(
            current_yaw_deg=0.0,
            heading_coverage={},
            max_local_rotations=6,
            state=state,
        )
        assert goal_fields is not None
        picked.append(float(goal_fields["relative_dyaw"]))
    assert any(value > 0 for value in picked)
    assert any(value < 0 for value in picked)
    # 不得出现连续 3 次同向（l30,l30,l30... 被禁止）。
    for i in range(len(picked) - 2):
        assert not (picked[i] > 0 and picked[i + 1] > 0 and picked[i + 2] > 0), picked
        assert not (picked[i] < 0 and picked[i + 1] < 0 and picked[i + 2] < 0), picked


def test_same_direction_repeat_penalty():
    """§7.3/§7.4：上一轮左转后，同分时优先右侧。"""
    state = LocalScanState(steps=1, last_direction=1, same_direction_count=1)
    goal_fields, next_state = select_local_scan_goal(
        current_yaw_deg=30.0,
        heading_coverage={},
        max_local_rotations=3,
        state=state,
    )
    assert goal_fields is not None
    # 左转被惩罚 -> 应选右转（负 dyaw）。
    assert float(goal_fields["relative_dyaw"]) < 0
    assert next_state.last_direction == -1


def test_dead_loop_guard_blocks_same_direction():
    """§7.4：同方向连续 >=2 且无新信息 -> 禁止再选同方向。"""
    state = LocalScanState(
        steps=2, last_direction=1, same_direction_count=2,
    )
    goal_fields, _ = select_local_scan_goal(
        current_yaw_deg=60.0,
        heading_coverage={},
        max_local_rotations=5,
        state=state,
        new_information=False,
    )
    assert goal_fields is not None
    assert float(goal_fields["relative_dyaw"]) < 0

    # 有新信息时允许同方向（避免“新信息但被迫反向”的次优行为）。
    state = LocalScanState(
        steps=2, last_direction=1, same_direction_count=2,
    )
    goal_fields, _ = select_local_scan_goal(
        current_yaw_deg=60.0,
        heading_coverage={},
        max_local_rotations=5,
        state=state,
        new_information=True,
    )
    # 同方向惩罚仍在，但不再是硬禁止；可能左或右，只断言不会返回 None。
    assert goal_fields is not None


def test_max_rotations_zero_disables_local_scan():
    goal_fields, state = select_local_scan_goal(
        current_yaw_deg=0.0,
        heading_coverage={},
        max_local_rotations=0,
    )
    assert goal_fields is None
    assert state.steps == 0
