"""Bounded LOCAL_SCAN rotation selection (计划书 §7 / §57-§58).

规则（计划书 §7.2-§7.4）：
* ``max_local_rotations`` 是“同一个 Place 最多执行多少次 LOCAL_SCAN 旋转动作”
  的配额，与 heading coverage 覆盖数彻底解耦；
* 候选左右对称：+30 / -30 / +60 / -60，不再固定左优先；
* 同方向重复惩罚 + 最近 sector 惩罚 + 死循环保护（同方向连续 >=2 且无新
  信息时禁止再选同方向）；
* 纯函数、无 I/O，便于离线回归测试。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from app.navigation.models import GOAL_ROTATE_VIEW


@dataclass
class LocalScanState:
    """同一个 Place 的 LOCAL_SCAN 配额与方向记忆。"""

    steps: int = 0
    last_direction: int | None = None  # +1 左转 / -1 右转
    same_direction_count: int = 0
    last_sector: int | None = None
    last_scan_cycle_info_gain: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any] | None) -> "LocalScanState":
        value = value or {}
        return cls(
            steps=int(value.get("steps", 0)),
            last_direction=value.get("last_direction"),
            same_direction_count=int(value.get("same_direction_count", 0)),
            last_sector=value.get("last_sector"),
            last_scan_cycle_info_gain=bool(value.get("last_scan_cycle_info_gain", False)),
        )


def select_local_scan_goal(
    *,
    current_yaw_deg: float,
    heading_coverage: dict[str, int] | None = None,
    max_local_rotations: int,
    state: LocalScanState | None = None,
    sector_deg: float = 30.0,
    sectors: int = 12,
    new_information: bool = False,
) -> tuple[dict[str, Any] | None, LocalScanState]:
    """返回 (goal_fields, updated_state)。

    ``goal_fields`` 为 ``None`` 表示配额已耗尽或无合法候选，调用方必须落到
    其他规划分支（计划书 §7.4：强制退出 LOCAL_SCAN）。
    """
    state = state or LocalScanState()
    sectors = max(1, int(sectors))
    sector_deg = max(1.0, float(sector_deg))
    max_local_rotations = max(0, int(max_local_rotations))
    if max_local_rotations <= 0 or state.steps >= max_local_rotations:
        return None, state

    covered = {int(key) for key in (heading_coverage or {})}
    current_sector = int(round(float(current_yaw_deg) / sector_deg)) % sectors
    # 左右对称候选（计划书 §7.3）：+30 / -30 / +60 / -60。
    deltas = (1, -1, 2, -2)
    scored: list[tuple[float, int, int]] = []
    for delta in deltas:
        sector = (current_sector + delta) % sectors
        if sector in covered:
            continue
        score = 1.0
        if state.last_direction is not None:
            same_direction = (delta > 0) == (state.last_direction > 0)
            if same_direction:
                score -= 0.6  # 同向重复惩罚
                if state.same_direction_count >= 2 and not new_information:
                    # 死循环保护：同方向连续 >=2 且没有新信息 -> 禁止同方向。
                    score -= 2.0
        if state.last_sector == sector:
            score -= 0.3  # recent sector 惩罚
        scored.append((score, delta, sector))
    if not scored:
        return None, state

    # 同分时优先选择与上次相反的方向（避免无限单向旋转）。
    scored.sort(key=lambda item: (-item[0], abs(item[1])))
    best = scored[0]
    if len(scored) > 1 and state.last_direction is not None:
        best_opposite = next(
            (
                item for item in scored
                if (item[1] > 0) != (state.last_direction > 0)
                and abs(item[0] - best[0]) < 1e-9
            ),
            None,
        )
        if best_opposite is not None:
            best = best_opposite
    _, delta, sector = best

    same_direction = (
        state.last_direction is not None
        and (delta > 0) == (state.last_direction > 0)
    )
    next_state = LocalScanState(
        steps=state.steps + 1,
        last_direction=1 if delta > 0 else -1,
        same_direction_count=(
            state.same_direction_count + 1 if same_direction else 1
        ),
        last_sector=sector,
        last_scan_cycle_info_gain=bool(new_information),
    )
    goal_fields: dict[str, Any] = {
        "goal_id": f"local_scan_{next_state.steps:03d}",
        "goal_type": GOAL_ROTATE_VIEW,
        "relative_dyaw": float(delta * sector_deg),
        "heading_sector": sector,
        "semantic_reason": (
            f"bounded local scan sector {sector} "
            f"(step {next_state.steps}/{max_local_rotations})"
        ),
        "expected_information_gain": 0.2,
        "provenance": {
            "source": "local_scan",
            "sector": sector,
            "delta_sector": delta,
            "delta_deg": float(delta * sector_deg),
            "same_direction_count": next_state.same_direction_count,
        },
    }
    return goal_fields, next_state
