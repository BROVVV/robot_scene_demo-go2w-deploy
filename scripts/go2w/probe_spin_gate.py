#!/usr/bin/env python3
"""探针：连跳闸门把搜索循环带向哪里——它在夹具里会不会**无界**。

回答的是 §2.3-3-1 那个结构问题：脱困支路是 ``continue``，**不涨 planning_cycles**，
脱困倒车又不一定涨 ``motion_steps``；于是 ``ExplorationBudget.remaining`` 里三个
循环边界饿死两个，只剩 ``wall-clock``（生产 ``max_search_seconds`` = 600 s）。
症状不是"更早脱困"，而是**整场搜索变成 600 秒的脱困抖动**。

做法：把 ``budget.remaining`` 钉死在 200 轮（墙钟在这个夹具里无意义），跑完看
``_in_place_turn_streak`` / ``no_information_classified`` 的次数与决策分布。
对照组 ``stuck_detection.enabled=False`` 就是闸门没接上时的旧行为。

用法：`env -u PYTHONNOUSERSITE .venv/bin/python3 scripts/go2w/probe_spin_gate.py`
"""
from __future__ import annotations

import collections
import os
import subprocess
import sys

REPO = subprocess.run(
    ["git", "rev-parse", "--show-toplevel"],
    cwd=os.path.dirname(os.path.abspath(__file__)),
    capture_output=True, text=True, check=True,
).stdout.strip()

sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "tests"))

from app.live_robot.autonomous_explorer import AutonomousExplorer  # noqa: E402
from app.navigation.backend_factory import MockBackend  # noqa: E402
from app.navigation.exploration_config import (  # noqa: E402
    ExplorationBudget,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph  # noqa: E402
from test_in_place_turn_streak import (  # noqa: E402
    _constant_scene,
    _perception,
    _snapshot,
)

CYCLE_BUDGET = 20
PRODUCTION_NO_INFO = 20


def build(stuck_enabled: bool) -> AutonomousExplorer:
    scene = _constant_scene()
    policy = load_exploration_policy()
    policy.budget = ExplorationBudget(
        max_motion_steps=CYCLE_BUDGET, max_planning_cycles=CYCLE_BUDGET,
        max_consecutive_no_information_cycles=PRODUCTION_NO_INFO,
    )
    explorer = AutonomousExplorer(
        target="t", observer=scene.observer(), matcher=scene.matcher(),
        verifier=scene.verifier(), backend=MockBackend(),
        graph=ExplorationGraph(session_id="t"), policy=policy,
        local_safety_provider=lambda obs: _snapshot(),
        perception_quality_provider=lambda obs, gain: _perception(),
    )
    explorer.search_policy.stuck_detection.enabled = stuck_enabled
    return explorer


def dump(explorer: AutonomousExplorer, loops: int) -> None:
    counts = collections.Counter(e["event"] for e in explorer.events)
    print(f"循环次数={loops} 打转计数={explorer._in_place_turn_streak} "
          f"无信息={explorer._no_information_cycles} 脱困={explorer._recovery_count} "
          f"无信息重规划={explorer._no_information_replans}")
    for key in ("no_information_classified", "stuck_detected", "recovery_triggered",
                "recovery_result", "recovery_backward_start", "recovery_reobserve",
                "recovery_budget_exceeded", "search_exhausted", "memory_update",
                "goal_executed", "goal_selected"):
        if counts.get(key):
            print(f"  {key}: {counts[key]}")
    classified = [e for e in explorer.events if e["event"] == "no_information_classified"]
    print(f"  决策分布={collections.Counter(e.get('decision') for e in classified)}")
    print(f"  前 12 条（决策,无信息计数,前沿数）="
          f"{[(e.get('decision'), e.get('no_information_cycles'), e.get('open_frontiers')) for e in classified[:12]]}")


def main() -> None:
    for enabled in (True, False):
        explorer = build(enabled)
        calls = {"n": 0}
        real = explorer.policy.budget.remaining

        def capped(*, elapsed_sec, planning_cycles, motion_steps, _real=real):
            calls["n"] += 1
            if calls["n"] > 200:
                return False
            return _real(elapsed_sec=elapsed_sec, planning_cycles=planning_cycles,
                         motion_steps=motion_steps)

        explorer.policy.budget.remaining = capped  # type: ignore[assignment]
        explorer.run()
        print(f"--- stuck_detection.enabled={enabled} ---")
        dump(explorer, calls["n"])


if __name__ == "__main__":
    main()
