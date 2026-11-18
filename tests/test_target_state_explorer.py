from __future__ import annotations

from app.live_robot.autonomous_explorer import AutonomousExplorer, SemanticMatch, VerificationOutcome
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import ExplorationBudget, load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import LiveObservation


def test_possible_target_is_reobserved_before_any_motion() -> None:
    policy = load_exploration_policy()
    policy.budget = ExplorationBudget(max_planning_cycles=1, max_motion_steps=1)
    observations = [
        LiveObservation(
            bundle_id=f"possible_{index}", timestamp=float(index),
            target_match={"target_state": "POSSIBLE", "target_present": False},
        )
        for index in range(3)
    ]
    cursor = [0]

    def observe() -> LiveObservation:
        index = min(cursor[0], len(observations) - 1)
        cursor[0] += 1
        return observations[index]

    backend = MockBackend()
    explorer = AutonomousExplorer(
        target="目标",
        observer=observe,
        matcher=lambda _observation: SemanticMatch(has_candidate=False),
        verifier=lambda _observation, _match: VerificationOutcome(
            confirmed=False, attempts=1, reason_zh="not confirmed"
        ),
        backend=backend,
        policy=policy,
        graph=ExplorationGraph(session_id="possible_test"),
    )
    result = explorer.run()
    events = [item["event"] for item in explorer.events]
    possible_events = [index for index, event in enumerate(events) if event == "target_possible_reobserve"]
    assert len(possible_events) == 2
    # 必须真的出现 action_start，否则下面那条 all(index < first_action) 在
    # ``first_action = len(events)`` 的兜底下**恒真** —— 一条恒真断言会让本用例
    # 在"explorer 压根没走到 planner"时也报绿，等于没测排序。
    assert "action_start" in events, (
        "本用例要求 POSSIBLE 耗尽（转 ABSENT）后 planner 恢复动作："
        "没有 action_start 就没有可比的边界，排序断言会退化成恒真"
    )
    first_action = events.index("action_start")
    assert all(index < first_action for index in possible_events)
    # 三次观察全部 POSSIBLE、verifier 恒否决 -> 每一次复核都必须排在那唯一一次
    # 真实动作之前。motion_steps 由 EXECUTE 成功自增，所以它这里必然 <= 1。
    assert result.motion_steps <= 1
