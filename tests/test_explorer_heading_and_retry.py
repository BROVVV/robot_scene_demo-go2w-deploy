"""计划书 §17.4/§17.7/§17.8：heading sector 来自当前 pose（stale 语义不能
控制导航 coverage）；Quick VLM 瞬态 timeout 可恢复；连续失败 cause 精确。

Explorer 级回归：用 MockBackend + 注入 observer 跑真实主循环。
"""

from __future__ import annotations

import math

from app.live_robot.autonomous_explorer import (
    AutonomousExplorer,
    PerceptionFailure,
    SemanticMatch,
    VerificationOutcome,
)
from app.navigation.backend_factory import create_backend
from app.navigation.exploration_config import load_exploration_policy
from app.navigation.models import LiveObservation

POLICY = load_exploration_policy(
    "configs/exploration/default.yaml",
    overrides={
        "exploration": {
            "budget": {
                "max_planning_cycles": 4,
                "max_search_seconds": 120.0,
                "max_motion_steps": 6,
            },
        },
    },
)


def _base_observation(bundle_id: str, heading_sector: int | None = 0) -> LiveObservation:
    return LiveObservation(
        bundle_id=bundle_id,
        timestamp=1.0,
        scene_objects=[],
        scene_relations=[],
        target_match={"target_present": False},
        heading_sector=heading_sector,
        sensor_health={"camera": True},
    )


def _matcher(observation: LiveObservation) -> SemanticMatch:
    return SemanticMatch(has_candidate=False)


def _verifier(observation: LiveObservation, match: SemanticMatch) -> VerificationOutcome:
    return VerificationOutcome(confirmed=False, attempts=1)


def _run_explorer(observer, backend) -> tuple[AutonomousExplorer, object]:
    explorer = AutonomousExplorer(
        target="白色垃圾桶",
        observer=observer,
        matcher=_matcher,
        verifier=_verifier,
        backend=backend,
        policy=POLICY,
        session_id="test_heading_retry",
        max_perception_retries=2,
    )
    result = explorer.run()
    return explorer, result


def test_navigation_heading_sector_uses_current_pose_not_stale_semantic():
    """§17.4（最重要回归）：语义永远返回 sector=0，导航 coverage 仍记录
    0,1,2,...，绝不能仍为 {"0": N}。"""
    yaws_deg = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0]
    index = [0]

    def pose_provider():
        value = yaws_deg[min(index[0], len(yaws_deg) - 1)]
        index[0] += 1
        return (0.0, 0.0, math.radians(value))

    backend = create_backend("mock", pose_provider=pose_provider)
    calls = [0]

    def observer() -> LiveObservation:
        calls[0] += 1
        # stale semantic：heading_sector 永远是 0。
        return _base_observation(f"bundle_{calls[0]}", heading_sector=0)

    explorer, result = _run_explorer(observer, backend)
    sectors = [
        event.get("navigation_heading_sector")
        for event in explorer.events
        if event.get("event") == "observation"
        and event.get("navigation_heading_sector") is not None
    ]
    assert len(sectors) >= 3, f"expected multiple observations, got {sectors}"
    assert sectors[0] == 0
    assert sectors[1] == 1
    assert sectors[2] == 2
    # observation.heading_sector（导航）也必须跟随当前 pose，而不是 0。
    nav_sectors = [
        event.get("heading_sector")
        for event in explorer.events
        if event.get("event") == "observation"
        and event.get("heading_sector") is not None
    ]
    assert nav_sectors[:3] == [0, 1, 2]
    assert result.result != "PERCEPTION_FAILURE"


def test_perception_retry_after_previous_success():
    """§17.7：cycle 1 成功 -> cycle 2 第一次 timeout -> retry 成功 -> 继续。"""
    backend = create_backend("mock", pose_provider=lambda: (0.0, 0.0, 0.0))
    calls = [0]

    def observer() -> LiveObservation:
        calls[0] += 1
        if calls[0] == 2:
            raise PerceptionFailure(
                "Quick VLM timed out",
                code="QUICK_VLM_TIMEOUT",
                recoverable=True,
                last_success_age_s=0.0,
            )
        return _base_observation(f"bundle_{calls[0]}")

    explorer, result = _run_explorer(observer, backend)
    retries = [
        event for event in explorer.events if event.get("event") == "perception_retry"
    ]
    assert len(retries) == 1
    assert retries[0]["code"] == "QUICK_VLM_TIMEOUT"
    assert retries[0]["observations_so_far"] == 1
    assert result.result != "PERCEPTION_FAILURE"
    assert explorer.state in {"FINISHED", "SEARCH_EXHAUSTED", "MAX_STEPS_REACHED", "TIMEOUT"}


def test_quick_continuous_failure_reports_precise_cause():
    """§17.8：连续超过重试次数 -> PERCEPTION_FAILURE + cause=QUICK_VLM_TIMEOUT
    + attempts 精确，而不是“未分类异常”。"""
    backend = create_backend("mock", pose_provider=lambda: (0.0, 0.0, 0.0))

    def observer() -> LiveObservation:
        raise PerceptionFailure(
            "Quick VLM timed out",
            code="QUICK_VLM_TIMEOUT",
            recoverable=True,
            last_success_age_s=None,
        )

    explorer, result = _run_explorer(observer, backend)
    assert result.result == "PERCEPTION_FAILURE"
    finish = next(
        event for event in explorer.events
        if event.get("event") == "session_finish"
    )
    assert finish.get("cause") == "QUICK_VLM_TIMEOUT"
    assert finish.get("attempts") == 3  # 原始 1 次 + retry 2 次
    assert finish.get("recoverable") is True
    summary = result.to_dict()["summary"]
    assert summary["perception_failure"]["code"] == "QUICK_VLM_TIMEOUT"


def test_bare_runtime_error_after_success_still_retries():
    """归档 search_20260831_192836_aa3ca51c 的回归：一次 VLM 超时判死整场搜索。

    那份 events.jsonl 里两次会话都是 `observer_error`
    (`RuntimeError: SiliconFlow vision API timed out`) 紧跟 `session_finish
    PERCEPTION_FAILURE`，中间没有任何 `observer_retry` —— 当时的重试条件是
    `observations == 0 and ...`，已经成功观察过就不再重试。daemon 客户端抛的是
    裸 RuntimeError 而不是 PerceptionFailure，走的是通用 except 分支，所以
    §17.7 那条只覆盖 PerceptionFailure 的回归拦不住它。
    """
    backend = create_backend("mock", pose_provider=lambda: (0.0, 0.0, 0.0))
    calls = [0]

    def observer() -> LiveObservation:
        calls[0] += 1
        if calls[0] == 2:
            raise RuntimeError("SiliconFlow vision API timed out")
        return _base_observation(f"bundle_{calls[0]}")

    explorer, result = _run_explorer(observer, backend)
    retries = [
        event for event in explorer.events if event.get("event") == "observer_retry"
    ]
    assert len(retries) == 1
    assert retries[0]["observations_so_far"] == 1
    assert result.result != "PERCEPTION_FAILURE"
