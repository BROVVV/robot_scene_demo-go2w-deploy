"""SearchStateStore tests (plan book §97: test_search_state_store)."""

from __future__ import annotations

from app.live_robot.search_event import (
    ACTION_FINISHED,
    ACTION_STARTED,
    CANDIDATES_GENERATED,
    GOAL_SELECTED,
    MAP_UPDATED,
    OBSERVATION_UPDATED,
    SEARCH_STATE_CHANGED,
    SESSION_CREATED,
    SESSION_STARTED,
    TARGET_CONFIRMED,
    TARGET_MATCH_UPDATED,
    SearchEvent,
)
from app.live_robot.search_state_store import (
    STATUS_TARGET_FOUND,
    SearchStateStore,
)
from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter
from app.live_robot.search_event_bus import SearchEventBus


def _event(event_type: str, session_id: str = "search_test", cycle: int | None = None,
           payload: dict | None = None, event_id: int = 1, timestamp: float = 100.0) -> SearchEvent:
    return SearchEvent(
        event_id=event_id,
        session_id=session_id,
        timestamp=timestamp,
        event_type=event_type,
        cycle=cycle,
        payload=payload or {},
    )


def test_store_session_lifecycle() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={
        "target": "饮水机旁边的蓝色垃圾桶", "reasoner": "semantic_navigation", "backend": "mock",
    }))
    store.apply(_event(SESSION_STARTED))
    snapshot = store.snapshot()
    assert snapshot["session_id"] == "search_test"
    assert snapshot["target"] == "饮水机旁边的蓝色垃圾桶"
    assert snapshot["status"] == "RUNNING"
    assert snapshot["timeline"][0]["event_type"] == SESSION_CREATED


def test_store_preserves_live_phase_detail() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    store.apply(_event(SEARCH_STATE_CHANGED, timestamp=123.0, payload={
        "phase": "OBSERVE",
        "phase_detail": "正在获取最新画面并分析目标与场景",
    }))
    snapshot = store.snapshot()
    assert snapshot["phase"] == "OBSERVE"
    assert snapshot["phase_detail"] == "正在获取最新画面并分析目标与场景"
    assert snapshot["phase_started_at"] == 123.0


def test_store_observation_and_objects() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    store.apply(_event(OBSERVATION_UPDATED, cycle=1, payload={
        "bundle_id": "bundle_1",
        "scene_objects": [{"label_zh": "饮水机", "confidence": 0.9}],
        "detections": [{"label": "water dispenser", "bbox_2d": [0.1, 0.2, 0.3, 0.4]}],
        "target_present": False,
        "heading_sector": 3,
        "pose": {"x": 1.0, "y": 0.5, "yaw_rad": 0.5},
    }))
    snapshot = store.snapshot()
    assert snapshot["observation"]["bundle_id"] == "bundle_1"
    assert snapshot["observation"]["objects"][0]["label_zh"] == "饮水机"
    assert snapshot["observation"]["target_present"] is False
    assert snapshot["cycle"] == 1
    assert snapshot["candidates"] == []
    assert len(snapshot["timeline"]) == 2


def test_store_target_match_and_candidates_and_goal() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    store.apply(_event(TARGET_MATCH_UPDATED, payload={
        "target_match_level": "partial_match",
        "target_score": 0.81,
        "anchor_labels": ["饮水机"],
        "directive": {"kind": "inspect_anchor"},
    }))
    store.apply(_event(CANDIDATES_GENERATED, payload={
        "candidates": [{
            "goal": {"goal_id": "goal_1", "goal_type": "INSPECT_ANCHOR"},
            "score": 0.82,
            "components": {"semantic_relevance": 0.91},
        }],
        "selected_goal_id": "goal_1",
    }))
    store.apply(_event(GOAL_SELECTED, payload={
        "goal": {"goal_id": "goal_1", "goal_type": "INSPECT_ANCHOR"},
        "score": 0.82,
        "components": {"semantic_relevance": 0.91},
        "reasons": ["锚点"],
    }))
    store.apply(_event(ACTION_STARTED))
    store.apply(_event(ACTION_FINISHED, payload={
        "status": "succeeded",
        "message": "turn l20",
        "elapsed_sec": 2.1,
    }))
    snapshot = store.snapshot()
    assert snapshot["target_match"]["level"] == "partial_match"
    assert snapshot["target_match"]["anchor_labels"] == ["饮水机"]
    assert snapshot["selected_goal"]["goal"]["goal_id"] == "goal_1"
    assert snapshot["candidates"][0]["selected"] is True
    assert snapshot["robot"]["motion_status"] == "SUCCEEDED"
    assert snapshot["robot"]["last_motion_result"]["message"] == "turn l20"
    evidence = snapshot["objects"]["target_evidence"]
    assert evidence["anchor_labels"] == ["饮水机"]


def test_action_started_clears_the_previous_goal_result() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    store.apply(_event(ACTION_FINISHED, payload={
        "status": "rejected",
        "message": "old goal was too narrow",
        "candidate_decision": {"accepted": False, "reject_reason": "TOO_NARROW"},
    }))
    assert store.snapshot()["robot"]["last_motion_result"] is not None

    store.apply(_event(ACTION_STARTED, payload={"goal_id": "new_goal"}))

    snapshot = store.snapshot()
    assert snapshot["robot"]["motion_status"] == "EXECUTING"
    assert snapshot["robot"]["last_motion_result"] is None


def test_store_graph_revision_monotonic() -> None:
    """图只有 `set_graph()` 一个写入口，revision 只增不减（§2.2）。

    `MAP_UPDATED` 本身不再携带图 —— 这条同时钉住"事件流写不动图"，免得日后有人
    把旧路径接回来。
    """
    store = SearchStateStore()
    store.apply(_event(MAP_UPDATED, payload={"revision": 1}))
    assert store.snapshot()["graph"] is None, "MAP_UPDATED 不许写图"
    for index in range(5):
        assert store.set_graph({
            "schema_version": "semantic_navigation_graph_v1",
            "revision": index + 1,
            "places": [], "edges": [], "objects": [], "frontiers": [],
            "current_place_id": None, "object_topology": None,
        }) is True
        assert store.snapshot()["graph_revision"] == index + 1
    final = store.snapshot()["graph"]
    assert final["schema_version"] == "semantic_navigation_graph_v1"
    assert final["revision"] == 5


def test_store_target_confirmed_status() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    store.apply(_event(TARGET_CONFIRMED, payload={"reason_zh": "verify pass"}))
    snapshot = store.snapshot()
    assert snapshot["status"] == STATUS_TARGET_FOUND
    assert snapshot["target_match"]["target_confirmed"] is True
    assert snapshot["target_match"]["level"] == "confirmed"
    assert snapshot["phase"] == "TARGET_FOUND"


def test_store_exhausted_and_error() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    store.apply(_event("SEARCH_EXHAUSTED", payload={"reason": "no candidates"}))
    assert store.snapshot()["status"] == "SEARCH_EXHAUSTED"
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    store.apply(_event("ERROR", payload={"error_type": "PERCEPTION_ERROR", "message": "no frame"}))
    snapshot = store.snapshot()
    assert snapshot["status"] == "FAILED"
    assert snapshot["error"]["error_type"] == "PERCEPTION_ERROR"


def test_budget_limit_is_normal_completion_not_error() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    # Compatibility with an older event stream that emitted ERROR immediately
    # before the terminal budget result.
    store.apply(_event("ERROR", payload={
        "error_type": "SEARCH_ERROR", "message": "MAX_STEPS_REACHED",
    }))
    store.apply(_event("SEARCH_FINISHED", payload={
        "result": "MAX_STEPS_REACHED",
        "finish_reason": "MAX_STEPS_REACHED",
    }))
    snapshot = store.snapshot()
    assert snapshot["status"] == "FINISHED"
    assert snapshot["result"] == "MAX_STEPS_REACHED"
    assert snapshot["error"] is None


def test_observer_retry_is_recoverable_warning_not_failed() -> None:
    store = SearchStateStore()
    bus = SearchEventBus()
    adapter = ExplorerSearchAdapter(bus, store, session_id="search_test")
    adapter.on_explorer_event({"event": "session_start", "state": "BOOTSTRAP"})
    adapter.on_explorer_event({
        "event": "observer_retry",
        "state": "OBSERVE",
        "attempt": 1,
        "error": "vision request timed out",
    })
    snapshot = store.snapshot()
    assert snapshot["status"] == "RUNNING"
    assert snapshot["error"] is None
    assert snapshot["last_warning"]["recoverable"] is True
    assert "vision request timed out" in snapshot["phase_detail"]


def test_bus_subscriber_sees_committed_store_state() -> None:
    archived: list[str] = []
    store = SearchStateStore(
        on_change=lambda snapshot, _event: archived.append(snapshot["status"])
    )
    bus = SearchEventBus()
    seen: list[tuple[str, str]] = []
    bus.subscribe(lambda event: seen.append(
        (event.event_type, store.snapshot()["status"])
    ))
    adapter = ExplorerSearchAdapter(bus, store, session_id="search_test")
    adapter.on_explorer_event({"event": "session_start", "state": "BOOTSTRAP"})
    assert seen
    assert archived
    assert seen[-1][1] == archived[-1]


def test_store_surfaces_terminal_quick_vlm_timeout() -> None:
    """Terminal perception failures retain the concrete VLM cause for WebUI."""
    store = SearchStateStore()
    bus = SearchEventBus()
    adapter = ExplorerSearchAdapter(bus, store, session_id="search_test")
    adapter.on_explorer_event({"event": "session_start", "state": "BOOTSTRAP"})
    adapter.on_explorer_event({
        "event": "session_finish",
        "state": "FINISHED",
        "result": "PERCEPTION_FAILURE",
        "cause": "QUICK_VLM_TIMEOUT",
        "attempts": 3,
        "error": "SiliconFlow vision API timed out",
        "error_detail": {
            "code": "QUICK_VLM_TIMEOUT",
            "message": "SiliconFlow vision API timed out",
            "detail": "daemon request exceeded 65 seconds",
        },
    })
    error = store.snapshot()["error"]
    assert error["code"] == "QUICK_VLM_TIMEOUT"
    assert error["message"] == "SiliconFlow vision API timed out"
    assert error["category"] == "reasoning"


def test_store_snapshot_is_a_copy() -> None:
    store = SearchStateStore()
    store.apply(_event(SESSION_CREATED, payload={"target": "x"}))
    first = store.snapshot()
    first["target"] = "mutated"
    assert store.snapshot()["target"] == "x"


def test_candidate_decision_reaches_the_snapshot() -> None:
    """计划书 §14：走廊判定/净空/速度上限必须能在快照里查到。

    这条记录在 nav2 后端里是 NavigationResult.provenance 的一个字段，一路要
    经过 explorer 事件 → 适配器 → 状态存储三跳。任何一跳把它丢掉，现场验收
    的第 7 项就没有可查的东西了。
    """
    store = SearchStateStore()
    bus = SearchEventBus()
    adapter = ExplorerSearchAdapter(bus, store, session_id="contract")

    decision = {
        "candidate_id": "goal_7",
        "goal_x": 1.5,
        "goal_y": -0.25,
        "goal_yaw": 12.0,
        "path_length_m": 2.4,
        "min_left_body_gap_m": 0.62,
        "min_right_body_gap_m": 0.48,
        "min_side_gap_m": 0.48,
        "corridor_decision": "PASS",
        "speed_limit_mps": 0.35,
        "center_score": 0.71,
        "accepted": True,
        "reject_reason": "",
    }
    adapter.on_explorer_event({
        "event": "navigation_result",
        "goal_id": "goal_7",
        "status": "succeeded",
        "message": "navigate to pose succeeded",
        "candidate_decision": decision,
    })

    recorded = store.snapshot()["robot"]["last_motion_result"]["candidate_decision"]
    assert recorded["corridor_decision"] == "PASS"
    assert recorded["min_left_body_gap_m"] == 0.62
    assert recorded["speed_limit_mps"] == 0.35
