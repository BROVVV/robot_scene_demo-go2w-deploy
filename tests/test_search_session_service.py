"""SearchSessionService tests (plan book §97, §102: session lifecycle and
mock search scenarios: target first frame, anchor case, exhausted,
navigation failure -> replan, pause/resume, operator stop)."""

from __future__ import annotations

import time

import pytest

from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.search_session_service import (
    SearchSessionService,
    make_mock_executor_factory,
)


_TEST_SESSION_DIR = "outputs/live_runs_test"


@pytest.fixture(autouse=True)
def _isolate_archives(tmp_path):
    global _TEST_SESSION_DIR
    _TEST_SESSION_DIR = str(tmp_path / "sessions")


def _service(**factory_kwargs) -> SearchSessionService:
    owner = ControlOwner()
    return SearchSessionService(
        owner=owner,
        executor_factory=make_mock_executor_factory(**factory_kwargs),
        session_dir=_TEST_SESSION_DIR,
    )


def _wait_status(service: SearchSessionService, statuses: set[str],
                 timeout: float = 60.0) -> str:
    deadline = time.time() + timeout
    last = ""
    while time.time() < deadline:
        session = service.current_session()
        last = session.status if session else "IDLE"
        if last in statuses:
            return last
        time.sleep(0.05)
    raise AssertionError(f"timed out waiting for {statuses}, last={last}")


def _start(service: SearchSessionService, target: str = "饮水机旁边的蓝色垃圾桶",
           **overrides) -> dict:
    params = {"target": target, "backend": "mock", "reasoner": "semantic_navigation", **overrides}
    return service.start_search(SearchStartRequest.from_dict(params))


def test_start_returns_immediately_with_session() -> None:
    service = _service()
    result = _start(service, max_planning_cycles=5)
    assert result["ok"] is True
    assert result["session_id"].startswith("search_")
    assert result["status"] == "STARTING"
    session = service.current_session()
    assert session is not None
    assert session.status == "STARTING"
    service.shutdown()


def test_duplicate_start_blocked_while_running() -> None:
    service = _service(scenario="no_target", backend_latency_sec=0.3,
                       mock_target_after=999)
    result = _start(service)
    assert result["ok"] is True
    _wait_status(service, {"RUNNING"})
    second = _start(service, target="另一个目标")
    assert second["ok"] is False
    assert second.get("conflict") is True
    service.stop_search()
    service.shutdown()


def test_target_first_frame_no_motion() -> None:
    service = _service(
        scene_steps=[
            {
                "objects": ["blue trash bin"],
                "target_present": True,
                "target_score": 0.95,
                "bundle_id": "obs_first",
            }
        ],
        confirm_after_seen=1,
    )
    result = _start(service)
    assert result["ok"] is True
    _wait_status(service, {"TARGET_FOUND"})
    state = service.state_snapshot()
    assert state["result"] == "TARGET_FOUND"
    assert state["finish_reason"] == "TARGET_FOUND"
    assert state["summary"]["planning_cycles"] == 0
    # 0 motion: no ACTION_STARTED / navigation_result events
    types = [item["event_type"] for item in service.recent_events(500)]
    assert "ACTION_STARTED" not in types
    service.shutdown()


def test_anchor_case_goal_priority_visible() -> None:
    service = _service(scenario="anchor_then_target", mock_target_after=3,
                       confirm_after_seen=1)
    result = _start(service)
    assert result["ok"] is True
    _wait_status(service, {"TARGET_FOUND"})
    events = service.recent_events(1000)
    types = [item["event_type"] for item in events]
    assert "TARGET_MATCH_UPDATED" in types
    assert "CANDIDATES_GENERATED" in types
    assert "GOAL_SELECTED" in types
    assert "TARGET_CONFIRMED" in types
    # candidates include score components
    candidate_events = [item for item in events if item["event_type"] == "CANDIDATES_GENERATED"]
    assert candidate_events
    first = candidate_events[0]["payload"]["candidates"]
    if first:
        components = first[0].get("components") or {}
        assert "semantic_relevance" in components
    service.shutdown()


def test_search_exhausted() -> None:
    service = _service(scenario="no_target")
    result = _start(service, max_planning_cycles=50, max_motion_steps=50)
    assert result["ok"] is True
    _wait_status(service, {"SEARCH_EXHAUSTED", "FINISHED", "FAILED",
                            "MAX_STEPS_REACHED", "MAX_PLANNING_CYCLES_REACHED"})
    state = service.state_snapshot()
    # 新语义：未找到目标时机器人会持续探索到预算为止，而不是在约 8 轮
    # “连续无新信息”就被 SEARCH_EXHAUSTED 提前终止。
    assert state["result"] in {"SEARCH_EXHAUSTED", "MAX_STEPS_REACHED",
                                "MAX_PLANNING_CYCLES_REACHED", "FINISHED"}
    assert (state.get("summary") or {}).get("planning_cycles", 0) >= 8
    service.shutdown()


def test_navigation_failure_replans() -> None:
    service = _service(scenario="anchor_then_target", mock_target_after=2,
                       confirm_after_seen=1,
                       outcome_sequence=["failed", "succeeded"])
    result = _start(service)
    assert result["ok"] is True
    _wait_status(service, {"TARGET_FOUND"})
    types = [item["event_type"] for item in service.recent_events(1000)]
    assert "REPLAN" in types
    assert types.count("ACTION_FINISHED") >= 2
    state = service.state_snapshot()
    assert state["summary"]["replans"] >= 1
    service.shutdown()


def test_pause_resume_flow() -> None:
    service = _service(scenario="no_target", backend_latency_sec=0.3,
                       mock_target_after=999)
    result = _start(service, max_planning_cycles=30, max_motion_steps=30)
    assert result["ok"] is True
    _wait_status(service, {"RUNNING"})
    pause = service.pause_search()
    assert pause["ok"] is True
    _wait_status(service, {"PAUSED"})
    # pause must not end the session
    assert service.current_session().status == "PAUSED"
    resume = service.resume_search()
    assert resume["ok"] is True
    _wait_status(service, {"RUNNING"})
    service.stop_search()
    _wait_status(service, {"OPERATOR_STOP", "FINISHED", "FAILED", "SEARCH_EXHAUSTED"})
    types = [item["event_type"] for item in service.recent_events(1000)]
    assert "PAUSED" in types
    assert "RESUMED" in types
    service.shutdown()


def test_pause_requires_running() -> None:
    service = _service()
    result = service.pause_search()
    assert result["ok"] is False
    service.shutdown()


def test_operator_stop() -> None:
    service = _service(scenario="no_target", backend_latency_sec=0.3,
                       mock_target_after=999)
    result = _start(service)
    assert result["ok"] is True
    _wait_status(service, {"RUNNING"})
    stop = service.stop_search()
    assert stop["ok"] is True
    _wait_status(service, {"OPERATOR_STOP", "FINISHED", "FAILED"})
    state = service.state_snapshot()
    assert state["result"] in {"OPERATOR_STOP", "SEARCH_EXHAUSTED"}
    # ownership released after stop
    assert service.owner.state().value == "NONE"
    service.shutdown()


def test_estop_search() -> None:
    service = _service(scenario="no_target", backend_latency_sec=0.3,
                       mock_target_after=999)
    result = _start(service)
    assert result["ok"] is True
    _wait_status(service, {"RUNNING"})
    result = service.estop_search()
    assert result["ok"] is True
    assert service.owner.is_estop()
    _wait_status(service, {"OPERATOR_STOP", "FINISHED", "FAILED", "TARGET_FOUND"})
    service.shutdown()


def test_history_records_sessions(tmp_path) -> None:
    owner = ControlOwner()
    service = SearchSessionService(
        owner=owner,
        executor_factory=make_mock_executor_factory(
            scenario="target_appears_after_n", mock_target_after=1,
            confirm_after_seen=1,
        ),
        session_dir=str(tmp_path),
    )
    result = _start(service)
    assert result["ok"] is True
    _wait_status(service, {"TARGET_FOUND"})
    history = service.history()
    assert len(history) == 1
    assert history[0]["result"] == "TARGET_FOUND"
    service.shutdown()


def test_invalid_target_rejected() -> None:
    service = _service()
    result = _start(service, target="")
    assert result["ok"] is False
    assert "target" in result["error"]
    result = _start(service, target="x" * 501)
    assert result["ok"] is False
    service.shutdown()
