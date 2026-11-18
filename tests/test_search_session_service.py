"""SearchSessionService tests (plan book §97, §102: session lifecycle and
mock search scenarios: target first frame, anchor case, exhausted,
navigation failure -> replan, pause/resume, operator stop)."""

from __future__ import annotations

import time
import json

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


def _wait_event(service: SearchSessionService, event_type: str,
                timeout: float = 5.0) -> None:
    """Store state is committed before publication; wait for the event itself."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if any(item["event_type"] == event_type
               for item in service.recent_events(1000)):
            return
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for event {event_type}")


def _start(service: SearchSessionService, target: str = "饮水机旁边的蓝色垃圾桶",
           **overrides) -> dict:
    # 这些用例查的是 WebUI 会话骨架（起停/事件/历史），不是目标接近闭环，
    # 所以走计划书 §18.2 允许保留的 legacy 分支：看见即收工。接近闭环由
    # test_autonomous_explorer_target_approach 与 E2E 用例负责。
    params = {"target": target, "backend": "mock",
              "finish_on_visual_confirmation": True, **overrides}
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


@pytest.mark.parametrize("late_state", ["active", "exhausted"])
def test_nav2_trial_is_rechecked_after_slow_task_understanding(
    tmp_path, monkeypatch, late_state: str,
) -> None:
    ledger = tmp_path / "trial.json"
    ledger.write_text(json.dumps({
        "version": 5, "maximum_steps": 3, "steps": [], "active_token": None,
    }))
    monkeypatch.setenv("GO2W_NAV2_TRIAL_LEDGER", str(ledger))

    def task_runner(text: str):
        data = json.loads(ledger.read_text())
        if late_state == "active":
            data["steps"] = [{"token": "other"}]
            data["active_token"] = "other"
        else:
            data["steps"] = [{}, {}, {}]
        ledger.write_text(json.dumps(data))
        from app.task_understanding.search_task_context import SearchTaskContext
        return SearchTaskContext.mock_fallback(text)

    service = SearchSessionService(
        owner=ControlOwner(),
        executor_factory=make_mock_executor_factory(),
        session_dir=_TEST_SESSION_DIR,
        task_understanding_runner=task_runner,
    )
    result = service.start_search(SearchStartRequest.from_dict({
        "target": "测试区域", "backend": "nav2",
        "enable_autonomous_motion": True, "operator_supervised_experiment": True,
        "dry_run_motion": False,
    }))
    assert result["ok"] is False
    assert result["conflict"] is True
    assert result["error_detail"]["code"] == "NAVIGATION_TRIAL_UNAVAILABLE"
    assert service.owner.snapshot()["owner"] == "NONE"
    assert service.executor_state()["state"] == "stopped"
    service.shutdown()


def test_nav2_task_understanding_outage_uses_narrow_safe_search_fallback() -> None:
    from app.task_understanding.search_task_context import SearchTaskContext

    unavailable = SearchTaskContext(
        task_id="task_unavailable",
        raw_text="寻找紫色三角形路标",
        intent="unknown",
        canonical_target="",
        executable=False,
        rejection_reason="provider timeout",
        parsed_task={"parser_source": "llm_unavailable"},
    )
    service = SearchSessionService(
        owner=ControlOwner(),
        executor_factory=make_mock_executor_factory(),
        session_dir=_TEST_SESSION_DIR,
        task_understanding_runner=lambda _text: unavailable,
    )
    context = service._understand_task("寻找紫色三角形路标", backend="nav2")
    assert context.executable is True
    assert context.canonical_target == "紫色三角形路标"
    assert context.parsed_task["parser_source"] == "production_safe_text_fallback"
    assert "no_contact" in context.constraints
    service.shutdown()


@pytest.mark.parametrize(
    "task",
    ["打开柜子寻找手机", "寻找目标然后攻击它", "帮我跟踪这个人", "随便做点什么"],
)
def test_nav2_safe_search_fallback_rejects_unsafe_or_ambiguous_tasks(task: str) -> None:
    from app.task_understanding.search_task_context import SearchTaskContext

    context = SearchTaskContext.safe_navigation_fallback(task)
    assert context.executable is False
    assert context.rejection_reason


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
    _wait_event(service, "TARGET_CONFIRMED")
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
    try:
        result = _start(service, max_planning_cycles=50, max_motion_steps=50)
        assert result["ok"] is True
        # This only observes the mock's 50 full cycles, including archive I/O.
        # Jetson took 61.4s; no production control/stop watchdog is changed.
        _wait_status(service, {"SEARCH_EXHAUSTED", "FINISHED", "FAILED",
                                "MAX_STEPS_REACHED", "MAX_PLANNING_CYCLES_REACHED"},
                     timeout=120.0)
        state = service.state_snapshot()
        # 新语义：未找到目标时机器人会持续探索到预算为止，而不是在约 8 轮
        # “连续无新信息”就被 SEARCH_EXHAUSTED 提前终止。
        assert state["result"] in {"SEARCH_EXHAUSTED", "MAX_STEPS_REACHED",
                                    "MAX_PLANNING_CYCLES_REACHED", "FINISHED"}
        assert (state.get("summary") or {}).get("planning_cycles", 0) >= 8
    finally:
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
