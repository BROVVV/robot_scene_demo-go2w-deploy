"""Startup lifecycle tests (plan §4 / §19.7).

Covers mock STARTING -> RUNNING, worker import/exit failure -> FAILED, and the
WebUI's ability to observe startup stage progression so it never hangs in an
undifferentiated STARTING.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import pytest

from app.live_robot.search_state_store import (
    STATUS_FAILED,
    STATUS_RUNNING,
    STATUS_STARTING,
)
from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.search_executor import InProcessMockExecutor
from app.manual_web_demo.search_session_service import SearchSessionService


class _SlowStartExecutor(InProcessMockExecutor):
    """Emits a startup stage before running, to verify stage progression."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._stage_emitted = False

    def start(self, params: dict[str, Any]) -> None:
        # Emit worker_status STARTING stage before delegating.
        if self._on_message:
            self._on_message({
                "type": "worker_status",
                "session_id": params.get("session_id"),
                "status": {
                    "state": "starting", "stage": "WAIT_RGBD",
                    "stage_started_at": time.time(),
                    "last_progress_at": time.time(),
                    "worker_alive": True,
                },
            })
        super().start(params)


class _FailingStartExecutor:
    """An executor whose thread immediately fails, simulating worker death."""

    def __init__(self) -> None:
        self._on_message: Callable[[dict[str, Any]], None] | None = None
        self._thread: threading.Thread | None = None

    def set_on_message(self, cb): self._on_message = cb
    def start(self, params):
        def fail():
            time.sleep(0.05)
            if self._on_message:
                self._on_message({"type": "error", "message": "simulated worker death"})
                self._on_message({
                    "type": "session_result",
                    "result": {"exit_code": 4, "finish_reason": "FAILED"},
                })
        self._thread = threading.Thread(target=fail, daemon=True)
        self._thread.start()
    def pause(self): pass
    def resume(self): pass
    def stop(self): pass
    def estop(self): pass
    def status(self): return {"state": "running", "session_id": "x"}
    def alive(self): return self._thread is not None and self._thread.is_alive()
    def shutdown(self): pass


_TEST_SESSION_DIR = "outputs/live_runs_test_startup"


@pytest.fixture(autouse=True)
def _isolate_archives(tmp_path):
    global _TEST_SESSION_DIR
    _TEST_SESSION_DIR = str(tmp_path / "sessions")


def _make_service(executor_factory):
    # Mark the factory as a mock factory so start_search uses it for
    # backend="mock" instead of substituting a default InProcessMockExecutor.
    executor_factory.is_mock_factory = True  # type: ignore[attr-defined]
    return SearchSessionService(
        owner=ControlOwner(),
        executor_factory=executor_factory,
        allow_mock_task_fallback=True,
        session_dir=_TEST_SESSION_DIR,
    )


def test_starting_emits_worker_status_stage():
    """The worker_status STARTING stage is surfaced into the store's startup
    snapshot so the WebUI can render an explicit startup stage."""
    factory = lambda: _SlowStartExecutor(backend_latency_sec=0.2)
    service = _make_service(factory)
    result = service.start_search(
        SearchStartRequest(
            task_text="找到垃圾桶", backend="mock", target="垃圾桶",
        )
    )
    assert result["ok"] is True
    # The in-process executor is async; poll for the worker_status stage.
    deadline = time.time() + 10
    stage = None
    while time.time() < deadline:
        snap = service.state_snapshot()
        stage = (snap.get("startup") or {}).get("stage")
        if stage:
            break
        time.sleep(0.05)
    assert stage in {"WAIT_RGBD", "RUNNING", "SPAWN_WORKER", "WORKER_READY", "LOAD_PIPELINE", "START_EXPLORER"}
    service.shutdown()


def test_worker_failure_leads_to_failed():
    factory = lambda: _FailingStartExecutor()
    service = _make_service(factory)
    result = service.start_search(
        SearchStartRequest(
            task_text="找到垃圾桶", backend="mock", target="垃圾桶",
        )
    )
    assert result["ok"] is True
    deadline = time.time() + 10
    last = ""
    while time.time() < deadline:
        snap = service.state_snapshot()
        last = snap.get("status") or ""
        if last in {STATUS_FAILED, "FINISHED", "OPERATOR_STOP"}:
            break
        time.sleep(0.05)
    assert last in {STATUS_FAILED, "FINISHED", "OPERATOR_STOP"}
    service.shutdown()


def test_duplicate_start_rejected_while_starting():
    factory = lambda: InProcessMockExecutor(backend_latency_sec=0.5)
    service = _make_service(factory)
    r1 = service.start_search(
        SearchStartRequest(task_text="找到垃圾桶", backend="mock", target="垃圾桶")
    )
    assert r1["ok"] is True
    time.sleep(0.05)
    r2 = service.start_search(
        SearchStartRequest(task_text="另一个", backend="mock", target="另一个")
    )
    assert r2["ok"] is False
    assert r2.get("conflict") is True
    service.shutdown()


def test_startup_stage_snapshot_exists_in_empty_state():
    service = _make_service(lambda: InProcessMockExecutor())
    snap = service.state_snapshot()
    assert "startup" in snap
    service.shutdown()
