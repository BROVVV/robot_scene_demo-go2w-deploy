"""Autonomous search Web routes tests (plan book §98: start / duplicate /
pause / resume / stop / state / map / objects / history + ownership)."""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from fastapi.testclient import TestClient

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.search_session_service import make_mock_executor_factory
from app.manual_web_demo.web_server import DemoRuntime, create_app


class FakeWorker:
    """Same shape as tests/test_manual_web_demo_api.py::FakeWorker."""

    def __init__(self) -> None:
        self._cb: Callable[[str, dict[str, Any]], None] | None = None
        self._status = {
            "state": "ready", "motion_available": True, "robot_mode": 1,
            "robot_error_code": 0, "state_fresh": True, "lease_alive": True,
            "lidar_fresh": True, "front_clearance_m": 1.0,
            "left_clearance_m": 1.0, "right_clearance_m": 1.0,
            "rotation_clearance_valid": True, "last_error": None,
            "odom_frame": "odom", "odom_pose": [0.0, 0.0, 0.0],
        }
        self.pulses: list[str] = []
        self.stops = 0
        self.estops = 0

    def set_on_message(self, cb): self._cb = cb
    def status(self): return dict(self._status)
    def alive(self): return True
    def start(self): pass
    def stop(self): pass
    def request_pulse(self, direction): self.pulses.append(direction)
    def request_stop(self): self.stops += 1
    def request_estop(self): self.estops += 1
    def request_status(self):
        if self._cb is not None:
            self._cb("camera_status", {"available": True, "width": 1920, "height": 1080})
    def shutdown(self): pass


def make_app(tmp_path: Path, **factory_kwargs) -> tuple[TestClient, FakeWorker]:
    config = replace(
        ManualDemoSettings(),
        runtime_dir=str(tmp_path / "runtime"),
        logs_dir=str(tmp_path / "logs"),
        analysis_frames_dir=str(tmp_path / "frames"),
    )
    worker = FakeWorker()
    runtime = DemoRuntime(
        config,
        worker=worker,
        camera_fresh=lambda: True,
        search_executor_factory=make_mock_executor_factory(**factory_kwargs),
        search_session_dir=str(tmp_path / "sessions"),
    )
    app = create_app(config=config, runtime=runtime)
    return TestClient(app), worker


def _wait_status(client: TestClient, statuses: set[str], timeout: float = 60.0) -> str:
    deadline = time.time() + timeout
    last = ""
    while time.time() < deadline:
        last = client.get("/api/search/state").json().get("status", "")
        if last in statuses:
            return last
        time.sleep(0.05)
    raise AssertionError(f"timed out waiting for {statuses}, last={last}")


def test_idle_state_map_objects_events(tmp_path) -> None:
    client, _ = make_app(tmp_path)
    with client:
        state = client.get("/api/search/state").json()
        assert state["status"] == "IDLE"
        assert client.get("/api/search/map").json()["map_mode"] == "topological"
        assert client.get("/api/search/objects").json() == {
            "current": [], "session_seen": [], "target_evidence": {},
        }
        events = client.get("/api/search/events").json()["events"]
        assert isinstance(events, list)
        assert client.get("/api/search/history").json()["sessions"] == []
        assert client.get("/api/search/readiness").json()["ready"] is True


def test_start_returns_immediately(tmp_path) -> None:
    client, _ = make_app(tmp_path, backend_latency_sec=0.4)
    with client:
        result = client.post(
            "/api/search/start",
            json={"target": "蓝色垃圾桶", "backend": "mock"},
        ).json()
        assert result["ok"] is True
        assert result["status"] == "STARTING"
        assert result["session_id"].startswith("search_")
        # The HTTP call must not block until completion: it already returned.
        status = client.get("/api/search/state").json()["status"]
        assert status in {"STARTING", "RUNNING", "TARGET_FOUND"}


def test_duplicate_start_409_while_running(tmp_path) -> None:
    client, _ = make_app(tmp_path, scenario="no_target", backend_latency_sec=0.3)
    with client:
        result = client.post(
            "/api/search/start", json={"target": "x", "backend": "mock"}
        ).json()
        assert result["ok"] is True
        _wait_status(client, {"RUNNING"})
        second = client.post(
            "/api/search/start", json={"target": "y", "backend": "mock"}
        )
        assert second.status_code == 409
        assert second.json()["ok"] is False
        client.post("/api/search/stop")
        _wait_status(client, {"OPERATOR_STOP", "FINISHED", "FAILED"})


def test_search_end_to_end_state_and_summary(tmp_path) -> None:
    client, _ = make_app(
        tmp_path,
        scene_steps=[{"objects": ["blue trash bin"], "target_present": True,
                      "target_score": 0.95}],
    )
    with client:
        client.post("/api/search/start", json={
            "target": "蓝色垃圾桶", "backend": "mock", "reasoner": "semantic_navigation",
        })
        _wait_status(client, {"TARGET_FOUND"})
        state = client.get("/api/search/state").json()
        assert state["result"] == "TARGET_FOUND"
        assert state["target"] == "蓝色垃圾桶"
        assert state["summary"]["planning_cycles"] == 0
        map_data = client.get("/api/search/map").json()
        assert map_data["revision"] >= 1
        assert client.get("/api/search/objects").json()["session_seen"]
        events = client.get("/api/search/events?limit=100").json()["events"]
        types = [item["event_type"] for item in events]
        assert "TARGET_CONFIRMED" in types
        assert "SEARCH_FINISHED" in types
        history = client.get("/api/search/history?limit=50").json()
        assert history["max_sessions"] == 10
        assert len(history["sessions"]) == 1
        session_id = history["sessions"][0]["session_id"]
        archived = client.get(f"/api/search/history/{session_id}").json()
        assert archived["ok"] is True
        assert archived["state"]["status"] == "TARGET_FOUND"
        assert archived["state"]["map"]["revision"] >= 1
        assert archived["events"]


def test_pause_resume_stop_via_api(tmp_path) -> None:
    client, _ = make_app(tmp_path, scenario="no_target", backend_latency_sec=0.3)
    with client:
        client.post("/api/search/start", json={"target": "x", "backend": "mock",
                                               "max_planning_cycles": 30})
        _wait_status(client, {"RUNNING"})
        assert client.post("/api/search/pause").json()["ok"] is True
        _wait_status(client, {"PAUSED"})
        assert client.post("/api/search/resume").json()["ok"] is True
        _wait_status(client, {"RUNNING"})
        assert client.post("/api/search/stop").json()["ok"] is True
        _wait_status(client, {"OPERATOR_STOP", "FINISHED", "FAILED"})


def test_manual_control_blocks_search_start(tmp_path) -> None:
    client, _ = make_app(tmp_path)
    with client:
        enable = client.post("/api/control/enable").json()
        assert enable["ok"] is True
        result = client.post(
            "/api/search/start", json={"target": "x", "backend": "mock"}
        )
        assert result.status_code == 409
        assert "manual" in result.json()["error"]
        # release manual first, then start works
        client.post("/api/control/disable")
        result = client.post(
            "/api/search/start", json={"target": "x", "backend": "mock"}
        ).json()
        assert result["ok"] is True


def test_autonomous_running_blocks_manual_control(tmp_path) -> None:
    client, _ = make_app(tmp_path, scenario="no_target", backend_latency_sec=0.3)
    with client:
        client.post("/api/search/start", json={"target": "x", "backend": "mock"})
        _wait_status(client, {"RUNNING"})
        result = client.post("/api/control/enable").json()
        assert result["ok"] is False
        assert "autonomous" in (result.get("reason") or "")


def test_estop_overrides_search_and_manual(tmp_path) -> None:
    client, _ = make_app(tmp_path, scenario="no_target", backend_latency_sec=0.3)
    with client:
        client.post("/api/search/start", json={"target": "x", "backend": "mock"})
        _wait_status(client, {"RUNNING"})
        client.post("/api/estop")
        _wait_status(client, {"OPERATOR_STOP", "FINISHED", "FAILED"})
        status = client.get("/api/status").json()
        assert status["owner"]["owner"] == "ESTOP"
        # estop latches: neither manual nor autonomous may start again
        assert client.post("/api/control/enable").json()["ok"] is False
        result = client.post(
            "/api/search/start", json={"target": "y", "backend": "mock"}
        )
        assert result.status_code == 409


def test_invalid_target_400(tmp_path) -> None:
    client, _ = make_app(tmp_path)
    with client:
        response = client.post("/api/search/start", json={"target": ""})
        assert response.status_code == 400
        assert response.json()["ok"] is False
        assert response.json()["error_detail"]["code"] == "TASK_INVALID"
        assert response.json()["error_detail"]["suggestion"]


def test_search_estop_endpoint(tmp_path) -> None:
    client, _ = make_app(tmp_path, scenario="no_target", backend_latency_sec=0.3)
    with client:
        client.post("/api/search/start", json={"target": "x", "backend": "mock"})
        _wait_status(client, {"RUNNING"})
        result = client.post("/api/search/estop").json()
        assert result["ok"] is True
        assert client.get("/api/status").json()["owner"]["owner"] == "ESTOP"


def test_estop_reset_allows_next_search_after_health_checks(tmp_path) -> None:
    client, _ = make_app(tmp_path)
    with client:
        client.post("/api/estop")
        assert client.post("/api/search/start", json={"target": "x"}).status_code == 409
        reset = client.post("/api/estop/reset")
        assert reset.status_code == 200
        assert reset.json()["status"] == "RESET"
        assert client.get("/api/status").json()["owner"]["owner"] == "NONE"
        started = client.post("/api/search/start", json={"target": "x"}).json()
        assert started["ok"] is True
        client.post("/api/search/stop")
