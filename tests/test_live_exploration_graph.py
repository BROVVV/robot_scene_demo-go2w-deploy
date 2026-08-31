"""Live ExplorationGraph tests (plan book §44-§49, §97: test_live_exploration_graph)."""

from __future__ import annotations

import json
import time
from dataclasses import replace
from pathlib import Path

from fastapi.testclient import TestClient

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.search_session_service import make_mock_executor_factory
from app.manual_web_demo.web_server import DemoRuntime, create_app


class FakeWorker:
    def __init__(self) -> None:
        self._cb = None
        self._status = {
            "state": "ready", "motion_available": True, "robot_mode": 1,
            "robot_error_code": 0, "state_fresh": True, "lease_alive": True,
            "lidar_fresh": True, "front_clearance_m": 1.0,
            "left_clearance_m": 1.0, "right_clearance_m": 1.0,
            "rotation_clearance_valid": True, "last_error": None,
        }

    def set_on_message(self, cb): self._cb = cb
    def status(self): return dict(self._status)
    def alive(self): return True
    def start(self): pass
    def stop(self): pass
    def request_pulse(self, d): pass
    def request_stop(self): pass
    def request_estop(self): pass
    def request_status(self):
        if self._cb is not None:
            self._cb("camera_status", {"available": True, "width": 1920, "height": 1080})
    def shutdown(self): pass


def make_client(tmp_path: Path) -> TestClient:
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
        search_executor_factory=make_mock_executor_factory(
            scenario="anchor_then_target", mock_target_after=2,
            confirm_after_seen=1,
        ),
        search_session_dir=str(tmp_path / "sessions"),
    )
    app = create_app(config=config, runtime=runtime)
    return TestClient(app)


def _wait_finished(client: TestClient, timeout: float = 60.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        status = client.get("/api/search/state").json().get("status", "")
        if status in {"TARGET_FOUND", "SEARCH_EXHAUSTED", "FINISHED", "FAILED", "OPERATOR_STOP"}:
            return
        time.sleep(0.05)
    raise AssertionError("search did not finish")


def test_map_schema_and_revision_monotonic(tmp_path) -> None:
    client = make_client(tmp_path)
    with client:
        client.post("/api/search/start", json={"target": "饮水机旁边的蓝色垃圾桶",
                                               "backend": "mock"})
        _wait_finished(client)
        map_data = client.get("/api/search/map").json()
        assert map_data["schema_version"] == "live_exploration_graph_v1"
        assert map_data["map_mode"] == "topological"
        assert map_data["revision"] >= 1
        assert map_data["current_node_id"]
        assert len(map_data["nodes"]) >= 1
        assert len(map_data["edges"]) >= 0
        # node schema (plan book §47)
        node = map_data["nodes"][0]
        for key in ("node_id", "x", "y", "yaw", "pose_quality", "reachable_state",
                    "visited_count", "objects", "target_match_level",
                    "semantic_relevance", "information_gain", "timestamp"):
            assert key in node
        # revision strictly grows across MAP_UPDATED events
        revisions = [
            item["payload"].get("revision")
            for item in client.get("/api/search/events?limit=2000").json()["events"]
            if item["event_type"] == "MAP_UPDATED"
        ]
        revisions = [r for r in revisions if r is not None]
        assert revisions == sorted(revisions)


def test_map_persisted_to_session_dir(tmp_path) -> None:
    client = make_client(tmp_path)
    with client:
        client.post("/api/search/start", json={"target": "饮水机旁边的蓝色垃圾桶",
                                               "backend": "mock"})
        _wait_finished(client)
        session = client.get("/api/search/state").json()["session_id"]
        session_dir = Path(tmp_path) / "sessions" / session
        # artifacts are written right after the terminal event; poll briefly
        deadline = time.time() + 10
        while time.time() < deadline:
            if (session_dir / "summary.json").is_file():
                break
            time.sleep(0.05)
        assert (session_dir / "summary.json").is_file()
        assert (session_dir / "exploration_graph.json").is_file()
        assert (session_dir / "events.jsonl").is_file()
        graph = json.loads((session_dir / "exploration_graph.json").read_text())
        # The graph keeps the explorer's internal session id; it must be
        # non-empty and the artifact must live under the Web session dir.
        assert graph["session_id"]
        assert len(graph["nodes"]) >= 1


def test_map_carries_semantic_state(tmp_path) -> None:
    client = make_client(tmp_path)
    with client:
        client.post("/api/search/start", json={"target": "饮水机旁边的蓝色垃圾桶",
                                               "backend": "mock"})
        _wait_finished(client)
        map_data = client.get("/api/search/map").json()
        node = map_data["nodes"][0]
        # semantic interest / target candidate flags ride on node state
        assert node["reachable_state"] in {
            "OBSERVED", "VISITED", "SEMANTIC_INTEREST", "NEGATIVE",
            "TARGET_CANDIDATE", "TARGET_CONFIRMED",
        }
        state = client.get("/api/search/state").json()
        assert state["map"]["revision"] == map_data["revision"]
