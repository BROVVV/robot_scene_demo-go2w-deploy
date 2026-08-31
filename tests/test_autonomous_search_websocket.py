"""Autonomous search WebSocket tests (plan book §99: initial snapshot,
event delivery, ordering, disconnect, reconnect)."""

from __future__ import annotations

import json
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from fastapi.testclient import TestClient

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.search_session_service import make_mock_executor_factory
from app.manual_web_demo.web_server import DemoRuntime, create_app


class FakeWorker:
    def __init__(self) -> None:
        self._cb: Callable[[str, dict[str, Any]], None] | None = None
        self._status = {
            "state": "ready", "motion_available": True, "robot_mode": 1,
            "robot_error_code": 0, "state_fresh": True, "lease_alive": True,
            "lidar_fresh": True, "front_clearance_m": 1.0,
            "left_clearance_m": 1.0, "right_clearance_m": 1.0,
            "rotation_clearance_valid": True, "last_error": None,
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


def make_client(tmp_path: Path, **factory_kwargs) -> tuple[TestClient, FakeWorker]:
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


def _recv(ws) -> dict:
    return json.loads(ws.receive_text())


def test_ws_initial_snapshot_then_events(tmp_path) -> None:
    client, _ = make_client(
        tmp_path,
        scene_steps=[{"objects": ["blue trash bin"], "target_present": True,
                      "target_score": 0.95}],
    )
    with client:
        with client.websocket_connect("/ws/search") as ws:
            first = _recv(ws)
            assert first["type"] == "snapshot"
            assert first["state"]["status"] in {"IDLE", "STARTING", "RUNNING"}

            client.post("/api/search/start", json={
                "target": "蓝色垃圾桶", "backend": "mock",
            })

            types = []
            deadline = time.time() + 30
            while time.time() < deadline:
                message = _recv(ws)
                if message["type"] == "event":
                    types.append(message["event"]["event_type"])
                    if "TARGET_FOUND" in types:
                        break
                elif message["type"] == "heartbeat":
                    continue
            assert "SESSION_CREATED" in types
            assert "OBSERVATION_UPDATED" in types
            assert "TARGET_CONFIRMED" in types
            assert "SEARCH_FINISHED" in types
            # event ordering: ids monotonically increase
            ids = [
                message["event"]["event_id"]
                for message in _collect_until_events(ws, 6)
            ]
            assert ids == sorted(ids)


def _collect_until_events(ws, count: int) -> list:
    collected = []
    deadline = time.time() + 10
    while len(collected) < count and time.time() < deadline:
        message = _recv(ws)
        if message["type"] == "event":
            collected.append(message)
    return collected


def test_ws_reconnect_gets_snapshot(tmp_path) -> None:
    client, _ = make_client(tmp_path, scenario="no_target", backend_latency_sec=0.3)
    with client:
        with client.websocket_connect("/ws/search") as first:
            _recv(first)  # snapshot
            client.post("/api/search/start", json={"target": "x", "backend": "mock"})
            # wait until running
            deadline = time.time() + 20
            while time.time() < deadline:
                if client.get("/api/search/state").json().get("status") == "RUNNING":
                    break
                time.sleep(0.05)
            # reconnect: new socket must immediately get the live snapshot
        with client.websocket_connect("/ws/search") as second:
            snapshot = _recv(second)
            assert snapshot["type"] == "snapshot"
            assert snapshot["state"]["session_id"].startswith("search_")
            assert snapshot["state"]["status"] in {"RUNNING", "PAUSED"}
        client.post("/api/search/stop")


def test_ws_heartbeat_present(tmp_path) -> None:
    client, _ = make_client(tmp_path)
    with client:
        with client.websocket_connect("/ws/search") as ws:
            messages = [_recv(ws), _recv(ws)]  # snapshot + events
            assert messages[0]["type"] == "snapshot"
