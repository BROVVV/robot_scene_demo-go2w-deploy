"""FastAPI endpoint tests with a mocked ROS worker (plan book §47)."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from fastapi.testclient import TestClient

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.web_server import DemoRuntime, create_app


class FakeWorker:
    def __init__(self) -> None:
        self._cb: Callable[[str, dict[str, Any]], None] | None = None
        self._status = {
            "state": "ready",
            "motion_available": True,
            "robot_mode": 1,
            "robot_error_code": 0,
            "state_fresh": True,
            "lease_alive": True,
            "lidar_fresh": True,
            "front_clearance_m": 1.0,
            "left_clearance_m": 1.0,
            "right_clearance_m": 1.0,
            "rotation_clearance_valid": True,
            "last_error": None,
            "odom_frame": "odom",
            "odom_pose": [0.0, 0.0, 0.0],
        }
        self.pulses: list[str] = []
        self.stops = 0
        self.estops = 0

    def set_on_message(self, callback: Callable[[str, dict[str, Any]], None]) -> None:
        self._cb = callback

    def status(self) -> dict[str, Any]:
        return dict(self._status)

    def alive(self) -> bool:
        return True

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def request_pulse(self, direction: str) -> None:
        self.pulses.append(direction)
        if self._cb is not None:
            self._cb(
                "motion_finished",
                {
                    "success": True,
                    "direction": direction,
                    "error_code": "none",
                    "message": "ok",
                },
            )

    def request_stop(self) -> None:
        self.stops += 1

    def request_estop(self) -> None:
        self.estops += 1

    def request_status(self) -> None:
        pass

    def shutdown(self) -> None:
        pass


def make_app(tmp_path: Path) -> tuple[TestClient, FakeWorker]:
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
    )
    app = create_app(config=config, runtime=runtime)
    return TestClient(app), worker


def test_index_served() -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        client, _ = make_app(Path(tmp))
        with client:
            response = client.get("/")
            assert response.status_code == 200
            assert "Go2-W" in response.text


def test_status_endpoint() -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        client, _ = make_app(Path(tmp))
        with client:
            response = client.get("/api/status")
            assert response.status_code == 200
            data = response.json()
            assert data["motion"]["available"] is True
            assert data["motion"]["control_enabled"] is False
            assert "camera" in data
            assert "llm" in data
            assert "directions" in data


def test_objects_endpoint() -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        client, _ = make_app(Path(tmp))
        with client:
            response = client.get("/api/objects")
            assert response.status_code == 200
            data = response.json()
            assert "objects" in data
            assert "status" in data


def test_control_enable_disable_estop() -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        client, _ = make_app(Path(tmp))
        with client:
            enable = client.post("/api/control/enable")
            assert enable.status_code == 200
            assert enable.json()["ok"] is True

            status = client.get("/api/status").json()
            assert status["motion"]["control_enabled"] is True

            disable = client.post("/api/control/disable")
            assert disable.json()["ok"] is True
            assert client.get("/api/status").json()["motion"]["control_enabled"] is False

            estop = client.post("/api/estop")
            assert estop.json()["ok"] is True
            assert client.get("/api/status").json()["motion"]["control_enabled"] is False


def test_websocket_drives_pulse() -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        client, worker = make_app(Path(tmp))
        with client:
            assert client.post("/api/control/enable").json()["ok"] is True
            with client.websocket_connect("/ws/control") as websocket:
                websocket.send_json({"type": "hello"})
                websocket.receive_json()  # initial state
                websocket.send_json({"type": "key_down", "key": "w"})
                websocket.receive_json()  # state / motion_started
                websocket.receive_json()  # motion_finished / state
            assert "forward" in worker.pulses


def test_websocket_disconnect_disables_control() -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        client, _ = make_app(Path(tmp))
        with client:
            assert client.post("/api/control/enable").json()["ok"] is True
            with client.websocket_connect("/ws/control") as websocket:
                websocket.send_json({"type": "heartbeat", "pressed": [], "seq": 1})
            # After WS close, control should be disabled.
            assert client.get("/api/status").json()["motion"]["control_enabled"] is False


def test_llm_toggle_endpoints() -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        client, _ = make_app(Path(tmp))
        with client:
            # LLM analysis is OFF by default (saves tokens).
            assert client.get("/api/status").json()["llm"]["enabled"] is False
            enable = client.post("/api/llm/enable")
            assert enable.status_code == 200
            assert enable.json()["enabled"] is True
            assert client.get("/api/status").json()["llm"]["enabled"] is True
            disable = client.post("/api/llm/disable")
            assert disable.json()["enabled"] is False
            assert client.get("/api/status").json()["llm"]["enabled"] is False


def test_camera_mjpeg_generator_emits_frames() -> None:
    """The infinite MJPEG generator yields frame chunks and stops on disconnect.

    Exercised directly (instead of through TestClient) because a never-ending
    streaming response cannot be cleanly shut down by the ASGI test client.
    """
    import asyncio
    import tempfile

    from app.manual_web_demo.web_server import _mjpeg_generator

    with tempfile.TemporaryDirectory() as tmp:
        runtime_dir = Path(tmp) / "runtime"
        runtime_dir.mkdir(parents=True, exist_ok=True)
        (runtime_dir / "latest.jpg").write_bytes(b"\xff\xd8\xff\xd9")
        config = replace(
            ManualDemoSettings(),
            runtime_dir=str(runtime_dir),
            logs_dir=str(Path(tmp) / "logs"),
        )
        runtime = DemoRuntime(config, worker=FakeWorker(), camera_fresh=lambda: True)

        class FakeRequest:
            def __init__(self, disconnected: bool) -> None:
                self._disconnected = disconnected

            async def is_disconnected(self) -> bool:
                return self._disconnected

        async def collect(disconnected: bool) -> list[bytes]:
            generator = _mjpeg_generator(runtime, FakeRequest(disconnected))
            chunks: list[bytes] = []
            async for chunk in generator:
                chunks.append(chunk)
                if len(chunks) >= 2:
                    break
            return chunks

        chunks = asyncio.run(collect(disconnected=False))
        assert len(chunks) >= 1
        assert b"frame" in chunks[0]
        assert b"Content-Type: image/jpeg" in chunks[0]

        # On disconnect the generator stops without yielding another frame.
        async def first_frame() -> list[bytes]:
            generator = _mjpeg_generator(runtime, FakeRequest(disconnected=True))
            out: list[bytes] = []
            async for chunk in generator:
                out.append(chunk)
                break
            return out

        assert asyncio.run(first_frame()) == []
