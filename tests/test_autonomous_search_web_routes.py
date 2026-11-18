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


def make_app(tmp_path: Path, *, executor_factory=None,
             **factory_kwargs) -> tuple[TestClient, FakeWorker]:
    config = replace(
        ManualDemoSettings(),
        runtime_dir=str(tmp_path / "runtime"),
        logs_dir=str(tmp_path / "logs"),
        analysis_frames_dir=str(tmp_path / "frames"),
        # 绝对路径：config.slam_map_snapshot_path 对相对路径会拼 _PROJECT_ROOT，
        # 那会把测试指到仓库里那份真机会话留下的 runtime/slam_map_3d.json 上。
        # 它一旦被判成"发散/冻结"，/api/search/start 会先以 409 MAPPING_FROZEN
        # 返回，executor 根本不会启动 —— 本文件所有"等状态变成 X"的用例就集体
        # 超时，看着像代码回归，其实是测试读了别人的运行态。
        # 同 tests/test_slam_map_snapshot_health_contract.py 的写法。
        slam_map_snapshot=str(tmp_path / "runtime" / "slam_map_3d.json"),
    )
    worker = FakeWorker()
    runtime = DemoRuntime(
        config,
        worker=worker,
        camera_fresh=lambda: True,
        search_executor_factory=(
            executor_factory or make_mock_executor_factory(**factory_kwargs)
        ),
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


def _wait_event(client: TestClient, event_type: str, timeout: float = 30.0) -> list[str]:
    """等到 `event_type` 出现在归档事件流里，返回当时全部事件类型。

    §2.2 删掉了 `/api/search/events`（它是 state 的第二份投影）。事件仍然完整地
    落在会话归档里，历史详情 API 是唯一的读口 —— 这条路径同时也把"归档里的事件
    和 live 事件是同一批"钉住了。

    ⚠ **不能**在 `_wait_status(TARGET_FOUND)` 之后直接取事件：状态机置位与事件
    落盘不是同一个原子步骤。实测（2026-09-16，12 次）**4 次**会读到"状态已是
    TARGET_FOUND、`SEARCH_FINISHED` 还没追加"的中间窗口，通过率只有 8/12。

    这不是产品漏发：同一进程里再等 1 秒 `SEARCH_FINISHED` 就出现了（而且出现两次）。
    是**测试没等**。所以这里加等待，**断言一个字不改**——超时返回最后一次读到的
    列表，断言照样会因为缺事件而红。
    """
    deadline = time.time() + timeout
    types: list[str] = []
    while time.time() < deadline:
        session_id = client.get("/api/search/state").json().get("session_id") or ""
        if session_id:
            record = client.get(f"/api/search/history/{session_id}").json()
            types = [item["event_type"] for item in (record.get("events") or [])]
            if event_type in types:
                return types
        time.sleep(0.05)
    return types


def test_idle_state_has_one_empty_graph(tmp_path) -> None:
    """空闲态：一份 `graph`（`null`）+ 空的小状态，没有第二张图、没有旧路由。

    §2.2 之后 `/api/search/map`、`/objects`、`/events` 都不存在了 —— 这条同时
    钉住"它们真的没了"（404 而不是返回一份影子数据）。
    """
    client, _ = make_app(tmp_path)
    with client:
        state = client.get("/api/search/state").json()
        assert state["status"] == "IDLE"
        assert state["graph"] is None
        assert state["graph_revision"] == 0
        assert state["objects"] == {
            "current": [], "session_seen": [], "target_evidence": {},
        }
        for stale in ("/api/search/map", "/api/search/objects", "/api/search/events",
                      "/api/search/decisions", "/api/search/frontiers",
                      "/api/search/place-graph", "/api/search/semantic-map"):
            assert client.get(stale).status_code == 404, f"{stale} 应当已删除"
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


def test_exhausted_bounded_trial_blocks_web_search_start(tmp_path, monkeypatch) -> None:
    ledger=tmp_path/'trial.json'
    ledger.write_text('{"version":6,"maximum_steps":1,"steps":[{}],"active_token":null}')
    monkeypatch.setenv('GO2W_NAV2_TRIAL_LEDGER',str(ledger))
    client,_=make_app(tmp_path)
    with client:
        readiness=client.get('/api/search/readiness').json()
        assert readiness['ready'] is False
        assert readiness['checks']['navigation_trial_available'] is False
        assert 'navigation_trial_available' in readiness['blocking']
        response=client.post('/api/search/start',json={'target':'蓝色垃圾桶','backend':'mock'})
        assert response.status_code==409
        assert 'trial' in response.json()['error'].lower()


def test_start_params_carry_the_detector(tmp_path) -> None:
    """物体识别后端必须活着走到 worker 的 params 上。

    2026-09-17 断的就是这一环：``search_session_service.start_search`` 里那份
    params 是逐个字段手写的，而 ``detector`` 当时根本不在其中 —— 于是请求里
    就算带了它，worker 也永远收不到，``--detector`` 恒定落到 argparse 默认值。
    """
    captured: dict[str, Any] = {}
    inner = make_mock_executor_factory()

    def factory():
        executor = inner()
        original_start = executor.start

        def start(params):
            captured.update(params)
            return original_start(params)

        executor.start = start
        return executor

    factory.is_mock_factory = True
    client, _ = make_app(tmp_path, executor_factory=factory)
    with client:
        client.post("/api/search/start",
                    json={"target": "蓝色垃圾桶", "backend": "mock",
                          "detector": "grounded_sam"})
        assert captured.get("detector") == "grounded_sam"


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
            "target": "蓝色垃圾桶", "backend": "mock",
            # 本用例查的是 WebUI 会话骨架（state/summary/map/objects/events/
            # history），不是目标接近闭环，所以走计划书 §18.2 允许保留的 legacy
            # 分支：看见即收工。接近闭环由 test_autonomous_explorer_target_approach
            # 与 E2E 用例负责（同 test_search_session_service.py:52-55 的理由）。
            "finish_on_visual_confirmation": True,
        })
        _wait_status(client, {"TARGET_FOUND"})
        state = client.get("/api/search/state").json()
        assert state["result"] == "TARGET_FOUND"
        assert state["target"] == "蓝色垃圾桶"
        assert state["summary"]["planning_cycles"] == 0
        assert state["graph"]["revision"] >= 1
        assert state["objects"]["session_seen"]
        types = _wait_event(client, "SEARCH_FINISHED")
        assert "TARGET_CONFIRMED" in types
        assert "SEARCH_FINISHED" in types
        history = client.get("/api/search/history?limit=50").json()
        assert history["max_sessions"] == 10
        assert len(history["sessions"]) == 1
        session_id = history["sessions"][0]["session_id"]
        archived = client.get(f"/api/search/history/{session_id}").json()
        assert archived["ok"] is True
        assert archived["state"]["status"] == "TARGET_FOUND"
        assert archived["state"]["graph"]["revision"] >= 1
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
