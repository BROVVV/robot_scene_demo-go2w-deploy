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
        # 见 test_autonomous_search_web_routes.py 同名字段的注释：不指到
        # tmp_path 就会读到仓库里那份真机会话留下的快照，被 409 挡在启动之前。
        slam_map_snapshot=str(tmp_path / "runtime" / "slam_map_3d.json"),
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


def test_graph_is_one_snapshot_with_a_monotonic_revision(tmp_path) -> None:
    """§2.2：`/api/search/state` 只有一份 `graph`，revision 由生产者单调推进。

    曾经这里读的是 `/api/search/map`（`live_exploration_graph_v1`，第二张图）。
    那条路由连同 `state.map` 一起删了 —— 现在唯一的图是
    `semantic_navigation_graph_v1`。
    """
    client = make_client(tmp_path)
    with client:
        client.post("/api/search/start", json={"target": "饮水机旁边的蓝色垃圾桶",
                                               "backend": "mock"})
        _wait_finished(client)
        state = client.get("/api/search/state").json()
        graph = state["graph"]
        assert graph is not None, "mock 会话跑完必须留下一份图"
        assert graph["schema_version"] == "semantic_navigation_graph_v1"
        assert graph["revision"] >= 1
        assert state["graph_revision"] == graph["revision"]
        for key in ("places", "objects", "frontiers", "edges",
                    "current_place_id", "object_topology"):
            assert key in graph, f"web_snapshot() 必须带上 {key}"
        assert graph["places"], "跑完 anchor_then_target 场景至少该留下一个 Place"
        assert "map" not in state, "state.map 已删；图只走 state.graph"

        # 归档里的事件日志中，MAP_UPDATED 只带 revision，且 revision 单调不减。
        session = state["session_id"]
        events_path = Path(tmp_path) / "sessions" / session / "webui_events.jsonl"
        deadline = time.time() + 10
        while time.time() < deadline and not events_path.is_file():
            time.sleep(0.05)
        revisions = [
            json.loads(line)["payload"].get("revision")
            for line in events_path.read_text(encoding="utf-8").splitlines()
            if line.strip() and json.loads(line)["event_type"] == "MAP_UPDATED"
        ]
        revisions = [r for r in revisions if r is not None]
        assert revisions, "至少要有一次 MAP_UPDATED 通知"
        assert revisions == sorted(revisions), f"revision 不许回退：{revisions}"
        assert max(revisions) == graph["revision"], (
            "最后一条通知的 revision 必须就是快照里那一份"
        )

        # §5：事件日志里**一条**整图都不能有。只断言 MAP_UPDATED 的形状不够 ——
        # 把整图塞进别的 event_type（旧的 observation / place_created 就这么干）
        # 一样会让日志随会话线性膨胀。判据有两个：payload 顶层出现整图键名，
        # 或任意层出现带图 schema_version 的 dict。不按 `nodes`/`objects` 这类
        # 数组名判 —— OBJECTS_UPDATED 的小列表、object_topology 的投影都是各
        # 面板要用的合法载荷，按数组名判会把它们一起误杀。
        GRAPH_KEYS = {"graph", "map", "semantic_navigation_graph", "spatial_map",
                      "place_graph", "semantic_graph"}

        def _graphs(node):
            if isinstance(node, dict):
                if str(node.get("schema_version") or "").startswith(
                    "semantic_navigation_graph"
                ):
                    yield node
                for value in node.values():
                    yield from _graphs(value)
            elif isinstance(node, list):
                for value in node:
                    yield from _graphs(value)

        records = [json.loads(line) for line in
                   events_path.read_text(encoding="utf-8").splitlines() if line.strip()]
        for record in records:
            payload = record["payload"]
            if record["event_type"] == "MAP_UPDATED":
                assert set(payload) == {"revision"}, (
                    f"MAP_UPDATED 只许带 revision：{payload}"
                )
            assert not GRAPH_KEYS & set(payload), (
                f"{record['event_type']} 的载荷里有整图键：{sorted(GRAPH_KEYS & set(payload))}"
            )
            found = list(_graphs(payload))
            assert not found, f"{record['event_type']} 的事件里带了整图：{found[:1]}"


def test_map_persisted_to_session_dir(tmp_path) -> None:
    client = make_client(tmp_path)
    with client:
        client.post("/api/search/start", json={"target": "饮水机旁边的蓝色垃圾桶",
                                               "backend": "mock"})
        _wait_finished(client)
        session = client.get("/api/search/state").json()["session_id"]
        session_dir = Path(tmp_path) / "sessions" / session
        # artifacts are written right after the terminal event; poll briefly.
        # 必须等**全部**三个文件，不能只等 summary.json —— 写手是一个接一个写的
        # （exploration_graph → summary → events → task → decisions → semantic_map），
        # 只等第一个就会在 events.jsonl 落盘之前跳出循环，断言随机变红。
        # 2026-09-21 实测：summary.json 已可见时 events.jsonl 还要再等 <0.5s。
        expected = ("summary.json", "exploration_graph.json", "events.jsonl")
        deadline = time.time() + 10
        while time.time() < deadline:
            if all((session_dir / name).is_file() for name in expected):
                break
            time.sleep(0.05)
        for name in expected:
            assert (session_dir / name).is_file(), name
        graph = json.loads((session_dir / "exploration_graph.json").read_text())
        # The graph keeps the explorer's internal session id; it must be
        # non-empty and the artifact must live under the Web session dir.
        assert graph["session_id"]
        assert len(graph["nodes"]) >= 1


def test_graph_carries_semantic_state(tmp_path) -> None:
    """图里带着语义状态：Place 有位姿质量、对象有状态与确认标记。"""
    client = make_client(tmp_path)
    with client:
        client.post("/api/search/start", json={"target": "饮水机旁边的蓝色垃圾桶",
                                               "backend": "mock"})
        _wait_finished(client)
        state = client.get("/api/search/state").json()
        graph = state["graph"]
        assert state["graph_revision"] == graph["revision"]

        place = graph["places"][0]
        assert place["place_id"].startswith("P")
        assert "pose_quality" in place

        assert graph["objects"], "场景里认出了物体，图里就该有对象条目"
        obj = graph["objects"][0]
        assert obj["status"] in {"CONFIRMED", "TENTATIVE", "STALE"}
        assert 0.0 <= obj["confidence"] <= 1.0

        # 对象拓扑是图的子视图，不是另一份数据。
        topology = graph["object_topology"]
        assert topology["stats"]["node_count"] == len(topology["nodes"])
