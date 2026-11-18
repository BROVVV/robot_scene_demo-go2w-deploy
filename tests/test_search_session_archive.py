from __future__ import annotations

from pathlib import Path

from app.live_robot.search_event import ERROR, EventIdAllocator, make_event
from app.manual_web_demo.search_session_archive import SearchSessionArchive
from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_session_service import SearchSessionService


def _graph(index: int) -> dict:
    """§2.2 的唯一图：一次会话一份，单独落盘。"""
    return {
        "schema_version": "semantic_navigation_graph_v1",
        "revision": index,
        "current_place_id": "P1",
        "places": [{"place_id": "P1"}],
        "objects": [],
        "frontiers": [],
        "edges": [],
        "object_topology": {
            "schema_version": "semantic_object_topology_v1",
            "revision": index,
            "nodes": [{"node_id": f"obj_{index:03d}"}],
            "edges": [],
        },
    }


def _snapshot(session_id: str, index: int, status: str = "FINISHED") -> dict:
    """小状态：**不含** graph —— 图由 :meth:`record_graph` 单独写。"""
    return {
        "session_id": session_id,
        "status": status,
        "result": status,
        "target": f"目标 {index}",
        "cycle": index,
        "elapsed_seconds": float(index),
        "spatial": {"route_plan": None},
        "decisions": [{"decision_id": f"d{index}"}],
        "timeline": [],
    }


def test_archive_preserves_full_state_and_events(tmp_path: Path) -> None:
    archive = SearchSessionArchive(tmp_path, max_sessions=10)
    sid = "search_20260821_120000_deadbeef"
    archive.begin(sid, {"task_text": "寻找蓝色垃圾桶", "target": "蓝色垃圾桶"})
    archive.record_graph(sid, _graph(1))
    event = make_event(
        allocator=EventIdAllocator(), session_id=sid, event_type=ERROR,
        payload={"code": "CAMERA_UNAVAILABLE", "message": "frame stale"},
    )
    state = _snapshot(sid, 1, "FAILED")
    state["error"] = event.payload
    archive.record(state, event)

    loaded = archive.load(sid)
    assert loaded is not None
    assert loaded["state"]["graph"]["object_topology"]["nodes"]
    assert loaded["state"]["graph_revision"] == 1
    assert loaded["state"]["decisions"][0]["decision_id"] == "d1"
    assert loaded["events"][0]["payload"]["code"] == "CAMERA_UNAVAILABLE"
    # 图与小状态分文件：小状态那一份**不含**图，否则每次事件都要整图落盘。
    import json as _json

    small = _json.loads((tmp_path / sid / "webui_state.json").read_text(encoding="utf-8"))
    assert "graph" not in small and "graph_revision" not in small


def test_archive_keeps_only_ten_managed_sessions_and_preserves_legacy(tmp_path: Path) -> None:
    legacy = tmp_path / "search_legacy_unmanaged"
    legacy.mkdir()
    (legacy / "summary.json").write_text("{}", encoding="utf-8")
    archive = SearchSessionArchive(tmp_path, max_sessions=10)
    for index in range(12):
        sid = f"search_20260821_12{index:02d}00_{index:08d}"
        archive.begin(sid, {"task_text": f"目标 {index}"})
        archive.record(_snapshot(sid, index))

    assert len(archive.list(50)) == 10
    assert legacy.is_dir()


def test_latest_snapshot_survives_archive_reconstruction(tmp_path: Path) -> None:
    sid = "search_20260821_130000_cafebabe"
    first = SearchSessionArchive(tmp_path)
    first.begin(sid, {"task_text": "找椅子"})
    first.record_graph(sid, _graph(7))
    first.record(_snapshot(sid, 7, "OPERATOR_STOP"))

    restored = SearchSessionArchive(tmp_path).latest()
    assert restored is not None
    assert restored["state"]["status"] == "OPERATOR_STOP"
    assert restored["state"]["graph"]["object_topology"]["nodes"][0]["node_id"] == "obj_007"


def test_graph_file_is_rewritten_once_per_revision_and_never_per_event(
    tmp_path: Path,
) -> None:
    """§2.2 的判别性证据：**图的落盘次数跟着 revision 走，不跟着事件走。**

    旧形状把整份图塞进每一次 `webui_state.json` 写入里，于是一条会话跑到最后，
    图的序列化次数 = 事件数。这里用 mtime 分辨：revision 不变时反复 record()，
    图文件的 mtime 必须纹丝不动。
    """
    import os
    import time as _time

    sid = "search_20260821_140000_rev00001"
    archive = SearchSessionArchive(tmp_path)
    archive.begin(sid, {"task_text": "找垃圾桶"})
    archive.record_graph(sid, _graph(1))
    graph_path = tmp_path / sid / "webui_graph.json"
    first_mtime = graph_path.stat().st_mtime_ns

    # 一串事件：图不许被重写。其中掺一份**自带 graph 键**的快照 —— 那是服务
    # 重启回填路径的形状；record() 必须把它摘掉，而不是跟着写一份旧图。
    for index in range(20):
        extra = {"graph": _graph(99), "graph_revision": 99} if index == 5 else {}
        archive.record({**_snapshot(sid, index), **extra})
        _time.sleep(0.001)
    assert graph_path.stat().st_mtime_ns == first_mtime, (
        "revision 没变却重写了图文件 —— 又回到「每个事件整图落盘」了"
    )
    assert archive.load(sid)["state"]["graph"]["revision"] == 1, (
        "快照里夹带的图不许覆盖单独的图文件"
    )

    # revision 变了才重写，并且落盘的是新的那一份。
    _time.sleep(0.01)
    archive.record_graph(sid, _graph(9))
    assert graph_path.stat().st_mtime_ns > first_mtime
    loaded = archive.load(sid)
    assert loaded["state"]["graph"]["revision"] == 9
    assert os.path.getsize(graph_path) > 0


def test_legacy_budget_failure_is_normalized_on_load(tmp_path: Path) -> None:
    sid = "search_20260821_135000_budget01"
    archive = SearchSessionArchive(tmp_path)
    archive.begin(sid, {"task_text": "找垃圾桶"})
    state = _snapshot(sid, 1, "FAILED")
    state.update({
        "result": "MAX_STEPS_REACHED",
        "finish_reason": "MAX_STEPS_REACHED",
        "error": {"message": "MAX_STEPS_REACHED"},
    })
    archive.record(state)

    loaded = archive.load(sid)
    assert loaded is not None
    assert loaded["state"]["status"] == "FINISHED"
    assert loaded["state"]["error"] is None
    assert archive.list(1)[0]["status"] == "FINISHED"


def test_legacy_retry_event_is_not_restored_as_terminal_error(tmp_path: Path) -> None:
    sid = "search_20260821_135500_retry001"
    archive = SearchSessionArchive(tmp_path)
    archive.begin(sid, {"task_text": "找垃圾桶"})
    state = _snapshot(sid, 0, "FAILED")
    state.update({
        "result": "FAILED",
        "finish_reason": "",
        "error": {
            "source": "observer_retry",
            "message": "vision request timed out",
        },
    })
    archive.record(state)

    loaded = archive.load(sid)
    assert loaded is not None
    assert loaded["state"]["status"] == "FINISHED"
    assert loaded["state"]["result"] == "INTERRUPTED_DURING_RETRY"
    assert loaded["state"]["error"] is None
    assert archive.list(1)[0]["status"] == "FINISHED"


def test_service_keeps_terminal_snapshot_in_history_without_resurrecting_live_session(
    tmp_path: Path,
) -> None:
    sid = "search_20260821_140000_1234abcd"
    archive = SearchSessionArchive(tmp_path)
    archive.begin(sid, {"task_text": "找红色椅子", "backend": "mock"})
    archive.record_graph(sid, _graph(4))
    state = _snapshot(sid, 4, "OPERATOR_STOP")
    state.update({
        "task": {"raw_text": "找红色椅子", "canonical_target": "红色椅子"},
        "backend": "mock", "reasoner": "semantic", "started_at": 1.0,
        "finished_at": 5.0, "objects": {"current": [], "session_seen": [],
                                          "target_evidence": {}},
    })
    archive.record(state)

    service = SearchSessionService(owner=ControlOwner(), session_dir=str(tmp_path))
    restored = service.state_snapshot()
    assert restored["session_id"] == ""
    assert restored["status"] == "IDLE"
    assert restored["graph"] is None, (
        "重启后 live 图必须是空的 —— 旧会话的图只许从历史里看"
    )
    history = service.history()
    assert history and history[0]["session_id"] == sid
    archived = service.history_session(sid)
    assert archived is not None
    assert archived["state"]["status"] == "OPERATOR_STOP"
    assert archived["state"]["graph"]["object_topology"]["nodes"]


def test_terminal_worker_is_not_reported_as_live_executor(tmp_path: Path) -> None:
    class DeadExecutor:
        def status(self):
            return {"state": "running", "session_id": "terminal"}

        def alive(self):
            return False

    service = SearchSessionService(
        owner=ControlOwner(), session_dir=str(tmp_path / "sessions")
    )
    service._executor = DeadExecutor()  # type: ignore[assignment]

    assert service.executor_state() == {
        "state": "stopped", "session_id": None, "alive": False
    }
