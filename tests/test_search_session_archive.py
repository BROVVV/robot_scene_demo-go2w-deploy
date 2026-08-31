from __future__ import annotations

from pathlib import Path

from app.live_robot.search_event import ERROR, EventIdAllocator, make_event
from app.manual_web_demo.search_session_archive import SearchSessionArchive
from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_session_service import SearchSessionService


def _snapshot(session_id: str, index: int, status: str = "FINISHED") -> dict:
    return {
        "session_id": session_id,
        "status": status,
        "result": status,
        "target": f"目标 {index}",
        "cycle": index,
        "elapsed_seconds": float(index),
        "map": {"nodes": [{"node_id": f"n{index}"}], "edges": []},
        "spatial": {
            "semantic_graph": {
                "object_topology": {
                    "nodes": [{"node_id": f"obj_{index:03d}"}],
                    "edges": [],
                }
            }
        },
        "decisions": [{"decision_id": f"d{index}"}],
        "timeline": [],
    }


def test_archive_preserves_full_state_and_events(tmp_path: Path) -> None:
    archive = SearchSessionArchive(tmp_path, max_sessions=10)
    sid = "search_20260821_120000_deadbeef"
    archive.begin(sid, {"task_text": "寻找蓝色垃圾桶", "target": "蓝色垃圾桶"})
    event = make_event(
        allocator=EventIdAllocator(), session_id=sid, event_type=ERROR,
        payload={"code": "CAMERA_UNAVAILABLE", "message": "frame stale"},
    )
    state = _snapshot(sid, 1, "FAILED")
    state["error"] = event.payload
    archive.record(state, event)

    loaded = archive.load(sid)
    assert loaded is not None
    assert loaded["state"]["spatial"]["semantic_graph"]["object_topology"]["nodes"]
    assert loaded["state"]["decisions"][0]["decision_id"] == "d1"
    assert loaded["events"][0]["payload"]["code"] == "CAMERA_UNAVAILABLE"


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
    first.record(_snapshot(sid, 7, "OPERATOR_STOP"))

    restored = SearchSessionArchive(tmp_path).latest()
    assert restored is not None
    assert restored["state"]["status"] == "OPERATOR_STOP"
    assert restored["state"]["map"]["nodes"][0]["node_id"] == "n7"


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


def test_service_restores_last_terminal_snapshot_after_restart(tmp_path: Path) -> None:
    sid = "search_20260821_140000_1234abcd"
    archive = SearchSessionArchive(tmp_path)
    archive.begin(sid, {"task_text": "找红色椅子", "backend": "mock"})
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
    assert restored["session_id"] == sid
    assert restored["status"] == "OPERATOR_STOP"
    assert restored["spatial"]["semantic_graph"]["object_topology"]["nodes"]
