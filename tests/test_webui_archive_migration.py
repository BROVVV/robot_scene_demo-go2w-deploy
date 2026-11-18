"""v1 归档一次性迁移：抽出图、删掉状态里的重复图字段、原文件先备份。

计划书 §2.2 要求 v1 归档做**一次性**迁移，并且运行时**不做**静默兼容 ——
所以迁移脚本的正确性只能在这里验：它动的是历史归档，现场跑之前不会有人
先拿真数据试。

判别点（每一条都能区分"真迁好了"和"看起来迁好了"）：

1. 图进了 ``webui_graph.json``，且是 §2.1 的 Web 图键集合（``nodes`` /
   ``observations`` 不进去 —— 前者是 places+objects+frontiers 的重复，
   后者是离线证据）。
2. 状态里 ``map`` 与五个重复 spatial 键都不见了，别的 spatial 键（RGB-D、
   位姿、map_health…）**一个都不能少** —— 迁移不是清空状态。
3. ``events.jsonl`` / ``webui_events.jsonl`` 是原始证据，逐字节不动。
4. 备份里有迁移前的 ``webui_state.json``。
5. 迁移后的目录能被运行时读口（``SearchSessionArchive.load``）读出图 revision。
6. 幂等：再跑一次不再投影，也不覆盖已经迁移好的图。
7. 没有图可提取的会话不造空图（§6：不虚构默认值）。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from app.manual_web_demo.search_session_archive import SearchSessionArchive
from scripts.go2w import migrate_webui_archive_v1 as migrate

SESSION = "search_20260101_000000_aaaa1111"


def _graph(*, revision: int = 7) -> dict:
    return {
        "schema_version": "semantic_navigation_graph_v1",
        "revision": revision,
        "current_place_id": "P1",
        "places": [{"place_id": "P1"}],
        "objects": [{"object_id": "o1"}],
        "frontiers": [{"frontier_id": "f1"}],
        "edges": [{"from": "P1", "to": "o1"}],
        # 这两个键是 v1 多出来的：nodes 是前三个数组的并集，observations 是离线证据。
        "nodes": [{"node_id": "P1"}, {"node_id": "o1"}, {"node_id": "f1"}],
        "observations": [{"observation_id": "obs1"}],
        "object_topology": {"nodes": [], "edges": [], "stats": {}},
    }


def _v1_state(*, graph: dict | None = None) -> dict:
    graph = _graph() if graph is None else graph
    return {
        "session_id": SESSION,
        "status": "FINISHED",
        "target": "垃圾桶",
        # v1 把同一张图存了三份，外加从它再投影一遍的字段。
        "map": {"semantic_navigation_graph": graph, "nodes": graph["nodes"], "revision": 7},
        "spatial": {
            "rgbd_frame": None,
            "spatial_pose": {"x": 1.0, "y": 2.0},
            "map_health": {"state": "HEALTHY"},
            "semantic_graph": graph,
            "spatial_map": graph,
            "place_graph": {"places": graph["places"]},
            "semantic_objects": graph["objects"],
            "frontiers": graph["frontiers"],
        },
        "objects": {"current": [], "session_seen": [], "target_evidence": {}},
    }


def _make_archive(tmp_path: Path, *, state: dict | None = None) -> Path:
    root = tmp_path / "live_runs"
    session = root / SESSION
    session.mkdir(parents=True)
    (session / "webui_session.json").write_text(
        json.dumps({"session_id": SESSION, "status": "FINISHED"}), encoding="utf-8")
    (session / "webui_state.json").write_text(
        json.dumps(state if state is not None else _v1_state()), encoding="utf-8")
    (session / "webui_events.jsonl").write_text('{"event_type":"X"}\n', encoding="utf-8")
    (session / "events.jsonl").write_text('{"event":"raw"}\n', encoding="utf-8")
    return root


def _run(root: Path, backup: Path) -> int:
    argv = ["migrate", "--root", str(root), "--apply", "--backup-root", str(backup)]
    original = sys.argv
    sys.argv = argv
    try:
        return migrate.main()
    finally:
        sys.argv = original


def test_migration_splits_the_graph_out_of_the_state(tmp_path: Path) -> None:
    root = _make_archive(tmp_path)
    backup = tmp_path / "backup"
    assert _run(root, backup) == 0

    session = root / SESSION
    graph = json.loads((session / "webui_graph.json").read_text(encoding="utf-8"))
    # §2.1 的 Web 图键集合：多一个都是把旧形状带进了新文件。
    assert set(graph) == {
        "schema_version", "revision", "current_place_id",
        "places", "objects", "frontiers", "edges", "object_topology",
    }
    assert graph["revision"] == 7

    state = json.loads((session / "webui_state.json").read_text(encoding="utf-8"))
    assert "map" not in state
    for gone in ("semantic_graph", "spatial_map", "place_graph",
                 "semantic_objects", "frontiers"):
        assert gone not in state["spatial"], f"{gone} 没删掉"
    # 迁移只删重复图字段，独立的空间数据一个都不能少。
    assert state["spatial"]["rgbd_frame"] is None
    assert state["spatial"]["spatial_pose"] == {"x": 1.0, "y": 2.0}
    assert state["spatial"]["map_health"] == {"state": "HEALTHY"}
    assert state["objects"] == {"current": [], "session_seen": [], "target_evidence": {}}


def test_migration_backs_up_and_never_touches_the_raw_evidence(tmp_path: Path) -> None:
    root = _make_archive(tmp_path)
    backup = tmp_path / "backup"
    _run(root, backup)

    before = json.loads(
        (backup / SESSION / "webui_state.json").read_text(encoding="utf-8"))
    assert before["map"]["semantic_navigation_graph"]["revision"] == 7
    session = root / SESSION
    assert (session / "events.jsonl").read_text(encoding="utf-8") == '{"event":"raw"}\n'
    assert (session / "webui_events.jsonl").read_text(encoding="utf-8") == '{"event_type":"X"}\n'


def test_the_migrated_archive_reads_through_the_runtime_reader(tmp_path: Path) -> None:
    root = _make_archive(tmp_path)
    _run(root, tmp_path / "backup")
    detail = SearchSessionArchive(root, max_sessions=10).load(SESSION)
    assert detail is not None
    assert detail["state"]["graph_revision"] == 7
    assert detail["state"]["graph"]["objects"] == [{"object_id": "o1"}]
    # 新图文件不是"归档附件"，它由读口按名字排除。
    assert detail["artifacts"] == ["events.jsonl"]


def test_migration_is_idempotent(tmp_path: Path) -> None:
    root = _make_archive(tmp_path)
    _run(root, tmp_path / "backup1")
    graph_path = root / SESSION / "webui_graph.json"
    first = graph_path.read_text(encoding="utf-8")
    # 第二次跑：既不再投影，也不覆盖已经迁好的图。
    assert _run(root, tmp_path / "backup2") == 0
    assert graph_path.read_text(encoding="utf-8") == first
    assert not (tmp_path / "backup2" / SESSION).exists()


def test_a_session_without_a_graph_gets_no_fabricated_one(tmp_path: Path) -> None:
    """只有会话索引的启动即失败会话：清重复字段，但**不造**空图。"""
    state = _v1_state()
    state["map"] = {"semantic_navigation_graph": None}
    state["spatial"] = {key: value for key, value in state["spatial"].items()
                        if key not in ("semantic_graph", "spatial_map")}
    root = _make_archive(tmp_path, state=state)
    assert _run(root, tmp_path / "backup") == 0
    session = root / SESSION
    assert not (session / "webui_graph.json").exists()
    assert "map" not in json.loads((session / "webui_state.json").read_text(encoding="utf-8"))


def test_a_dry_run_writes_nothing(tmp_path: Path) -> None:
    root = _make_archive(tmp_path)
    before = (root / SESSION / "webui_state.json").read_text(encoding="utf-8")
    original = sys.argv
    sys.argv = ["migrate", "--root", str(root)]
    try:
        assert migrate.main() == 0
    finally:
        sys.argv = original
    assert (root / SESSION / "webui_state.json").read_text(encoding="utf-8") == before
    assert not (root / SESSION / "webui_graph.json").exists()
