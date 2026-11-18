"""WebUI 链路契约：AutonomousExplorer 实际使用的 SemanticNavigationGraph 也要
输出 object_topology（Live 前端真机/Mock 都是走这个图，而非 SemanticEntityGraph）。

这锁住计划书最终目标：WebUI 「语义拓扑」页读取 `state.graph.object_topology`
能拿到持久物体关系。

**2026-10-08 §2.1 之后**：图不再搭在 `memory_update` 事件的
`semantic_navigation_graph` 字段上，而是走 worker stdout 的 `graph_snapshot`
消息 → `ExplorerSearchAdapter.on_graph_snapshot()`。本文件的两条适配器用例
随之改走这条真实入口。
"""

from __future__ import annotations

from app.live_robot.search_event import MAP_UPDATED, SearchEvent
from app.live_robot.search_state_store import SearchStateStore
from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph
from app.spatial.semantic_object_map import SemanticObjectMap


def _scene_item(obj_id: str, label: str, xyz: tuple):
    return {"id": obj_id, "label": label, "map_xyz": list(xyz), "confidence": 0.9}


def _two_frame_graph() -> SemanticNavigationGraph:
    g = SemanticNavigationGraph(relation_confirm_min_observations=2)
    g.update_observation(
        observation_id="bundle_1", heading_sector=0,
        scene_objects=[
            _scene_item("semantic_obj_001", "办公桌", (1.0, 0.0, 0.0)),
            _scene_item("semantic_obj_002", "绿色垃圾桶", (1.2, 0.0, 0.0)),
            _scene_item("semantic_obj_003", "办公椅", (0.2, 0.1, 0.0)),
        ],
        scene_relations=[
            {"subject_id": "semantic_obj_001", "object_id": "semantic_obj_002", "relation": "near", "confidence": 0.7},
            {"subject_id": "semantic_obj_003", "object_id": "semantic_obj_001", "relation": "left_of", "confidence": 0.8},
        ],
        pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0}, timestamp=1.0,
    )
    g.update_observation(
        observation_id="bundle_2", heading_sector=1,
        scene_objects=[
            _scene_item("semantic_obj_010", "办公桌", (1.02, 0.0, 0.0)),
            _scene_item("semantic_obj_011", "绿色垃圾桶", (1.18, 0.0, 0.0)),
            _scene_item("semantic_obj_012", "办公椅", (0.22, 0.1, 0.0)),
        ],
        scene_relations=[
            {"subject_id": "semantic_obj_010", "object_id": "semantic_obj_011", "relation": "near", "confidence": 0.8},
        ],
        pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.2}, timestamp=2.0,
    )
    return g


def test_semantic_navigation_graph_outputs_object_topology():
    g = _two_frame_graph()
    d = g.to_dict()
    assert d["schema_version"] == "semantic_navigation_graph_v1"
    assert "object_topology" in d
    ot = d["object_topology"]
    assert ot["schema_version"] == "semantic_object_topology_v1"
    # 3 个持久物体 + 当前 place P1（P1 是中转站，拓扑才连成一张图）
    assert len(ot["nodes"]) == 4
    assert {n["node_id"] for n in ot["nodes"]} == {"obj_001", "obj_002", "obj_003", "P1"}
    # 2 条物体关系 + 3 条 P1 -> 物体的 OBSERVED_FROM
    assert len(ot["edges"]) == 5
    near = [e for e in ot["edges"] if e["relation"] == "near"]
    assert len(near) == 1
    assert near[0]["observation_count"] == 2
    assert near[0]["status"] == "CONFIRMED"
    # 跨帧 frame id 变化仍映射到同一对 persistent 物体
    assert near[0]["from"].startswith("obj_") and near[0]["to"].startswith("obj_")


def test_graph_reuses_the_observers_object_associations_without_second_ingest():
    objects = SemanticObjectMap()
    update = objects.update_with_associations(
        [ObjectSpatialObservation(
            object_id="frame_bin", label="垃圾桶",
            map_xyz=(2.0, 1.0, 0.0), spatial_quality="METRIC_LIDAR",
            confidence=0.9,
            provenance={"map_frame": "pslam_map", "observation_id": "bundle_1"},
        )],
        place_id="P1", now=1.0, frame_id="camera_1",
    )
    graph = SemanticNavigationGraph(object_map=objects)
    result = graph.update_observation(
        observation_id="bundle_1", heading_sector=0,
        scene_objects=[_scene_item("frame_bin", "垃圾桶", (2.0, 1.0, 0.0))],
        scene_relations=[], pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
        timestamp=1.0, object_update_result=update,
    )
    assert len(objects.objects) == 1
    entry = next(iter(objects.objects.values()))
    assert entry.observation_count == 1
    assert entry.provenance["map_frame"] == "pslam_map"
    assert result["frame_object_ids"]["frame_bin"] == entry.object_id


def test_store_carries_the_web_snapshot_verbatim():
    """`set_graph(web_snapshot())` → `state.graph.object_topology` 原样可读。"""
    graph = _two_frame_graph().web_snapshot()
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    assert store.set_graph(graph) is True
    sg = store.snapshot()["graph"]
    assert sg.get("schema_version") == "semantic_navigation_graph_v1"
    ot = sg.get("object_topology") or {}
    assert ot.get("schema_version") == "semantic_object_topology_v1"
    assert len(ot.get("nodes") or []) == 4
    assert len(ot.get("edges") or []) == 5


# --------------------------------------------------------------------------- #
# 适配器层：走真实 ExplorerSearchAdapter，锁住 explorer 事件 → store 拓扑    #
# --------------------------------------------------------------------------- #

def test_adapter_graph_snapshot_is_the_only_way_the_graph_reaches_the_store():
    """`on_graph_snapshot()` → store 一份图 + 总线上一条轻量 MAP_UPDATED。"""
    from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter
    from app.live_robot.search_event_bus import SearchEventBus

    graph = _two_frame_graph().web_snapshot()

    bus = SearchEventBus()
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    adapter = ExplorerSearchAdapter(bus, store, session_id="s1")

    seen: list[SearchEvent] = []
    bus.subscribe(seen.append)
    assert adapter.on_graph_snapshot(graph) is True

    sg = store.snapshot()["graph"] or {}
    assert sg.get("schema_version") == "semantic_navigation_graph_v1"
    ot = sg.get("object_topology") or {}
    assert ot.get("schema_version") == "semantic_object_topology_v1"
    assert len(ot.get("nodes") or []) == 4
    assert len(ot.get("edges") or []) == 5
    assert any(
        e["relation"] == "near" and e["observation_count"] == 2 and e["status"] == "CONFIRMED"
        for e in (ot.get("edges") or [])
    )

    # 总线上的 MAP_UPDATED 只带 revision —— 图不许搭事件走（§2.1）。
    map_events = [e for e in seen if e.event_type == MAP_UPDATED]
    assert len(map_events) == 1, f"每次图版本变化应当恰好一条 MAP_UPDATED：{seen}"
    assert map_events[0].payload == {"revision": graph["revision"]}

    # 普通 explorer 事件（哪怕自带 graph 字段）不得改图
    adapter.on_explorer_event({
        "event": "observation",
        "state": "OBSERVE",
        "session_id": "s1",
        "bundle_id": "bundle_9",
        "graph": {"session_id": "s1", "nodes": [], "edges": []},
    })
    ot_again = (store.snapshot()["graph"] or {}).get("object_topology") or {}
    assert len(ot_again.get("nodes") or []) == 4, "observation 不得覆盖 object_topology"


def test_adapter_rejects_an_older_graph_snapshot():
    """乱序到达的旧图既不改 store，也不发 MAP_UPDATED —— 页面不会被拽回去。"""
    from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter
    from app.live_robot.search_event_bus import SearchEventBus

    bus = SearchEventBus()
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    adapter = ExplorerSearchAdapter(bus, store, session_id="s1")

    seen: list[SearchEvent] = []
    bus.subscribe(seen.append)
    newest = _two_frame_graph().web_snapshot()
    old = dict(newest, revision=newest["revision"] - 1, objects=[])
    assert adapter.on_graph_snapshot(newest) is True
    assert adapter.on_graph_snapshot(old) is False
    assert (store.snapshot()["graph"] or {}).get("objects"), "旧图不许抹掉物件"
    assert sum(1 for e in seen if e.event_type == MAP_UPDATED) == 1
