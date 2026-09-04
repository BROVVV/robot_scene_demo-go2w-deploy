"""WebUI 链路契约：AutonomousExplorer 实际使用的 SemanticNavigationGraph 也要
输出 object_topology（Live 前端真机/Mock 都是走这个图，而非 SemanticEntityGraph）。

这锁住计划书最终目标：WebUI 「语义拓扑」页读取
state.spatial.semantic_graph.object_topology 能拿到持久物体关系。
"""

from __future__ import annotations

from app.live_robot.search_event import MAP_UPDATED, SearchEvent
from app.live_robot.search_state_store import SearchStateStore
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph


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


def test_adapter_graph_reaches_web_contract():
    """store.apply(MAP_UPDATED, semantic_navigation_graph) -> spatial.semantic_graph.object_topology"""
    g = _two_frame_graph()
    graph = g.to_dict()
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    store.apply(SearchEvent(
        event_id=1, session_id="s1", timestamp=1.0,
        event_type=MAP_UPDATED,
        payload={"graph": graph, "revision": graph["revision"]},
    ))
    spatial = store.spatial_snapshot()
    sg = spatial.get("semantic_graph") or {}
    assert sg.get("schema_version") == "semantic_navigation_graph_v1"
    ot = sg.get("object_topology") or {}
    assert ot.get("schema_version") == "semantic_object_topology_v1"
    assert len(ot.get("nodes") or []) == 4
    assert len(ot.get("edges") or []) == 5


# --------------------------------------------------------------------------- #
# 适配器层：走真实 ExplorerSearchAdapter，锁住 explorer 事件 → store 拓扑    #
# --------------------------------------------------------------------------- #

def test_adapter_memory_update_carries_topology_to_store():
    from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter
    from app.live_robot.search_event_bus import SearchEventBus

    g = _two_frame_graph()
    graph = g.to_dict()

    bus = SearchEventBus()
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    adapter = ExplorerSearchAdapter(bus, store, session_id="s1")

    # memory_update 事件：与 AutonomousExplorer._emit("memory_update", ...) 同构
    adapter.on_explorer_event({
        "event": "memory_update",
        "state": "UPDATE_MEMORY",
        "session_id": "s1",
        "node_id": "P1",
        "semantic_navigation_graph": graph,
    })

    spatial = store.spatial_snapshot()
    sg = spatial.get("semantic_graph") or {}
    assert sg.get("schema_version") == "semantic_navigation_graph_v1"
    ot = sg.get("object_topology") or {}
    assert ot.get("schema_version") == "semantic_object_topology_v1"
    assert len(ot.get("nodes") or []) == 4
    assert len(ot.get("edges") or []) == 5
    assert any(
        e["relation"] == "near" and e["observation_count"] == 2 and e["status"] == "CONFIRMED"
        for e in (ot.get("edges") or [])
    )

    # observation 事件（ExplorationGraph，无 object_topology）不得覆盖语义拓扑
    adapter.on_explorer_event({
        "event": "observation",
        "state": "OBSERVE",
        "session_id": "s1",
        "bundle_id": "bundle_9",
        "graph": {"session_id": "s1", "nodes": [], "edges": []},
    })
    sg_again = (store.spatial_snapshot().get("semantic_graph") or {})
    ot_again = sg_again.get("object_topology") or {}
    assert len(ot_again.get("nodes") or []) == 4, "observation 不得覆盖 object_topology"
