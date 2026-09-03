"""Tests for spatial event handling in SearchStateStore."""

from __future__ import annotations

from app.live_robot.search_event import (
    FRONTIERS_UPDATED,
    LONG_TERM_GOAL_SELECTED,
    MAP_UPDATED,
    PLACE_CREATED,
    SearchEvent,
)
from app.live_robot.search_state_store import SearchStateStore


def _event(event_type: str, payload: dict, event_id: int = 1) -> SearchEvent:
    return SearchEvent(
        event_id=event_id,
        session_id="s1",
        timestamp=1.0,
        event_type=event_type,
        payload=payload,
    )


def test_spatial_snapshot_updates():
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    store.apply(_event(FRONTIERS_UPDATED, {"frontiers": [{"frontier_id": "F1"}]}))
    store.apply(_event(PLACE_CREATED, {"place": {"place_id": "P1"}}))
    store.apply(_event(LONG_TERM_GOAL_SELECTED, {"intent": {"intent_id": "i1"}}))
    spatial = store.spatial_snapshot()
    assert spatial["frontiers"][0]["frontier_id"] == "F1"
    assert spatial["place_graph"]["places"][0]["place_id"] == "P1"
    assert spatial["long_term_goal"]["intent_id"] == "i1"


def test_semantic_map_projection_preserves_canonical_current_place():
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    graph = {
        "schema_version": "semantic_navigation_graph_v1",
        "current_place_id": "P1",
        "places": [{"place_id": "P1"}],
        "nodes": [{"node_id": "P1", "node_type": "PLACE"}],
        "edges": [],
    }
    store.apply(_event(MAP_UPDATED, {"graph": graph}, event_id=2))
    spatial = store.spatial_snapshot()
    assert spatial["place_graph"]["current_place_id"] == "P1"
    assert (spatial["semantic_graph"] or {})["current_place_id"] == "P1"


def test_object_topology_reaches_the_webui_next_to_the_internal_nav_graph():
    """计划书 §7.3：内部导航图保留 Place/Frontier，语义拓扑只投影对象。"""
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    graph = {
        "schema_version": "semantic_navigation_graph_v1",
        "revision": 5,
        "current_place_id": "P1",
        "places": [{"place_id": "P1"}],
        "frontiers": [{"frontier_id": "F01", "label": "F01"}],
        "objects": [{"object_id": "obj_001", "label": "办公桌"}],
        "nodes": [
            {"node_id": "P1", "node_type": "PLACE"},
            {"node_id": "F01", "node_type": "FRONTIER"},
            {"node_id": "obj_001", "node_type": "OBJECT", "label": "办公桌"},
        ],
        "edges": [
            {"from": "P1", "to": "obj_001", "relation": "OBSERVED_FROM"},
            {"from": "P1", "to": "F01", "relation": "FRONTIER_TO"},
        ],
        "object_topology": {
            "schema_version": "semantic_object_topology_v1",
            "revision": 5,
            "nodes": [{"node_id": "obj_001", "node_type": "OBJECT",
                       "label": "办公桌", "status": "CONFIRMED"}],
            "edges": [],
            "stats": {"node_count": 1, "edge_count": 0},
        },
    }
    store.apply(_event(MAP_UPDATED, {"graph": graph}, event_id=2))
    spatial = store.spatial_snapshot()

    topology = (spatial["semantic_graph"] or {})["object_topology"]
    assert topology["schema_version"] == "semantic_object_topology_v1"
    assert [node["node_id"] for node in topology["nodes"]] == ["obj_001"]
    assert topology["edges"] == []

    # 内部导航图不许被删掉：规划器和 Place/Frontier 端点仍然要看到它们。
    internal_types = {node["node_type"] for node in spatial["semantic_graph"]["nodes"]}
    assert internal_types == {"PLACE", "FRONTIER", "OBJECT"}
    assert spatial["place_graph"]["current_place_id"] == "P1"
    assert spatial["frontiers"][0]["frontier_id"] == "F01"
