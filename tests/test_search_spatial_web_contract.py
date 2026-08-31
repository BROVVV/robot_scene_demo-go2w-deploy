"""Web contract tests (plan §19.8).

Verifies that /api/search/state / WebSocket snapshots expose the persistent
semantic graph, route plan, startup lifecycle and next-motion data required by
the WebUI.
"""

from __future__ import annotations

from app.live_robot.search_event import (
    DECISION_RECORDED,
    MAP_UPDATED,
    SEARCH_STATE_CHANGED,
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


def test_snapshot_has_semantic_graph_and_route():
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    graph = {
        "schema_version": "semantic_entity_graph_v1",
        "revision": 3,
        "current_place_id": "P2",
        "nodes": [
            {"node_id": "P1", "node_type": "PLACE"},
            {"node_id": "obj_001", "node_type": "OBJECT", "label": "桌"},
        ],
        "edges": [
            {"edge_id": "P1__observed_from__obj_001", "relation": "OBSERVED_FROM"},
        ],
        "places": [{"place_id": "P1"}],
        "objects": [{"object_id": "obj_001", "label": "桌", "status": "CONFIRMED"}],
        "frontiers": [],
        "route_plan": {
            "route_id": "route_001", "frame_id": "map", "target_type": "FRONTIER_CANDIDATE",
            "target_id": "F1", "target_position": [1.0, 1.0],
            "waypoints": [[0.0, 0.0]], "reachable": True,
            "planner_source": "grid_astar", "cost_components": {},
        },
    }
    store.apply(_event(MAP_UPDATED, {"graph": graph}))
    spatial = store.spatial_snapshot()
    assert spatial["semantic_graph"] is not None
    assert spatial["route_plan"]["route_id"] == "route_001"


def test_snapshot_has_last_decision_and_next_motion():
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    decision = {
        "decision_id": "D00017",
        "cycle": 5,
        "reason_zh": "目标尚未确认，选择信息增益最高的前沿。",
        "score": 0.73,
        "score_breakdown": {"semantic_relevance": 0.81},
        "next_motion_command": {"instruction_zh": "右转 24°"},
        "alternatives": [{"candidate_id": "F2", "score": 0.48}],
    }
    store.apply(_event(DECISION_RECORDED, {"decision": decision}))
    snap = store.snapshot()
    assert snap["last_decision"]["decision_id"] == "D00017"
    assert snap["next_motion_command"]["instruction_zh"] == "右转 24°"


def test_snapshot_has_startup_stage():
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    startup = {
        "stage": "WAIT_RGBD",
        "stage_started_at": 1.0,
        "last_progress_at": 1.0,
        "worker_alive": True,
        "worker_state": "starting",
    }
    store.apply(_event(SEARCH_STATE_CHANGED, {"startup": startup}))
    snap = store.snapshot()
    assert snap["startup"]["stage"] == "WAIT_RGBD"
    assert snap["startup"]["worker_alive"] is True


def test_start_btn_state_gates_active():
    """The WebUI should not allow a second start while STARTING/RUNNING."""
    active = {"STARTING", "RUNNING", "PAUSED", "STOPPING"}
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    store.apply(_event(SEARCH_STATE_CHANGED, {"phase": "STARTING"}))
    # The service-level gate already rejects a second start; verify that the
    # snapshot reflects STARTING so the front-end disables the button.
    assert store.snapshot()["status"] == "STARTING"


def test_snapshot_has_object_topology_projection():
    """Plan §35 / §38: spatial.semantic_graph.object_topology must exist.

    The WebUI semantic-topology view reads
    ``state.spatial.semantic_graph.object_topology`` directly - no second state
    store, no extra endpoint.
    """
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    graph = {
        "schema_version": "semantic_entity_graph_v1",
        "revision": 7,
        "nodes": [],
        "edges": [],
        "places": [],
        "objects": [],
        "frontiers": [],
        "object_topology": {
            "schema_version": "semantic_object_topology_v1",
            "revision": 7,
            "generated_at": 1.0,
            "nodes": [
                {"node_id": "obj_001", "node_type": "OBJECT", "label": "桌",
                 "status": "CONFIRMED", "confidence": 0.9, "observation_count": 4}
            ],
            "edges": [
                {"edge_id": "obj_001__near__obj_002", "from": "obj_001", "to": "obj_002",
                 "relation": "near", "relation_scope": "STRUCTURAL", "directed": False,
                 "status": "CONFIRMED", "confidence": 0.8, "observation_count": 2}
            ],
            "stats": {"node_count": 1, "edge_count": 1, "connected_components": 1},
        },
        "route_plan": None,
    }
    store.apply(_event(MAP_UPDATED, {"graph": graph}))
    spatial = store.spatial_snapshot()
    assert spatial["semantic_graph"] is not None
    topology = spatial["semantic_graph"].get("object_topology")
    assert topology is not None
    assert topology["schema_version"] == "semantic_object_topology_v1"
    assert isinstance(topology["nodes"], list)
    assert isinstance(topology["edges"], list)
    assert topology["revision"] == 7
