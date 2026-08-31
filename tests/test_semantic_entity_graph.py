"""Tests for SemanticEntityGraph (plan §19.4)."""

from __future__ import annotations

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.models import SpatialPose
from app.spatial.semantic_entity_graph import SemanticEntityGraph


def _obs(label, map_xyz, frame, place):
    return ObjectSpatialObservation(
        object_id=None, label=label, map_xyz=map_xyz,
        spatial_quality="METRIC_RGBD", confidence=0.9,
        provenance={"frame_id": frame, "place_id": place,
                    "observation_id": f"obs_{frame}"},
    )


def test_graph_has_place_object_nodes_and_edges():
    graph = SemanticEntityGraph()
    eg = graph.place_graph
    # Create two places
    place1, _ = eg.register_observation(
        observation_id="obs_0", heading_sector=0, objects=["桌"],
        pose=SpatialPose(x=0.0, y=0.0),
    )
    place2, _ = eg.register_observation(
        observation_id="obs_1", heading_sector=0, objects=[],
        pose=SpatialPose(x=1.0, y=0.0),
    )
    # Update with persistent objects
    result = graph.object_map.update_with_associations(
        [_obs("桌", (1.0, 0.5, 0.0), "f1", place1)],
        place_id=place1, frame_id="f1", now=1.0,
    )
    object_map = graph.object_map
    graph.sync_from_observation(
        observation_id="obs_0",
        heading_sector=0,
        labels=["桌"],
        spatial_objects=[_obs("桌", (1.0, 0.5, 0.0), "f1", place1)],
        pose=SpatialPose(x=0.0, y=0.0),
        timestamp=1.0,
        place_id=place1,
        update_result=result,
    )
    snap = graph.snapshot()
    assert snap["schema_version"] == "semantic_entity_graph_v1"
    types = {node["node_type"] for node in snap["nodes"]}
    assert "PLACE" in types
    assert "OBJECT" in types
    relations = {edge["relation"] for edge in snap["edges"]}
    # Object was observed at place1 -> OBSERVED_FROM edge
    assert any(
        edge["relation"] == "OBSERVED_FROM"
        for edge in snap["edges"]
    )


def test_moved_to_edge_from_place_graph():
    graph = SemanticEntityGraph()
    eg = graph.place_graph
    eg.register_observation(
        observation_id="obs_0", heading_sector=0, objects=[],
        pose=SpatialPose(x=0.0, y=0.0),
    )
    eg.register_observation(
        observation_id="obs_1", heading_sector=0, objects=[],
        pose=SpatialPose(x=1.5, y=0.0),
    )
    snap = graph.snapshot()
    assert any(
        edge["relation"] == "MOVED_TO"
        for edge in snap["edges"]
    )


def test_observed_object_ids_are_persistent_ids():
    """Plan §6.4: Place observed_object_ids must hold persistent obj ids."""
    graph = SemanticEntityGraph()
    eg = graph.place_graph
    place1, _ = eg.register_observation(
        observation_id="obs_0", heading_sector=0, objects=["桌"],
        pose=SpatialPose(x=0.0, y=0.0),
    )
    result = graph.object_map.update_with_associations(
        [_obs("桌", (1.0, 0.5, 0.0), "f1", place1)],
        place_id=place1, frame_id="f1", now=1.0,
    )
    oid = result.created_ids[0]
    graph.sync_from_observation(
        observation_id="obs_0", heading_sector=0, labels=["桌"],
        spatial_objects=[_obs("桌", (1.0, 0.5, 0.0), "f1", place1)],
        pose=SpatialPose(x=0.0, y=0.0),
        timestamp=1.0, place_id=place1, update_result=result,
    )
    assert oid in graph.place_graph.places[place1].observed_object_ids
    assert oid.startswith("obj_")


def test_route_plan_in_snapshot():
    graph = SemanticEntityGraph()
    graph.set_route_plan({
        "route_id": "r1", "frame_id": "map", "target_type": "FRONTIER_CANDIDATE",
        "target_id": "F1", "target_position": [1.0, 1.0],
        "waypoints": [[0.0, 0.0]], "reachable": True,
        "planner_source": "grid_astar", "cost_components": {},
    })
    snap = graph.snapshot()
    assert snap["route_plan"]["route_id"] == "r1"


def test_summary_stats():
    graph = SemanticEntityGraph()
    eg = graph.place_graph
    eg.register_observation(
        observation_id="obs_0", heading_sector=0, objects=["桌"],
        pose=SpatialPose(x=0.0, y=0.0),
    )
    stats = graph.summary_stats()
    assert stats["unique_places"] == 1
