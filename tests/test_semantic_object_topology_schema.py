"""Plan §32 tests: object_topology projection schema.

Checks the projection contains only persistent OBJECT nodes with stable ids,
its edges reference existing nodes, and the revision stays in sync with the
main graph revision (no second, desynchronised revision counter).
"""

from __future__ import annotations

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.models import SpatialPose
from app.spatial.semantic_entity_graph import (
    OBJECT_TOPOLOGY_SCHEMA_VERSION,
    SemanticEntityGraph,
)


def _obs(label, object_id, map_xyz, frame_id, place_id):
    return ObjectSpatialObservation(
        object_id=object_id, label=label, map_xyz=map_xyz,
        spatial_quality="METRIC_RGBD", confidence=0.9,
        provenance={"frame_id": frame_id, "place_id": place_id, "observation_id": f"obs_{frame_id}"},
    )


def _build_topology() -> SemanticEntityGraph:
    graph = SemanticEntityGraph()
    p1, _ = graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["桌", "桶"],
        pose=SpatialPose(x=0.0, y=0.0), timestamp=1.0,
    )
    obs = [
        _obs("桌", "semantic_obj_001", (1.0, 0.0, 0.0), "f1", p1),
        _obs("桶", "semantic_obj_002", (1.2, 0.0, 0.0), "f1", p1),
    ]
    result = graph.object_map.update_with_associations(obs, place_id=p1, frame_id="f1", now=1.0)
    graph.sync_from_observation(
        observation_id="bundle_1", heading_sector=0, labels=["桌", "桶"],
        spatial_objects=obs, pose=SpatialPose(x=0.0, y=0.0), timestamp=1.0,
        place_id=p1, update_result=result,
        relations=[{
            "subject_id": "semantic_obj_001", "object_id": "semantic_obj_002",
            "relation": "near", "confidence": 0.9,
        }],
    )
    return graph


def test_object_topology_schema_contract():
    graph = _build_topology()
    topology = graph.object_topology_snapshot()
    assert topology["schema_version"] == OBJECT_TOPOLOGY_SCHEMA_VERSION
    assert isinstance(topology["revision"], int)
    assert topology["revision"] == graph.revision
    assert isinstance(topology["generated_at"], (int, float))
    assert "nodes" in topology and isinstance(topology["nodes"], list)
    assert "edges" in topology and isinstance(topology["edges"], list)
    assert "stats" in topology


def test_nodes_are_persistent_objects_only():
    topology = _build_topology().object_topology_snapshot()
    assert len(topology["nodes"]) == 2
    for node in topology["nodes"]:
        assert node["node_id"].startswith("obj_")
        assert node["node_type"] == "OBJECT"
        assert "label" in node
        assert "status" in node
        assert "observation_count" in node


def test_edge_endpoints_exist_in_nodes():
    topology = _build_topology().object_topology_snapshot()
    node_ids = {node["node_id"] for node in topology["nodes"]}
    assert len(topology["edges"]) == 1
    for edge in topology["edges"]:
        assert edge["from"] in node_ids
        assert edge["to"] in node_ids
        assert "relation" in edge
        assert "relation_scope" in edge
        assert "observation_count" in edge


def test_no_place_nodes_in_projection():
    """Plan §25 / §28: semantic topology must never mix Places in."""
    graph = _build_topology()
    topology = graph.object_topology_snapshot()
    types = {node["node_type"] for node in topology["nodes"]}
    assert "PLACE" not in types
    assert types == {"OBJECT"}


def test_stats_are_consistent():
    topology = _build_topology().object_topology_snapshot()
    stats = topology["stats"]
    assert stats["node_count"] == len(topology["nodes"])
    assert stats["edge_count"] == len(topology["edges"])
    assert isinstance(stats["connected_components"], int)
    assert stats["connected_components"] >= 1


def test_revision_stays_in_sync_with_main_graph():
    graph = _build_topology()
    top1 = graph.object_topology_snapshot()
    assert top1["revision"] == graph.revision
    graph.set_route_plan({"route_id": "r1"})
    # a route plan change bumps the main revision; the projection must follow.
    assert graph.object_topology_snapshot()["revision"] == graph.revision
