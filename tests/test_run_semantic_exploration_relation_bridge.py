"""Plan §34 tests: runner bridges ``semantic.relations`` into the EntityGraph.

A source-level check ensures the live runner passes ``relations`` to
``sync_from_observation`` (the place where relation data used to be dropped),
and a functional check drives the full stack the runner uses
(SemanticObjectMap + SemanticEntityGraph + relations) to verify edges appear.
"""

from __future__ import annotations

from pathlib import Path

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.semantic_entity_graph import SemanticEntityGraph
from app.spatial.semantic_object_map import SemanticObjectMap


RUNNER = Path(__file__).resolve().parents[1] / "scripts/go2w/run_semantic_exploration.py"


def test_runner_source_passes_relations_to_sync():
    src = RUNNER.read_text(encoding="utf-8")
    assert "sync_from_observation(" in src
    assert "relations=list(getattr(semantic, \"relations\", None) or [])" in src
    # label-based identity must be gone from the topology path
    assert "persistent_id_for_label" not in src
    assert "persistent_by_label" not in src


def test_runner_stack_relations_flow_to_object_topology():
    """The exact stack the runner constructs keeps relation data alive."""
    semantic_map = SemanticObjectMap(merge_distance_m=0.4, confirm_min_observations=2)
    graph = SemanticEntityGraph(object_map=semantic_map)
    p1, _ = graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["桌", "桶"],
        pose=None, timestamp=1.0,
    )
    obs = [
        ObjectSpatialObservation(
            object_id="semantic_obj_001", label="桌", map_xyz=(1.0, 0.0, 0.0),
            spatial_quality="METRIC_RGBD", confidence=0.9,
            provenance={"frame_id": "f1", "place_id": p1},
        ),
        ObjectSpatialObservation(
            object_id="semantic_obj_002", label="桶", map_xyz=(1.2, 0.0, 0.0),
            spatial_quality="METRIC_RGBD", confidence=0.9,
            provenance={"frame_id": "f1", "place_id": p1},
        ),
    ]
    # runner calls this with semantic_map.update_with_associations(...)
    result = semantic_map.update_with_associations(
        obs, place_id=p1, now=1.0, frame_id="f1",
    )
    # runner calls entity_graph.sync_from_observation(... relations=...)
    graph.sync_from_observation(
        observation_id="bundle_1",
        heading_sector=0,
        labels=["桌", "桶"],
        spatial_objects=obs,
        pose=None,
        timestamp=1.0,
        place_id=p1,
        update_result=result,
        relations=[
            {
                "subject_id": "semantic_obj_001",
                "object_id": "semantic_obj_002",
                "relation": "near",
                "confidence": 0.92,
            }
        ],
    )
    topology = graph.object_topology_snapshot()
    assert len(topology["nodes"]) == 2
    assert len(topology["edges"]) == 1
    assert topology["edges"][0]["relation"] == "near"
    assert topology["edges"][0]["observation_count"] == 1
