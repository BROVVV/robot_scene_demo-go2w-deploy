"""Plan §5 / §36 tests: persistent id mapping must not use labels.

The runner (``scripts/go2w/run_semantic_exploration.py``) builds
``association_by_index`` / ``persistent_by_source_id`` from
``SemanticMapUpdateResult.associations`` and publishes object events with the
persistent id from the association - never from a ``label -> id`` dict.  These
tests lock that contract at the data layer and mirror the runner's mapping
loop.
"""

from __future__ import annotations

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.semantic_entity_graph import SemanticEntityGraph
from app.spatial.semantic_object_map import SemanticObjectMap


def _obs(label, object_id, map_xyz):
    return ObjectSpatialObservation(
        object_id=object_id, label=label, map_xyz=map_xyz,
        spatial_quality="METRIC_RGBD", confidence=0.9,
        provenance={"frame_id": f"f_{object_id}"},
    )


def _association_map(update_result):
    return {
        assoc.observation_index: assoc.persistent_object_id
        for assoc in update_result.associations
    }


def test_two_chairs_produce_two_distinct_persistent_ids():
    """Two chairs with same label must map to obj_004 and obj_005 (distinct)."""
    semantic_map = SemanticObjectMap(merge_distance_m=0.4, confirm_min_observations=2)
    obs = [
        _obs("椅子", "semantic_obj_001", (0.5, 0.0, 0.0)),
        _obs("椅子", "semantic_obj_002", (1.5, 0.0, 0.0)),
    ]
    result = semantic_map.update_with_associations(obs, frame_id="f1", now=1.0)
    by_index = _association_map(result)
    assert len(by_index) == 2
    id0 = by_index[0]
    id1 = by_index[1]
    assert id0 != id1
    # source ids are preserved (frame_object_id -> persistent)
    sources = {assoc.source_object_id for assoc in result.associations}
    assert sources == {"semantic_obj_001", "semantic_obj_002"}


def test_runner_style_mapping_loop_uses_association_not_label():
    """Mirror the runner's event loop: object_id must come from association."""
    semantic_map = SemanticObjectMap(merge_distance_m=0.4, confirm_min_observations=2)
    obs = [
        _obs("椅子", "semantic_obj_001", (0.5, 0.0, 0.0)),
        _obs("椅子", "semantic_obj_002", (1.5, 0.0, 0.0)),
    ]
    result = semantic_map.update_with_associations(obs, frame_id="f1", now=1.0)
    association_by_index = {
        assoc.observation_index: assoc for assoc in result.associations
    }
    emitted = []
    for index, obs_item in enumerate(obs):
        assoc = association_by_index.get(index)
        if assoc is None:
            continue
        entity = semantic_map.objects.get(assoc.persistent_object_id)
        emitted.append(
            {
                "object_id": assoc.persistent_object_id,
                "label": obs_item.label,
                "source_object_id": assoc.source_object_id,
                "association_action": assoc.action,
            }
        )
    assert len(emitted) == 2
    assert emitted[0]["object_id"] != emitted[1]["object_id"]
    assert emitted[0]["object_id"].startswith("obj_")
    assert emitted[1]["object_id"].startswith("obj_")
    # same label but distinct persistent ids - the whole point of the change
    assert emitted[0]["label"] == emitted[1]["label"] == "椅子"


def test_entity_graph_relations_use_persistent_mapping_not_label():
    """Relation endpoints resolve through association, not label collapse."""
    graph = SemanticEntityGraph()
    graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["椅子", "椅子"],
        pose=None, timestamp=1.0,
    )
    p1 = graph.place_graph.current_place().place_id
    obs = [
        _obs("椅子", "semantic_obj_001", (0.5, 0.0, 0.0)),
        _obs("椅子", "semantic_obj_002", (1.5, 0.0, 0.0)),
    ]
    result = graph.object_map.update_with_associations(
        obs, place_id=p1, frame_id="f1", now=1.0
    )
    graph.sync_from_observation(
        observation_id="bundle_1", heading_sector=0, labels=["椅子", "椅子"],
        spatial_objects=obs, timestamp=1.0, place_id=p1, update_result=result,
        relations=[{
            "subject_id": "semantic_obj_001",
            "object_id": "semantic_obj_002",
            "relation": "near",
        }],
    )
    assert len(graph.object_relations) == 1
    (key,) = graph.object_relations.keys()
    assert key[0] != key[2]
