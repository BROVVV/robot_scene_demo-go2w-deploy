"""Plan §33 cases for persistent OBJECT -> OBJECT relations.

Validates the identity bridge (frame object id -> association -> persistent
id), evidence merge, symmetric canonicalisation, directed relations, unresolved
endpoints and status lifecycle in :class:`SemanticEntityGraph`.
"""

from __future__ import annotations

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.models import SpatialPose
from app.spatial.semantic_entity_graph import (
    SemanticEntityGraph,
    normalize_relation,
    relation_is_symmetric,
)


def _obs(label, object_id, map_xyz, frame_id, place_id):
    return ObjectSpatialObservation(
        object_id=object_id,
        label=label,
        map_xyz=map_xyz,
        spatial_quality="METRIC_RGBD",
        confidence=0.9,
        provenance={
            "frame_id": frame_id,
            "place_id": place_id,
            "observation_id": f"obs_{frame_id}",
        },
    )


def _add_frame(graph, *, obs_list, relations, place_id, bundle_id, now):
    update_result = graph.object_map.update_with_associations(
        obs_list,
        place_id=place_id,
        frame_id=obs_list[0].provenance["frame_id"] if obs_list else "f",
        now=now,
    )
    graph.sync_from_observation(
        observation_id=bundle_id,
        heading_sector=0,
        labels=[o.label for o in obs_list],
        spatial_objects=obs_list,
        pose=SpatialPose(x=0.0, y=0.0),
        timestamp=now,
        place_id=place_id,
        update_result=update_result,
        relations=relations,
    )
    return update_result


def test_relation_remap_and_merge_across_frames():
    """Plan §33 case 1+2: frame ids change, persistent edge stays one."""
    graph = SemanticEntityGraph()
    p1, _ = graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["桌", "桶"],
        pose=SpatialPose(x=0.0, y=0.0), timestamp=1.0,
    )
    obs1 = [
        _obs("桌", "semantic_obj_001", (1.0, 0.0, 0.0), "f1", p1),
        _obs("桶", "semantic_obj_002", (1.2, 0.0, 0.0), "f1", p1),
    ]
    _add_frame(
        graph,
        obs_list=obs1,
        relations=[{
            "subject_id": "semantic_obj_001",
            "object_id": "semantic_obj_002",
            "relation": "near",
            "confidence": 0.9,
            "description_zh": "桌靠近桶",
        }],
        place_id=p1,
        bundle_id="bundle_1",
        now=1.0,
    )
    id_by_label = {o.label: o.object_id for o in graph.object_map.objects.values()}
    desk, bin_ = id_by_label["桌"], id_by_label["桶"]
    assert len(graph.object_map.objects) == 2
    assert len(graph.object_relations) == 1
    rel = graph.object_relations[(desk, "near", bin_)]
    assert rel.observation_count == 1
    assert rel.status == "TENTATIVE"
    assert rel.relation_scope == "STRUCTURAL"

    # Frame 2: different frame ids, same physical objects
    p2, _ = graph.place_graph.register_observation(
        observation_id="bundle_2", heading_sector=1, objects=["桌", "桶"],
        pose=SpatialPose(x=0.0, y=0.1), timestamp=2.0,
    )
    obs2 = [
        _obs("桌", "semantic_obj_010", (1.05, 0.0, 0.0), "f10", p2),
        _obs("桶", "semantic_obj_011", (1.22, 0.0, 0.0), "f11", p2),
    ]
    _add_frame(
        graph,
        obs_list=obs2,
        relations=[{
            "subject_id": "semantic_obj_010",
            "object_id": "semantic_obj_011",
            "relation": "near",
            "confidence": 0.8,
        }],
        place_id=p2,
        bundle_id="bundle_2",
        now=2.0,
    )
    assert len(graph.object_map.objects) == 2
    assert len(graph.object_relations) == 1
    rel2 = graph.object_relations[(desk, "near", bin_)]
    assert rel2.observation_count == 2
    assert rel2.status == "CONFIRMED"
    assert abs(rel2.confidence - (0.9 + 0.8) / 2) < 1e-6


def test_same_label_objects_stay_distinct():
    """Plan §33 case 3: two chairs merge only via association, never by label."""
    graph = SemanticEntityGraph()
    p1, _ = graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["椅子", "椅子"],
        pose=SpatialPose(x=0.0, y=0.0), timestamp=1.0,
    )
    obs1 = [
        _obs("椅子", "semantic_obj_001", (0.5, 0.0, 0.0), "f1", p1),
        _obs("椅子", "semantic_obj_002", (1.5, 0.0, 0.0), "f1", p1),
    ]
    _add_frame(
        graph,
        obs_list=obs1,
        relations=[{
            "subject_id": "semantic_obj_001",
            "object_id": "semantic_obj_002",
            "relation": "near",
            "confidence": 0.9,
        }],
        place_id=p1,
        bundle_id="bundle_1",
        now=1.0,
    )
    ids = sorted(o.object_id for o in graph.object_map.objects.values())
    assert len(graph.object_map.objects) == 2
    assert len(ids) == 2
    assert ids[0] != ids[1]
    # relation uses the two distinct persistent ids
    assert len(graph.object_relations) == 1
    (key,) = graph.object_relations.keys()
    assert key[0] != key[2]


def test_unresolved_and_self_relation_are_rejected():
    """Plan §33 case 4+5: no crash, no bogus edge."""
    graph = SemanticEntityGraph()
    p1, _ = graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["桌", "桶"],
        pose=SpatialPose(x=0.0, y=0.0), timestamp=1.0,
    )
    obs1 = [
        _obs("桌", "semantic_obj_001", (1.0, 0.0, 0.0), "f1", p1),
        _obs("桶", "semantic_obj_002", (1.2, 0.0, 0.0), "f1", p1),
    ]
    _add_frame(
        graph,
        obs_list=obs1,
        relations=[
            {
                "subject_id": "semantic_obj_999",  # unresolved
                "object_id": "semantic_obj_001",
                "relation": "near",
            },
            {
                "subject_id": "semantic_obj_001",
                "object_id": "semantic_obj_001",  # self
                "relation": "near",
            },
            {
                "subject_id": "semantic_obj_001",
                "object_id": "semantic_obj_002",  # distinct endpoints
                "relation": "fuzzy_unknown",  # unknown vocabulary
            },
        ],
        place_id=p1,
        bundle_id="bundle_1",
        now=1.0,
    )
    assert len(graph.object_relations) == 0
    rejected = [
        item for item in graph.association_debug
        if item.get("type") == "relation_association" and item.get("result") == "rejected"
    ]
    assert rejected, "expected debug rejection entries"
    reasons = {item.get("reason") for item in rejected}
    assert "source_endpoint_unresolved" in reasons
    assert "self_relation_rejected" in reasons
    assert "relation_not_allowed" in reasons


def test_symmetric_relation_canonicalised():
    """Plan §33 case 6: near is symmetric -> one canonical edge."""
    graph = SemanticEntityGraph()
    p1, _ = graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["桌", "桶"],
        pose=SpatialPose(x=0.0, y=0.0), timestamp=1.0,
    )
    obs1 = [
        _obs("桌", "semantic_obj_001", (1.0, 0.0, 0.0), "f1", p1),
        _obs("桶", "semantic_obj_002", (1.2, 0.0, 0.0), "f1", p1),
    ]
    _add_frame(
        graph,
        obs_list=obs1,
        relations=[{
            "subject_id": "semantic_obj_002",  # reversed order
            "object_id": "semantic_obj_001",
            "relation": "near",
            "confidence": 0.9,
        }],
        place_id=p1,
        bundle_id="bundle_1",
        now=1.0,
    )
    id_by_label = {o.label: o.object_id for o in graph.object_map.objects.values()}
    desk, bin_ = id_by_label["桌"], id_by_label["桶"]
    assert len(graph.object_relations) == 1
    assert (desk, "near", bin_) in graph.object_relations
    assert graph.object_relations[(desk, "near", bin_)].directed is False


def test_directed_relation_keeps_direction_and_scope():
    """Plan §33 case 7: left_of is VIEW_RELATIVE + directed."""
    graph = SemanticEntityGraph()
    p1, _ = graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["桌", "桶"],
        pose=SpatialPose(x=0.0, y=0.0), timestamp=1.0,
    )
    obs1 = [
        _obs("桌", "semantic_obj_001", (1.0, 0.0, 0.0), "f1", p1),
        _obs("桶", "semantic_obj_002", (1.2, 0.0, 0.0), "f1", p1),
    ]
    _add_frame(
        graph,
        obs_list=obs1,
        relations=[{
            "subject_id": "semantic_obj_001",
            "object_id": "semantic_obj_002",
            "relation": "left_of",
        }],
        place_id=p1,
        bundle_id="bundle_1",
        now=1.0,
    )
    id_by_label = {o.label: o.object_id for o in graph.object_map.objects.values()}
    desk, bin_ = id_by_label["桌"], id_by_label["桶"]
    rel = graph.object_relations[(desk, "left_of", bin_)]
    assert rel.directed is True
    assert rel.relation_scope == "VIEW_RELATIVE"
    # reversed direction is a different edge
    assert not (bin_, "left_of", desk) in graph.object_relations


def test_relation_status_lifecycle_tentative_to_confirmed():
    """Plan §33 case 8: 1st observation TENTATIVE, 2nd CONFIRMED."""
    graph = SemanticEntityGraph(
        relation_confirm_min_observations=2,
    )
    p1, _ = graph.place_graph.register_observation(
        observation_id="bundle_1", heading_sector=0, objects=["桌", "桶"],
        pose=SpatialPose(x=0.0, y=0.0), timestamp=1.0,
    )
    obs1 = [
        _obs("桌", "semantic_obj_001", (1.0, 0.0, 0.0), "f1", p1),
        _obs("桶", "semantic_obj_002", (1.2, 0.0, 0.0), "f1", p1),
    ]
    rel = {"subject_id": "semantic_obj_001", "object_id": "semantic_obj_002", "relation": "near"}
    _add_frame(graph, obs_list=obs1, relations=[rel], place_id=p1, bundle_id="bundle_1", now=1.0)
    desk = [o.object_id for o in graph.object_map.objects.values() if o.label == "桌"][0]
    bin_ = [o.object_id for o in graph.object_map.objects.values() if o.label == "桶"][0]
    assert graph.object_relations[(desk, "near", bin_)].status == "TENTATIVE"
    _add_frame(graph, obs_list=obs1, relations=[rel], place_id=p1, bundle_id="bundle_2", now=2.0)
    assert graph.object_relations[(desk, "near", bin_)].status == "CONFIRMED"


def test_normalize_relation_helpers():
    assert normalize_relation("left") == "left_of"
    assert normalize_relation("next_to") == "adjacent_to"
    assert normalize_relation("unknown_xyz") is None
    assert normalize_relation("") is None
    assert relation_is_symmetric("near") is True
    assert relation_is_symmetric("left_of") is False
