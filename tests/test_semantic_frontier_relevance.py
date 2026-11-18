from types import SimpleNamespace

from app.navigation.semantic_frontier_relevance import score_frontier_semantics
from app.spatial.models import FrontierCandidate, PlaceNode, SpatialPose


def _object(object_id, label, position, variance=None):
    return SimpleNamespace(
        object_id=object_id,
        label=label,
        label_zh=label,
        category="object",
        map_xyz=position,
        position_variance_xyz=variance,
    )


def test_anchor_geometry_changes_frontier_ranking_and_preserves_provenance():
    profile = SimpleNamespace(
        canonical_name_zh="红色背包",
        primary_labels_en=["backpack"],
        aliases_zh=["背包"],
        relation_anchor_labels_en=["water dispenser"],
        relation_anchor_labels_zh=["饮水机"],
    )
    semantic_map = SimpleNamespace(objects={
        "obj_anchor": _object("obj_anchor", "饮水机", (4.0, 0.0, 0.0), (0.01, 0.01, 0.04)),
    })
    frontiers = [
        FrontierCandidate("near", position=(4.2, 0.0)),
        FrontierCandidate("far", position=(0.0, 4.0)),
    ]
    scores, audit = score_frontier_semantics(
        frontiers,
        target_profile=profile,
        semantic_map=semantic_map,
        place_graph=None,
        route_costs={"near": {"reachable": True}, "far": {"reachable": True}},
    )
    assert scores["near"] > scores["far"]
    assert audit["near"].anchor_proximity is not None
    assert audit["near"].object_ids == ("obj_anchor",)
    assert "object:obj_anchor" in audit["near"].evidence_refs


def test_unknown_is_not_claimed_as_measured_semantic_evidence():
    frontier = FrontierCandidate("f1", position=(1.0, 0.0))
    scores, audit = score_frontier_semantics(
        [frontier], target_profile={}, semantic_map=None, place_graph=None
    )
    assert scores == {"f1": 0.0}
    assert audit["f1"].score is None
    assert "no measured" in audit["f1"].reasons[0]


def test_unreachable_route_cannot_gain_semantic_preference():
    profile = SimpleNamespace(canonical_name_zh="背包", primary_labels_en=[], aliases_zh=[],
                              relation_anchor_labels_en=[], relation_anchor_labels_zh=[])
    semantic_map = SimpleNamespace(objects={
        "bag": _object("bag", "背包", (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    })
    frontier = FrontierCandidate("f1", position=(1.1, 0.0))
    scores, audit = score_frontier_semantics(
        [frontier], target_profile=profile, semantic_map=semantic_map,
        place_graph=None, route_costs={"f1": {"reachable": False}},
    )
    assert scores["f1"] == 0.0
    assert audit["f1"].route_factor == 0.0


def test_relevant_place_is_a_measured_signal_without_object_coordinates():
    profile = SimpleNamespace(canonical_name_zh="背包", primary_labels_en=[], aliases_zh=[],
                              relation_anchor_labels_en=[], relation_anchor_labels_zh=[])
    place = PlaceNode("P1", pose=SpatialPose(2.0, 0.0, frame_id="map"),
                      observed_object_labels=["背包"])
    scores, audit = score_frontier_semantics(
        [FrontierCandidate("f1", position=(2.1, 0.0))],
        target_profile=profile,
        semantic_map=SimpleNamespace(objects={}),
        place_graph=SimpleNamespace(places={"P1": place}),
    )
    assert scores["f1"] > 0.0
    assert audit["f1"].place_ids == ("P1",)
