from app.video.scene_map_search_ranker import rank_places_for_target_search
from app.video.target_navigation_decision_builder import build_target_navigation_decision
from app.video.video_target_state import apply_target_state


def _television_profile() -> dict:
    return {
        "raw_target": "电视",
        "canonical_name_zh": "电视",
        "zh_terms": ["电视", "电视机"],
        "en_terms": ["television", "TV", "tv screen", "monitor", "display"],
        "context_terms": ["sofa", "TV stand", "living room wall", "remote control"],
        "likely_regions_zh": ["living_room", "客厅"],
        "negative_terms": ["window", "mirror", "poster", "painting", "whiteboard"],
    }


def _living_room_topology() -> dict:
    return {
        "nodes": [
            {"node_id": "place_003", "node_type": "place", "label": "living_room"},
            {"node_id": "obj_sofa", "node_type": "obstacle", "label": "sofa"},
            {"node_id": "obj_tv_stand", "node_type": "landmark", "label": "TV stand"},
            {"node_id": "obj_wall", "node_type": "landmark", "label": "living room wall"},
            {"node_id": "free_001", "node_type": "free_space", "label": "free_space"},
        ],
        "edges": [
            {"from": "place_003", "to": "obj_sofa", "relation": "contains"},
            {"from": "place_003", "to": "obj_tv_stand", "relation": "contains"},
            {"from": "place_003", "to": "obj_wall", "relation": "contains"},
            {"from": "place_003", "to": "free_001", "relation": "passable_in"},
        ],
    }


def test_context_cannot_confirm_television() -> None:
    result = {"target_found": False, "timeline": []}
    ranked = rank_places_for_target_search(
        _living_room_topology(),
        _television_profile(),
        result,
        config=None,
    )

    apply_target_state(result, ranked_places=ranked)

    assert ranked[0]["place_id"] == "place_003"
    assert ranked[0]["target_search_score"] >= 0.7
    assert ranked[0]["can_confirm_target"] is False
    assert result["target_confirmed"] is False
    assert result["target_status"] == "target_unconfirmed_but_likely_area_found"


def test_navigation_decision_requires_visual_confirmation_for_likely_area() -> None:
    result = {
        "target": "电视",
        "target_found": False,
        "target_status": "target_unconfirmed_but_likely_area_found",
    }
    ranked = [
        {
            "place_id": "place_003",
            "target_search_score": 0.82,
            "reason": "living-room-like place with TV stand",
            "recommended_observation": "look toward the wall above the TV stand",
        }
    ]

    decision = build_target_navigation_decision(
        target_search_result=result,
        navigation_topology=_living_room_topology(),
        ranked_places=ranked,
        target_profile=_television_profile(),
        config=None,
    )

    assert decision["target_confirmed"] is False
    assert decision["next_place_id"] == "place_003"
    assert decision["requires_visual_confirmation"] is True
