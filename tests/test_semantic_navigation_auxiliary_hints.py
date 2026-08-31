from __future__ import annotations

from app.reasoning.semantic_navigation.auxiliary_hints import (
    build_precomputed_situated_prior_hints,
    build_psg_auxiliary_hints,
)
from app.video.schemas import SceneGraph, SceneGraphNode


def test_psg_hint_is_grounded_in_observed_left_door_and_turn_only():
    graph = SceneGraph(nodes=[SceneGraphNode(
        node_id="door_1",
        node_type="object",
        label="door",
        label_zh="门",
        category="passage",
        source="observed",
        confidence=0.9,
        evidence_level="observed",
        attributes={
            "navigation_role": "passage",
            "stable_position_2d": "left",
        },
    )])
    result = build_psg_auxiliary_hints(graph, enabled=True)
    assert result["status"]["available"] is True
    assert len(result["hints"]) == 1
    hint = result["hints"][0]
    assert hint["heading_delta_deg"] == 30.0
    assert hint["anchor_node_id"] == "door_1"
    assert hint["can_confirm_target"] is False
    assert hint["allow_forward"] is False


def test_psg_disabled_or_missing_graph_yields_no_hint():
    disabled = build_psg_auxiliary_hints(None, enabled=False)
    missing = build_psg_auxiliary_hints(None, enabled=True)
    assert disabled["hints"] == []
    assert disabled["status"]["reason"] == "disabled_by_config"
    assert missing["hints"] == []
    assert missing["status"]["reason"] == "observed_scene_graph_unavailable"


def test_precomputed_situated_prior_requires_non_confirming_contract():
    unsafe = build_precomputed_situated_prior_hints(
        {"situated_prior": {
            "can_confirm_target": True,
            "next_view_plan": [{"action": "turn_left"}],
        }},
        enabled=True,
    )
    assert unsafe["hints"] == []
    safe = build_precomputed_situated_prior_hints(
        {"situated_prior": {
            "can_confirm_target": False,
            "next_view_plan": [{
                "hint_id": "situated_prior:left",
                "action": "turn_left",
                "expected_information_gain": 0.7,
                "reason_zh": "检查左侧可见区域",
            }],
        }},
        enabled=True,
    )
    assert safe["status"]["available"] is True
    assert safe["hints"][0]["heading_delta_deg"] == 30.0
    assert safe["hints"][0]["can_confirm_target"] is False
    assert safe["hints"][0]["allow_forward"] is False
