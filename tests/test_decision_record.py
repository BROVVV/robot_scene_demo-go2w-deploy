"""Tests for the structured DecisionRecord (plan §19.6)."""

from __future__ import annotations

from app.navigation.decision_record import (
    alternative_from_candidate,
    build_decision_record,
    build_reason_zh,
)


def _base_kwargs():
    return {
        "cycle": 3,
        "match_state": "PARTIAL",
        "selected_intent": {
            "intent_id": "intent_003",
            "intent_type": "EXPLORE_FRONTIER",
            "target_frontier_id": "F3",
            "semantic_score": 0.81,
            "spatial_gain": 0.67,
        },
        "selected_goal": {"goal_id": "local_003", "goal_type": "ROTATE_VIEW", "relative_dyaw": 24.0},
        "next_motion_command": {"instruction_zh": "右转 24°"},
        "score": 0.73,
        "score_breakdown": {
            "semantic_relevance": 0.81,
            "spatial_gain": 0.67,
            "psg_prior": 0.52,
            "novelty": 0.74,
            "route_cost_penalty": -0.21,
            "visited_penalty": 0.0,
            "negative_evidence_penalty": 0.0,
        },
        "evidence": {"anchor_labels": ["办公桌"]},
        "alternatives": [
            {"candidate_id": "F2", "score": 0.48, "rejected_reason_zh": "路线更长"}
        ],
        "map_revision": 31,
        "session_id": "s1",
        "task_text": "找到绿色垃圾桶",
        "canonical_target": "绿色垃圾桶",
        "current_place_id": "P3",
    }


def test_decision_selected_candidate_and_breakdown():
    rec = build_decision_record(**_base_kwargs())
    assert rec.cycle == 3
    assert rec.score == 0.73
    assert rec.selected_intent["target_frontier_id"] == "F3"
    assert rec.score_breakdown["semantic_relevance"] == 0.81
    assert rec.map_revision == 31


def test_reason_zh_rule_template():
    rec = build_decision_record(**_base_kwargs())
    assert "办公桌" in rec.reason_zh
    assert "下一步动作" in rec.reason_zh or "路线" in rec.reason_zh


def test_decision_partial_message():
    rec = build_decision_record(**_base_kwargs())
    assert "PARTIAL" in rec.to_dict()["match_state"] or "PARTIAL" in str(rec.match_state)
    assert rec.reason_code in {"EXPLORE_SEMANTIC_FRONTIER", "INSPECT_ANCHOR_REGION"}


def test_reason_zh_has_no_llm_cot():
    rec = build_decision_record(**_base_kwargs())
    forbidden = ["我认为", "AI觉得", "chain-of-thought", "推理如下"]
    for word in forbidden:
        assert word not in rec.reason_zh


def test_next_motion_present():
    rec = build_decision_record(**_base_kwargs())
    assert rec.next_motion_command["instruction_zh"] == "右转 24°"


def test_alternative_reason():
    rec = build_decision_record(**_base_kwargs())
    assert len(rec.alternatives) == 1
    assert "F2" in rec.alternatives[0]["candidate_id"]


def test_alternative_from_candidate():
    alt = alternative_from_candidate(
        candidate_id="F2", score=0.48,
        top_penalty_name="已访问惩罚", top_penalty_value=0.20,
    )
    assert alt["score"] == 0.48
    assert "已访问惩罚" in alt["rejected_reason_zh"]


def test_to_dict_json_safe():
    rec = build_decision_record(**_base_kwargs())
    import json
    data = rec.to_dict()
    json.dumps(data, ensure_ascii=False)  # must not raise


def test_strong_reason_mentions_confirmation():
    kwargs = _base_kwargs()
    kwargs["match_state"] = "STRONG"
    kwargs["selected_intent"] = {"intent_type": "APPROACH_TARGET", "target_object_id": "obj_004"}
    rec = build_decision_record(**kwargs)
    assert "验证" in rec.reason_zh or "确认" in rec.reason_zh
