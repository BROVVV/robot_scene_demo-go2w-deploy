from __future__ import annotations

import json

import pytest

from app.perception.passive_target_watch import (
    CandidateCooldown,
    TargetWatchConfig,
    active_verification_specs,
    atomic_write_json,
    automatic_slime_hazard_spec,
    bbox_area_ratio,
    deterministic_spec,
    load_json_object,
    schedule_terms,
    target_profile_from_spec,
    validate_request,
)


@pytest.mark.parametrize(
    ("query", "target", "anchor", "mode"),
    [
        ("黑椅子", "chair", None, "attribute"),
        ("饮水机旁边的凳子", "stool", "water dispenser", "relation"),
        ("烟雾", "smoke", None, "diffuse_hazard"),
        ("泄漏的液体", "liquid leak", "pipe", "diffuse_hazard"),
        ("史莱姆泥", "slime", "pipe", "diffuse_hazard"),
    ],
)
def test_required_deterministic_profiles(query, target, anchor, mode):
    spec = deterministic_spec(query)
    assert target in spec.target_terms_en
    assert spec.verification_mode == mode
    if anchor is not None:
        assert anchor in spec.anchor_terms_en


def test_prompt_budget_keeps_target_anchor_and_scene():
    spec = deterministic_spec("饮水机旁边的凳子")
    terms, slots, truncated = schedule_terms(
        spec,
        ["door", "cabinet", "person", "table"],
        ["smoke", "fluid spill"],
        max_terms=8,
        max_target_terms=3,
        max_anchor_terms=2,
        reserve_scene_terms=2,
    )
    assert terms[:2] == ["stool", "chair"]
    assert "water dispenser" in slots["anchor"]
    assert len(slots["scene"]) >= 2
    assert len(terms) <= 8
    assert isinstance(truncated, list)


def test_mapping_and_candidate_thresholds_are_separate():
    config = TargetWatchConfig.from_mapping({})
    assert config.candidate_score("grounded_sam") < 0.35
    assert config.candidate_score("yoloe_seg") < 0.35
    assert bbox_area_ratio([0.1, 0.1, 0.5, 0.5]) == pytest.approx(0.16)


def test_cooldown_is_generation_keyed_by_caller():
    cooldown = CandidateCooldown(20)
    assert cooldown.permits("1:obj:x", now=100)
    assert not cooldown.permits("1:obj:x", now=110)
    assert cooldown.permits("2:obj:x", now=110)


def test_atomic_json_and_request_validation(tmp_path):
    path = tmp_path / "request.json"
    spec = deterministic_spec("黑椅子")
    request = {
        "schema_version": 1,
        "mapping_session_id": "teleop_0",
        "generation": 4,
        "enabled": True,
        "command": "set",
        "raw_query": spec.raw_query,
        "profile": spec.to_dict(),
        "submitted_at": 1.0,
    }
    atomic_write_json(path, request)
    assert load_json_object(path) == request
    assert validate_request(request, session_id="teleop_0")["generation"] == 4
    with pytest.raises(ValueError, match="mismatch"):
        validate_request(request, session_id="teleop_1")
    path.write_text("{broken", encoding="utf-8")
    assert load_json_object(path) is None


def test_target_profile_bridge_preserves_anchor_role():
    profile = target_profile_from_spec(deterministic_spec("饮水机旁边的凳子"))
    assert profile.primary_labels_en[:2] == ["stool", "chair"]
    assert profile.relation_anchor_labels_en == ["water dispenser"]


def test_config_rejects_unbounded_concurrency():
    with pytest.raises(ValueError, match="max_inflight"):
        TargetWatchConfig.from_mapping({"verify": {"max_inflight": 2}})


def test_automatic_slime_hazard_profile_is_active_without_operator_text():
    profiles = active_verification_specs(None, auto_hazard_enabled=True)

    assert len(profiles) == 1
    spec, automatic = profiles[0]
    assert automatic is True
    assert spec == automatic_slime_hazard_spec()
    assert spec.verification_mode == "diffuse_hazard"
    assert "simulated liquid leak" in spec.target_terms_en
    assert "green slime" in spec.target_terms_en
    assert "pipe" in spec.anchor_terms_en


def test_automatic_hazard_profile_precedes_unrelated_operator_target():
    operator = deterministic_spec("黑椅子")
    profiles = active_verification_specs(operator, auto_hazard_enabled=True)

    assert [(item.canonical_name_zh, auto) for item, auto in profiles] == [
        ("疑似管道史莱姆泥泄漏污染源", True),
        ("黑椅子", False),
    ]


def test_automatic_hazard_can_be_disabled_explicitly():
    config = TargetWatchConfig.from_mapping(
        {"auto_hazard": {"enabled": False}}
    )
    assert config.auto_hazard_enabled is False
    assert active_verification_specs(
        None, auto_hazard_enabled=config.auto_hazard_enabled
    ) == ()
