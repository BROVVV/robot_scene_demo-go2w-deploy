from app.navigation.strategic_vlm import constrain_strategic_decision


def test_strategic_vlm_accepts_only_program_candidate_ids():
    decision = constrain_strategic_decision(
        {
            "target_state": "absent",
            "selected_search_candidate_id": "junction_2",
            "reason": "right branch is plausible",
            "cmd_vel": {"linear": 1.0},
        },
        detection_ids=["det_1"],
        candidate_ids=["frontier_1", "junction_2"],
    )
    assert decision.selected_search_candidate_id == "junction_2"
    assert "cmd_vel" not in decision.to_dict()


def test_unknown_or_invalid_id_uses_first_program_ranked_candidate():
    for payload in (
        {"target_state": "unknown"},
        {"target_state": "absent", "selected_search_candidate_id": "invented"},
    ):
        decision = constrain_strategic_decision(
            payload,
            detection_ids=[],
            candidate_ids=["frontier_best", "frontier_second"],
        )
        assert decision.selected_search_candidate_id == "frontier_best"
        assert decision.fallback_used is True


def test_visual_claim_without_local_detection_stays_possible_without_geometry():
    decision = constrain_strategic_decision(
        {"target_state": "present", "target_detection_id": "invented"},
        detection_ids=["det_real"],
        candidate_ids=["frontier_1"],
    )
    assert decision.target_state == "possible"
    assert decision.target_detection_id is None
    assert decision.selected_search_candidate_id is None
