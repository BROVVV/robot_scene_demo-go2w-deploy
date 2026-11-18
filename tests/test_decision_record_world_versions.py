from app.navigation.decision_record import build_decision_record


def test_decision_record_binds_snapshot_versions_and_downstream_selection():
    record = build_decision_record(
        cycle=3, match_state="ZERO",
        selected_intent={"intent_type": "EXPLORE_FRONTIER"},
        selected_goal={"goal_id": "g1", "goal_type": "NAVIGATE_POSE"},
        next_motion_command={}, score=0.4, score_breakdown={"semantic_relevance": 0.2},
        task_id="t1", snapshot_id="s1", map_id="m1", map_epoch="e1",
        graph_revision=4, grid_revision=5, semantic_revision=6,
        sensor_profile_id="sha256:profile",
        source_observation_ids=["obs1"], evidence_age_ms=20.0,
        candidates=[{"id": "f1", "reachability_status": "REACHABLE"}],
        selected_candidate_id="f1", downstream_selected_id="g1",
    )
    value = record.to_dict()
    assert value["snapshot_id"] == "s1"
    assert value["map_epoch"] == "e1"
    assert value["grid_revision"] == 5
    assert value["sensor_profile_id"] == "sha256:profile"
    assert value["candidates"][0]["id"] == "f1"
    assert value["selected_candidate_id"] == "f1"
    assert value["downstream_selected_id"] == "g1"
    assert value["arrival_verified"] is False
    assert value["target_verified"] is False
