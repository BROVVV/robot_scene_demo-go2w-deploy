from __future__ import annotations

from scripts.go2w.analyze_search_run import analyse_events


def test_historical_rotation_lio_failure_is_detected_by_replay_metrics() -> None:
    events = [
        {
            "event": "spatial_pose_updated",
            "pose": {"x": 1.0, "y": -3.0, "yaw": 0.0},
        },
        {
            "event": "spatial_pose_updated",
            "pose": {"x": 3605.4, "y": 3516.6, "yaw": 0.5},
        },
        {
            "event": "place_updated",
            "place": {"place_id": "P1"},
        },
        {
            "event": "place_updated",
            "place": {"place_id": "P7"},
        },
        {
            "event": "local_scan_selected",
            "goal_id": "local_scan_001",
            "relative_dyaw": 30.0,
            "steps": 1,
            "place_id": "P1",
        },
        {
            "event": "local_scan_selected",
            "goal_id": "local_scan_001",
            "relative_dyaw": 30.0,
            "steps": 1,
            "place_id": "P7",
        },
        {"event": "semantic_error", "error_code": "FULL_SEMANTIC_ERROR"},
    ]

    result = analyse_events(events)

    assert result["place_count"] == 2
    assert result["pslam"]["max_radius_m"] > 5000.0
    assert result["local_scan"]["step_sequence"] == [1, 1]
    assert result["local_scan"]["turn_sequence_deg"] == [30.0, 30.0]
    assert result["full_semantic_error_count"] == 1
