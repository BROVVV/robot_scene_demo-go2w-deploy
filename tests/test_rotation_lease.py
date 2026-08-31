from datetime import datetime, timedelta, timezone
import hashlib
import json

import pytest

from app.live_robot.rotation_lease import (
    CAPTURE_ROLES,
    REQUIRED_CHECKS,
    SECTORS,
    evaluate_rotation_lease,
    evaluate_rotation_lease_step,
    load_rotation_lease,
    resolve_rotation_clearance_source,
    rotation_lease_stage2_scope_errors,
)


def evidence(tmp_path, now: datetime) -> tuple[dict, object]:
    sources = {}
    capture_payloads = {}
    for role in CAPTURE_ROLES:
        path = tmp_path / f"{role}.json"
        sector_summaries = {
            sector: {
                "sector": sector,
                "expected_distance_m": 0.70,
                "frame_count": 30,
            }
            for sector in SECTORS
        }
        capture_payload = {
            "validation_type": "go2w_rotation_sector_capture",
            "capture_role": role,
            "captured_at": now.isoformat(),
            "robot_model": "Unitree Go2-W",
            "posture": "stationary_standing",
            "robot_motion_commanded": False,
            "marker_cloud_topic": "/go2w/lidar/obstacles",
            "marker_cloud_purpose": "controlled_direction_and_yaw_crosscheck_only",
            "marker_cloud_safety_authoritative_for_collision": False,
            "collision_cloud_topic": "/go2w/lidar/collision_obstacles",
            "passed": True,
            "stationarity": {"passed": True},
            "sector_summaries": sector_summaries,
            "sector_profiles": {},
        }
        capture_payloads[role] = capture_payload
        path.write_text(json.dumps(capture_payload), encoding="utf-8")
        sources[role] = {
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    expires = now + timedelta(minutes=10)
    payload = {
        "schema_version": "1.0",
        "validation_type": "go2w_rotation_clearance_physical_crosscheck",
        "validation_tool": "validate_rotation_clearance_physical_ros.py",
        "robot_model": "Unitree Go2-W",
        "robot_motion_commanded": False,
        "operator": "TEST_OPERATOR",
        "posture": "stationary_standing",
        "passed": True,
        "validated_at": now.isoformat(),
        "expires_at": expires.isoformat(),
        "capture_window": {
            "earliest_at": now.isoformat(),
            "latest_at": now.isoformat(),
            "span_seconds": 0.0,
            "maximum_age_seconds": 900,
        },
        "validated_rotation_envelope_radius_m": 0.511,
        "maximum_cross_capture_translation_m": 0.002,
        "maximum_cross_capture_yaw_change_rad": 0.005,
        "physical_inspection": {
            "swept_clearance_confirmed": True,
            "measured_clearance_radius_m": 0.60,
        },
        "authorization_scope": {
            "type": "initial_pose_in_place_rotation_only",
            "frame": "odom_wheel",
            "origin_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            "maximum_translation_m": 0.03,
            "expires_at": expires.isoformat(),
        },
        "checks": {key: True for key in REQUIRED_CHECKS},
        "sector_validations": {
            sector: {
                "sector": sector,
                "passed": True,
                "checks": {
                    "minimum_frames": True,
                    "target_repeatably_detected": True,
                    "target_point_count_contrast": True,
                    "target_selected_point_contrast": True,
                    "target_range_contrast": False,
                    "target_contrast": True,
                    "target_bearing_matches": True,
                    "target_range_matches": True,
                },
                "baseline": capture_payloads["baseline"]["sector_summaries"][sector],
                "target": capture_payloads[sector]["sector_summaries"][sector],
            }
            for sector in SECTORS
        },
        "capture_checks": {role: True for role in CAPTURE_ROLES},
        "capture_evidence": sources,
    }
    evidence_path = tmp_path / "evidence.json"
    evidence_path.write_text(json.dumps(payload), encoding="utf-8")
    return payload, evidence_path


def test_valid_lease_is_loaded_and_bound_to_origin(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    _, path = evidence(tmp_path, now)
    loaded = load_rotation_lease(
        path,
        required_envelope_radius_m=0.511,
        project_root=tmp_path,
        now=now + timedelta(seconds=1),
    )
    assert evaluate_rotation_lease(
        loaded,
        current_pose=(1.02, 2.0, 1.5),
        current_frame="odom_wheel",
        now=now + timedelta(minutes=1),
    ).allowed
    wrong_frame = evaluate_rotation_lease(
        loaded,
        current_pose=(1.0, 2.0, 1.5),
        current_frame="odom",
        now=now + timedelta(minutes=1),
    )
    assert not wrong_frame.allowed
    assert "requires odom_wheel" in wrong_frame.reason
    moved = evaluate_rotation_lease(
        loaded,
        current_pose=(1.031, 2.0, 1.5),
        current_frame="odom_wheel",
        now=now + timedelta(minutes=1),
    )
    assert not moved.allowed
    assert "translation" in moved.reason


def test_lease_expires_and_capture_tampering_is_rejected(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    payload, path = evidence(tmp_path, now)
    (tmp_path / "front.json").write_text("tampered", encoding="utf-8")
    with pytest.raises(ValueError, match="hash mismatch: front"):
        load_rotation_lease(
            path,
            required_envelope_radius_m=0.511,
            project_root=tmp_path,
            now=now + timedelta(seconds=1),
        )
    front_payload = json.loads((tmp_path / "right.json").read_text(encoding="utf-8"))
    front_payload["capture_role"] = "front"
    (tmp_path / "front.json").write_text(json.dumps(front_payload), encoding="utf-8")
    payload["sector_validations"]["front"]["target"] = front_payload[
        "sector_summaries"
    ]["front"]
    payload["capture_evidence"]["front"]["sha256"] = hashlib.sha256(
        (tmp_path / "front.json").read_bytes()
    ).hexdigest()
    path.write_text(json.dumps(payload), encoding="utf-8")
    loaded = load_rotation_lease(
        path,
        required_envelope_radius_m=0.511,
        project_root=tmp_path,
        now=now + timedelta(seconds=1),
    )
    assert not evaluate_rotation_lease(
        loaded,
        current_pose=(1.0, 2.0, 0.3),
        current_frame="odom_wheel",
        now=now + timedelta(minutes=11),
    ).allowed


def test_pose_bound_lease_cannot_be_relabelled_persistent(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    payload, path = evidence(tmp_path, now)
    payload["authorization_scope"]["type"] = "persistent_sensor_coverage"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="scope is unsupported"):
        load_rotation_lease(
            path,
            required_envelope_radius_m=0.511,
            project_root=tmp_path,
            now=now + timedelta(seconds=1),
        )


def test_sector_result_must_match_hashed_capture(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    payload, path = evidence(tmp_path, now)
    payload["sector_validations"]["left"]["target"]["frame_count"] = 999
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="target capture mismatch: left"):
        load_rotation_lease(
            path,
            required_envelope_radius_m=0.511,
            project_root=tmp_path,
            now=now + timedelta(seconds=1),
        )


def test_cross_capture_yaw_change_is_enforced(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    payload, path = evidence(tmp_path, now)
    payload["maximum_cross_capture_yaw_change_rad"] = 0.02
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="yaw change is excessive"):
        load_rotation_lease(
            path,
            required_envelope_radius_m=0.511,
            project_root=tmp_path,
            now=now + timedelta(seconds=1),
        )


def test_lease_cannot_outlive_baseline_capture_window(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    payload, path = evidence(tmp_path, now)
    payload["expires_at"] = (now + timedelta(minutes=16)).isoformat()
    payload["authorization_scope"]["expires_at"] = payload["expires_at"]
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="lifetime must be between"):
        load_rotation_lease(
            path,
            required_envelope_radius_m=0.511,
            project_root=tmp_path,
            now=now + timedelta(seconds=1),
        )


def test_raw_side_clearance_requires_live_local_lease(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    _, path = evidence(tmp_path, now)
    lease = load_rotation_lease(
        path,
        required_envelope_radius_m=0.511,
        project_root=tmp_path,
        now=now + timedelta(seconds=1),
    )
    selected = resolve_rotation_clearance_source(
        formal_left_clearance_m=None,
        formal_right_clearance_m=None,
        formal_valid=False,
        lease=lease,
        current_pose=(1.0, 2.0, 1.2),
        current_frame="odom_wheel",
        diagnostic_left_clearance_m=float("inf"),
        diagnostic_right_clearance_m=0.60,
        diagnostic_age_seconds=0.1,
        lidar_fresh=True,
        now=now + timedelta(minutes=1),
    )
    assert selected.valid is True
    assert selected.source == "pose_bound_physical_lease_plus_live_raw_lidar"

    stale = resolve_rotation_clearance_source(
        formal_left_clearance_m=None,
        formal_right_clearance_m=None,
        formal_valid=False,
        lease=lease,
        current_pose=(1.0, 2.0, 1.2),
        current_frame="odom_wheel",
        diagnostic_left_clearance_m=float("inf"),
        diagnostic_right_clearance_m=float("inf"),
        diagnostic_age_seconds=0.31,
        lidar_fresh=True,
        now=now + timedelta(minutes=1),
    )
    assert stale.valid is False
    assert "stale" in stale.reason


def test_persistent_gate_never_uses_diagnostic_values() -> None:
    selected = resolve_rotation_clearance_source(
        formal_left_clearance_m=0.70,
        formal_right_clearance_m=0.80,
        formal_valid=True,
        lease=None,
        current_pose=(0.0, 0.0, 0.0),
        current_frame="odom_wheel",
        diagnostic_left_clearance_m=0.10,
        diagnostic_right_clearance_m=0.10,
        diagnostic_age_seconds=0.0,
        lidar_fresh=True,
    )
    assert selected.valid is True
    assert selected.left_clearance_m == 0.70
    assert selected.source == "persistent_sensor_gate"


def test_pose_bound_lease_allows_only_small_explicit_turns() -> None:
    assert evaluate_rotation_lease_step("l30").allowed
    assert evaluate_rotation_lease_step("r0.5").allowed
    for step in ("f", "l0", "r31", "lbad", "observe"):
        assert not evaluate_rotation_lease_step(step).allowed


def test_stage2_cli_scope_is_exact_and_reports_all_unsafe_variants() -> None:
    safe = {
        "mode": "state_machine_search",
        "semantic_reasoning": True,
        "search_reasoner": "hybrid",
        "search_reasoner_mode": "active",
        "turn_only": True,
        "front_half_plane_only": True,
        "max_motion_steps": 1,
        "max_radius_m": 1.5,
        "semantic_allow_forward": False,
    }
    assert rotation_lease_stage2_scope_errors(**safe) == []
    unsafe = dict(safe)
    unsafe.update({
        "mode": "wander",
        "semantic_reasoning": False,
        "search_reasoner": "legacy",
        "search_reasoner_mode": "shadow",
        "turn_only": False,
        "front_half_plane_only": False,
        "max_motion_steps": 2,
        "max_radius_m": 1.51,
        "semantic_allow_forward": True,
    })
    errors = rotation_lease_stage2_scope_errors(**unsafe)
    assert len(errors) == 9
    assert "--semantic-allow-forward is forbidden" in errors
