"""Tests for binding the pose-bound rotation lease to current hardware."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import hashlib
import json

import pytest

from app.live_robot.rotation_lease import (
    CAPTURE_ROLES,
    REQUIRED_CHECKS,
    SECTORS,
    build_rotation_lease_binding,
    evaluate_rotation_lease,
    load_rotation_lease,
)


def _build_lease(tmp_path, now, *, binding: dict | None = None) -> object:
    sources = {}
    for role in CAPTURE_ROLES:
        path = tmp_path / f"{role}.json"
        payload = {
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
            "sector_summaries": {
                sector: {"sector": sector, "expected_distance_m": 0.70, "frame_count": 30}
                for sector in SECTORS
            },
            "sector_profiles": {},
        }
        path.write_text(json.dumps(payload), encoding="utf-8")
        sources[role] = {
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    expires = now + timedelta(minutes=10)
    lease = {
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
                "baseline": {"sector": sector, "expected_distance_m": 0.70, "frame_count": 30},
                "target": {"sector": sector, "expected_distance_m": 0.70, "frame_count": 30},
            }
            for sector in SECTORS
        },
        "capture_checks": {role: True for role in CAPTURE_ROLES},
        "capture_evidence": sources,
    }
    if binding is not None:
        lease["hardware_binding"] = binding
    path = tmp_path / "evidence.json"
    path.write_text(json.dumps(lease), encoding="utf-8")
    return path


def test_build_binding_has_required_keys() -> None:
    binding = build_rotation_lease_binding(
        hardware_state_hash="state-hash",
        geometry_hash="geometry-hash",
        extrinsic_version="ext-1",
        clock_tier="host_receive_time_only",
    )
    assert set(binding) == {
        "hardware_state_hash",
        "geometry_hash",
        "extrinsic_version",
        "clock_tier",
    }


def test_matching_binding_loads(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    binding = {
        "hardware_state_hash": "s1",
        "geometry_hash": "g1",
        "extrinsic_version": "e1",
        "clock_tier": "host_receive_time_only",
    }
    path = _build_lease(tmp_path, now, binding=binding)
    loaded = load_rotation_lease(
        path,
        required_envelope_radius_m=0.511,
        project_root=tmp_path,
        now=now,
        expected_binding=binding,
    )
    assert loaded["hardware_binding"] == binding


def test_mismatched_binding_rejected(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    binding = {
        "hardware_state_hash": "s1",
        "geometry_hash": "g1",
        "extrinsic_version": "e1",
        "clock_tier": "host_receive_time_only",
    }
    path = _build_lease(tmp_path, now, binding=binding)
    wrong = {**binding, "hardware_state_hash": "s2"}
    with pytest.raises(ValueError, match="hardware_state_hash mismatch"):
        load_rotation_lease(
            path,
            required_envelope_radius_m=0.511,
            project_root=tmp_path,
            now=now,
            expected_binding=wrong,
        )


def test_missing_binding_key_rejected(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    binding = {
        "hardware_state_hash": "s1",
        "geometry_hash": "g1",
        "extrinsic_version": "e1",
        "clock_tier": "host_receive_time_only",
    }
    path = _build_lease(tmp_path, now, binding=binding)
    # Caller expects a dual_lidar_evidence_hash the lease does not carry.
    with pytest.raises(ValueError, match="missing"):
        load_rotation_lease(
            path,
            required_envelope_radius_m=0.511,
            project_root=tmp_path,
            now=now,
            expected_binding={**binding, "dual_lidar_evidence_hash": "dl1"},
        )


def test_lease_without_binding_still_loads_when_not_required(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    path = _build_lease(tmp_path, now)
    # Legacy lease without a binding remains valid when no expected binding is
    # supplied (backward compatibility).
    load_rotation_lease(
        path,
        required_envelope_radius_m=0.511,
        project_root=tmp_path,
        now=now,
        expected_binding=None,
    )


def test_evaluate_rejects_stale_binding(tmp_path) -> None:
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    binding = {
        "hardware_state_hash": "s1",
        "geometry_hash": "g1",
        "extrinsic_version": "e1",
        "clock_tier": "host_receive_time_only",
    }
    path = _build_lease(tmp_path, now, binding=binding)
    lease = load_rotation_lease(
        path,
        required_envelope_radius_m=0.511,
        project_root=tmp_path,
        now=now,
        expected_binding=binding,
    )
    # Current hardware state changed (mount_changed_since_calibration) so the
    # caller now expects a different state hash; the lease must be invalid.
    decision = evaluate_rotation_lease(
        lease,
        current_pose=(1.0, 2.0, 0.3),
        current_frame="odom_wheel",
        now=now + timedelta(seconds=1),
        expected_binding={**binding, "hardware_state_hash": "s2"},
    )
    assert decision.allowed is False
