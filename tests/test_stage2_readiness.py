"""Tests for the machine-readable SemanticNavigation Stage 2/3 readiness."""

from __future__ import annotations

from app.live_robot.stage2_readiness import (
    compute_stage2_readiness,
    compute_stage3_readiness,
)


def _all_true_kwargs() -> dict:
    return {
        "semantic_v1_ready": True,
        "pandar_raw_ready": True,
        "pandar_preprocess_ready": True,
        "pandar_extrinsics_validated": True,
        "current_hardware_geometry_loaded": True,
        "dual_lidar_rotation_observability_valid": True,
        "current_hardware_four_direction_evidence_valid": True,
        "pose_bound_rotation_lease_valid": True,
        "odom_fresh": True,
        "mode_ok": True,
        "motion_action_available": True,
        "no_stage2_error": True,
    }


def test_stage2_ready_when_all_true() -> None:
    report = compute_stage2_readiness(**_all_true_kwargs())
    assert report.ready is True
    assert all(report.checks.values())


def test_stage2_fails_when_any_check_false() -> None:
    kwargs = _all_true_kwargs()
    for key in (
        "semantic_v1_ready",
        "pandar_raw_ready",
        "pandar_extrinsics_validated",
        "current_hardware_geometry_loaded",
        "dual_lidar_rotation_observability_valid",
        "current_hardware_four_direction_evidence_valid",
        "pose_bound_rotation_lease_valid",
        "odom_fresh",
        "mode_ok",
        "motion_action_available",
    ):
        single_fail = {**kwargs, key: False}
        assert compute_stage2_readiness(**single_fail).ready is False, key


def test_stage2_fail_closed_on_missing_inputs() -> None:
    # An empty call must fail closed, not raise.
    report = compute_stage2_readiness()
    assert report.ready is False
    assert not report.checks["pandar_extrinsics_validated"]


def test_stage3_depends_on_stage2() -> None:
    stage2_pass = compute_stage2_readiness(**_all_true_kwargs())
    stage3 = compute_stage3_readiness(
        stage2=stage2_pass,
        stage2_active_turn_only_pass=True,
        semantic_forward_enabled=True,
        front_clearance_valid=True,
        short_forward_scope_valid=True,
    )
    assert stage3.ready is True

    # Even with all forward checks true, a not-ready Stage 2 blocks Stage 3.
    blocked = compute_stage3_readiness(
        stage2=compute_stage2_readiness(),
        stage2_active_turn_only_pass=True,
        semantic_forward_enabled=True,
        front_clearance_valid=True,
        short_forward_scope_valid=True,
    )
    assert blocked.ready is False
    assert blocked.checks["stage2_ready"] is False
