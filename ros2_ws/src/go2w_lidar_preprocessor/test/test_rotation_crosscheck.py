from datetime import datetime, timezone
import json
import math

import numpy as np
import pytest

from go2w_lidar_preprocessor.rotation_crosscheck import (
    SECTOR_BEARINGS_RAD,
    build_rotation_evidence,
    compare_sector_capture,
    compare_capture_context,
    matching_baseline_summary,
    summarize_sector_frames,
)


def target_frames(sector: str, count: int = 30) -> list[np.ndarray]:
    bearing = SECTOR_BEARINGS_RAD[sector]
    frames = []
    for index in range(count):
        distance = 0.70 + 0.005 * math.sin(index)
        offsets = (-0.02, 0.0, 0.02)
        frames.append(np.asarray([
            [
                distance * math.cos(bearing + offset),
                distance * math.sin(bearing + offset),
                -0.10,
            ]
            for offset in offsets
        ]))
    return frames


def summaries(frames: list[np.ndarray]) -> dict:
    return {
        sector: summarize_sector_frames(
            frames, sector=sector, expected_distance_m=0.70
        )
        for sector in SECTOR_BEARINGS_RAD
    }


def capture(
    role: str,
    frames: list[np.ndarray],
    x: float = 0.0,
    yaw: float = 0.0,
    captured_at: str = "2026-08-13T12:00:00+00:00",
) -> dict:
    return {
        "schema_version": "1.0",
        "validation_type": "go2w_rotation_sector_capture",
        "capture_role": role,
        "captured_at": captured_at,
        "robot_model": "Unitree Go2-W",
        "posture": "stationary_standing",
        "robot_motion_commanded": False,
        "stationarity": {
            "passed": True,
            "origin_pose": {"x": x, "y": 0.0, "yaw": yaw},
        },
        "sector_summaries": summaries(frames),
        "passed": True,
    }


def test_sector_crosscheck_requires_new_repeatable_return() -> None:
    empty = [np.empty((0, 3)) for _ in range(30)]
    baseline = summarize_sector_frames(
        empty, sector="left", expected_distance_m=0.70
    )
    target = summarize_sector_frames(
        target_frames("left"), sector="left", expected_distance_m=0.70
    )
    report = compare_sector_capture(baseline, target)
    assert report["passed"] is True
    assert report["checks"]["baseline_roi_clear"] is True
    assert report["checks"]["target_repeatably_detected"] is True
    assert target["absolute_bearing_error_deg"] < 1.0


def test_sector_crosscheck_rejects_preexisting_baseline_obstacle() -> None:
    baseline = summarize_sector_frames(
        target_frames("front"), sector="front", expected_distance_m=0.70
    )
    target = summarize_sector_frames(
        target_frames("front"), sector="front", expected_distance_m=0.70
    )
    report = compare_sector_capture(baseline, target)
    assert report["passed"] is False
    assert report["checks"]["baseline_roi_clear"] is False
    assert report["checks"]["target_point_count_contrast"] is False


def test_occupied_baseline_can_pass_only_with_strong_before_after_gain() -> None:
    weak_frames = [frame[:2] for frame in target_frames("left")]
    strong_frames = [
        np.vstack((frame, frame * np.asarray([1.0, 1.0, 1.0]), frame))
        for frame in target_frames("left")
    ]
    baseline = summarize_sector_frames(
        weak_frames, sector="left", expected_distance_m=0.70
    )
    target = summarize_sector_frames(
        strong_frames, sector="left", expected_distance_m=0.70
    )
    report = compare_sector_capture(baseline, target)
    assert report["checks"]["baseline_roi_clear"] is False
    assert report["checks"]["target_point_count_contrast"] is True
    assert report["passed"] is True


def test_occupied_baseline_can_pass_with_stable_range_replacement() -> None:
    def frames(distance: float) -> list[np.ndarray]:
        return [
            np.asarray(
                [[distance, -0.02, 0.3], [distance, 0.0, 0.4], [distance, 0.02, 0.5]]
            )
            for _ in range(30)
        ]

    baseline = summarize_sector_frames(
        frames(0.70), sector="front", expected_distance_m=0.65
    )
    target = summarize_sector_frames(
        frames(0.60), sector="front", expected_distance_m=0.65
    )
    report = compare_sector_capture(baseline, target)
    assert report["checks"]["target_point_count_contrast"] is False
    assert report["checks"]["target_selected_point_contrast"] is False
    assert report["checks"]["target_range_contrast"] is True
    assert report["checks"]["target_contrast"] is True
    assert report["observed_range_shift_m"] == pytest.approx(0.10, abs=1e-4)
    assert report["passed"] is True


def test_matching_baseline_profile_allows_per_sector_target_distance() -> None:
    empty = [np.empty((0, 3)) for _ in range(30)]
    occupied = summarize_sector_frames(
        target_frames("rear"), sector="rear", expected_distance_m=0.70
    )
    clear = summarize_sector_frames(
        empty, sector="rear", expected_distance_m=1.00
    )
    target = summarize_sector_frames(
        [
            np.asarray([[-1.0, -0.02, -0.1], [-1.0, 0.0, -0.1], [-1.0, 0.02, -0.1]])
            for _ in range(30)
        ],
        sector="rear",
        expected_distance_m=1.00,
    )
    selected = matching_baseline_summary(
        {
            "sector_summaries": {"rear": occupied},
            "sector_profiles": {"rear": [clear]},
        },
        target,
    )
    assert selected["expected_distance_m"] == 1.00
    assert compare_sector_capture(selected, target)["passed"] is True


def test_capture_context_enforces_time_translation_and_yaw() -> None:
    baseline = capture("baseline", [], captured_at="2026-08-13T12:00:00+00:00")
    target = capture(
        "front",
        [],
        x=0.02,
        yaw=math.radians(0.8),
        captured_at="2026-08-13T12:10:00+00:00",
    )
    assert compare_capture_context(baseline, target)["passed"] is True
    target["stationarity"]["origin_pose"]["x"] = 0.031
    target["stationarity"]["origin_pose"]["yaw"] = math.radians(1.1)
    target["captured_at"] = "2026-08-13T12:16:00+00:00"
    report = compare_capture_context(baseline, target)
    assert report["passed"] is False
    assert report["checks"]["capture_time_separation_within_limit"] is False
    assert report["checks"]["translation_within_limit"] is False
    assert report["checks"]["yaw_change_within_limit"] is False

    baseline["captured_at"] = "2026-08-13T12:05:00+00:00"
    target["captured_at"] = "2026-08-13T12:00:00+00:00"
    target["stationarity"]["origin_pose"] = {
        "x": 0.0, "y": 0.0, "yaw": 0.0
    }
    reverse_order = compare_capture_context(baseline, target)
    assert reverse_order["passed"] is True
    assert reverse_order["baseline_precedes_target"] is False


def test_complete_evidence_is_short_lived_and_pose_bound(tmp_path) -> None:
    empty = [np.empty((0, 3)) for _ in range(30)]
    captures = {"baseline": capture("baseline", empty)}
    captures.update({
        sector: capture(sector, target_frames(sector))
        for sector in SECTOR_BEARINGS_RAD
    })
    paths = {}
    for role, payload in captures.items():
        path = tmp_path / f"{role}.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        paths[role] = path
    now = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)
    evidence = build_rotation_evidence(
        operator="TEST_OPERATOR",
        configured_envelope_radius_m=0.511,
        physical_clearance_radius_m=0.60,
        swept_clearance_confirmed=True,
        standing_posture_confirmed=True,
        captures=captures,
        capture_paths=paths,
        validity_seconds=600,
        now=now,
    )
    assert evidence["passed"] is True
    assert evidence["authorization_scope"]["type"] == (
        "initial_pose_in_place_rotation_only"
    )
    assert evidence["authorization_scope"]["maximum_translation_m"] == 0.03
    assert evidence["validated_at"] == "2026-08-13T12:00:00+00:00"
    assert evidence["expires_at"] == "2026-08-13T12:10:00+00:00"
    assert all(item["passed"] for item in evidence["sector_validations"].values())


def test_evidence_rejects_insufficient_physical_sweep_and_pose_drift(tmp_path) -> None:
    empty = [np.empty((0, 3)) for _ in range(30)]
    captures = {"baseline": capture("baseline", empty)}
    captures.update({
        sector: capture(
            sector,
            target_frames(sector),
            x=0.04 if sector == "rear" else 0.0,
        )
        for sector in SECTOR_BEARINGS_RAD
    })
    paths = {}
    for role, payload in captures.items():
        path = tmp_path / f"{role}.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        paths[role] = path
    evidence = build_rotation_evidence(
        operator="TEST_OPERATOR",
        configured_envelope_radius_m=0.511,
        physical_clearance_radius_m=0.50,
        swept_clearance_confirmed=True,
        standing_posture_confirmed=True,
        captures=captures,
        capture_paths=paths,
        now=datetime(2026, 8, 13, 12, 5, tzinfo=timezone.utc),
    )
    assert evidence["passed"] is False
    assert evidence["checks"]["near_field_blind_zone_mitigated"] is False
    assert evidence["checks"]["standing_posture_validated"] is False


def test_evidence_rejects_cross_capture_yaw_change(tmp_path) -> None:
    empty = [np.empty((0, 3)) for _ in range(30)]
    captures = {"baseline": capture("baseline", empty)}
    captures.update({
        sector: capture(
            sector,
            target_frames(sector),
            yaw=math.radians(1.1) if sector == "right" else 0.0,
        )
        for sector in SECTOR_BEARINGS_RAD
    })
    paths = {}
    for role, payload in captures.items():
        path = tmp_path / f"{role}.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        paths[role] = path
    evidence = build_rotation_evidence(
        operator="TEST_OPERATOR",
        configured_envelope_radius_m=0.511,
        physical_clearance_radius_m=0.60,
        swept_clearance_confirmed=True,
        standing_posture_confirmed=True,
        captures=captures,
        capture_paths=paths,
        now=datetime(2026, 8, 13, 12, 5, tzinfo=timezone.utc),
    )
    assert evidence["passed"] is False
    assert evidence["maximum_cross_capture_yaw_change_rad"] > math.radians(1.0)
    assert evidence["checks"]["standing_posture_validated"] is False


def test_evidence_validity_cannot_exceed_fifteen_minutes(tmp_path) -> None:
    with pytest.raises(ValueError, match="between 60 and 900"):
        build_rotation_evidence(
            operator="TEST_OPERATOR",
            configured_envelope_radius_m=0.511,
            physical_clearance_radius_m=0.60,
            swept_clearance_confirmed=True,
            standing_posture_confirmed=True,
            captures={},
            capture_paths={},
            validity_seconds=901,
        )


def test_evidence_rejects_stale_capture_window(tmp_path) -> None:
    empty = [np.empty((0, 3)) for _ in range(30)]
    captures = {"baseline": capture("baseline", empty)}
    captures.update({
        sector: capture(sector, target_frames(sector))
        for sector in SECTOR_BEARINGS_RAD
    })
    paths = {}
    for role, payload in captures.items():
        path = tmp_path / f"{role}.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        paths[role] = path
    with pytest.raises(ValueError, match="older than 15 minutes"):
        build_rotation_evidence(
            operator="TEST_OPERATOR",
            configured_envelope_radius_m=0.511,
            physical_clearance_radius_m=0.60,
            swept_clearance_confirmed=True,
            standing_posture_confirmed=True,
            captures=captures,
            capture_paths=paths,
            now=datetime(2026, 8, 13, 12, 16, tzinfo=timezone.utc),
        )
