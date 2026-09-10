"""Plan section 32: the 24-sector local safety snapshot must be fail-closed.

The single rule every case below defends: UNKNOWN geometry is never free
space, and no direction reports a clearance number it did not measure.
"""

from __future__ import annotations

from app.navigation.safety_models import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    STATE_OCCUPIED,
    STATE_SELF_OCCLUDED,
    STATE_STALE,
    STATE_UNKNOWN,
    build_local_safety_snapshot,
    clearance_sector_evidence,
    evaluate_perception_quality,
    local_safety_from_clearance,
    make_safety_sector,
)


def sweep(state: str, clearance_m: float | None, *, only: range | None = None) -> list[dict]:
    indices = only if only is not None else range(SECTOR_COUNT)
    return [
        {
            "bearing_deg": index * SECTOR_WIDTH_DEG,
            "clearance_m": clearance_m,
            "state": state,
            "confidence": 1.0,
        }
        for index in indices
    ]


def test_clear_corridor_is_forward_safe_and_fully_rotatable() -> None:
    snapshot = build_local_safety_snapshot(
        sector_evidence=sweep(STATE_CLEAR, 3.0),
        lidar_fresh=True,
        geometry_validated=True,
    )

    assert len(snapshot.sectors) == SECTOR_COUNT
    assert snapshot.forward_safe is True
    assert snapshot.backward_safe is True
    assert snapshot.front_clearance_m == 3.0
    assert snapshot.front_state == STATE_CLEAR
    assert snapshot.safe_turn_left_deg == 180.0
    assert snapshot.safe_turn_right_deg == 180.0
    assert snapshot.nearest_free_sector_deg == 0.0


def test_front_wall_blocks_forward_but_keeps_side_escape() -> None:
    evidence = sweep(STATE_CLEAR, 2.5)
    for item in evidence:
        if abs(item["bearing_deg"]) <= 20.0 or item["bearing_deg"] >= 340.0:
            item["state"] = STATE_OCCUPIED
            item["clearance_m"] = 0.30

    snapshot = build_local_safety_snapshot(
        sector_evidence=evidence,
        lidar_fresh=True,
        geometry_validated=True,
    )

    assert snapshot.forward_safe is False
    assert snapshot.front_state == STATE_OCCUPIED
    assert snapshot.forward_swept_clearance_m == 0.30
    assert snapshot.backward_safe is True
    assert snapshot.nearest_free_sector_deg == 30.0
    assert snapshot.traversable_at(0.0) is False
    assert snapshot.traversable_at(90.0) is True


def test_left_obstacle_limits_left_turn_only() -> None:
    evidence = sweep(STATE_CLEAR, 2.0)
    for item in evidence:
        if 40.0 <= item["bearing_deg"] <= 140.0:
            item["clearance_m"] = 0.25

    snapshot = build_local_safety_snapshot(
        sector_evidence=evidence,
        lidar_fresh=True,
        geometry_validated=True,
        rotation_clearance_m=0.40,
    )

    assert snapshot.forward_safe is True
    assert snapshot.left_clearance_m == 0.25
    assert snapshot.right_clearance_m == 2.0
    assert snapshot.safe_turn_left_deg == 30.0
    assert snapshot.safe_turn_right_deg == 180.0


def test_unknown_sector_is_not_free_space() -> None:
    evidence = sweep(STATE_CLEAR, 2.0)
    for item in evidence:
        if item["bearing_deg"] == 90.0:
            item["state"] = STATE_UNKNOWN
            item["clearance_m"] = 5.0

    snapshot = build_local_safety_snapshot(
        sector_evidence=evidence,
        lidar_fresh=True,
        geometry_validated=True,
    )

    unknown = snapshot.sector_at(90.0)
    assert unknown.state == STATE_UNKNOWN
    assert unknown.clearance_m is None
    assert unknown.observable is False
    assert unknown.traversable is False
    # An unobservable bearing poisons the whole arc instead of averaging away.
    assert snapshot.left_clearance_m is None
    assert snapshot.turn_left_swept_clearance_m is None
    assert snapshot.safe_turn_left_deg == 75.0
    assert snapshot.safe_turn_right_deg == 180.0
    assert snapshot.forward_safe is True


def test_stale_lidar_never_authorises_motion() -> None:
    snapshot = build_local_safety_snapshot(
        sector_evidence=sweep(STATE_STALE, 3.0),
        lidar_fresh=False,
        geometry_validated=True,
    )

    assert snapshot.front_state == STATE_STALE
    assert snapshot.front_clearance_m is None
    assert snapshot.forward_safe is False
    assert snapshot.backward_safe is False
    assert snapshot.nearest_free_sector_deg is None
    assert snapshot.safe_turn_left_deg == 0.0


def test_fresh_geometry_but_unvalidated_extrinsics_blocks_forward() -> None:
    snapshot = build_local_safety_snapshot(
        sector_evidence=sweep(STATE_CLEAR, 3.0),
        lidar_fresh=True,
        geometry_validated=False,
    )

    assert snapshot.forward_swept_clearance_m == 3.0
    assert snapshot.forward_safe is False
    assert snapshot.backward_safe is False


def test_self_occluded_sector_reports_no_clearance() -> None:
    evidence = sweep(STATE_CLEAR, 2.0)
    for item in evidence:
        if item["bearing_deg"] == 180.0:
            item["state"] = STATE_SELF_OCCLUDED
            item["clearance_m"] = 0.10

    snapshot = build_local_safety_snapshot(
        sector_evidence=evidence,
        lidar_fresh=True,
        geometry_validated=True,
    )

    assert snapshot.sector_at(180.0).state == STATE_SELF_OCCLUDED
    assert snapshot.sector_at(180.0).clearance_m is None
    assert snapshot.rear_clearance_m is None
    assert snapshot.backward_safe is False
    assert snapshot.forward_safe is True


def test_unobserved_rear_stays_unknown() -> None:
    snapshot = build_local_safety_snapshot(
        sector_evidence=sweep(STATE_CLEAR, 2.0, only=range(-2, 3)),
        lidar_fresh=True,
        geometry_validated=True,
    )

    assert len(snapshot.sectors) == SECTOR_COUNT
    assert snapshot.rear_state == STATE_UNKNOWN
    assert snapshot.rear_clearance_m is None
    assert snapshot.backward_safe is False
    assert snapshot.forward_safe is True
    assert snapshot.front_clearance_m is None  # front arc is +-45 deg, wider than the scan


def test_conflicting_evidence_in_one_sector_takes_the_worst() -> None:
    snapshot = build_local_safety_snapshot(
        sector_evidence=[
            {"bearing_deg": 3.0, "clearance_m": 2.0, "state": STATE_CLEAR},
            {"bearing_deg": -4.0, "clearance_m": 0.20, "state": STATE_OCCUPIED},
        ],
        lidar_fresh=True,
        geometry_validated=True,
    )

    front = snapshot.sector_at(0.0)
    assert front.state == STATE_OCCUPIED
    assert front.clearance_m == 0.20
    assert front.traversable is False


def test_make_safety_sector_drops_clearance_when_unobservable() -> None:
    sector = make_safety_sector(
        bearing_deg=200.0, clearance_m=1.5, state="unknown", hard_clearance_m=0.4
    )

    assert sector.bearing_deg == -160.0
    assert sector.clearance_m is None
    assert sector.observable is False
    assert sector.traversable is False


def test_bundle_clearance_section_becomes_a_snapshot() -> None:
    snapshot = local_safety_from_clearance(
        {
            "front_m": 0.32,
            "left_m": None,
            "right_m": 1.80,
            "front_status": "measured",
            "left_status": "unknown",
            "right_status": "measured",
            "lidar_fresh": True,
            "rotation_clearance_valid": True,
        },
        geometry_validated=True,
    )

    assert snapshot.forward_safe is False
    assert snapshot.front_state == STATE_OCCUPIED
    assert snapshot.forward_swept_clearance_m == 0.32
    # The +-45 deg side arcs are wider than the bridge's side fans, so only the
    # rotation swept volume carries a number.
    assert snapshot.left_clearance_m is None
    assert snapshot.right_clearance_m is None
    assert snapshot.turn_right_swept_clearance_m == 1.80
    assert snapshot.turn_left_swept_clearance_m is None
    assert snapshot.rear_clearance_m is None
    assert snapshot.backward_safe is False
    # The near front wall stops the rotation span in both directions.
    assert snapshot.safe_turn_left_deg == 0.0
    assert snapshot.safe_turn_right_deg == 0.0
    assert snapshot.nearest_free_sector_deg == -75.0


def test_bundle_no_return_fan_counts_as_clear_and_stale_bundle_does_not() -> None:
    open_corridor = local_safety_from_clearance(
        {
            "front_m": None,
            "left_m": None,
            "right_m": None,
            "front_status": "no_return",
            "left_status": "no_return",
            "right_status": "no_return",
            "lidar_fresh": True,
        },
        geometry_validated=True,
        no_return_clearance_m=3.0,
    )
    assert open_corridor.forward_safe is True
    assert open_corridor.front_clearance_m is None  # +-45 deg arc is wider than the fan
    assert open_corridor.forward_swept_clearance_m == 3.0

    stale = local_safety_from_clearance(
        {
            "front_m": None,
            "left_m": None,
            "right_m": None,
            "front_status": "unknown",
            "left_status": "unknown",
            "right_status": "unknown",
            "lidar_fresh": False,
        },
        geometry_validated=True,
    )
    assert clearance_sector_evidence({"front_status": "unknown"}) == []
    assert stale.forward_safe is False
    assert stale.front_state == STATE_UNKNOWN


def test_perception_quality_flags_camera_against_a_wall() -> None:
    blocked = evaluate_perception_quality(
        depth_valid_ratio=0.95,
        near_field_ratio=0.82,
        visual_feature_count=40,
        stable_object_count=1,
        estimated_information_gain=0.05,
        front_clearance_m=0.30,
    )
    assert blocked.camera_blocked is True
    assert blocked.too_close_to_surface is True
    assert "CAMERA_BLOCKED_BY_NEAR_SURFACE" in blocked.reason

    unknown_front = evaluate_perception_quality(
        depth_valid_ratio=0.9,
        near_field_ratio=0.60,
        estimated_information_gain=0.10,
        front_clearance_m=None,
    )
    assert unknown_front.camera_blocked is True

    open_view = evaluate_perception_quality(
        depth_valid_ratio=0.9,
        near_field_ratio=0.10,
        visual_feature_count=300,
        stable_object_count=6,
        estimated_information_gain=0.65,
        front_clearance_m=2.4,
    )
    assert open_view.camera_blocked is False
    assert open_view.too_close_to_surface is False
    assert open_view.low_information is False
    assert open_view.reason == ""
