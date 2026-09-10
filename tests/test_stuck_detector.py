"""Plan section 32: stuck classification must name the physical cause.

"Nothing new was observed" is not an explanation.  Each case below pins one
cause so the explorer can pick recovery instead of declaring the area searched.
"""

from __future__ import annotations

from app.navigation.recovery_manager import select_escape_rotation
from app.navigation.safety_models import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    STATE_OCCUPIED,
    STATE_UNKNOWN,
    build_local_safety_snapshot,
    evaluate_perception_quality,
    wrap_deg,
)
from app.navigation.search_policy_config import StuckDetectionConfig
from app.navigation.stuck_detector import (
    StuckDetectorInput,
    StuckState,
    detect_stuck,
    requires_recovery,
)


def snapshot(
    *,
    blocked_half_angle_deg: float = 0.0,
    blocked_clearance_m: float = 0.30,
    open_clearance_m: float = 2.5,
    unknown_beyond_deg: float | None = None,
    lidar_fresh: bool = True,
    geometry_validated: bool = True,
):
    evidence = []
    for index in range(SECTOR_COUNT):
        bearing = wrap_deg(index * SECTOR_WIDTH_DEG)
        if blocked_half_angle_deg > 0.0 and abs(bearing) <= blocked_half_angle_deg:
            state, clearance = STATE_OCCUPIED, blocked_clearance_m
        elif unknown_beyond_deg is not None and abs(bearing) > unknown_beyond_deg:
            state, clearance = STATE_UNKNOWN, None
        else:
            state, clearance = STATE_CLEAR, open_clearance_m
        evidence.append(
            {"bearing_deg": bearing, "clearance_m": clearance, "state": state}
        )
    return build_local_safety_snapshot(
        sector_evidence=evidence,
        lidar_fresh=lidar_fresh,
        geometry_validated=geometry_validated,
    )


def perception(**kwargs):
    defaults = dict(
        depth_valid_ratio=0.9,
        near_field_ratio=0.1,
        visual_feature_count=250,
        stable_object_count=5,
        estimated_information_gain=0.6,
        front_clearance_m=2.5,
    )
    defaults.update(kwargs)
    return evaluate_perception_quality(**defaults)


def test_open_corridor_is_free() -> None:
    state = detect_stuck(StuckDetectorInput(), snapshot(), perception())

    assert state is StuckState.FREE
    assert requires_recovery(state) is False


def test_stale_geometry_is_sensor_degraded_not_free() -> None:
    state = detect_stuck(
        StuckDetectorInput(), snapshot(lidar_fresh=False), perception()
    )
    assert state is StuckState.SENSOR_DEGRADED
    # A degraded sensor must not authorise a blind reverse (plan section 44).
    assert requires_recovery(state) is False

    unvalidated = detect_stuck(
        StuckDetectorInput(), snapshot(geometry_validated=False), perception()
    )
    assert unvalidated is StuckState.SENSOR_DEGRADED

    assert detect_stuck(StuckDetectorInput(), None, perception()) is StuckState.SENSOR_DEGRADED


def test_camera_against_wall_is_camera_occluded() -> None:
    state = detect_stuck(
        StuckDetectorInput(),
        snapshot(blocked_half_angle_deg=10.0, blocked_clearance_m=0.30),
        perception(
            near_field_ratio=0.85, estimated_information_gain=0.05, front_clearance_m=0.30
        ),
    )

    assert state is StuckState.CAMERA_OCCLUDED
    assert requires_recovery(state) is True


def test_front_blocked_keeps_side_escape() -> None:
    # Narrow obstacle straight ahead: forward is unsafe, but turning is still
    # possible, so this is FRONT_BLOCKED rather than CORNERED.
    state = detect_stuck(
        StuckDetectorInput(),
        snapshot(blocked_half_angle_deg=10.0, blocked_clearance_m=0.30),
        perception(front_clearance_m=0.55, near_field_ratio=0.3),
    )

    assert state is StuckState.FRONT_BLOCKED
    assert requires_recovery(state) is True


def test_cornered_when_no_turn_direction_is_safe() -> None:
    tight = snapshot(blocked_half_angle_deg=180.0, blocked_clearance_m=0.30)

    state = detect_stuck(StuckDetectorInput(), tight, perception(front_clearance_m=0.30))

    assert state is StuckState.CORNERED
    assert tight.safe_turn_left_deg == 0.0
    assert tight.safe_turn_right_deg == 0.0


def test_commanded_forward_without_odometry_progress_is_no_progress() -> None:
    history = StuckDetectorInput(
        pose_history=[(0.0, 0.0, 0.0), (0.005, 0.0, 0.0), (0.01, 0.0, 0.0)],
        command_history=[{"distance_m": 0.10}, {"distance_m": 0.10}],
        result_history=["SUCCEEDED", "SUCCEEDED"],
    )

    state = detect_stuck(history, snapshot(), perception())

    assert state is StuckState.NO_PROGRESS
    assert requires_recovery(state) is True


def test_real_movement_is_not_no_progress() -> None:
    history = StuckDetectorInput(
        pose_history=[(0.0, 0.0, 0.0), (0.10, 0.0, 0.0), (0.20, 0.0, 0.0)],
        command_history=[{"distance_m": 0.10}, {"distance_m": 0.10}],
    )

    assert detect_stuck(history, snapshot(), perception()) is StuckState.FREE


def test_repeated_viewpoints_count_as_no_progress() -> None:
    history = StuckDetectorInput(repeated_viewpoint_count=2)
    assert detect_stuck(history, snapshot(), perception()) is StuckState.NO_PROGRESS

    history = StuckDetectorInput(repeated_heading_count=3)
    assert detect_stuck(history, snapshot(), perception()) is StuckState.NO_PROGRESS


def test_repeated_navigation_failures_require_recovery() -> None:
    history = StuckDetectorInput(navigation_failures=2)

    assert detect_stuck(history, snapshot(), perception()) is StuckState.RECOVERY_REQUIRED


def test_tight_but_passable_corridor_is_constrained() -> None:
    state = detect_stuck(
        StuckDetectorInput(),
        snapshot(open_clearance_m=0.60),
        perception(front_clearance_m=0.60),
    )

    assert state is StuckState.CONSTRAINED
    assert requires_recovery(state) is False


def test_disabled_detector_never_reports_stuck() -> None:
    state = detect_stuck(
        StuckDetectorInput(navigation_failures=9),
        snapshot(blocked_half_angle_deg=180.0),
        perception(front_clearance_m=0.2),
        StuckDetectionConfig(enabled=False),
    )

    assert state is StuckState.FREE


def test_snapshot_and_perception_may_come_from_the_input() -> None:
    history = StuckDetectorInput(
        local_safety=snapshot(blocked_half_angle_deg=10.0),
        perception_quality=perception(front_clearance_m=0.55, near_field_ratio=0.3),
    )

    assert detect_stuck(history) is StuckState.FRONT_BLOCKED


def test_escape_rotation_prefers_the_open_heading() -> None:
    # A narrow pillar at +-10 deg: rotating 30 deg clears it, 10 deg does not.
    decision = select_escape_rotation(snapshot(blocked_half_angle_deg=10.0))

    assert decision.allowed is True
    assert abs(decision.yaw_deg) == 30.0
    assert decision.post_turn_clearance_m == 2.5


def test_escape_rotation_fails_closed_when_boxed_in() -> None:
    decision = select_escape_rotation(snapshot(blocked_half_angle_deg=180.0))

    assert decision.allowed is False
    assert "RECOVERY_BLOCKED" in decision.reason
    assert len(decision.rejected) == 8

    stale = select_escape_rotation(snapshot(lidar_fresh=False))
    assert stale.allowed is False
    assert "RECOVERY_BLOCKED" in stale.reason


def test_escape_rotation_refuses_unobservable_headings() -> None:
    decision = select_escape_rotation(
        snapshot(blocked_half_angle_deg=10.0, unknown_beyond_deg=20.0)
    )

    assert decision.allowed is False
    assert any("UNOBSERVABLE" in item["reason"] for item in decision.rejected)
