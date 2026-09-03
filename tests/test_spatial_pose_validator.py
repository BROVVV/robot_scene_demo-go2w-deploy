from __future__ import annotations

from app.spatial.models import SpatialPose
from app.spatial.spatial_pose_validator import (
    DEGRADED,
    HEALTHY,
    RECOVERING,
    MotionEvidence,
    SpatialPoseValidator,
)


def test_rotation_only_lio_translation_is_rejected() -> None:
    validator = SpatialPoseValidator(recovery_valid_samples=3)
    assert validator.validate(SpatialPose(0.0, 0.0), timestamp=1.0).accepted
    result = validator.validate(
        SpatialPose(0.31, 0.0, yaw=0.5),
        MotionEvidence(command_type="ROTATE", requested_turn_deg=30.0, wheel_delta_xy_m=0.01),
        timestamp=2.0,
    )
    assert not result.accepted
    assert result.health == DEGRADED
    assert result.reason_code == "LIO_DRIFT_DURING_ROTATION"
    assert result.accepted_pose is not None
    assert result.accepted_pose.x == 0.0


def test_validator_requires_a_short_recovery_streak() -> None:
    validator = SpatialPoseValidator(recovery_valid_samples=3)
    validator.validate(SpatialPose(0.0, 0.0), timestamp=1.0)
    validator.validate(
        SpatialPose(0.35, 0.0),
        MotionEvidence(requested_turn_deg=30.0, wheel_delta_xy_m=0.0),
        timestamp=2.0,
    )
    one = validator.validate(SpatialPose(0.01, 0.0), timestamp=3.0)
    two = validator.validate(SpatialPose(0.02, 0.0), timestamp=4.0)
    three = validator.validate(SpatialPose(0.03, 0.0), timestamp=5.0)
    assert one.health == RECOVERING and not one.accepted
    assert two.health == RECOVERING and not two.accepted
    assert three.health == HEALTHY and three.accepted


def test_invalid_pose_does_not_replace_last_good() -> None:
    validator = SpatialPoseValidator()
    first = validator.validate(SpatialPose(1.0, 2.0), timestamp=1.0)
    assert first.accepted
    result = validator.validate(None, timestamp=2.0)
    assert not result.accepted
    assert result.accepted_pose is not None
    assert result.accepted_pose.x == 1.0
