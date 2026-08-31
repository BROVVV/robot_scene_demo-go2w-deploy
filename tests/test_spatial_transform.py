"""Tests for the pure-Python spatial transform chain (plan §5.8).

Covers the optical -> base -> world mapping, yaw handling, axis sign
conventions, quality assignment and the nominal-extrinsic fallback.
"""

from __future__ import annotations

import math

from app.spatial.models import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_METRIC_RGBD,
    SPATIAL_QUALITY_RELATIVE_RGBD,
    SpatialPose,
)
from app.spatial.spatial_transform import (
    camera_optical_to_base,
    camera_point_to_map,
    circular_mean,
    dynamic_merge_distance_m,
    position_variance,
    quality_weight,
    transform_quality,
    weighted_position_mean,
)


def test_optical_to_base_axis():
    """camera x>0 (image right) -> base y negative; camera z -> base x."""
    xyz_base = camera_optical_to_base((0.0, 0.0, 2.0))
    # camera z(forward 2m) + nominal x offset 0.10 -> base x ~2.1
    assert xyz_base[0] > 2.0
    assert abs(xyz_base[1]) < 1e-6
    # camera x>0 (right) -> base y negative
    xyz_right = camera_optical_to_base((0.3, 0.0, 1.0))
    assert xyz_right[1] < 0


def test_pose_zero_forward():
    """Case 1: robot at (0,0,yaw=0), object 2m in front of camera lands ahead."""
    pose = SpatialPose(x=0.0, y=0.0, yaw=0.0)
    mapped = camera_point_to_map((0.0, 0.0, 2.0), pose)
    assert mapped is not None
    assert mapped[0] > 2.0  # forward + camera x-offset
    assert abs(mapped[1]) < 0.01


def test_pose_yaw_90():
    """Case 2: robot yaw=90° -> straight-ahead camera ray maps along +Y in world."""
    pose = SpatialPose(x=1.0, y=1.0, yaw=math.radians(90.0))
    mapped = camera_point_to_map((0.0, 0.0, 1.0), pose)
    assert mapped is not None
    # camera z (forward) rotated by +90 yaw -> +Y world
    assert mapped[1] > 1.5
    assert abs(mapped[0] - 1.0) < 0.15


def test_camera_x_sign_maps_to_base_negative_y():
    """Case 3: camera x>0 (image right) maps to base y<0 direction."""
    xyz_base = camera_optical_to_base((0.4, 0.0, 1.0))
    assert xyz_base[1] < 0
    # Then world y flips sign by yaw rotation.


def test_no_pose_returns_none():
    """Without a robot pose, camera-local cannot be lifted to map -> None."""
    assert camera_point_to_map((0.0, 0.0, 1.0), None) is None


def test_quality_metric_requires_tf_and_map():
    """Case 4: TF unavailable must never claim METRIC_RGBD."""
    q = transform_quality(
        map_available=True,
        pose_available=True,
        transform_source="nominal_extrinsic",
        has_map_frame=True,
    )
    assert q == SPATIAL_QUALITY_RELATIVE_RGBD
    q2 = transform_quality(
        map_available=True,
        pose_available=True,
        transform_source="tf2",
        has_map_frame=True,
    )
    assert q2 == SPATIAL_QUALITY_METRIC_RGBD


def test_weighted_position_mean_and_variance():
    positions = [(1.0, 2.0, 0.0), (1.1, 1.9, 0.0)]
    mean = weighted_position_mean(positions, [1.0, 1.0])
    assert abs(mean[0] - 1.05) < 1e-3
    var = position_variance(positions, mean)
    assert var[0] > 0


def test_circular_mean_wraparound():
    # 350° and 10° are the same heading (≈0°); the result may normalize to
    # 0 or 360 which are the same point on the circle.
    result = circular_mean([350.0, 10.0])
    assert min(abs(result - 0.0), abs(result - 360.0)) < 1e-6


def test_quality_weight():
    assert quality_weight(SPATIAL_QUALITY_METRIC_RGBD) == 1.0
    assert quality_weight(SPATIAL_QUALITY_RELATIVE_RGBD) > 0
    assert quality_weight(SPATIAL_QUALITY_CAMERA_LOCAL) == 0.0


def test_dynamic_merge_distance_size_aware():
    large = dynamic_merge_distance_m(label="办公桌")
    small = dynamic_merge_distance_m(label="水瓶")
    assert large >= small


def test_camera_optical_to_base_nominal_translation():
    """The default nominal translation (0.10, 0, 0.30) offsets the origin."""
    origin_base = camera_optical_to_base((0.0, 0.0, 0.0))
    assert abs(origin_base[0] - 0.10) < 1e-6
    assert origin_base[2] > 0.29
