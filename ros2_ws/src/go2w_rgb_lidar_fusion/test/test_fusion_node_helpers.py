from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

from go2w_rgb_lidar_fusion.fusion_node import (
    nearest,
    transform_camera_to_lidar,
    transform_points,
)
from go2w_rgb_lidar_fusion.overlay_core import (
    LidarToCameraTransform,
    transform_lidar_to_camera,
)


def stamp_message(nanoseconds):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=nanoseconds // 1_000_000_000,
                nanosec=nanoseconds % 1_000_000_000,
            )
        )
    )


def test_nearest_enforces_timestamp_threshold():
    messages = deque([stamp_message(100), stamp_message(200)])
    assert nearest(messages, 190, 20) is messages[1]
    assert nearest(messages, 300, 20) is None


def test_transform_points_applies_quaternion_then_translation():
    half = np.sqrt(0.5)
    transform = SimpleNamespace(
        rotation=SimpleNamespace(x=0.0, y=0.0, z=half, w=half),
        translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
    )
    result = transform_points(np.asarray([[1.0, 0.0, 0.0]]), transform)
    assert result[0] == pytest.approx((1.0, 3.0, 3.0))


def test_camera_to_lidar_is_exact_inverse_of_calibrated_transform():
    angle = np.deg2rad(23.0)
    transform = LidarToCameraTransform(
        rotation=np.asarray(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        translation=np.asarray([0.31, -0.07, 0.12]),
        source="synthetic",
    )
    points_lidar = np.asarray([[1.2, -0.4, 2.8], [-0.2, 0.5, 1.0]])
    points_camera = transform_lidar_to_camera(points_lidar, transform)
    assert transform_camera_to_lidar(points_camera, transform) == pytest.approx(
        points_lidar
    )
