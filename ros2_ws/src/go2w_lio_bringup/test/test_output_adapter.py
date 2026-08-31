import numpy as np
from geometry_msgs.msg import Pose
import pytest

from go2w_lio_bringup.output_adapter import transform_lidar_points_to_odom


def test_registered_cloud_applies_lidar_extrinsic_then_odom_pose():
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    pose.orientation.z = np.sin(np.pi / 4.0)
    pose.orientation.w = np.cos(np.pi / 4.0)
    lidar2base = (0.0, 0.0, 0.0, 1.0, 0.25, 0.0, 0.5)
    result = transform_lidar_points_to_odom(
        np.asarray([[1.0, 0.0, 0.0]]), pose, lidar2base
    )
    np.testing.assert_allclose(result, [[1.0, 3.25, 3.5]], atol=1e-12)


def test_registered_cloud_rejects_nonfinite_points():
    pose = Pose()
    pose.orientation.w = 1.0
    with pytest.raises(ValueError, match="finite"):
        transform_lidar_points_to_odom(
            np.asarray([[float("nan"), 0.0, 0.0]]),
            pose,
            (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        )
