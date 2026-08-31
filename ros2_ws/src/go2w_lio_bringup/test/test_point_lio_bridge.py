import math
import socket

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from go2w_lio_bringup.bridge_protocol import (
    ProtocolError,
    receive_frame,
    require_message_type,
    send_frame,
)
from go2w_lio_bringup.point_lio_bridge import (
    imu_pose_to_base_pose,
    point_cloud_frame,
)


def test_protocol_round_trip_preserves_binary_cloud_payload():
    left, right = socket.socketpair()
    try:
        send_frame(left, {"type": "cloud_in", "frame_id": "utlidar_lidar"}, b"\x00\x7f\xff")
        metadata, payload = receive_frame(right)
    finally:
        left.close()
        right.close()
    assert require_message_type(metadata, {"cloud_in", "imu_in"}) == "cloud_in"
    assert payload == b"\x00\x7f\xff"


def test_protocol_rejects_control_or_generic_topic_message_types():
    with pytest.raises(ProtocolError, match="allow list"):
        require_message_type({"type": "cmd_vel"}, {"cloud_in", "imu_in"})
    with pytest.raises(ProtocolError, match="allow list"):
        require_message_type({"type": "generic_ros_topic"}, {"cloud_in", "imu_in"})


def test_point_cloud_frame_preserves_stamp_fields_layout_and_data():
    cloud = PointCloud2()
    cloud.header = Header(stamp=Time(sec=12, nanosec=34), frame_id="utlidar_lidar")
    cloud.height = 1
    cloud.width = 1
    cloud.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1)]
    cloud.point_step = 4
    cloud.row_step = 4
    cloud.data = b"\x00\x00\x80?"
    metadata, payload = point_cloud_frame(cloud)
    assert metadata["stamp_sec"] == 12
    assert metadata["stamp_nanosec"] == 34
    assert metadata["fields"] == [
        {"name": "x", "offset": 0, "datatype": PointField.FLOAT32, "count": 1}
    ]
    assert payload == b"\x00\x00\x80?"


def test_imu_pose_to_base_pose_inverts_fixed_base_to_imu_transform():
    position, quaternion = imu_pose_to_base_pose(
        position=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        imu2base_quat_xyzw_xyz=(0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0),
    )
    assert position == pytest.approx((-1.0, -2.0, -3.0))
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)


def test_imu_pose_to_base_pose_yaw_reflect_mirrors_world_across_xz_plane():
    position, quaternion = imu_pose_to_base_pose(
        position=(1.0, 2.0, 3.0),
        orientation=(0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
        imu2base_quat_xyzw_xyz=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        yaw_reflect=True,
    )
    assert position == pytest.approx((1.0, -2.0, 3.0))
    assert quaternion == pytest.approx(
        (0.0, 0.0, -math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    )
