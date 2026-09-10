#!/usr/bin/env python3
"""ROS 1 side of the localhost-only, sensor/odometry-only Point-LIO bridge.

The wire protocol has exactly two ROS 2 -> ROS 1 message types (PointCloud2 and
Imu) and exactly two ROS 1 -> ROS 2 types (Odometry and registered PointCloud2).
It has no generic topic forwarding and no control-message representation.
"""

from __future__ import annotations

import socket
import sys
import threading
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_SOURCE = PROJECT_ROOT / "ros2_ws/src/go2w_lio_bringup"
sys.path.insert(0, str(PACKAGE_SOURCE))

import rospy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu, PointCloud2, PointField  # noqa: E402

from go2w_lio_bringup.bridge_protocol import (  # noqa: E402
    ProtocolError,
    receive_frame,
    require_message_type,
    send_frame,
)


HOST = "127.0.0.1"
PORT = 29876
ALLOWED_INPUT = {"cloud_in", "imu_in"}
REQUIRED_CLOUD_FIELDS = {"x", "y", "z", "intensity", "ring", "time"}


def _stamp(metadata: dict) -> rospy.Time:
    seconds = int(metadata["stamp_sec"])
    nanoseconds = int(metadata["stamp_nanosec"])
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ProtocolError("invalid sensor timestamp")
    return rospy.Time(seconds, nanoseconds)


def _point_cloud_from_frame(metadata: dict, payload: bytes) -> PointCloud2:
    frame_id = str(metadata["frame_id"])
    if frame_id != "utlidar_lidar":
        raise ProtocolError("unexpected LiDAR frame")
    fields = []
    for item in metadata["fields"]:
        fields.append(
            PointField(
                name=str(item["name"]),
                offset=int(item["offset"]),
                datatype=int(item["datatype"]),
                count=int(item["count"]),
            )
        )
    if not REQUIRED_CLOUD_FIELDS.issubset({field.name for field in fields}):
        raise ProtocolError("Point-LIO input cloud is missing required fields")
    message = PointCloud2()
    message.header.stamp = _stamp(metadata)
    message.header.frame_id = frame_id
    message.height = int(metadata["height"])
    message.width = int(metadata["width"])
    message.fields = fields
    message.is_bigendian = bool(metadata["is_bigendian"])
    message.point_step = int(metadata["point_step"])
    message.row_step = int(metadata["row_step"])
    message.is_dense = bool(metadata["is_dense"])
    if len(payload) != message.row_step * message.height:
        raise ProtocolError("PointCloud2 payload length does not match row layout")
    message.data = payload
    return message


def _vector(target, values) -> None:
    target.x, target.y, target.z = (float(value) for value in values)


def _quaternion(target, values) -> None:
    target.x, target.y, target.z, target.w = (float(value) for value in values)


def _imu_from_frame(metadata: dict) -> Imu:
    if str(metadata["frame_id"]) != "utlidar_imu":
        raise ProtocolError("unexpected IMU frame")
    message = Imu()
    message.header.stamp = _stamp(metadata)
    message.header.frame_id = "utlidar_imu"
    _quaternion(message.orientation, metadata["orientation"])
    _vector(message.angular_velocity, metadata["angular_velocity"])
    _vector(message.linear_acceleration, metadata["linear_acceleration"])
    message.orientation_covariance = [float(v) for v in metadata["orientation_covariance"]]
    message.angular_velocity_covariance = [
        float(v) for v in metadata["angular_velocity_covariance"]
    ]
    message.linear_acceleration_covariance = [
        float(v) for v in metadata["linear_acceleration_covariance"]
    ]
    return message


def _point_cloud_frame(message: PointCloud2) -> tuple[dict, bytes]:
    return (
        {
            "type": "cloud_out",
            "stamp_sec": int(message.header.stamp.secs),
            "stamp_nanosec": int(message.header.stamp.nsecs),
            "frame_id": str(message.header.frame_id),
            "height": int(message.height),
            "width": int(message.width),
            "fields": [
                {
                    "name": field.name,
                    "offset": int(field.offset),
                    "datatype": int(field.datatype),
                    "count": int(field.count),
                }
                for field in message.fields
            ],
            "is_bigendian": bool(message.is_bigendian),
            "point_step": int(message.point_step),
            "row_step": int(message.row_step),
            "is_dense": bool(message.is_dense),
        },
        bytes(message.data),
    )


def _odometry_frame(message: Odometry) -> dict:
    pose = message.pose.pose
    twist = message.twist.twist
    return {
        "type": "odom_out",
        "stamp_sec": int(message.header.stamp.secs),
        "stamp_nanosec": int(message.header.stamp.nsecs),
        "source_frame_id": str(message.header.frame_id),
        "source_child_frame_id": str(message.child_frame_id),
        "position": [pose.position.x, pose.position.y, pose.position.z],
        "orientation": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        "pose_covariance": list(message.pose.covariance),
        "linear_velocity": [twist.linear.x, twist.linear.y, twist.linear.z],
        "angular_velocity": [twist.angular.x, twist.angular.y, twist.angular.z],
        "twist_covariance": list(message.twist.covariance),
    }


class ReadOnlyPointLioEndpoint:
    def __init__(self) -> None:
        self._connection = None
        self._connection_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((HOST, PORT))
        self._server.listen(1)
        self._cloud_pub = rospy.Publisher("/unilidar/cloud", PointCloud2, queue_size=10)
        self._imu_pub = rospy.Publisher("/unilidar/imu", Imu, queue_size=2000)
        self._odom_sub = rospy.Subscriber(
            "/pointlio/odom", Odometry, self._odom, queue_size=100, tcp_nodelay=True
        )
        self._cloud_sub = rospy.Subscriber(
            "/pointlio/cloud_registered",
            PointCloud2,
            self._cloud,
            queue_size=10,
            tcp_nodelay=True,
        )
        rospy.on_shutdown(self.close)

    def _send(self, metadata: dict, payload: bytes = b"") -> None:
        with self._connection_lock:
            connection = self._connection
        if connection is None:
            return
        try:
            with self._send_lock:
                send_frame(connection, metadata, payload)
        except (OSError, ProtocolError):
            self._disconnect(connection)

    def _odom(self, message: Odometry) -> None:
        if (
            message.header.frame_id != "camera_init"
            or message.child_frame_id != "aft_mapped"
        ):
            rospy.logerr_throttle(1.0, "Point-LIO emitted an unexpected odometry frame")
            return
        self._send(_odometry_frame(message))

    def _cloud(self, message: PointCloud2) -> None:
        if message.header.frame_id != "camera_init":
            rospy.logerr_throttle(1.0, "Point-LIO emitted an unexpected cloud frame")
            return
        metadata, payload = _point_cloud_frame(message)
        self._send(metadata, payload)

    def _disconnect(self, connection) -> None:
        with self._connection_lock:
            if self._connection is connection:
                self._connection = None
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        connection.close()

    def serve(self) -> None:
        while not rospy.is_shutdown():
            rospy.loginfo("Point-LIO read-only bridge waiting on %s:%d", HOST, PORT)
            connection, peer = self._server.accept()
            if peer[0] != HOST:
                connection.close()
                continue
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._connection_lock:
                previous = self._connection
                self._connection = connection
            if previous is not None:
                self._disconnect(previous)
            rospy.loginfo("Point-LIO read-only ROS 2 peer connected")
            try:
                while not rospy.is_shutdown():
                    metadata, payload = receive_frame(connection)
                    message_type = require_message_type(metadata, ALLOWED_INPUT)
                    if message_type == "cloud_in":
                        self._cloud_pub.publish(
                            _point_cloud_from_frame(metadata, payload)
                        )
                    else:
                        if payload:
                            raise ProtocolError("IMU frame must not have binary payload")
                        self._imu_pub.publish(_imu_from_frame(metadata))
            except (EOFError, OSError, KeyError, TypeError, ValueError, ProtocolError) as exc:
                if not rospy.is_shutdown():
                    rospy.logwarn("Point-LIO bridge connection closed: %s", exc)
            finally:
                self._disconnect(connection)

    def close(self) -> None:
        with self._connection_lock:
            connection = self._connection
            self._connection = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        self._server.close()


def main() -> None:
    rospy.init_node("go2w_point_lio_readonly_endpoint", disable_signals=False)
    ReadOnlyPointLioEndpoint().serve()


if __name__ == "__main__":
    main()
