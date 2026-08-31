"""Read-only ROS 2 endpoint for an isolated official ROS 1 Point-LIO process."""

from __future__ import annotations

import math
import queue
import socket
import threading
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import TransformBroadcaster

from .bridge_protocol import ProtocolError, receive_frame, require_message_type, send_frame


HOST = "127.0.0.1"
PORT = 29876
ALLOWED_OUTPUT = {"odom_out", "cloud_out"}


def _finite(values) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ProtocolError("non-finite Point-LIO output")
    return result


def _normalize_quaternion(values) -> np.ndarray:
    quaternion = np.asarray(_finite(values), dtype=np.float64)
    if quaternion.shape != (4,):
        raise ProtocolError("quaternion must contain four values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ProtocolError("invalid zero quaternion")
    return quaternion / norm


def _quaternion_multiply_xyzw(left, right) -> np.ndarray:
    lx, ly, lz, lw = _normalize_quaternion(left)
    rx, ry, rz, rw = _normalize_quaternion(right)
    return _normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def _rotation_matrix_xyzw(values) -> np.ndarray:
    x, y, z, w = _normalize_quaternion(values)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _quaternion_from_matrix_xyzw(matrix) -> np.ndarray:
    """Eigen-style quaternion (x, y, z, w) from a 3x3 rotation matrix."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        root = math.sqrt(trace + 1.0)
        w = 0.5 * root
        root = 0.5 / root
        x = (matrix[2, 1] - matrix[1, 2]) * root
        y = (matrix[0, 2] - matrix[2, 0]) * root
        z = (matrix[1, 0] - matrix[0, 1]) * root
    else:
        diagonal = np.diag(matrix)
        axis = int(np.argmax(diagonal))
        next_axis = (axis + 1) % 3
        next_next = (axis + 2) % 3
        root = math.sqrt(diagonal[axis] - diagonal[next_axis] -
                         diagonal[next_next] + 1.0)
        quaternion = np.zeros(4)
        quaternion[axis] = 0.5 * root
        root = 0.5 / root
        w = (matrix[next_next, next_axis] -
             matrix[next_axis, next_next]) * root
        quaternion[next_axis] = (
            matrix[next_axis, axis] + matrix[axis, next_axis]) * root
        quaternion[next_next] = (
            matrix[next_next, axis] + matrix[axis, next_next]) * root
        x, y, z = quaternion[0], quaternion[1], quaternion[2]
    return _normalize_quaternion((x, y, z, w))


def imu_pose_to_base_pose(
    position,
    orientation,
    imu2base_quat_xyzw_xyz,
    yaw_reflect: bool = False,
):
    """Convert Point-LIO's world<-IMU pose into the audited world<-base pose."""

    extrinsic = _finite(imu2base_quat_xyzw_xyz)
    if len(extrinsic) != 7:
        raise ProtocolError("IMU-to-base extrinsic must contain seven values")
    world_position_imu = np.asarray(_finite(position), dtype=np.float64)
    if world_position_imu.shape != (3,):
        raise ProtocolError("position must contain three values")
    world_quaternion_imu = _normalize_quaternion(orientation)
    base_quaternion_imu = _normalize_quaternion(extrinsic[:4])
    base_translation_imu = np.asarray(extrinsic[4:], dtype=np.float64)

    imu_quaternion_base = np.asarray(
        (-base_quaternion_imu[0], -base_quaternion_imu[1], -base_quaternion_imu[2], base_quaternion_imu[3])
    )
    imu_rotation_base = _rotation_matrix_xyzw(imu_quaternion_base)
    imu_translation_base = -(imu_rotation_base @ base_translation_imu)
    world_rotation_imu = _rotation_matrix_xyzw(world_quaternion_imu)
    world_position_base = world_position_imu + world_rotation_imu @ imu_translation_base
    world_quaternion_base = _quaternion_multiply_xyzw(
        world_quaternion_imu, imu_quaternion_base
    )
    if yaw_reflect:
        # Go2-W's lidar world frame is yaw-mirrored relative to the physical
        # base frame (live-confirmed: a physical left turn of +10 deg is
        # reported as -9 deg by the stable LiDAR-only LIO). Reflect the world
        # across the X-Z plane so odom yaw/position follow the physical frame.
        world_position_base = (
            world_position_base[0],
            -world_position_base[1],
            world_position_base[2],
        )
        world_rotation_base = _rotation_matrix_xyzw(world_quaternion_base)
        reflection = np.diag((1.0, -1.0, 1.0))
        reflected = reflection @ world_rotation_base @ reflection
        world_quaternion_base = _quaternion_from_matrix_xyzw(reflected)
    return world_position_base, world_quaternion_base


def _stamp_metadata(message) -> dict:
    return {
        "stamp_sec": int(message.header.stamp.sec),
        "stamp_nanosec": int(message.header.stamp.nanosec),
        "frame_id": str(message.header.frame_id),
    }


def point_cloud_frame(message: PointCloud2) -> tuple[dict, bytes]:
    metadata = _stamp_metadata(message)
    metadata.update(
        {
            "type": "cloud_in",
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
        }
    )
    return metadata, bytes(message.data)


def imu_frame(message: Imu, gyro_sign: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> dict:
    if len(gyro_sign) != 3 or any(abs(value) != 1.0 for value in gyro_sign):
        raise ProtocolError("gyro sign correction must be three +/-1 values")
    metadata = _stamp_metadata(message)
    metadata.update(
        {
            "type": "imu_in",
            "orientation": [
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ],
            "orientation_covariance": list(message.orientation_covariance),
            "angular_velocity": [
                float(message.angular_velocity.x) * gyro_sign[0],
                float(message.angular_velocity.y) * gyro_sign[1],
                float(message.angular_velocity.z) * gyro_sign[2],
            ],
            "angular_velocity_covariance": list(message.angular_velocity_covariance),
            "linear_acceleration": [
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ],
            "linear_acceleration_covariance": list(
                message.linear_acceleration_covariance
            ),
        }
    )
    return metadata


class PointLioReadOnlyBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2w_point_lio_readonly_bridge")
        self.declare_parameter("input_timeout_seconds", 0.3)
        self.declare_parameter("maximum_path_poses", 10000)
        self.declare_parameter("path_publish_period_seconds", 1.0)
        self.declare_parameter(
            "extrinsic_imu2base_quat_xyzw_xyz", Parameter.Type.DOUBLE_ARRAY
        )
        self.declare_parameter(
            "gyro_sign_correction",
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter("yaw_reflect", False)
        self._imu2base = tuple(
            float(value)
            for value in self.get_parameter("extrinsic_imu2base_quat_xyzw_xyz").value
        )
        if len(self._imu2base) != 7:
            raise ValueError("official IMU-to-base extrinsic is required")
        self._gyro_sign = tuple(
            float(value)
            for value in self.get_parameter("gyro_sign_correction").value
        )
        if (
            len(self._gyro_sign) != 3
            or any(abs(value) != 1.0 for value in self._gyro_sign)
        ):
            raise ValueError("gyro sign correction must be three +/-1 values")
        self._yaw_reflect = bool(self.get_parameter("yaw_reflect").value)
        self._outbound: queue.Queue[tuple[dict, bytes]] = queue.Queue(maxsize=8192)
        self._inbound: queue.Queue[tuple[dict, bytes]] = queue.Queue(maxsize=32)
        self._socket = None
        self._socket_lock = threading.Lock()
        self._stop = threading.Event()
        self._connected = False
        self._dropped = 0
        self._last_odom_monotonic = None
        self._path = Path()
        self._last_path_publish_stamp = None

        self._odom_pub = self.create_publisher(Odometry, "/lio/odom", 10)
        self._path_pub = self.create_publisher(Path, "/lio/path", 10)
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/lio/cloud_registered", qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(DiagnosticArray, "/lio/status", 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(
            PointCloud2,
            "/go2w/lio_input/cloud_raw",
            self._cloud_input,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/go2w/lio_input/imu_raw",
            self._imu_input,
            qos_profile_sensor_data,
        )
        self.create_timer(0.002, self._drain_inbound)
        self.create_timer(0.1, self._watchdog)
        self._connector = threading.Thread(target=self._connect_loop, daemon=True)
        self._sender = threading.Thread(target=self._send_loop, daemon=True)
        self._connector.start()
        self._sender.start()

    def _enqueue(self, metadata: dict, payload: bytes = b"") -> None:
        if not self._connected:
            self._dropped += 1
            return
        try:
            self._outbound.put_nowait((metadata, payload))
        except queue.Full:
            self._dropped += 1

    def _cloud_input(self, message: PointCloud2) -> None:
        if message.header.frame_id != "utlidar_lidar":
            self._dropped += 1
            return
        self._enqueue(*point_cloud_frame(message))

    def _imu_input(self, message: Imu) -> None:
        if message.header.frame_id != "utlidar_imu":
            self._dropped += 1
            return
        self._enqueue(imu_frame(message, self._gyro_sign))

    def _connect_loop(self) -> None:
        while not self._stop.is_set():
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                connection.connect((HOST, PORT))
                with self._socket_lock:
                    self._socket = connection
                    self._connected = True
                while not self._stop.is_set():
                    frame = receive_frame(connection)
                    require_message_type(frame[0], ALLOWED_OUTPUT)
                    self._inbound.put(frame, timeout=0.2)
            except (EOFError, OSError, ProtocolError, queue.Full):
                pass
            finally:
                with self._socket_lock:
                    if self._socket is connection:
                        self._socket = None
                        self._connected = False
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                connection.close()
            self._stop.wait(0.25)

    def _send_loop(self) -> None:
        while not self._stop.is_set():
            try:
                metadata, payload = self._outbound.get(timeout=0.1)
            except queue.Empty:
                continue
            with self._socket_lock:
                connection = self._socket
            if connection is None:
                self._dropped += 1
                continue
            try:
                send_frame(connection, metadata, payload)
            except (OSError, ProtocolError):
                self._dropped += 1
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    def _drain_inbound(self) -> None:
        for _ in range(32):
            try:
                metadata, payload = self._inbound.get_nowait()
            except queue.Empty:
                return
            try:
                message_type = require_message_type(metadata, ALLOWED_OUTPUT)
                if message_type == "odom_out":
                    if payload:
                        raise ProtocolError("odometry frame must not have binary payload")
                    self._publish_odometry(metadata)
                else:
                    self._publish_cloud(metadata, payload)
            except (KeyError, TypeError, ValueError, ProtocolError) as exc:
                self._diagnostic(DiagnosticStatus.ERROR, str(exc))

    @staticmethod
    def _assign_vector(target, values) -> None:
        target.x, target.y, target.z = _finite(values)

    @staticmethod
    def _assign_quaternion(target, values) -> None:
        target.x, target.y, target.z, target.w = _normalize_quaternion(values)

    def _publish_odometry(self, metadata: dict) -> None:
        if (
            metadata["source_frame_id"] != "camera_init"
            or metadata["source_child_frame_id"] != "aft_mapped"
        ):
            raise ProtocolError("unexpected Point-LIO frame contract")
        position, orientation = imu_pose_to_base_pose(
            metadata["position"],
            metadata["orientation"],
            self._imu2base,
            self._yaw_reflect,
        )
        message = Odometry()
        message.header.stamp.sec = int(metadata["stamp_sec"])
        message.header.stamp.nanosec = int(metadata["stamp_nanosec"])
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        self._assign_vector(message.pose.pose.position, position)
        self._assign_quaternion(message.pose.pose.orientation, orientation)
        message.pose.covariance = list(_finite(metadata["pose_covariance"]))
        self._assign_vector(message.twist.twist.linear, metadata["linear_velocity"])
        self._assign_vector(message.twist.twist.angular, metadata["angular_velocity"])
        message.twist.covariance = list(_finite(metadata["twist_covariance"]))
        self._odom_pub.publish(message)

        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self._path.header = message.header
        self._path.poses.append(pose)
        limit = int(self.get_parameter("maximum_path_poses").value)
        if len(self._path.poses) > limit:
            self._path.poses = self._path.poses[-limit:]
        stamp_seconds = float(message.header.stamp.sec) + float(
            message.header.stamp.nanosec
        ) / 1e9
        period = float(self.get_parameter("path_publish_period_seconds").value)
        if (
            self._last_path_publish_stamp is None
            or stamp_seconds - self._last_path_publish_stamp >= period
        ):
            self._path_pub.publish(self._path)
            self._last_path_publish_stamp = stamp_seconds

        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._tf.sendTransform(transform)
        self._last_odom_monotonic = time.monotonic()
        self._diagnostic(DiagnosticStatus.OK, "valid")

    def _publish_cloud(self, metadata: dict, payload: bytes) -> None:
        if metadata["frame_id"] != "camera_init":
            raise ProtocolError("unexpected Point-LIO registered-cloud frame")
        message = PointCloud2()
        message.header.stamp.sec = int(metadata["stamp_sec"])
        message.header.stamp.nanosec = int(metadata["stamp_nanosec"])
        message.header.frame_id = "odom"
        message.height = int(metadata["height"])
        message.width = int(metadata["width"])
        message.fields = [
            PointField(
                name=str(item["name"]),
                offset=int(item["offset"]),
                datatype=int(item["datatype"]),
                count=int(item["count"]),
            )
            for item in metadata["fields"]
        ]
        message.is_bigendian = bool(metadata["is_bigendian"])
        message.point_step = int(metadata["point_step"])
        message.row_step = int(metadata["row_step"])
        message.is_dense = bool(metadata["is_dense"])
        if len(payload) != message.row_step * message.height:
            raise ProtocolError("registered cloud payload length mismatch")
        message.data = payload
        self._cloud_pub.publish(message)

    def _watchdog(self) -> None:
        if not self._connected:
            self._diagnostic(DiagnosticStatus.ERROR, "ros1_bridge_disconnected")
            return
        if self._last_odom_monotonic is None:
            self._diagnostic(DiagnosticStatus.ERROR, "no_odometry")
            return
        age = time.monotonic() - self._last_odom_monotonic
        if age > float(self.get_parameter("input_timeout_seconds").value):
            self._diagnostic(
                DiagnosticStatus.ERROR, "odometry_stale", age_seconds=f"{age:.6f}"
            )

    def _diagnostic(self, level: int, message: str, **values: str) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "go2w_lio/status"
        status.hardware_id = "go2w_utlidar_point_lio"
        status.message = message
        details = {
            "implementation": "unitree_point_lio_unilidar",
            "bridge_host": HOST,
            "generic_topic_forwarding": "false",
            "pose_republished_when_stale": "false",
            "robot_motion_authorized": "false",
            "validation_scope": "stationary_read_only",
            "dropped_bridge_messages": str(self._dropped),
        }
        details.update(values)
        status.values = [KeyValue(key=key, value=value) for key, value in details.items()]
        array.status = [status]
        self._status_pub.publish(array)

    def destroy_node(self):
        self._stop.set()
        with self._socket_lock:
            connection = self._socket
            self._socket = None
            self._connected = False
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        self._connector.join(timeout=1.0)
        self._sender.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointLioReadOnlyBridge()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
