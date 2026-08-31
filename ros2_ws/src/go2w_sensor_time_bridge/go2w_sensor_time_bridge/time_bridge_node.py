"""Preserve raw LIO inputs and publish host-clock-aligned sensor copies."""

from __future__ import annotations

import copy
from pathlib import Path

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2

from .time_sync_core import load_time_sync, transform_seconds


def seconds_from_stamp(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def stamp_from_seconds(value: float) -> Time:
    seconds = int(value)
    nanoseconds = int(round((value - seconds) * 1_000_000_000.0))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


class TimeBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2w_sensor_time_bridge")
        self.declare_parameter("config_file", "")
        self.declare_parameter("allow_unstable_alignment", False)
        path = str(self.get_parameter("config_file").value).strip()
        self._config = load_time_sync(path) if path else None
        self._allow_unstable = bool(
            self.get_parameter("allow_unstable_alignment").value
        )
        # Unitree's official /utlidar publishers offer RELIABLE data.  Use a
        # matching profile here; the generic sensor-data BEST_EFFORT profile
        # can discover the topics but does not receive samples on this rig.
        reliable_sensor_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._raw_cloud_pub = self.create_publisher(
            PointCloud2, "/go2w/lio_input/cloud_raw", reliable_sensor_qos
        )
        self._raw_imu_pub = self.create_publisher(
            Imu, "/go2w/lio_input/imu_raw", reliable_sensor_qos
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/go2w/sensors/cloud", reliable_sensor_qos
        )
        self._imu_pub = self.create_publisher(
            Imu, "/go2w/sensors/lidar_imu", reliable_sensor_qos
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/go2w/sensors/time_status", 10
        )
        self.create_subscription(
            PointCloud2, "/utlidar/cloud", self._cloud, reliable_sensor_qos
        )
        self.create_subscription(
            Imu, "/utlidar/imu", self._imu, reliable_sensor_qos
        )

    def _cloud(self, message: PointCloud2) -> None:
        self._raw_cloud_pub.publish(message)
        self._align_and_publish(message, self._cloud_pub, "cloud")

    def _imu(self, message: Imu) -> None:
        self._raw_imu_pub.publish(message)
        self._align_and_publish(message, self._imu_pub, "imu")

    def _align_and_publish(self, message, publisher, stream: str) -> None:
        config = self._config
        if config is None:
            self._diagnostic(DiagnosticStatus.ERROR, stream, "config_missing")
            return
        if not config["stable"] and not self._allow_unstable:
            self._diagnostic(DiagnosticStatus.ERROR, stream, "fit_unstable")
            return
        stream_config = config.get(stream, config)
        if not stream_config["stable"] and not self._allow_unstable:
            self._diagnostic(
                DiagnosticStatus.ERROR, stream, "stream_fit_unstable"
            )
            return
        sensor_seconds = seconds_from_stamp(message.header.stamp)
        try:
            aligned_seconds = transform_seconds(
                sensor_seconds,
                stream_config["scale"],
                stream_config["offset_seconds"],
            )
        except ValueError as exc:
            self._diagnostic(DiagnosticStatus.ERROR, stream, str(exc))
            return
        aligned = copy.deepcopy(message)
        aligned.header.stamp = stamp_from_seconds(aligned_seconds)
        publisher.publish(aligned)
        self._diagnostic(
            DiagnosticStatus.OK,
            stream,
            "aligned",
            original_stamp=f"{sensor_seconds:.9f}",
            aligned_stamp=f"{aligned_seconds:.9f}",
            alignment_fit=stream,
        )

    def _diagnostic(self, level, stream, message, **values) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = f"go2w_sensor_time_bridge/{stream}"
        status.hardware_id = "go2w_utlidar"
        status.message = message
        base = {
            "raw_relative_time_preserved": "true",
            "point_time_field_modified": "false",
            "aligned_output_enabled": str(
                bool(
                    self._config
                    and (self._config["stable"] or self._allow_unstable)
                )
            ).lower(),
        }
        base.update(values)
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in base.items()]
        array.status = [status]
        self._diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimeBridge()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
