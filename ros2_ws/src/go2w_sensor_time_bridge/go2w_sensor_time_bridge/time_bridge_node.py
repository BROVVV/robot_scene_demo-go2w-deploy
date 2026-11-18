"""Preserve raw LIO inputs and publish host-clock-aligned sensor copies."""

from __future__ import annotations

import copy
import math
from pathlib import Path

import rclpy
# RCLError 所在的 C 扩展模块在两个发行版上不同名：Foxy 是 rclpy._rclpy（扩展名
# _rclpy.cpython-38-aarch64-linux-gnu.so），Galactic 起才改叫 _rclpy_pybind11。
# 原来只写后者，于是这个节点在机器狗（Foxy）上**启动即崩**：
#   ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# 它恰好是 /go2w/sensors/cloud 的唯一发布者，而 lidar_preprocessor 只订阅那条话题
# —— 结果整个雷达障碍链断在源头：/go2w/lidar/scan 一个发布者都不发帧，
# global_costmap 的 obstacle_layer 永远没有输入，bridge 的 lidar_stale 恒真。
# rclpy.exceptions 里两个发行版都没有 RCLError，只能这样取。
try:  # Foxy（机器狗）
    from rclpy._rclpy import RCLError
except ImportError:  # Humble 及以后（控制主机）
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
        self.declare_parameter("timestamp_policy", "host_receive")
        self.declare_parameter("publish_raw_lio", True)
        self.declare_parameter("forward_imu", True)
        publish_raw_lio = bool(self.get_parameter("publish_raw_lio").value)
        forward_imu = bool(self.get_parameter("forward_imu").value)
        self._policy = str(self.get_parameter("timestamp_policy").value)
        if self._policy not in {"host_receive", "static_fit"}:
            raise ValueError("timestamp_policy must be host_receive or static_fit")
        self._previous = {}
        path = str(self.get_parameter("config_file").value).strip()
        self._config = load_time_sync(path) if path and self._policy == "static_fit" else None
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
        ) if publish_raw_lio else None
        self._raw_imu_pub = self.create_publisher(
            Imu, "/go2w/lio_input/imu_raw", reliable_sensor_qos
        ) if publish_raw_lio and forward_imu else None
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/go2w/sensors/cloud", reliable_sensor_qos
        )
        self._imu_pub = self.create_publisher(
            Imu, "/go2w/sensors/lidar_imu", reliable_sensor_qos
        ) if forward_imu else None
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/go2w/sensors/time_status", 10
        )
        self.create_subscription(
            PointCloud2, "/utlidar/cloud", self._cloud, reliable_sensor_qos
        )
        if forward_imu:
            self.create_subscription(
                Imu, "/utlidar/imu", self._imu, reliable_sensor_qos
            )

    def _cloud(self, message: PointCloud2) -> None:
        if self._raw_cloud_pub is not None:
            self._raw_cloud_pub.publish(message)
        self._align_and_publish(message, self._cloud_pub, "cloud")

    def _imu(self, message: Imu) -> None:
        if self._raw_imu_pub is not None:
            self._raw_imu_pub.publish(message)
        self._align_and_publish(message, self._imu_pub, "imu")

    def _align_and_publish(self, message, publisher, stream: str) -> None:
        sensor_seconds = seconds_from_stamp(message.header.stamp)
        host_seconds = self.get_clock().now().nanoseconds / 1e9
        previous = self._previous.get(stream)
        self._previous[stream] = (sensor_seconds, host_seconds)
        if (not math.isfinite(sensor_seconds) or sensor_seconds <= 0
                or previous and (sensor_seconds <= previous[0] or host_seconds <= previous[1])):
            self._diagnostic(DiagnosticStatus.ERROR, stream, "clock_regression")
            return
        if self._policy == "host_receive":
            aligned = copy.deepcopy(message)
            aligned.header.stamp = stamp_from_seconds(host_seconds)
            publisher.publish(aligned)
            self._diagnostic(DiagnosticStatus.OK, stream, "host_receive",
                             original_stamp=f"{sensor_seconds:.9f}",
                             aligned_stamp=f"{host_seconds:.9f}",
                             timestamp_policy=self._policy)
            return
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
                    self._policy == "host_receive" or self._config
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
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
