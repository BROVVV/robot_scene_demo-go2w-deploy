"""Collect a bounded live clock fit while the Go2-W remains stationary."""

from __future__ import annotations

import argparse
import datetime as dt
import os
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from .time_sync_core import fit_clock, fit_cloud_imu_relative_clock


def stamp_seconds(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


_POINT_FIELD_DTYPES = {
    1: "i1",  # INT8
    2: "u1",  # UINT8
    3: "i2",  # INT16
    4: "u2",  # UINT16
    5: "i4",  # INT32
    6: "u4",  # UINT32
    7: "f4",  # FLOAT32
    8: "f8",  # FLOAT64
}


def point_time_range(message: PointCloud2) -> dict:
    """Return bounded statistics for the existing per-point ``time`` field.

    The view honours row padding and point stride and never changes the message.
    No unit is inferred: that requires vendor documentation or a separate test.
    """
    field = next((item for item in message.fields if item.name == "time"), None)
    if field is None:
        return {"available": False, "reason": "field_missing"}
    dtype_code = _POINT_FIELD_DTYPES.get(field.datatype)
    if dtype_code is None or field.count != 1:
        return {
            "available": False,
            "reason": "unsupported_datatype_or_count",
            "datatype": int(field.datatype),
            "count": int(field.count),
        }
    if message.height <= 0 or message.width <= 0 or message.point_step <= 0:
        return {"available": False, "reason": "empty_cloud"}
    byte_order = ">" if message.is_bigendian else "<"
    dtype = np.dtype(byte_order + dtype_code)
    required = (
        (int(message.height) - 1) * int(message.row_step)
        + (int(message.width) - 1) * int(message.point_step)
        + int(field.offset)
        + dtype.itemsize
    )
    if required > len(message.data):
        return {
            "available": False,
            "reason": "data_buffer_too_short",
            "required_bytes": required,
            "actual_bytes": len(message.data),
        }
    values = np.ndarray(
        shape=(int(message.height), int(message.width)),
        dtype=dtype,
        buffer=message.data,
        offset=int(field.offset),
        strides=(int(message.row_step), int(message.point_step)),
    )
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return {"available": False, "reason": "no_finite_values"}
    minimum = float(np.min(finite))
    maximum = float(np.max(finite))
    return {
        "available": True,
        "datatype": int(field.datatype),
        "count": int(field.count),
        "unit": "unknown_not_inferred",
        "minimum": minimum,
        "maximum": maximum,
        "span": maximum - minimum,
        "finite_points": int(finite.size),
        "total_points": int(message.height) * int(message.width),
    }


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("go2w_time_sync_measurement")
        self.cloud = []
        self.imu = []
        self.cloud_point_time = []
        self.create_subscription(
            PointCloud2, "/utlidar/cloud", self._cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "/utlidar/imu", self._imu, qos_profile_sensor_data
        )

    def _cloud(self, message):
        self.cloud.append((stamp_seconds(message), self.get_clock().now().nanoseconds / 1e9))
        self.cloud_point_time.append(point_time_range(message))

    def _imu(self, message):
        self.imu.append((stamp_seconds(message), self.get_clock().now().nanoseconds / 1e9))


def atomic_yaml(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(value, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main(args=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--output", required=True, type=Path)
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.duration < 120.0:
        raise SystemExit("duration must be at least 120 seconds")
    rclpy.init(args=ros_args)
    node = Collector()
    deadline = time.monotonic() + parsed.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        cloud_fit = fit_clock(node.cloud, minimum_duration_sec=parsed.duration * 0.95)
        imu_fit = fit_clock(node.imu, minimum_duration_sec=parsed.duration * 0.95)
        relative_fit = fit_cloud_imu_relative_clock(
            node.cloud,
            node.imu,
            minimum_duration_sec=parsed.duration * 0.95,
        )
        selected = cloud_fit
        stable = cloud_fit.stable and imu_fit.stable and relative_fit.stable
        available_point_times = [
            item for item in node.cloud_point_time if item.get("available")
        ]
        point_time_summary = {
            "frames_observed": len(node.cloud_point_time),
            "frames_with_valid_time": len(available_point_times),
            "field_modified": False,
            "unit": "unknown_not_inferred",
        }
        if available_point_times:
            point_time_summary.update(
                {
                    "minimum_observed": min(
                        item["minimum"] for item in available_point_times
                    ),
                    "maximum_observed": max(
                        item["maximum"] for item in available_point_times
                    ),
                    "minimum_frame_span": min(
                        item["span"] for item in available_point_times
                    ),
                    "maximum_frame_span": max(
                        item["span"] for item in available_point_times
                    ),
                    "first_frame": available_point_times[0],
                    "last_frame": available_point_times[-1],
                }
            )
        elif node.cloud_point_time:
            point_time_summary["first_error"] = node.cloud_point_time[0]
        payload = {
            "source_clock": "utlidar_cloud_and_imu_header",
            "scale": selected.scale,
            "offset_seconds": selected.offset_seconds,
            "drift_ppm": selected.drift_ppm,
            "fit_rmse_ms": selected.fit_rmse_ms,
            "stable": stable,
            "measurement_duration_sec": min(
                cloud_fit.duration_sec, imu_fit.duration_sec
            ),
            "measured_at": dt.datetime.now(dt.timezone.utc).isoformat(),
            "timestamp_semantics": "host ROS receive time fit; not hardware exposure time",
            "cloud": cloud_fit.__dict__,
            "imu": imu_fit.__dict__,
            "cloud_imu_relative_clock": relative_fit.__dict__,
            "alignment_policy": "per_stream_linear_fit_for_ros_aligned_copies",
            "raw_lio_relative_time_preserved": True,
            "point_time_field_modified": False,
            "point_time_field_statistics": point_time_summary,
        }
        atomic_yaml(parsed.output, payload)
        print(yaml.safe_dump(payload, sort_keys=False))
        return 0 if stable else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
