"""Diagnostic-only preprocessor for the external PandarXT-16 stream.

Subscribes to ``/hesai/pandarxt16/points_raw`` (raw, unvalidated sensor
frame), applies an explicit zero/near-zero return filter, and republishes an
isolated diagnostic cloud plus per-frame status, self-occlusion and
observability debug topics under ``/go2w/hesai/*``.

This node never replaces ``/utlidar/cloud`` or ``/go2w/lidar/scan`` and never
publishes ``/go2w/safety/rotation_clearance_valid``.  Its output is
diagnostic-only until the Pandar transform, clock and self-occlusion are
validated and dual-lidar safety fusion is enabled.
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from .hesai_diagnostics import (
    analyze_pandar_frame,
    azimuth_occupied_bins,
    estimate_self_occlusion_fraction,
)


class PandarConfigError(RuntimeError):
    pass


def load_pandar_preprocess_config(path: str) -> dict:
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if config.get("diagnostic_only") is not True:
        raise PandarConfigError("Pandar preprocessor must be diagnostic_only")
    if config.get("authorizes_motion") is not False:
        raise PandarConfigError("Pandar preprocessor never authorizes motion")
    if config.get("authorizes_safety_integration") is not False:
        raise PandarConfigError("Pandar preprocessor never authorizes safety integration")
    required = (
        "input_topic",
        "zero_return",
        "range_m",
        "ring",
        "freshness",
        "self_occlusion",
        "observability",
        "outputs",
    )
    missing = [key for key in required if key not in config]
    if missing:
        raise PandarConfigError("missing config keys: " + ", ".join(missing))
    zero_max = float(config["zero_return"]["maximum_range_m"])
    if not math.isfinite(zero_max) or zero_max <= 0.0:
        raise PandarConfigError("zero_return maximum_range_m must be positive")
    return config


class PandarDiagnosticPreprocessor(Node):
    def __init__(self) -> None:
        super().__init__("go2w_hesai_diagnostic_preprocessor")
        self.declare_parameter("config_file", "")
        self._ready = False
        self._last_valid_ns = None
        self._last_status = None
        try:
            self._config = load_pandar_preprocess_config(
                str(self.get_parameter("config_file").value)
            )
            self._ready = True
        except (OSError, ValueError, KeyError, TypeError, PandarConfigError) as exc:
            self.get_logger().error(f"Pandar preprocessor gate closed: {exc}")

        self._filtered_pub = self.create_publisher(
            PointCloud2,
            self._config["outputs"]["points_filtered"],
            qos_profile_sensor_data,
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray,
            self._config["outputs"]["status"],
            qos_profile_sensor_data,
        )
        self._self_occlusion_pub = self.create_publisher(
            DiagnosticArray,
            self._config["outputs"]["self_occlusion_debug"],
            qos_profile_sensor_data,
        )
        self._observability_pub = self.create_publisher(
            DiagnosticArray,
            self._config["outputs"]["observability_debug"],
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            self._config["input_topic"],
            self._cloud,
            qos_profile_sensor_data,
        )
        period = float(self._config["freshness"]["status_publish_period_seconds"])
        self.create_timer(max(0.1, period), self._publish_status_timer)

    # -- helpers ------------------------------------------------------------

    def _diagnostic_array(self, name: str, level: int, message: str, values: list) -> DiagnosticArray:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = f"go2w_hesai_diagnostic_preprocessor/{name}"
        status.message = message
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values]
        array.status.append(status)
        return array

    def _status_array(self, payload: dict) -> DiagnosticArray:
        level = DiagnosticStatus.OK if self._ready else DiagnosticStatus.ERROR
        message = "diagnostic stream healthy" if self._ready else "gate closed"
        values = [
            ("diagnostic_only", str(payload.get("diagnostic_only", True))),
            ("frame_id", str(payload.get("frame_id", ""))),
            ("source_topic", str(payload.get("source_topic", ""))),
            ("total_points", str(payload.get("total_points", 0))),
            ("zero_return_fraction", str(payload.get("zero_return_fraction", 0.0))),
            ("valid_return_fraction", str(payload.get("valid_return_fraction", 0.0))),
            ("valid_rings", str(payload.get("valid_rings", []))),
            ("candidate_transform_available", str(payload.get("candidate_transform_available", False))),
            ("transform_validated", str(payload.get("transform_validated", False))),
            ("fresh", str(payload.get("fresh", False))),
        ]
        return self._diagnostic_array("status", level, message, values)

    def _self_occlusion_array(self, payload: dict) -> DiagnosticArray:
        values = [
            ("valid_points", str(payload.get("valid_points", 0))),
            ("inside_body_region", str(payload.get("inside_body_region", 0))),
            ("self_occlusion_fraction", str(payload.get("self_occlusion_fraction", 0.0))),
        ]
        return self._diagnostic_array(
            "self_occlusion", DiagnosticStatus.OK, "self-occlusion estimate", values
        )

    def _observability_array(self, payload: dict) -> DiagnosticArray:
        values = [
            ("bearing_bin_count", str(payload.get("bearing_bin_count", 0))),
            ("occupied_bins", str(payload.get("occupied_bins", 0))),
            ("occupied_fraction", str(payload.get("occupied_fraction", 0.0))),
            ("empty_bins", str(payload.get("empty_bins", 0))),
        ]
        return self._diagnostic_array(
            "observability", DiagnosticStatus.OK, "azimuth occupancy", values
        )

    # -- callbacks -----------------------------------------------------------

    def _cloud(self, message: PointCloud2) -> None:
        if not self._ready:
            return
        try:
            records = point_cloud2.read_points(
                message,
                field_names=("x", "y", "z", "ring", "timestamp"),
                skip_nans=False,
            )
            xyz = np.column_stack(
                tuple(
                    np.asarray(records[name], dtype=np.float64)
                    for name in ("x", "y", "z")
                )
            )
            ring = np.asarray(records["ring"], dtype=np.int64)
            point_time = np.asarray(records["timestamp"], dtype=np.float64)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f"Pandar frame parse failed: {exc}")
            return

        zero_max = float(self._config["zero_return"]["maximum_range_m"])
        ranges = np.linalg.norm(xyz, axis=1)
        finite = np.isfinite(xyz).all(axis=1)
        keep = finite & (ranges > zero_max)
        filtered = xyz[keep]

        freshness = (
            float(self.get_clock().now().nanoseconds - self._last_valid_ns) / 1e9
            if self._last_valid_ns is not None
            else None
        )
        status = analyze_pandar_frame(
            xyz=xyz,
            ring=ring,
            point_timestamp=point_time,
            zero_return_max_m=zero_max,
            expected_rings=int(self._config["ring"]["expected_rings"]),
            frame_id=message.header.frame_id,
            source_topic=self._config["input_topic"],
            candidate_transform_available=True,
            transform_validated=False,
            freshness_seconds=freshness,
        )
        occlusion = estimate_self_occlusion_fraction(
            xyz=xyz,
            body_region=self._config["self_occlusion"]["body_region"],
            zero_return_max_m=zero_max,
        )
        observability = azimuth_occupied_bins(
            xyz=xyz,
            bearing_bin_count=int(self._config["observability"]["azimuth_bin_count"]),
            zero_return_max_m=zero_max,
            min_range_m=float(self._config["observability"]["min_range_m"]),
            max_range_m=float(self._config["observability"]["max_range_m"]),
        )

        header = copy.deepcopy(message.header)
        header.frame_id = self._config["output_frame"]
        if filtered.shape[0]:
            self._filtered_pub.publish(
                point_cloud2.create_cloud_xyz32(header, filtered.tolist())
            )
        self._status_pub.publish(self._status_array(status))
        self._self_occlusion_pub.publish(self._self_occlusion_array(occlusion))
        self._observability_pub.publish(self._observability_array(observability))
        self._last_valid_ns = self.get_clock().now().nanoseconds
        self._last_status = status

    def _publish_status_timer(self) -> None:
        if self._last_status is None:
            return
        timeout_ns = int(float(self._config["freshness"]["timeout_seconds"]) * 1e9)
        age = self.get_clock().now().nanoseconds - self._last_valid_ns
        fresh = 0 <= age <= timeout_ns
        self._last_status["fresh"] = fresh
        self._last_status["freshness_seconds"] = age / 1e9 if self._last_valid_ns else None
        self._status_pub.publish(self._status_array(self._last_status))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PandarDiagnosticPreprocessor()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
