#!/usr/bin/env python3
"""Validate diagnostic RGB-LiDAR readiness without opening metric 3D gates."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_msgs.msg import Detection3DArray


def diagnostic_level(value) -> int:
    return int.from_bytes(value, "little") if isinstance(value, bytes) else int(value)


class GateWatcher(Node):
    def __init__(self) -> None:
        super().__init__("go2w_rgb_lidar_fusion_gate_validator")
        self.fusion_ready_values: list[bool] = []
        self.extrinsics_values: list[bool] = []
        self.overlay_ready_values: list[bool] = []
        self.metric_3d_output_count = 0
        self.diagnostics: list[dict] = []
        self.create_subscription(
            Bool,
            "/perception/fusion_ready",
            lambda message: self.fusion_ready_values.append(bool(message.data)),
            10,
        )
        self.create_subscription(
            Bool,
            "/perception/rgb_lidar_extrinsics_validated",
            lambda message: self.extrinsics_values.append(bool(message.data)),
            10,
        )
        self.create_subscription(
            Bool,
            "/perception/rgb_lidar_overlay_ready",
            lambda message: self.overlay_ready_values.append(bool(message.data)),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/perception/fusion_status",
            self._diagnostic,
            10,
        )
        self.create_subscription(
            Detection3DArray,
            "/perception/detections_3d",
            self._metric_3d_output,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/perception/target_pose_relative",
            self._metric_3d_output,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/perception/target_pose_odom",
            self._metric_3d_output,
            10,
        )

    def _metric_3d_output(self, _message) -> None:
        self.metric_3d_output_count += 1

    def _diagnostic(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "go2w_rgb_lidar_fusion/gate":
                continue
            values = {item.key: item.value for item in status.values}
            self.diagnostics.append(
                {
                    "level": diagnostic_level(status.level),
                    "message": status.message,
                    "blocker": values.get("blocker"),
                    "extrinsics_blocker": values.get("extrinsics_blocker"),
                    "diagnostic_overlay_ready": values.get(
                        "diagnostic_overlay_ready"
                    ),
                    "authorizes_3d_output": values.get("authorizes_3d_output"),
                    "authorizes_motion": values.get("authorizes_motion"),
                }
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=float, default=8.0)
    parser.add_argument("--minimum-samples", type=int, default=3)
    parser.add_argument(
        "--expected-extrinsics-blocker",
        default=(
            "camera-LiDAR extrinsics are not navigation-grade calibrated and "
            "confirmed"
        ),
    )
    args = parser.parse_args()

    rclpy.init()
    node = GateWatcher()
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                len(node.fusion_ready_values) >= args.minimum_samples
                and len(node.extrinsics_values) >= args.minimum_samples
                and len(node.overlay_ready_values) >= args.minimum_samples
                and len(node.diagnostics) >= args.minimum_samples
            ):
                break

        checks = {
            "fusion_ready_received": len(node.fusion_ready_values)
            >= args.minimum_samples,
            "fusion_ready_always_false": bool(node.fusion_ready_values)
            and not any(node.fusion_ready_values),
            "extrinsics_received": len(node.extrinsics_values) >= args.minimum_samples,
            "extrinsics_always_false": bool(node.extrinsics_values)
            and not any(node.extrinsics_values),
            "overlay_ready_received": len(node.overlay_ready_values)
            >= args.minimum_samples,
            "overlay_ready_always_true": bool(node.overlay_ready_values)
            and all(node.overlay_ready_values),
            "diagnostics_received": len(node.diagnostics) >= args.minimum_samples,
            "metric_3d_gate_closed": bool(node.diagnostics)
            and all(
                item["level"] == diagnostic_level(DiagnosticStatus.WARN)
                and item["message"]
                == "diagnostic overlay ready; metric 3D gate closed"
                and item["blocker"]
                == "RGB-LiDAR fusion is disabled or unvalidated"
                and item["extrinsics_blocker"] == args.expected_extrinsics_blocker
                and item["diagnostic_overlay_ready"] == "true"
                and item["authorizes_3d_output"] == "false"
                and item["authorizes_motion"] == "false"
                for item in node.diagnostics
            ),
            "metric_3d_topics_silent": node.metric_3d_output_count == 0,
        }
        payload = {
            "schema_version": "1.0",
            "validation_type": "rgb_lidar_fusion_fail_closed_runtime",
            "robot_connection_assessed": False,
            "robot_motion_commanded": False,
            "fusion_ready_values": node.fusion_ready_values,
            "extrinsics_values": node.extrinsics_values,
            "overlay_ready_values": node.overlay_ready_values,
            "metric_3d_output_count": node.metric_3d_output_count,
            "diagnostics": node.diagnostics,
            "checks": checks,
            "passed": all(checks.values()),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if payload["passed"] else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
