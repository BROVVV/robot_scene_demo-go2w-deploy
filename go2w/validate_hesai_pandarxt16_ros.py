#!/usr/bin/env python3
"""Read-only acceptance check for the isolated external PandarXT-16 stream."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from hesai_ros_driver.msg import LossPacket
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


REQUIRED_FIELDS = {"x", "y", "z", "intensity", "ring", "timestamp"}
EXPECTED_RINGS = set(range(16))


class PandarAudit(Node):
    def __init__(self, cloud_topic: str, loss_topic: str, sample_count: int) -> None:
        super().__init__("pandarxt16_readonly_validator")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sample_count = sample_count
        self.frames: list[dict] = []
        self.loss_samples: list[dict] = []
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos)
        self.create_subscription(LossPacket, loss_topic, self._on_loss, qos)

    def _on_loss(self, message: LossPacket) -> None:
        self.loss_samples.append(
            {
                "total_packet_count": int(message.total_packet_count),
                "total_packet_loss_count": int(message.total_packet_loss_count),
            }
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        if len(self.frames) >= self.sample_count:
            return
        field_names = {field.name for field in message.fields}
        if not REQUIRED_FIELDS.issubset(field_names):
            self.frames.append(
                {
                    "stamp": message.header.stamp.sec
                    + message.header.stamp.nanosec * 1e-9,
                    "frame_id": message.header.frame_id,
                    "fields": sorted(field_names),
                    "points": int(message.width * message.height),
                    "parse_error": "required_fields_missing",
                }
            )
            return

        records = point_cloud2.read_points(
            message,
            field_names=("x", "y", "z", "intensity", "ring", "timestamp"),
            skip_nans=False,
        )
        x = np.asarray(records["x"], dtype=np.float64)
        y = np.asarray(records["y"], dtype=np.float64)
        z = np.asarray(records["z"], dtype=np.float64)
        ranges = np.sqrt(x * x + y * y + z * z)
        finite = np.isfinite(ranges)
        valid = finite & (ranges > 0.05)
        rings = np.asarray(records["ring"], dtype=np.int64)
        point_times = np.asarray(records["timestamp"], dtype=np.float64)
        valid_ranges = ranges[valid]
        self.frames.append(
            {
                "stamp": message.header.stamp.sec
                + message.header.stamp.nanosec * 1e-9,
                "arrival_monotonic": time.monotonic(),
                "frame_id": message.header.frame_id,
                "fields": sorted(field_names),
                "height": int(message.height),
                "width": int(message.width),
                "point_step": int(message.point_step),
                "points": int(ranges.size),
                "finite_points": int(np.count_nonzero(finite)),
                "valid_points_gt_5cm": int(np.count_nonzero(valid)),
                "valid_rings": sorted({int(value) for value in rings[valid]}),
                "point_times_finite": bool(np.all(np.isfinite(point_times))),
                "point_time_span_s": float(np.ptp(point_times)),
                "valid_range_quantiles_m": (
                    [
                        float(value)
                        for value in np.quantile(
                            valid_ranges, (0.0, 0.01, 0.5, 0.99, 1.0)
                        )
                    ]
                    if valid_ranges.size
                    else []
                ),
            }
        )


def build_report(node: PandarAudit, requested_samples: int) -> dict:
    frames = node.frames
    stamps = [float(frame["stamp"]) for frame in frames]
    header_intervals = np.diff(stamps)
    arrival_intervals = np.diff(
        [float(frame.get("arrival_monotonic", math.nan)) for frame in frames]
    )
    valid_fractions = [
        frame.get("valid_points_gt_5cm", 0) / max(frame.get("points", 0), 1)
        for frame in frames
    ]
    last_loss = node.loss_samples[-1] if node.loss_samples else None
    header_rate = (
        float(1.0 / np.median(header_intervals))
        if len(header_intervals) and np.all(header_intervals > 0.0)
        else None
    )
    arrival_rate = (
        float(1.0 / np.median(arrival_intervals))
        if len(arrival_intervals)
        and np.all(np.isfinite(arrival_intervals))
        and np.all(arrival_intervals > 0.0)
        else None
    )
    checks = {
        "requested_frames_received": len(frames) >= requested_samples,
        "unvalidated_sensor_frame_preserved": bool(frames)
        and all(
            frame.get("frame_id") == "pandarxt16_link_unvalidated"
            for frame in frames
        ),
        "required_fields_present": bool(frames)
        and all(REQUIRED_FIELDS.issubset(frame.get("fields", [])) for frame in frames),
        "64000_points_per_complete_frame": bool(frames)
        and all(frame.get("points") == 64_000 for frame in frames),
        "all_16_channels_have_valid_returns": bool(frames)
        and all(set(frame.get("valid_rings", [])) == EXPECTED_RINGS for frame in frames),
        "strictly_increasing_header_stamps": len(stamps) >= 2
        and all(new > old for old, new in zip(stamps, stamps[1:])),
        "nominal_10hz_rate": header_rate is not None and 9.0 <= header_rate <= 11.0,
        "valid_return_fraction_at_least_80pct": bool(valid_fractions)
        and min(valid_fractions) >= 0.80,
        "finite_per_point_timestamps": bool(frames)
        and all(frame.get("point_times_finite", False) for frame in frames),
        "one_revolution_timestamp_span": bool(frames)
        and all(0.08 <= frame.get("point_time_span_s", math.inf) <= 0.12 for frame in frames),
        "packet_loss_sample_received": last_loss is not None,
        "reported_packet_loss_is_zero": last_loss is not None
        and last_loss["total_packet_count"] > 0
        and last_loss["total_packet_loss_count"] == 0,
    }
    return {
        "schema": "go2w.hesai_pandarxt16.readonly_acceptance.v1",
        "passed": all(checks.values()),
        "diagnostic_only": True,
        "authorizes_motion": False,
        "authorizes_safety_integration": False,
        "checks": checks,
        "summary": {
            "frames": len(frames),
            "header_rate_hz": header_rate,
            "arrival_rate_hz": arrival_rate,
            "minimum_valid_return_fraction": min(valid_fractions, default=None),
            "median_valid_return_fraction": (
                float(np.median(valid_fractions)) if valid_fractions else None
            ),
            "last_packet_counters": last_loss,
            "last_valid_range_quantiles_m": (
                frames[-1].get("valid_range_quantiles_m") if frames else None
            ),
        },
        "limitations": [
            "mounting transform and full-body swept envelope are not measured",
            "PTP is free-running; host receive timestamps are used",
            "firetime correction file is not installed",
            "zero and near-zero returns must be filtered before downstream use",
            "stream is not connected to /go2w/lidar/* or the motion safety chain",
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout-seconds", type=float, default=8.0)
    parser.add_argument("--cloud-topic", default="/hesai/pandarxt16/points_raw")
    parser.add_argument("--loss-topic", default="/hesai/pandarxt16/packet_loss")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.samples < 2:
        parser.error("--samples must be at least 2")
    if args.timeout_seconds <= 0.0:
        parser.error("--timeout-seconds must be positive")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = PandarAudit(args.cloud_topic, args.loss_topic, args.samples)
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while (
            rclpy.ok()
            and (len(node.frames) < args.samples or not node.loss_samples)
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
        report = build_report(node, args.samples)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
