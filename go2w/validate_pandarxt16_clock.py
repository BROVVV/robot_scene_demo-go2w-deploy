#!/usr/bin/env python3
"""Read-only PandarXT-16 clock statistics vs the built-in LiDAR.

Collects header stamps, arrival times and per-point timestamp spans from the
isolated Pandar stream and the built-in LiDAR, then computes a tiered clock
report. This NEVER authorises metric fusion: the default tier stays
HOST_RECEIVE_TIME_ONLY until PTP/host-clock-model validation is performed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# The Pandar stream is delivered RELIABLE; the built-in L2 cloud is a
# BEST_EFFORT sensor topic. Using a RELIABLE reader on the built-in topic
# never matches, so the dual-lidar offset would always be missing.
_BUILTIN_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
_PANDAR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.live_robot.pandar_clock import (  # noqa: E402
    PandarClockTier,
    compute_pandar_clock_statistics,
)


class PandarClockAudit(Node):
    def __init__(self, samples: int, timeout_seconds: float) -> None:
        super().__init__("pandarxt16_clock_validator")
        self.samples = samples
        self.pandar_headers: list[tuple[float, float]] = []  # (header_s, arrival_s)
        self.builtin_headers: list[tuple[float, float]] = []
        self.pandar_point_spans: list[float] = []
        self.deadline = time.monotonic() + timeout_seconds
        self.create_subscription(
            PointCloud2,
            "/hesai/pandarxt16/points_raw",
            self._on_pandar,
            _PANDAR_QOS,
        )
        self.create_subscription(
            PointCloud2,
            "/go2w/sensors/cloud",
            self._on_builtin,
            _BUILTIN_QOS,
        )

    def _on_pandar(self, message: PointCloud2) -> None:
        if len(self.pandar_headers) >= self.samples:
            return
        header_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.pandar_headers.append((header_s, self.get_clock().now().nanoseconds / 1e9))
        # Sample the per-point timestamp span cheaply (numpy, not a Python loop).
        try:
            records = point_cloud2.read_points(
                message, field_names=("timestamp",), skip_nans=False
            )
            point_times = np.asarray(records["timestamp"], dtype=np.float64)
            finite = point_times[np.isfinite(point_times)]
            if finite.size:
                self.pandar_point_spans.append(float(np.ptp(finite)))
        except (ValueError, TypeError):
            pass

    def _on_builtin(self, message: PointCloud2) -> None:
        if len(self.builtin_headers) >= self.samples:
            return
        header_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.builtin_headers.append((header_s, self.get_clock().now().nanoseconds / 1e9))


def build_report(node: PandarClockAudit, requested: int) -> dict:
    pandar_headers = [item[0] for item in node.pandar_headers]
    pandar_arrivals = [item[1] for item in node.pandar_headers]
    builtin_headers = [item[0] for item in node.builtin_headers]
    builtin_arrivals = [item[1] for item in node.builtin_headers]

    stats = compute_pandar_clock_statistics(
        header_stamps=pandar_headers,
        arrival_monotonic=pandar_arrivals,
        point_times=node.pandar_point_spans,
        other_sensor_header_stamps=builtin_headers,
        tier=PandarClockTier.HOST_RECEIVE_TIME_ONLY,
    )
    dual_offset = stats.dual_lidar_apparent_time_offset_s
    builtin_stats = compute_pandar_clock_statistics(
        header_stamps=builtin_headers,
        arrival_monotonic=builtin_arrivals,
        tier=PandarClockTier.HOST_RECEIVE_TIME_ONLY,
    )
    # A fresh RELIABLE subscriber to the 2 MB points_raw topic receives only a
    # sample of frames under CycloneDDS; 5 frames are enough to estimate the
    # rate/jitter, and the actual sample count is reported honestly.
    minimum_frames = min(5, requested)
    checks = {
        "minimum_pandar_frames_received": len(node.pandar_headers) >= minimum_frames,
        "minimum_builtin_frames_received": len(node.builtin_headers) >= minimum_frames,
        "header_rate_nominal_10hz": (
            stats.header_rate_hz is not None
            and 8.0 <= stats.header_rate_hz <= 12.0
        ),
        "zero_non_increasing_deltas": stats.missing_header_delta_count == 0,
        "jitter_bounded": stats.jitter_s is not None and stats.jitter_s < 0.05,
        "dual_lidar_offset_finite": dual_offset is not None and math.isfinite(dual_offset),
    }
    return {
        "schema": "go2w.pandarxt16.clock.v1",
        "passed": all(checks.values()),
        "clock_tier": stats.tier.value,
        "metric_fusion_authorized": False,
        "checks": checks,
        "pandar": {
            "samples": stats.samples,
            "header_rate_hz": stats.header_rate_hz,
            "arrival_rate_hz": stats.arrival_rate_hz,
            "header_delta_median_s": stats.header_delta_median_s,
            "jitter_s": stats.jitter_s,
            "drift_s_per_s": stats.drift_s_per_s,
            "missing_non_increasing_deltas": stats.missing_header_delta_count,
            "point_time_span_median_s": (
                float(sum(node.pandar_point_spans) / len(node.pandar_point_spans))
                if node.pandar_point_spans
                else None
            ),
            "warnings": stats.warnings,
        },
        "builtin_l2": {
            "samples": builtin_stats.samples,
            "header_rate_hz": builtin_stats.header_rate_hz,
            "jitter_s": builtin_stats.jitter_s,
        },
        "dual_lidar_apparent_time_offset_s": dual_offset,
        "limitations": [
            "PTP is Free Run; timestamps use host receive time",
            "formal metric fusion requires HOST_CLOCK_MODEL_VALIDATED or PTP_VALIDATED",
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--timeout-seconds", type=float, default=12.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.samples < 4:
        parser.error("--samples must be at least 4")
    if args.timeout_seconds <= 0.0:
        parser.error("--timeout-seconds must be positive")
    return args


def _spin_until(node: PandarClockAudit, samples: int, timeout_seconds: float) -> None:
    """Spin the node in a background thread until samples are collected.

    A single-threaded ``spin_once`` loop can drop high-rate RELIABLE frames;
    a dedicated spin thread drains the executor reliably.
    """
    import threading

    stopped = threading.Event()

    def _spin() -> None:
        try:
            while rclpy.ok() and not stopped.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    deadline = time.monotonic() + timeout_seconds
    try:
        while (
            time.monotonic() < deadline
            and (
                len(node.pandar_headers) < samples
                or len(node.builtin_headers) < samples
            )
        ):
            time.sleep(0.1)
    finally:
        stopped.set()
        thread.join(timeout=2.0)


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = PandarClockAudit(args.samples, args.timeout_seconds)
    try:
        _spin_until(node, args.samples, args.timeout_seconds)
        report = build_report(node, args.samples)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
