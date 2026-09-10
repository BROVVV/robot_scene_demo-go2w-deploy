#!/usr/bin/env python3
"""Read-only dual-LiDAR rotation observability validation.

Computes, per bearing, whether the swept band ``[footprint, envelope]`` is
fully observable by at least one validated sensor (built-in L2 and/or the
externally mounted PandarXT-16).  The built-in L2 blind intervals come from the
confirmed lidar_preprocess geometry; the Pandar coverage uses a default model
until formal self-occlusion validation replaces it.

The Pandar contributes to formal observability only when its extrinsic is
validated.  This tool never authorises motion and never publishes a transform.
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
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

sys.path.insert(0, str(PROJECT_ROOT / "ros2_ws" / "src" / "go2w_lidar_preprocessor"))

from go2w_lidar_preprocessor.dual_lidar_config import (  # noqa: E402
    load_dual_lidar_safety_config,
    observability_params,
)
from go2w_lidar_preprocessor.dual_lidar_observability import (  # noqa: E402
    compute_dual_lidar_rotation_observability,
    generate_pandar_unobservable_profile,
)
from go2w_lidar_preprocessor.preprocess_core import (  # noqa: E402
    _merge_intervals,
    _ray_box_interval,
    _rectangle_ray_exit_radius,
    rotation_observability_report,
)
from go2w_lidar_preprocessor.config import load_safety_ready_config  # noqa: E402


class FreshnessProbe(Node):
    def __init__(self, pandar_topic: str, builtin_topic: str) -> None:
        super().__init__("dual_lidar_observability_probe")
        # The built-in cloud is a BEST_EFFORT sensor topic; the Pandar raw
        # topic is also best-effort in practice. Using RELIABLE readers never
        # matches, so freshness would always report stale.
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pandar_frames = 0
        self.builtin_frames = 0
        self.pandar_last = None
        self.builtin_last = None
        # Pandar liveness is read from the diagnostic status topic (small,
        # reliable) because a fresh RELIABLE reader on the 2 MB points_raw
        # topic is throttled by CycloneDDS and would look stale.
        self.create_subscription(
            DiagnosticArray,
            "/go2w/hesai/status",
            self._on_pandar_status,
            sensor_qos,
        )
        self.create_subscription(PointCloud2, builtin_topic, self._on_builtin, sensor_qos)

    def _on_pandar_status(self, message: DiagnosticArray) -> None:
        self.pandar_frames += 1
        self.pandar_last = self.get_clock().now().nanoseconds / 1e9

    def _on_builtin(self, message: PointCloud2) -> None:
        self.builtin_frames += 1
        self.builtin_last = self.get_clock().now().nanoseconds / 1e9


def per_bearing_builtin_unobservable(
    p,
    *,
    angular_samples: int = 720,
) -> dict[float, list[tuple[float, float]]]:
    """Recompute per-bearing built-in L2 unobservable intervals."""
    angles = np.linspace(-math.pi, math.pi, angular_samples, endpoint=False)
    result: dict[float, list[tuple[float, float]]] = {}
    for index, angle in enumerate(angles):
        cosine = float(math.cos(angle))
        sine = float(math.sin(angle))
        footprint_edge = _rectangle_ray_exit_radius(
            cosine, sine, p.self_half_length, p.self_half_width
        )
        envelope_edge = p.rotation_envelope_radius
        if footprint_edge >= envelope_edge:
            result[math.degrees(angle)] = []
            continue
        intervals: list[tuple[float, float]] = []

        def add(start: float, end: float) -> None:
            clipped_start = max(footprint_edge, float(start))
            clipped_end = min(envelope_edge, float(end))
            if clipped_end > clipped_start + 1e-9:
                intervals.append((clipped_start, clipped_end))

        add(0.0, p.minimum_range)
        base = _ray_box_interval(
            cosine,
            sine,
            -p.self_half_length - p.self_filter_margin,
            p.self_half_length + p.self_filter_margin,
            -p.self_half_width - p.self_filter_margin,
            p.self_half_width + p.self_filter_margin,
        )
        if base is not None:
            add(*base)
        for x_min, x_max, y_min, y_max, z_min, z_max in p.self_regions:
            if z_max <= p.ground_height or z_min > p.collision_maximum_height:
                continue
            interval = _ray_box_interval(cosine, sine, x_min, x_max, y_min, y_max)
            if interval is not None:
                add(*interval)
        result[math.degrees(angle)] = _merge_intervals(intervals)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dual-lidar-config",
        type=Path,
        default=Path("configs/go2w/dual_lidar_safety.yaml"),
    )
    parser.add_argument(
        "--lidar-config",
        type=Path,
        default=Path("configs/go2w/lidar_preprocess.yaml"),
    )
    parser.add_argument(
        "--geometry-config",
        type=Path,
        default=Path("configs/go2w/official_reference.yaml"),
    )
    parser.add_argument("--pandar-topic", default="/hesai/pandarxt16/points_raw")
    parser.add_argument("--builtin-topic", default="/go2w/lidar/cloud_filtered")
    parser.add_argument("--timeout-seconds", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    dual_config = load_dual_lidar_safety_config(args.dual_lidar_config)
    params = observability_params(dual_config)
    _, preprocess_params = load_safety_ready_config(
        str(args.lidar_config), str(args.geometry_config)
    )

    rclpy.init()
    probe = FreshnessProbe(args.pandar_topic, args.builtin_topic)
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and (probe.pandar_frames < 5 or probe.builtin_frames < 5)
        ):
            rclpy.spin_once(probe, timeout_sec=0.2)
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    builtin_unobservable = per_bearing_builtin_unobservable(preprocess_params)
    pandar_unobservable = generate_pandar_unobservable_profile(
        bearings_deg=builtin_unobservable.keys(),
        footprint_radius_m=params["footprint_radius_m"],
        envelope_radius_m=params["envelope_radius_m"],
        pandar_min_range_m=0.30,
    )
    pandar_extrinsics_validated = bool(
        (dual_config.get("sources") or {}).get("pandarxt16", {}).get(
            "transform_tier"
        )
        == "validated_tf"
    )
    observability = compute_dual_lidar_rotation_observability(
        footprint_radius_m=params["footprint_radius_m"],
        envelope_radius_m=params["envelope_radius_m"],
        builtin_unobservable=builtin_unobservable,
        pandar_unobservable=pandar_unobservable,
        pandar_extrinsics_validated=pandar_extrinsics_validated,
        requested_turn_range_deg=params["requested_turn_range_deg"],
    )

    now = time.time()
    pandar_fresh = (
        probe.pandar_last is not None and (now - probe.pandar_last) < 1.0
    )
    builtin_fresh = (
        probe.builtin_last is not None and (now - probe.builtin_last) < 1.0
    )
    checks = {
        "pandar_stream_fresh": pandar_fresh,
        "builtin_stream_fresh": builtin_fresh,
        "pandar_frames_received": probe.pandar_frames >= 5,
        "builtin_frames_received": probe.builtin_frames >= 5,
        "full_rotation_observability_valid": observability.full_rotation_observability_valid,
        "requested_turn_observability_valid": observability.requested_turn_observability_valid,
    }
    report = {
        "schema": "go2w.dual_lidar.rotation_observability.v1",
        "passed": all(checks.values()),
        "pandar_extrinsics_validated": pandar_extrinsics_validated,
        "checks": checks,
        "observability": observability.to_dict(),
        "unobservable_bearings": observability.unobservable_bearings[:40],
        "authorizes_motion": False,
        "authorizes_safety_integration": False,
        "limitations": [
            "Pandar coverage uses the default self-occlusion model; formal "
            "self-occlusion validation must replace it",
            "Pandar contributes only when its extrinsic is validated",
        ],
    }
    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
