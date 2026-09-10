#!/usr/bin/env python3
"""Read-only four-sector physical cross-check for a stationary Go2-W.

The robot never moves in this workflow.  An operator first captures a clear
baseline, then places the same low, LiDAR-visible target at 0/−90/180/+90
degrees.  ``finalize`` combines the five captures with a measured physical
swept-clearance inspection into short-lived, initial-pose-bound evidence.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

from go2w_lidar_preprocessor.config import load_safety_ready_config
from go2w_lidar_preprocessor.rotation_crosscheck import (
    SECTOR_BEARINGS_RAD,
    build_rotation_evidence,
    compare_capture_context,
    compare_sector_capture,
    matching_baseline_summary,
    summarize_sector_frames,
    wrapped_angle,
)
from app.live_robot.current_hardware import (
    geometry_hash,
    load_current_hardware_geometry,
    load_current_hardware_state,
    state_hash,
)
from app.live_robot.pandar_clock import DEFAULT_PANDAR_CLOCK_TIER
from app.live_robot.rotation_lease import build_rotation_lease_binding


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LIDAR_CONFIG = PROJECT_ROOT / "configs/go2w/lidar_preprocess.yaml"
DEFAULT_GEOMETRY_CONFIG = PROJECT_ROOT / "configs/go2w/official_reference.yaml"


def cloud_xyz(message: PointCloud2) -> np.ndarray:
    records = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True
    )
    if len(records) == 0:
        return np.empty((0, 3), dtype=np.float64)
    return np.column_stack(
        tuple(np.asarray(records[name], dtype=np.float64) for name in ("x", "y", "z"))
    )


def quaternion_yaw(quaternion) -> float:
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


class CaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("go2w_rotation_clearance_physical_readonly_validator")
        self.marker_cloud_frames: list[tuple[str, np.ndarray]] = []
        self.collision_cloud_frames: list[tuple[str, np.ndarray]] = []
        self.odom: list[dict] = []
        self.lidar_fresh: list[bool] = []
        self.rotation_valid: list[bool] = []
        self.create_subscription(
            PointCloud2,
            "/go2w/lidar/obstacles",
            lambda message: self.marker_cloud_frames.append(
                (message.header.frame_id, cloud_xyz(message))
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/go2w/lidar/collision_obstacles",
            lambda message: self.collision_cloud_frames.append(
                (message.header.frame_id, cloud_xyz(message))
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "/go2w/odom/wheel", self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            Bool,
            "/go2w/safety/lidar_fresh",
            lambda message: self.lidar_fresh.append(bool(message.data)),
            10,
        )
        self.create_subscription(
            Bool,
            "/go2w/safety/rotation_clearance_valid",
            lambda message: self.rotation_valid.append(bool(message.data)),
            10,
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        self.odom.append(
            {
                "frame": message.header.frame_id,
                "child_frame": message.child_frame_id,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": quaternion_yaw(pose.orientation),
                "linear_speed_m_s": math.hypot(
                    float(twist.linear.x), float(twist.linear.y)
                ),
                "angular_speed_rad_s": abs(float(twist.angular.z)),
            }
        )


def stationarity_summary(samples: list[dict]) -> dict:
    if not samples:
        return {
            "passed": False,
            "samples": 0,
            "origin_pose": None,
            "maximum_translation_m": None,
            "maximum_yaw_change_rad": None,
            "maximum_linear_speed_m_s": None,
            "maximum_angular_speed_rad_s": None,
        }
    origin = samples[0]
    translations = [
        math.hypot(item["x"] - origin["x"], item["y"] - origin["y"])
        for item in samples
    ]
    yaw_changes = [
        abs(wrapped_angle(item["yaw"] - origin["yaw"])) for item in samples
    ]
    maximum_translation = max(translations)
    maximum_yaw_change = max(yaw_changes)
    maximum_linear = max(item["linear_speed_m_s"] for item in samples)
    maximum_angular = max(item["angular_speed_rad_s"] for item in samples)
    frames_valid = all(
        item["frame"] == "odom_wheel" and item["child_frame"] == "base_link"
        for item in samples
    )
    return {
        "passed": bool(
            len(samples) >= 5
            and frames_valid
            and maximum_translation <= 0.015
            and maximum_yaw_change <= math.radians(1.0)
            and maximum_linear <= 0.02
            and maximum_angular <= 0.03
        ),
        "samples": len(samples),
        "frames_valid": frames_valid,
        "origin_pose": {
            "x": origin["x"],
            "y": origin["y"],
            "yaw": origin["yaw"],
        },
        "last_pose": {
            "x": samples[-1]["x"],
            "y": samples[-1]["y"],
            "yaw": samples[-1]["yaw"],
        },
        "maximum_translation_m": maximum_translation,
        "maximum_yaw_change_rad": maximum_yaw_change,
        "maximum_linear_speed_m_s": maximum_linear,
        "maximum_angular_speed_rad_s": maximum_angular,
    }


def capture(args: argparse.Namespace) -> int:
    rclpy.init()
    node = CaptureNode()
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and (
                len(node.marker_cloud_frames) < args.minimum_samples
                or len(node.collision_cloud_frames) < args.minimum_samples
                or len(node.odom) < 5
                or sum(node.lidar_fresh) < 5
                or len(node.rotation_valid) < 5
            )
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    frames = [points for _, points in node.marker_cloud_frames]
    summaries = {
        sector: summarize_sector_frames(
            frames,
            sector=sector,
            expected_distance_m=args.expected_distance_m,
            bearing_tolerance_deg=args.bearing_tolerance_deg,
            distance_tolerance_m=args.distance_tolerance_m,
            minimum_points_per_frame=args.minimum_points_per_frame,
        )
        for sector in SECTOR_BEARINGS_RAD
    }
    profile_distances = np.arange(
        args.profile_min_distance_m,
        args.profile_max_distance_m + args.profile_step_m / 2.0,
        args.profile_step_m,
    )
    profiles = {
        sector: [
            summarize_sector_frames(
                frames,
                sector=sector,
                expected_distance_m=round(float(distance), 6),
                bearing_tolerance_deg=args.bearing_tolerance_deg,
                distance_tolerance_m=args.distance_tolerance_m,
                minimum_points_per_frame=args.minimum_points_per_frame,
            )
            for distance in profile_distances
        ]
        for sector in SECTOR_BEARINGS_RAD
    }
    recommended_distances = {
        sector: [
            item["expected_distance_m"]
            for item in sorted(
                items,
                key=lambda item: (
                    item["hit_fraction"],
                    item["median_points_per_frame"],
                    item["expected_distance_m"],
                ),
            )[:3]
        ]
        for sector, items in profiles.items()
    }
    stationarity = stationarity_summary(node.odom)
    checks = {
        "minimum_marker_cloud_samples": len(frames) >= args.minimum_samples,
        "marker_cloud_frame_base_link": bool(node.marker_cloud_frames)
        and all(frame == "base_link" for frame, _ in node.marker_cloud_frames),
        "minimum_collision_cloud_samples": len(node.collision_cloud_frames)
        >= args.minimum_samples,
        "collision_cloud_frame_base_link": bool(node.collision_cloud_frames)
        and all(
            frame == "base_link" for frame, _ in node.collision_cloud_frames
        ),
        "lidar_fresh": sum(node.lidar_fresh) >= 5
        and node.lidar_fresh[-1] is True,
        "persistent_rotation_gate_remains_closed": bool(node.rotation_valid)
        and not any(node.rotation_valid),
        "robot_stationary": stationarity["passed"],
    }
    captured_at = datetime.now(timezone.utc).isoformat()
    target_crosscheck = None
    baseline_compatibility = None
    target_crosscheck_status = "not_applicable"
    if args.role != "baseline" and not args.defer_baseline_crosscheck:
        baseline = load_capture(args.baseline, "baseline")
        baseline_compatibility = compare_capture_context(
            baseline,
            {
                "captured_at": captured_at,
                "stationarity": stationarity,
            },
        )
        target_summary = summaries[args.role]
        target_crosscheck = compare_sector_capture(
            matching_baseline_summary(baseline, target_summary),
            target_summary,
        )
        checks["baseline_time_pose_compatible"] = baseline_compatibility["passed"]
        checks["target_crosscheck"] = target_crosscheck["passed"]
        target_crosscheck_status = "passed" if target_crosscheck["passed"] else "failed"
    elif args.role != "baseline":
        target_crosscheck_status = "deferred_until_baseline_capture"
    result = {
        "schema_version": "1.0",
        "validation_type": "go2w_rotation_sector_capture",
        "capture_role": args.role,
        "captured_at": captured_at,
        "robot_model": "Unitree Go2-W",
        "posture": "stationary_standing",
        "robot_motion_commanded": False,
        "marker_cloud_topic": "/go2w/lidar/obstacles",
        "marker_cloud_purpose": "controlled_direction_and_yaw_crosscheck_only",
        "marker_cloud_safety_authoritative_for_collision": False,
        "collision_cloud_topic": "/go2w/lidar/collision_obstacles",
        "marker_cloud_samples": len(frames),
        "collision_cloud_samples": len(node.collision_cloud_frames),
        "stationarity": stationarity,
        "sector_summaries": summaries,
        "sector_profiles": profiles,
        "recommended_low_occupancy_distances_m": recommended_distances,
        "target_crosscheck": target_crosscheck,
        "target_crosscheck_status": target_crosscheck_status,
        "baseline_compatibility": baseline_compatibility,
        "checks": checks,
        "passed": all(checks.values()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


def load_capture(path: Path, expected_role: str) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("validation_type") != "go2w_rotation_sector_capture":
        raise ValueError(f"not a rotation sector capture: {path}")
    if payload.get("capture_role") != expected_role:
        raise ValueError(f"capture role mismatch for {path}: expected {expected_role}")
    return payload


def finalize(args: argparse.Namespace) -> int:
    _, parameters = load_safety_ready_config(
        str(args.lidar_config), str(args.geometry_config)
    )
    paths = {
        "baseline": args.baseline,
        "front": args.front,
        "right": args.right,
        "rear": args.rear,
        "left": args.left,
    }
    captures = {role: load_capture(path, role) for role, path in paths.items()}
    hardware_binding = None
    try:
        geometry = load_current_hardware_geometry(args.hardware_geometry_config)
        state = load_current_hardware_state(args.hardware_state_config)
        hardware_binding = build_rotation_lease_binding(
            hardware_state_hash=state_hash(state),
            geometry_hash=geometry_hash(geometry),
            extrinsic_version="hesai_pandarxt16_extrinsics_20260813_unconfirmed",
            clock_tier=DEFAULT_PANDAR_CLOCK_TIER.value,
        )
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"warning: hardware binding not attached: {exc}", file=sys.stderr)
    result = build_rotation_evidence(
        operator=args.operator,
        configured_envelope_radius_m=parameters.rotation_envelope_radius,
        physical_clearance_radius_m=args.physical_clearance_radius_m,
        swept_clearance_confirmed=args.swept_clearance_confirmed,
        standing_posture_confirmed=args.standing_posture_confirmed,
        captures=captures,
        capture_paths=paths,
        validity_seconds=args.validity_seconds,
        hardware_binding=hardware_binding,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        description="Read-only, stationary Go2-W rotation-clearance cross-check"
    )
    commands = root.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture")
    capture_parser.add_argument(
        "--role", choices=("baseline", "front", "right", "rear", "left"), required=True
    )
    capture_parser.add_argument(
        "--defer-baseline-crosscheck",
        action="store_true",
        help="capture a target before the empty baseline; the file cannot "
             "authorize anything until finalize performs the A/B check",
    )
    capture_parser.add_argument(
        "--baseline",
        type=Path,
        help="baseline capture; required for every non-baseline role",
    )
    capture_parser.add_argument("--output", type=Path, required=True)
    capture_parser.add_argument("--expected-distance-m", type=float, default=0.70)
    capture_parser.add_argument("--bearing-tolerance-deg", type=float, default=15.0)
    capture_parser.add_argument("--distance-tolerance-m", type=float, default=0.15)
    capture_parser.add_argument("--minimum-points-per-frame", type=int, default=2)
    capture_parser.add_argument("--minimum-samples", type=int, default=30)
    capture_parser.add_argument("--timeout-seconds", type=float, default=20.0)
    capture_parser.add_argument("--profile-min-distance-m", type=float, default=0.55)
    capture_parser.add_argument("--profile-max-distance-m", type=float, default=1.50)
    capture_parser.add_argument("--profile-step-m", type=float, default=0.05)
    capture_parser.set_defaults(function=capture)

    finalize_parser = commands.add_parser("finalize")
    for role in ("baseline", "front", "right", "rear", "left"):
        finalize_parser.add_argument(f"--{role}", type=Path, required=True)
    finalize_parser.add_argument("--operator", required=True)
    finalize_parser.add_argument(
        "--physical-clearance-radius-m", type=float, required=True
    )
    finalize_parser.add_argument(
        "--swept-clearance-confirmed", action="store_true", required=True
    )
    finalize_parser.add_argument(
        "--standing-posture-confirmed", action="store_true", required=True
    )
    finalize_parser.add_argument("--validity-seconds", type=int, default=600)
    finalize_parser.add_argument("--output", type=Path, required=True)
    finalize_parser.add_argument(
        "--hardware-geometry-config",
        type=Path,
        default=PROJECT_ROOT / "configs/go2w/current_hardware_geometry.yaml",
        help="current whole-machine geometry; binds the lease to 0.70x0.43x0.70 m",
    )
    finalize_parser.add_argument(
        "--hardware-state-config",
        type=Path,
        default=PROJECT_ROOT / "configs/go2w/current_hardware_state.yaml",
        help="current hardware state manifest; binds the lease to the rig",
    )
    finalize_parser.add_argument(
        "--lidar-config", type=Path, default=DEFAULT_LIDAR_CONFIG
    )
    finalize_parser.add_argument(
        "--geometry-config", type=Path, default=DEFAULT_GEOMETRY_CONFIG
    )
    finalize_parser.set_defaults(function=finalize)
    return root


def main() -> int:
    args = parser().parse_args()
    if getattr(args, "minimum_samples", 30) < 20:
        raise SystemExit("at least 20 cloud samples are required")
    if args.command == "capture" and not (
        0.0 < args.profile_min_distance_m < args.profile_max_distance_m
        and 0.0 < args.profile_step_m <= 0.25
    ):
        raise SystemExit("invalid radial-profile distance range or step")
    if args.command == "capture":
        if args.role == "baseline" and (
            args.baseline is not None or args.defer_baseline_crosscheck
        ):
            raise SystemExit(
                "baseline role must not receive --baseline or defer crosscheck"
            )
        if (
            args.role != "baseline"
            and args.baseline is None
            and not args.defer_baseline_crosscheck
        ):
            raise SystemExit(
                "non-baseline capture requires --baseline or explicit "
                "--defer-baseline-crosscheck"
            )
        if args.baseline is not None and args.defer_baseline_crosscheck:
            raise SystemExit("choose --baseline or deferred crosscheck, not both")
    return int(args.function(args))


if __name__ == "__main__":
    raise SystemExit(main())
