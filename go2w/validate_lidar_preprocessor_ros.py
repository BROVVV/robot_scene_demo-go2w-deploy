#!/usr/bin/env python3
"""Validate the read-only Go2-W LiDAR preprocessing ROS outputs."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

from go2w_lidar_preprocessor.config import load_safety_ready_config
from go2w_lidar_preprocessor.preprocess_core import rotation_observability_report


def finite_or_none(value: float) -> float | None:
    return float(value) if math.isfinite(value) else None


def clearance_summary(values: list[float]) -> dict:
    finite = np.asarray(
        [value for value in values if math.isfinite(value)], dtype=np.float64
    )
    return {
        "samples": len(values),
        "finite_samples": int(len(finite)),
        "no_return_samples": sum(math.isinf(value) for value in values),
        "unknown_samples": sum(math.isnan(value) for value in values),
        "minimum_finite_m": float(np.min(finite)) if len(finite) else None,
        "median_finite_m": float(np.median(finite)) if len(finite) else None,
        "maximum_finite_m": float(np.max(finite)) if len(finite) else None,
    }


def cloud_xyz(message: PointCloud2) -> np.ndarray:
    records = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True
    )
    return np.column_stack(
        tuple(np.asarray(records[name], dtype=np.float64) for name in ("x", "y", "z"))
    )


class Validator(Node):
    def __init__(self) -> None:
        super().__init__("go2w_lidar_preprocessor_readonly_validator")
        self.scans: list[LaserScan] = []
        self.obstacles: list[tuple[str, np.ndarray]] = []
        self.collision_obstacles: list[tuple[str, np.ndarray]] = []
        self.filtered: list[tuple[str, np.ndarray]] = []
        self.clearances: list[Vector3Stamped] = []
        self.freshness: list[bool] = []
        self.rotation_clearance_validity: list[bool] = []
        self.create_subscription(
            LaserScan, "/go2w/lidar/scan", self.scans.append, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/go2w/lidar/obstacles",
            lambda message: self.obstacles.append(
                (message.header.frame_id, cloud_xyz(message))
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/go2w/lidar/collision_obstacles",
            lambda message: self.collision_obstacles.append(
                (message.header.frame_id, cloud_xyz(message))
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/go2w/lidar/cloud_filtered",
            lambda message: self.filtered.append(
                (message.header.frame_id, cloud_xyz(message))
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3Stamped,
            "/go2w/lidar/clearance",
            self.clearances.append,
            10,
        )
        self.create_subscription(
            Bool,
            "/go2w/safety/lidar_fresh",
            lambda message: self.freshness.append(bool(message.data)),
            10,
        )
        self.create_subscription(
            Bool,
            "/go2w/safety/rotation_clearance_valid",
            lambda message: self.rotation_clearance_validity.append(
                bool(message.data)
            ),
            10,
        )


def result(
    node: Validator,
    minimum_samples: int,
    ground_height: float,
    collision_maximum_height: float,
    front_corridor_half_width: float,
    rotation_envelope_radius: float,
    rotation_observability: dict,
) -> dict:
    obstacle_points = np.concatenate(
        [points for _, points in node.obstacles], axis=0
    )
    collision_points = np.concatenate(
        [points for _, points in node.collision_obstacles], axis=0
    )
    self_points = (
        (np.abs(obstacle_points[:, 0]) <= 0.35 + 0.04)
        & (np.abs(obstacle_points[:, 1]) <= 0.215 + 0.04)
    )
    finite_ranges = [
        float(value)
        for scan in node.scans
        for value in scan.ranges
        if math.isfinite(value)
    ]
    front_clearances = [float(message.vector.x) for message in node.clearances]
    left_clearances = [float(message.vector.y) for message in node.clearances]
    right_clearances = [float(message.vector.z) for message in node.clearances]
    raw_front_clearances = []
    raw_left_clearances = []
    raw_right_clearances = []
    raw_front_left_clearances = []
    raw_rear_left_clearances = []
    raw_front_right_clearances = []
    raw_rear_right_clearances = []
    rotation_envelope_point_counts = []
    for _, points in node.collision_obstacles:
        radial = np.hypot(points[:, 0], points[:, 1])

        def minimum(mask) -> float:
            values = radial[mask]
            return float(np.min(values)) if len(values) else math.inf

        front = (
            (points[:, 0] > 0.0)
            & (np.abs(points[:, 1]) <= front_corridor_half_width)
        )
        raw_front_clearances.append(
            float(np.min(points[front, 0])) if np.any(front) else math.inf
        )
        raw_left_clearances.append(minimum(points[:, 1] > 0.0))
        raw_right_clearances.append(minimum(points[:, 1] < 0.0))
        raw_front_left_clearances.append(
            minimum((points[:, 0] >= 0.0) & (points[:, 1] > 0.0))
        )
        raw_rear_left_clearances.append(
            minimum((points[:, 0] < 0.0) & (points[:, 1] > 0.0))
        )
        raw_front_right_clearances.append(
            minimum((points[:, 0] >= 0.0) & (points[:, 1] < 0.0))
        )
        raw_rear_right_clearances.append(
            minimum((points[:, 0] < 0.0) & (points[:, 1] < 0.0))
        )
        rotation_envelope_point_counts.append(
            int(np.count_nonzero(radial <= rotation_envelope_radius))
        )
    checks = {
        "minimum_samples": min(
            len(node.scans),
            len(node.obstacles),
            len(node.collision_obstacles),
            len(node.filtered),
            len(node.clearances),
        )
        >= minimum_samples,
        "fresh_true_seen": sum(node.freshness) >= 5,
        "scan_frame_base_link": bool(node.scans)
        and all(scan.header.frame_id == "base_link" for scan in node.scans),
        "cloud_frames_base_link": bool(node.obstacles)
        and all(
            frame == "base_link"
            for frame, _ in (
                node.obstacles + node.collision_obstacles + node.filtered
            )
        ),
        "scan_has_720_bins": bool(node.scans)
        and all(len(scan.ranges) == 720 for scan in node.scans),
        "scan_increment_is_half_degree": bool(node.scans)
        and all(
            abs(scan.angle_increment - math.pi / 360.0) <= 1e-6
            for scan in node.scans
        ),
        "self_envelope_removed": not bool(np.any(self_points)),
        "obstacles_above_ground_threshold": float(np.min(obstacle_points[:, 2]))
        > ground_height - 1e-6,
        "collision_points_within_vertical_envelope": bool(len(collision_points))
        and float(np.min(collision_points[:, 2])) > ground_height - 1e-6
        and float(np.max(collision_points[:, 2]))
        <= collision_maximum_height + 1e-6,
        "full_height_semantic_obstacles_retained": float(
            np.max(obstacle_points[:, 2])
        ) > collision_maximum_height,
        "rotation_clearance_fails_closed": bool(node.rotation_clearance_validity)
        and not any(node.rotation_clearance_validity)
        and all(math.isnan(value) for value in left_clearances)
        and all(math.isnan(value) for value in right_clearances),
        "finite_scan_obstacles_seen": bool(finite_ranges),
        "rotation_observability_limitation_reported": bool(
            rotation_observability.get(
                "requires_independent_physical_validation"
            )
        ),
    }
    last_clearance = node.clearances[-1].vector
    return {
        "schema_version": "1.0",
        "validation_type": "live_read_only_lidar_preprocessor",
        "robot_motion_commanded": False,
        "samples": {
            "scan": len(node.scans),
            "obstacles": len(node.obstacles),
            "collision_obstacles": len(node.collision_obstacles),
            "filtered": len(node.filtered),
            "clearance": len(node.clearances),
            "fresh_messages": len(node.freshness),
            "fresh_true": sum(node.freshness),
            "rotation_clearance_valid_messages": len(
                node.rotation_clearance_validity
            ),
        },
        "obstacles": {
            "points": int(len(obstacle_points)),
            "z_min_m": float(np.min(obstacle_points[:, 2])),
            "z_median_m": float(np.median(obstacle_points[:, 2])),
            "z_max_m": float(np.max(obstacle_points[:, 2])),
            "inside_self_envelope_points": int(np.sum(self_points)),
        },
        "collision_obstacles": {
            "points": int(len(collision_points)),
            "z_min_m": float(np.min(collision_points[:, 2])),
            "z_median_m": float(np.median(collision_points[:, 2])),
            "z_max_m": float(np.max(collision_points[:, 2])),
            "configured_z_max_m": collision_maximum_height,
        },
        "scan": {
            "finite_ranges": len(finite_ranges),
            "minimum_finite_m": min(finite_ranges),
            "maximum_finite_m": max(finite_ranges),
        },
        "last_clearance_m": {
            "front": finite_or_none(last_clearance.x),
            "left": finite_or_none(last_clearance.y),
            "right": finite_or_none(last_clearance.z),
        },
        "clearance_samples": {
            "front": clearance_summary(front_clearances),
            "left": clearance_summary(left_clearances),
            "right": clearance_summary(right_clearances),
        },
        "diagnostic_collision_cloud_clearance": {
            "safety_authoritative": False,
            "reason": (
                "raw collision-cloud sectors are reported for self-filter "
                "diagnosis; rotation remains unknown while its validity gate "
                "is false"
            ),
            "front": clearance_summary(raw_front_clearances),
            "left": clearance_summary(raw_left_clearances),
            "right": clearance_summary(raw_right_clearances),
            "front_left": clearance_summary(raw_front_left_clearances),
            "rear_left": clearance_summary(raw_rear_left_clearances),
            "front_right": clearance_summary(raw_front_right_clearances),
            "rear_right": clearance_summary(raw_rear_right_clearances),
            "rotation_envelope_radius_m": rotation_envelope_radius,
            "frames_with_any_point_inside_rotation_envelope": int(
                np.count_nonzero(rotation_envelope_point_counts)
            ),
            "points_inside_rotation_envelope": int(
                np.sum(rotation_envelope_point_counts)
            ),
            "maximum_points_inside_rotation_envelope_in_one_frame": int(
                np.max(rotation_envelope_point_counts, initial=0)
            ),
        },
        "rotation_observability": rotation_observability,
        "checks": checks,
        "passed": all(checks.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-samples", type=int, default=20)
    parser.add_argument("--timeout-seconds", type=float, default=15.0)
    args = parser.parse_args()
    if args.minimum_samples < 10:
        raise SystemExit("at least 10 samples are required")

    rclpy.init()
    node = Validator()
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and (
                len(node.scans) < args.minimum_samples
                or len(node.obstacles) < args.minimum_samples
                or len(node.collision_obstacles) < args.minimum_samples
                or len(node.filtered) < args.minimum_samples
                or len(node.clearances) < args.minimum_samples
                or sum(node.freshness) < 5
            )
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.obstacles or not node.clearances:
            raise SystemExit("LiDAR preprocessor outputs were not received")
        project_root = Path(__file__).resolve().parents[2]
        lidar_config, preprocess_parameters = load_safety_ready_config(
            str(project_root / "configs/go2w/lidar_preprocess.yaml"),
            str(project_root / "configs/go2w/official_reference.yaml"),
        )
        ground_height = float(lidar_config["ground_separation_height_m"])
        collision_maximum_height = float(
            lidar_config["collision_height_m"]["maximum"]
        )
        front_corridor_half_width = float(
            lidar_config["front_corridor_half_width_m"]
        )
        rotation_envelope_radius = float(
            lidar_config["rotation_envelope_radius_m"]
        )
        payload = result(
            node,
            args.minimum_samples,
            ground_height,
            collision_maximum_height,
            front_corridor_half_width,
            rotation_envelope_radius,
            rotation_observability_report(preprocess_parameters),
        )
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
