#!/usr/bin/env python3
"""Read-only stationary Go2-W LiDAR orientation and threshold audit."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


def rotation_matrix_from_rpy(rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def fit_ground_plane(points: np.ndarray) -> dict:
    radial = np.hypot(points[:, 0], points[:, 1])
    candidates = points[
        (radial >= 0.55)
        & (radial <= 4.5)
        & (points[:, 2] >= -0.8)
        & (points[:, 2] <= 0.15)
    ]
    if len(candidates) < 1000:
        raise ValueError("insufficient points in the ground search region")
    edges = np.arange(-0.8, 0.151, 0.01)
    counts, _ = np.histogram(candidates[:, 2], bins=edges)
    index = int(np.argmax(counts))
    peak = float((edges[index] + edges[index + 1]) / 2.0)
    plane = candidates[np.abs(candidates[:, 2] - peak) <= 0.05]
    coefficients = np.zeros(3, dtype=np.float64)
    for _ in range(4):
        design = np.column_stack((plane[:, 0], plane[:, 1], np.ones(len(plane))))
        coefficients = np.linalg.lstsq(design, plane[:, 2], rcond=None)[0]
        residual = plane[:, 2] - design @ coefficients
        median = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median)))
        tolerance = max(0.015, 3.5 * 1.4826 * mad)
        plane = plane[np.abs(residual - median) <= tolerance]
        if len(plane) < 1000:
            raise ValueError("ground plane fit lost too many inliers")
    a, b, c = (float(value) for value in coefficients)
    residual = plane[:, 2] - (a * plane[:, 0] + b * plane[:, 1] + c)
    return {
        "histogram_peak_z_m": peak,
        "equation_z_equals_ax_plus_by_plus_c": [a, b, c],
        "origin_height_z_m": c,
        "tilt_deg": math.degrees(math.atan(math.hypot(a, b))),
        "inlier_points": int(len(plane)),
        "residual_rmse_m": float(np.sqrt(np.mean(np.square(residual)))),
    }


def recommended_parameters(ground_z: float, length: float, width: float) -> dict:
    half_length = length / 2.0
    half_width = width / 2.0
    return {
        "height_m": {
            "minimum": round(ground_z - 0.06, 3),
            "maximum": round(ground_z + 1.50, 3),
        },
        "ground_separation_height_m": round(ground_z + 0.08, 3),
        "self_filter_margin_m": 0.04,
        "front_corridor_half_width_m": round(half_width + 0.10, 3),
        "rotation_envelope_radius_m": round(
            math.hypot(half_length, half_width) + 0.10, 3
        ),
        "voxel_size_m": 0.05,
        "policy": {
            "below_ground_tolerance_m": 0.06,
            "minimum_obstacle_height_above_ground_m": 0.08,
            "maximum_obstacle_height_above_ground_m": 1.50,
            "footprint_margin_m": 0.04,
            "corridor_side_clearance_m": 0.10,
            "rotation_clearance_m": 0.10,
        },
    }


class StationaryLidarAudit(Node):
    def __init__(self, transform: np.ndarray, translation: np.ndarray) -> None:
        super().__init__("go2w_stationary_lidar_geometry_audit")
        self.transform = transform
        self.translation = translation
        self.clouds: list[np.ndarray] = []
        self.frame_ids: set[str] = set()
        self.gyro: list[tuple[float, float, float]] = []
        self.acceleration: list[tuple[float, float, float]] = []
        self.started_monotonic = time.monotonic()
        self.create_subscription(
            PointCloud2, "/utlidar/cloud", self._cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "/utlidar/imu", self._imu, qos_profile_sensor_data
        )

    def _cloud(self, message: PointCloud2) -> None:
        self.frame_ids.add(message.header.frame_id)
        records = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        values = np.column_stack(
            tuple(np.asarray(records[name], dtype=np.float64) for name in ("x", "y", "z"))
        )
        if values.size:
            self.clouds.append(values @ self.transform.T + self.translation)

    def _imu(self, message: Imu) -> None:
        self.gyro.append(
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            )
        )
        self.acceleration.append(
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            )
        )


def audit_result(node: StationaryLidarAudit, reference: dict) -> dict:
    points = np.concatenate(node.clouds, axis=0)
    gyro = np.asarray(node.gyro, dtype=np.float64)
    acceleration = np.asarray(node.acceleration, dtype=np.float64)
    ground = fit_ground_plane(points)
    gyro_norm = np.linalg.norm(gyro, axis=1)
    acceleration_norm = np.linalg.norm(acceleration, axis=1)
    envelope = reference["dimensions"]["standing_envelope_m"]
    requirements = {
        "cloud_frame_is_utlidar_lidar": node.frame_ids == {"utlidar_lidar"},
        "minimum_cloud_frames": len(node.clouds) >= 60,
        "minimum_imu_samples": len(node.gyro) >= 500,
        "stationary_gyro_p95_at_most_0_05_rad_s": float(
            np.percentile(gyro_norm, 95)
        )
        <= 0.05,
        "ground_tilt_at_most_3_deg": ground["tilt_deg"] <= 3.0,
        "ground_rmse_at_most_0_05_m": ground["residual_rmse_m"] <= 0.05,
        "ground_origin_height_plausible": -0.70
        <= ground["origin_height_z_m"]
        <= -0.15,
    }
    return {
        "schema_version": "1.0",
        "robot_model": "Unitree Go2-W",
        "audit_type": "read_only_stationary_lidar_geometry",
        "robot_motion_commanded": False,
        "duration_seconds": time.monotonic() - node.started_monotonic,
        "cloud_frames": len(node.clouds),
        "imu_samples": len(node.gyro),
        "cloud_frame_ids": sorted(node.frame_ids),
        "point_count": int(len(points)),
        "base_link_z_percentiles_m": dict(
            zip(
                ("p01", "p05", "p50", "p95", "p99"),
                (float(value) for value in np.percentile(points[:, 2], [1, 5, 50, 95, 99])),
            )
        ),
        "ground_plane": ground,
        "stationarity": {
            "gyro_rms_xyz_rad_s": [
                float(value) for value in np.sqrt(np.mean(np.square(gyro), axis=0))
            ],
            "gyro_norm_p95_rad_s": float(np.percentile(gyro_norm, 95)),
            "acceleration_norm_mean_m_s2": float(np.mean(acceleration_norm)),
            "acceleration_norm_std_m_s2": float(np.std(acceleration_norm)),
        },
        "official_reference": {
            "unitree_ros_commit": reference["sources"]["go2w_urdf"]["commit"],
            "base_to_lidar": reference["frames"]["base_to_lidar"],
            "standing_envelope_m": envelope,
        },
        "recommended_lidar_preprocess": recommended_parameters(
            ground["origin_height_z_m"],
            float(envelope["length"]),
            float(envelope["width"]),
        ),
        "requirements": requirements,
        "passed": all(requirements.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-file", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cloud-frames", type=int, default=120)
    parser.add_argument("--minimum-imu-samples", type=int, default=1000)
    parser.add_argument("--timeout-seconds", type=float, default=20.0)
    args = parser.parse_args()
    if args.cloud_frames < 60 or args.minimum_imu_samples < 500:
        raise SystemExit("stationary audit requires at least 60 cloud frames and 500 IMU samples")

    reference = yaml.safe_load(args.reference_file.read_text(encoding="utf-8")) or {}
    if (
        reference.get("robot_model") != "Unitree Go2-W"
        or reference.get("reference_status") != "manufacturer_published"
    ):
        raise SystemExit("invalid Unitree Go2-W manufacturer reference")
    lidar = (reference.get("frames") or {}).get("base_to_lidar") or {}
    if lidar.get("accepted") is not True or lidar.get("ros_child") != "utlidar_lidar":
        raise SystemExit("official base-to-LiDAR frame mapping is not accepted")
    transform = rotation_matrix_from_rpy(lidar["rotation_rpy_rad"])
    translation = np.asarray(lidar["translation_m"], dtype=np.float64)

    rclpy.init()
    node = StationaryLidarAudit(transform, translation)
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and (
                len(node.clouds) < args.cloud_frames
                or len(node.gyro) < args.minimum_imu_samples
            )
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.clouds) < args.cloud_frames or len(node.gyro) < args.minimum_imu_samples:
            raise SystemExit(
                f"insufficient samples: cloud={len(node.clouds)}, imu={len(node.gyro)}"
            )
        result = audit_result(node, reference)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["passed"] else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
