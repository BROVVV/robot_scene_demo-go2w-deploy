#!/usr/bin/env python3
"""Validate live LIO while the Go2-W remains completely stationary."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

from go2w_description.description_config import (
    load_official_reference,
    official_sensor_to_base_extrinsics,
)


def rotation_matrix_xyzw(quaternion) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_from_xyzw(quaternion) -> float:
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def diagnostic_level(value) -> int:
    return int.from_bytes(value, "little") if isinstance(value, bytes) else int(value)


def stamp_seconds(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1e9


def stamp_key(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def windowed_pose_speeds(arrivals, positions, quaternions, window_seconds=1.0):
    """Measure sustained pose change over approximately one-second windows."""

    linear = []
    angular = []
    normalized = quaternions / np.linalg.norm(quaternions, axis=1)[:, None]
    for index in range(1, len(arrivals)):
        earlier = int(np.searchsorted(arrivals, arrivals[index] - window_seconds))
        if earlier >= index:
            continue
        elapsed = float(arrivals[index] - arrivals[earlier])
        if elapsed < window_seconds * 0.8:
            continue
        linear.append(float(np.linalg.norm(positions[index] - positions[earlier]) / elapsed))
        dot = float(abs(np.dot(normalized[index], normalized[earlier])))
        angular.append(2.0 * math.acos(min(1.0, max(0.0, dot))) / elapsed)
    if not linear:
        raise ValueError("not enough odometry duration for sustained-speed check")
    return np.asarray(linear), np.asarray(angular)


class StationaryLioValidator(Node):
    def __init__(self) -> None:
        super().__init__("go2w_stationary_lio_readonly_validator")
        self.start_monotonic = time.monotonic()
        self.odom: list[tuple[float, Odometry]] = []
        self.paths: list[RosPath] = []
        self.path_message_count = 0
        self.clouds: list[dict] = []
        self.status: list[dict] = []
        self.imu_accel: list[tuple[float, float, float]] = []
        self.imu_gyro_norm: list[float] = []
        self.tf_odom_base: list[object] = []
        self.create_subscription(Odometry, "/lio/odom", self._odom, 10)
        self.create_subscription(RosPath, "/lio/path", self._path, 1)
        self.create_subscription(
            PointCloud2,
            "/lio/cloud_registered",
            self._cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(DiagnosticArray, "/lio/status", self._status, 10)
        self.create_subscription(
            Imu, "/go2w/lio_input/imu_raw", self._imu, qos_profile_sensor_data
        )
        self.create_subscription(TFMessage, "/tf", self._tf, 100)

    def _elapsed(self) -> float:
        return time.monotonic() - self.start_monotonic

    def _odom(self, message: Odometry) -> None:
        self.odom.append((self._elapsed(), message))

    def _cloud(self, message: PointCloud2) -> None:
        records = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        xyz = np.column_stack(
            tuple(
                np.asarray(records[name], dtype=np.float64)
                for name in ("x", "y", "z")
            )
        )
        self.clouds.append(
            {
                "frame": message.header.frame_id,
                "points": int(len(xyz)),
                "finite": bool(len(xyz) and np.isfinite(xyz).all()),
            }
        )

    def _path(self, message: RosPath) -> None:
        self.path_message_count += 1
        self.paths[:] = [message]

    def _status(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "go2w_lio/status":
                self.status.append(
                    {
                        "elapsed_seconds": self._elapsed(),
                        "level": diagnostic_level(status.level),
                        "message": status.message,
                        "values": {item.key: item.value for item in status.values},
                    }
                )

    def _imu(self, message: Imu) -> None:
        accel = message.linear_acceleration
        gyro = message.angular_velocity
        self.imu_accel.append((accel.x, accel.y, accel.z))
        self.imu_gyro_norm.append(math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2))

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id == "odom"
                and transform.child_frame_id == "base_link"
            ):
                self.tf_odom_base.append(transform)


def build_result(node: StationaryLioValidator, reference_path: Path, minimum_odom: int) -> dict:
    arrivals = np.asarray([item[0] for item in node.odom], dtype=np.float64)
    messages = [item[1] for item in node.odom]
    message_times = np.asarray([stamp_seconds(message) for message in messages])
    positions = np.asarray(
        [
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            for msg in messages
        ],
        dtype=np.float64,
    )
    quaternions = np.asarray(
        [
            (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            for msg in messages
        ],
        dtype=np.float64,
    )
    velocities = np.asarray(
        [
            (
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            )
            for msg in messages
        ],
        dtype=np.float64,
    )
    angular_velocities = np.asarray(
        [
            (
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            )
            for msg in messages
        ],
        dtype=np.float64,
    )
    duration = float(message_times[-1] - message_times[0])
    intervals = np.diff(message_times)
    arrival_intervals = np.diff(arrivals)
    displacement = float(np.linalg.norm(positions[-1] - positions[0]))
    distance_from_first = np.linalg.norm(positions - positions[0], axis=1)
    path_length = float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))
    yaws = np.unwrap(np.asarray([yaw_from_xyzw(q) for q in quaternions]))
    quaternion_norms = np.linalg.norm(quaternions, axis=1)
    linear_speed = np.linalg.norm(velocities, axis=1)
    angular_speed = np.linalg.norm(angular_velocities, axis=1)
    derived_linear_speed = np.linalg.norm(np.diff(positions, axis=0), axis=1) / intervals
    normalized_quaternions = quaternions / quaternion_norms[:, None]
    quaternion_dots = np.abs(
        np.sum(normalized_quaternions[1:] * normalized_quaternions[:-1], axis=1)
    )
    derived_angular_speed = 2.0 * np.arccos(np.clip(quaternion_dots, 0.0, 1.0)) / intervals
    sustained_linear_speed, sustained_angular_speed = windowed_pose_speeds(
        message_times, positions, quaternions
    )

    reference = load_official_reference(reference_path)
    imu_extrinsic = official_sensor_to_base_extrinsics(reference)[
        "imu2base_quat_xyzw_xyz"
    ]
    median_accel_imu = np.median(np.asarray(node.imu_accel, dtype=np.float64), axis=0)
    accel_base = rotation_matrix_xyzw(imu_extrinsic[:4]) @ median_accel_imu
    median_q = np.median(quaternions, axis=0)
    accel_odom = rotation_matrix_xyzw(median_q) @ accel_base
    accel_norm = float(np.linalg.norm(accel_odom))
    gravity_horizontal = float(np.linalg.norm(accel_odom[:2]))

    tf_by_stamp = {stamp_key(item): item for item in node.tf_odom_base}
    odom_by_stamp = {stamp_key(item): item for item in messages}
    common_stamps = sorted(set(tf_by_stamp).intersection(odom_by_stamp))
    if not common_stamps:
        raise ValueError("no timestamp-matched odometry/TF pair")
    matched_stamp = common_stamps[-1]
    tf_matched = tf_by_stamp[matched_stamp]
    odom_matched = odom_by_stamp[matched_stamp]
    tf_translation = tf_matched.transform.translation
    tf_rotation = tf_matched.transform.rotation
    tf_position_error = float(
        np.linalg.norm(
            np.asarray((tf_translation.x, tf_translation.y, tf_translation.z))
            - np.asarray(
                (
                    odom_matched.pose.pose.position.x,
                    odom_matched.pose.pose.position.y,
                    odom_matched.pose.pose.position.z,
                )
            )
        )
    )
    tf_quaternion = np.asarray(
        (tf_rotation.x, tf_rotation.y, tf_rotation.z, tf_rotation.w)
    )
    odom_quaternion = np.asarray(
        (
            odom_matched.pose.pose.orientation.x,
            odom_matched.pose.pose.orientation.y,
            odom_matched.pose.pose.orientation.z,
            odom_matched.pose.pose.orientation.w,
        )
    )
    tf_orientation_error = float(
        min(
            np.linalg.norm(tf_quaternion - odom_quaternion),
            np.linalg.norm(tf_quaternion + odom_quaternion),
        )
    )
    final_status = node.status[-5:]
    checks = {
        "minimum_odom_samples": len(messages) >= minimum_odom,
        "duration_at_least_20_seconds": duration >= 20.0,
        "strictly_increasing_odom_stamps": bool(np.all(intervals > 0.0)),
        "odom_rate_at_least_10_hz": float(1.0 / np.median(intervals)) >= 10.0,
        "maximum_odom_stamp_gap_at_most_0_20_seconds": float(np.max(intervals))
        <= 0.20,
        "odom_frames_valid": all(
            msg.header.frame_id == "odom" and msg.child_frame_id == "base_link"
            for msg in messages
        ),
        "finite_odometry": bool(
            np.isfinite(positions).all()
            and np.isfinite(quaternions).all()
            and np.isfinite(velocities).all()
            and np.isfinite(angular_velocities).all()
        ),
        "unit_quaternions": float(np.max(np.abs(quaternion_norms - 1.0))) <= 1e-3,
        "stationary_final_displacement_at_most_0_15_m": displacement <= 0.15,
        "stationary_max_displacement_at_most_0_25_m": float(np.max(distance_from_first)) <= 0.25,
        "stationary_yaw_span_at_most_5_deg": float(np.ptp(yaws)) <= math.radians(5.0),
        "linear_speed_p95_at_most_0_20_mps": float(np.percentile(linear_speed, 95)) <= 0.20,
        "angular_speed_p95_at_most_0_20_radps": float(np.percentile(angular_speed, 95)) <= 0.20,
        "sustained_linear_speed_p95_at_most_0_20_mps": float(
            np.percentile(sustained_linear_speed, 95)
        )
        <= 0.20,
        "sustained_angular_speed_p95_at_most_0_20_radps": float(
            np.percentile(sustained_angular_speed, 95)
        )
        <= 0.20,
        "stationary_input_gyro_p95_at_most_0_05_radps": float(
            np.percentile(node.imu_gyro_norm, 95)
        )
        <= 0.05,
        "gravity_magnitude_8_to_11_5_mps2": 8.0 <= accel_norm <= 11.5,
        "gravity_horizontal_at_most_1_5_mps2": gravity_horizontal <= 1.5,
        "registered_clouds_in_odom": len(node.clouds) >= minimum_odom // 2
        and all(item["frame"] == "odom" and item["finite"] for item in node.clouds),
        "path_tracks_odometry": bool(node.paths)
        and node.paths[-1].header.frame_id == "odom"
        and len(node.paths[-1].poses) >= len(messages) - 20,
        "odom_base_tf_timestamp_match_ratio_at_least_0_90": len(common_stamps)
        >= math.floor(len(messages) * 0.90),
        "tf_matches_odometry": tf_position_error <= 1e-6
        and tf_orientation_error <= 1e-6,
        "final_diagnostics_valid": len(final_status) == 5
        and all(
            item["level"] == diagnostic_level(DiagnosticStatus.OK)
            and item["message"] == "valid"
            for item in final_status
        ),
    }
    return {
        "schema_version": "1.0",
        "validation_type": "live_read_only_stationary_lio",
        "robot_motion_commanded": False,
        "motion_components_started": False,
        "implementation": {
            "name": next(
                (
                    item["values"].get("implementation")
                    for item in reversed(node.status)
                    if item["values"].get("implementation")
                ),
                "rko_lio",
            )
        },
        "official_reference": {
            "urdf_commit": reference["sources"]["go2w_urdf"]["commit"],
            "extrinsic_convention": "sensor_to_base_quat_xyzw_xyz",
        },
        "samples": {
            "odometry": len(messages),
            "path_messages": node.path_message_count,
            "registered_clouds": len(node.clouds),
            "imu": len(node.imu_accel),
            "odom_base_tf": len(node.tf_odom_base),
            "diagnostics": len(node.status),
        },
        "timing": {
            "duration_seconds": duration,
            "median_rate_hz": float(1.0 / np.median(intervals)),
            "minimum_rate_hz": float(1.0 / np.max(intervals)),
            "maximum_stamp_gap_seconds": float(np.max(intervals)),
            "maximum_callback_gap_seconds": float(np.max(arrival_intervals)),
        },
        "stationary_drift": {
            "final_displacement_m": displacement,
            "maximum_displacement_from_start_m": float(np.max(distance_from_first)),
            "accumulated_path_length_m": path_length,
            "yaw_span_deg": math.degrees(float(np.ptp(yaws))),
            "linear_speed_p95_mps": float(np.percentile(linear_speed, 95)),
            "angular_speed_p95_radps": float(np.percentile(angular_speed, 95)),
            "derived_linear_speed_p95_mps": float(
                np.percentile(derived_linear_speed, 95)
            ),
            "derived_angular_speed_p95_radps": float(
                np.percentile(derived_angular_speed, 95)
            ),
            "sustained_window_seconds": 1.0,
            "sustained_linear_speed_p95_mps": float(
                np.percentile(sustained_linear_speed, 95)
            ),
            "sustained_angular_speed_p95_radps": float(
                np.percentile(sustained_angular_speed, 95)
            ),
        },
        "gravity": {
            "median_acceleration_odom_mps2": [float(value) for value in accel_odom],
            "magnitude_mps2": accel_norm,
            "horizontal_magnitude_mps2": gravity_horizontal,
            "stationary_gyro_p95_radps": float(np.percentile(node.imu_gyro_norm, 95)),
        },
        "tf_consistency": {
            "position_error_m": tf_position_error,
            "quaternion_chord_error": tf_orientation_error,
            "timestamp_matched_pairs": len(common_stamps),
            "matched_stamp_nanoseconds": matched_stamp,
        },
        "registered_cloud": {
            "minimum_points": min(item["points"] for item in node.clouds),
            "maximum_points": max(item["points"] for item in node.clouds),
        },
        "final_diagnostics": final_status,
        "checks": checks,
        "passed": all(checks.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-odom", type=int, default=300)
    parser.add_argument("--collection-seconds", type=float, default=25.0)
    parser.add_argument("--startup-timeout-seconds", type=float, default=20.0)
    args = parser.parse_args()
    if args.minimum_odom < 100 or args.collection_seconds < 20.0:
        raise SystemExit("stationary LIO acceptance requires >=100 odometry samples and >=20 seconds")

    rclpy.init()
    node = StationaryLioValidator()
    startup_deadline = time.monotonic() + args.startup_timeout_seconds
    try:
        while rclpy.ok() and time.monotonic() < startup_deadline and not node.odom:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.odom:
            raise SystemExit("LIO odometry was not received before the startup timeout")
        collection_start = time.monotonic()
        while rclpy.ok() and time.monotonic() - collection_start < args.collection_seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
        if (
            len(node.odom) < 2
            or not node.clouds
            or not node.imu_accel
            or not node.tf_odom_base
        ):
            raise SystemExit("required LIO outputs were not received")
        payload = build_result(node, args.reference, args.minimum_odom)
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
