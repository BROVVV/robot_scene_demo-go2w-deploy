from __future__ import annotations

import math
from pathlib import Path

import yaml
from go2w_description.description_config import (
    load_official_reference,
    official_sensor_to_base_extrinsics,
)


ALLOWED_STATIONARY_STATUSES = {
    "stationary_trial_authorized",
    "stationary_read_only_validated",
}

POINT_LIO_COMMIT = "18ed5976d8fab2bd8a5148c26a40692bd3c0dc91"


def _finite_positive(value, label: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"LIO parameter must be finite and positive: {label}")
    return result


def load_lio_gate(lio_path: str | Path, reference_path: str | Path, time_path: str | Path) -> dict:
    """Resolve the read-only stationary RKO-LIO runtime configuration.

    Passing this gate authorizes sensor processing and odometry publication only.
    It never authorizes a velocity command, navigation execution, or robot motion.
    """

    lio = yaml.safe_load(Path(lio_path).read_text(encoding="utf-8")) or {}
    timing = yaml.safe_load(Path(time_path).read_text(encoding="utf-8")) or {}
    reference = load_official_reference(reference_path)
    if lio.get("implementation") != "rko_lio" or str(
        lio.get("implementation_version")
    ) != "0.3.0":
        raise ValueError("only the audited native ROS 2 rko_lio 0.3.0 implementation is allowed")
    if not lio.get("enabled") or lio.get("tuning_status") not in ALLOWED_STATIONARY_STATUSES:
        raise ValueError("LIO parameters are disabled or not authorized for a stationary trial")
    if lio.get("validation_scope") != "stationary_read_only":
        raise ValueError("LIO validation scope must remain stationary and read-only")

    safety = lio.get("safety") or {}
    if safety.get("stationary_only") is not True or safety.get("authorizes_motion") is not False:
        raise ValueError("LIO safety contract must explicitly forbid motion")
    if safety.get("publish_tf") is not True:
        raise ValueError("the audited odom-to-base TF publisher must be enabled")
    if safety.get("allow_second_odom_base_publisher") is not False:
        raise ValueError("a second odom-to-base publisher must be forbidden")
    if (reference.get("safety") or {}).get("authorizes_motion") is not False:
        raise ValueError("manufacturer reference must not authorize motion")

    if lio.get("base_frame") != "base_link" or lio.get("odom_frame") != "odom":
        raise ValueError("LIO must publish the audited odom-to-base_link frame contract")
    if lio.get("lidar_topic") != "/go2w/lio_input/cloud_raw" or lio.get(
        "imu_topic"
    ) != "/go2w/lio_input/imu_raw":
        raise ValueError("LIO must consume the raw timestamp-preserving sensor copies")

    if not timing.get("stable") or not (timing.get("cloud_imu_relative_clock") or {}).get(
        "stable"
    ):
        raise ValueError("stable live cloud/IMU timing is required")
    if timing.get("raw_lio_relative_time_preserved") is not True or timing.get(
        "point_time_field_modified"
    ) is not False:
        raise ValueError("raw LIO timing preservation is not proven")
    point_time = lio.get("point_time") or {}
    if (
        point_time.get("field") != "time"
        or point_time.get("interpretation") != "relative_to_cloud_header"
        or float(point_time.get("multiplier_to_seconds", 0.0)) != 1.0
        or point_time.get("field_modified") is not False
    ):
        raise ValueError("per-point relative time contract is invalid")

    parameters = lio.get("parameters") or {}
    if parameters.get("initialization_phase") is not True:
        raise ValueError("stationary initialization must be enabled")
    if parameters.get("deskew") not in (True, False):
        raise ValueError("LiDAR deskew selection must be explicit")
    if parameters.get("deskew") is False and lio.get("tuning_status") != "stationary_trial_authorized":
        raise ValueError("deskew may only be disabled for a stationary diagnostic trial")
    voxel = _finite_positive(parameters.get("voxel_size_m"), "voxel_size_m")
    minimum = _finite_positive(parameters.get("minimum_range_m"), "minimum_range_m")
    maximum = _finite_positive(parameters.get("maximum_range_m"), "maximum_range_m")
    correspondence = _finite_positive(
        parameters.get("max_correspondence_distance_m"),
        "max_correspondence_distance_m",
    )
    if not (0.02 <= voxel <= 1.0 and 0.1 <= minimum < maximum <= 30.0):
        raise ValueError("LIO range or voxel parameters are outside the stationary trial envelope")
    if correspondence > 2.0:
        raise ValueError("LIO correspondence distance is outside the stationary trial envelope")
    timeout = _finite_positive(safety.get("input_timeout_seconds"), "input_timeout_seconds")
    if timeout > 1.0:
        raise ValueError("LIO input timeout must close within one second")

    extrinsics = lio.get("extrinsics") or {}
    if (
        extrinsics.get("source") != "unitree_official_reference"
        or extrinsics.get("rko_convention") != "sensor_to_base_quat_xyzw_xyz"
        or extrinsics.get("runtime_resolution") != "compose_pinned_reference_frames"
    ):
        raise ValueError("LIO official extrinsic provenance contract is incomplete")

    resolved = official_sensor_to_base_extrinsics(reference)
    if not all(
        len(vector) == 7 and all(math.isfinite(value) for value in vector)
        for vector in resolved.values()
    ):
        raise ValueError("resolved LIO extrinsics are invalid")
    lio["resolved_extrinsics"] = resolved
    lio["resolved_reference"] = {
        "robot_model": reference["robot_model"],
        "reference_status": reference["reference_status"],
        "urdf_commit": reference["sources"]["go2w_urdf"]["commit"],
        "urdf_sha256": reference["sources"]["go2w_urdf"]["sha256"],
    }
    return lio


def load_point_lio_gate(
    lio_path: str | Path, reference_path: str | Path, time_path: str | Path
) -> dict:
    """Resolve the isolated official Point-LIO stationary-only runtime."""

    lio = yaml.safe_load(Path(lio_path).read_text(encoding="utf-8")) or {}
    timing = yaml.safe_load(Path(time_path).read_text(encoding="utf-8")) or {}
    reference = load_official_reference(reference_path)
    source = (reference.get("sources") or {}).get("point_lio_unilidar") or {}
    if (
        lio.get("implementation") != "unitree_point_lio_unilidar"
        or lio.get("upstream_commit") != POINT_LIO_COMMIT
        or source.get("publisher") != "Unitree Robotics"
        or source.get("commit") != POINT_LIO_COMMIT
    ):
        raise ValueError("Point-LIO must use the pinned official Unitree implementation")
    if not lio.get("enabled") or lio.get("tuning_status") not in ALLOWED_STATIONARY_STATUSES:
        raise ValueError("Point-LIO is disabled or not authorized for a stationary trial")
    if lio.get("validation_scope") != "stationary_read_only":
        raise ValueError("Point-LIO validation scope must remain stationary and read-only")

    isolation = lio.get("isolation") or {}
    if (
        isolation.get("environment") != "go2w_point_lio_noetic"
        or isolation.get("ros_distribution") != "noetic"
        or isolation.get("host") != "127.0.0.1"
        or int(isolation.get("port", 0)) != 29876
    ):
        raise ValueError("Point-LIO must remain in the audited localhost Noetic isolation")
    bridge = lio.get("bridge") or {}
    required_bridge = {
        "generic_topic_forwarding": False,
        "ros2_cloud_input": "/go2w/lio_input/cloud_raw",
        "ros2_imu_input": "/go2w/lio_input/imu_raw",
        "ros1_cloud_input": "/unilidar/cloud",
        "ros1_imu_input": "/unilidar/imu",
        "ros1_odometry_output": "/pointlio/odom",
        "ros1_registered_cloud_output": "/pointlio/cloud_registered",
    }
    if any(bridge.get(key) != value for key, value in required_bridge.items()):
        raise ValueError("Point-LIO bridge violates the fixed read-only topic allow list")

    safety = lio.get("safety") or {}
    if (
        safety.get("stationary_only") is not True
        or safety.get("authorizes_motion") is not False
        or safety.get("forwards_control_messages") is not False
        or safety.get("pose_republished_when_stale") is not False
    ):
        raise ValueError("Point-LIO safety contract must explicitly forbid motion")
    timeout = _finite_positive(safety.get("input_timeout_seconds"), "input_timeout_seconds")
    if timeout > 1.0:
        raise ValueError("Point-LIO timeout must close within one second")
    if (reference.get("safety") or {}).get("authorizes_motion") is not False:
        raise ValueError("manufacturer reference must not authorize motion")
    if not timing.get("stable") or not (timing.get("cloud_imu_relative_clock") or {}).get(
        "stable"
    ):
        raise ValueError("stable live cloud/IMU timing is required")
    if timing.get("raw_lio_relative_time_preserved") is not True or timing.get(
        "point_time_field_modified"
    ) is not False:
        raise ValueError("Point-LIO requires unmodified raw per-point time")

    frames = lio.get("frames") or {}
    if frames != {
        "lidar": "utlidar_lidar",
        "imu": "utlidar_imu",
        "odom": "odom",
        "base": "base_link",
    }:
        raise ValueError("Point-LIO frame contract is not the audited contract")
    imu_frame = lio.get("imu_frame") or {}
    gyro_sign = imu_frame.get("gyro_sign") or []
    if (
        len(gyro_sign) != 3
        or not all(isinstance(value, (int, float)) for value in gyro_sign)
        or not all(float(value) in (-1.0, 1.0) for value in gyro_sign)
    ):
        raise ValueError("Point-LIO IMU gyro sign correction must be three +/-1 values")
    if not isinstance(imu_frame.get("yaw_reflect"), bool):
        raise ValueError("Point-LIO IMU frame yaw_reflect must be a bool")
    lio["imu_frame"] = imu_frame
    resolved = official_sensor_to_base_extrinsics(reference)
    lio["resolved_extrinsics"] = resolved
    return lio
