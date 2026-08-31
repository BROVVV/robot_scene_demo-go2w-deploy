"""Validate measured geometry or pinned Unitree sensor-frame references."""

from __future__ import annotations

import math
from pathlib import Path
from xml.sax.saxutils import escape

import yaml


LENGTH_KEYS = (
    "body_max_length_m",
    "body_max_width_m",
    "stationary_standing_height_m",
    "wheel_outer_envelope_length_m",
    "wheel_outer_envelope_width_m",
    "base_link_ground_height_m",
)

POSE_PREFIXES = ("front_camera", "lidar", "lidar_imu")
POSE_SUFFIXES = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
REQUIRED_KEYS = LENGTH_KEYS + tuple(
    f"{prefix}_{suffix}" for prefix in POSE_PREFIXES for suffix in POSE_SUFFIXES
)

OFFICIAL_FRAME_KEYS = ("base_to_lidar", "lidar_to_lidar_imu")


def _rpy_to_quaternion_xyzw(rpy: tuple[float, float, float]) -> tuple[float, ...]:
    """Return the ROS quaternion for a fixed-axis roll/pitch/yaw rotation."""

    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_multiply_xyzw(left, right) -> tuple[float, ...]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion, vector) -> tuple[float, ...]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # Expanded q * [v, 0] * q^-1 for a unit quaternion.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def load_confirmed_measurements(path: str | Path) -> dict[str, float]:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if payload.get("robot_model") != "Unitree Go2-W":
        raise ValueError("geometry is not explicitly identified as Unitree Go2-W")
    if payload.get("measurement_status") != "measured" or not payload.get("confirmed"):
        raise ValueError("Go2-W geometry is unmeasured or not operator-confirmed")
    measurements = payload.get("measurements") or {}
    values = {}
    for key in REQUIRED_KEYS:
        record = measurements.get(key) or {}
        value = record.get("value")
        metadata = (
            record.get("measurement_method"),
            record.get("operator"),
            record.get("timestamp"),
            record.get("uncertainty"),
        )
        if value is None or not all(item not in (None, "") for item in metadata):
            raise ValueError(f"measurement incomplete: {key}")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"measurement is not finite: {key}")
        if key in LENGTH_KEYS and value <= 0.0:
            raise ValueError(f"length measurement must be positive: {key}")
        values[key] = value
    return values


def _finite_vector(value, *, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"official reference vector must have {length} values: {label}")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"official reference vector is not finite: {label}")
    return result


def load_official_reference(path: str | Path) -> dict:
    """Load the pinned manufacturer reference without treating it as calibration."""

    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if payload.get("robot_model") != "Unitree Go2-W":
        raise ValueError("official reference is not explicitly identified as Unitree Go2-W")
    if payload.get("reference_status") != "manufacturer_published":
        raise ValueError("Go2-W manufacturer reference is not approved")
    sources = payload.get("sources") or {}
    urdf_source = sources.get("go2w_urdf") or {}
    if (
        urdf_source.get("publisher") != "Unitree Robotics"
        or len(str(urdf_source.get("commit", ""))) != 40
        or len(str(urdf_source.get("sha256", ""))) != 64
        or not str(urdf_source.get("raw_url", "")).startswith(
            "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        )
    ):
        raise ValueError("Go2-W official URDF provenance is incomplete")

    dimensions = payload.get("dimensions") or {}
    envelope = dimensions.get("standing_envelope_m") or {}
    envelope_values = tuple(
        float(envelope.get(key, 0.0)) for key in ("length", "width", "height")
    )
    if envelope.get("source") != "product_page" or not all(
        math.isfinite(item) and item > 0.0 for item in envelope_values
    ):
        raise ValueError("Go2-W manufacturer standing envelope is invalid")

    frames = payload.get("frames") or {}
    mapping = frames.get("base_frame_mapping") or {}
    if (
        mapping.get("official_urdf_frame") != "base"
        or mapping.get("ros_frame") != "base_link"
        or mapping.get("accepted") is not True
    ):
        raise ValueError("official Go2-W base frame mapping is not accepted")
    expected = {
        "base_to_lidar": ("utlidar_lidar", "go2w_urdf_radar_joint"),
        "lidar_to_lidar_imu": (
            "utlidar_imu",
            "unilidar_sdk2_coordinate_system_definition",
        ),
    }
    for key in OFFICIAL_FRAME_KEYS:
        record = frames.get(key) or {}
        expected_child, expected_source = expected[key]
        child = record.get("ros_child") if key == "base_to_lidar" else record.get("child")
        if (
            child != expected_child
            or record.get("source") != expected_source
            or record.get("accepted") is not True
        ):
            raise ValueError(f"official Go2-W frame reference is incomplete: {key}")
        record["translation_m"] = _finite_vector(
            record.get("translation_m"), length=3, label=f"{key}.translation_m"
        )
        record["rotation_rpy_rad"] = _finite_vector(
            record.get("rotation_rpy_rad"), length=3, label=f"{key}.rotation_rpy_rad"
        )
    return payload


def official_sensor_to_base_extrinsics(reference: dict) -> dict[str, tuple[float, ...]]:
    """Resolve the pinned Unitree poses as RKO-LIO sensor-to-base transforms."""

    frames = reference["frames"]
    lidar = frames["base_to_lidar"]
    lidar_imu = frames["lidar_to_lidar_imu"]
    lidar_q = _rpy_to_quaternion_xyzw(lidar["rotation_rpy_rad"])
    lidar_t = tuple(lidar["translation_m"])
    imu_in_lidar_q = _rpy_to_quaternion_xyzw(lidar_imu["rotation_rpy_rad"])
    imu_in_lidar_t = tuple(lidar_imu["translation_m"])
    rotated_imu_t = _rotate_vector(lidar_q, imu_in_lidar_t)
    imu_t = tuple(a + b for a, b in zip(lidar_t, rotated_imu_t))
    imu_q = _quaternion_multiply_xyzw(lidar_q, imu_in_lidar_q)
    return {
        "lidar2base_quat_xyzw_xyz": (*lidar_q, *lidar_t),
        "imu2base_quat_xyzw_xyz": (*imu_q, *imu_t),
    }


def _fixed_joint(name: str, parent: str, child: str, xyz, rpy) -> str:
    xyz_text = " ".join(f"{value:.9g}" for value in xyz)
    rpy_text = " ".join(f"{value:.9g}" for value in rpy)
    return (
        f'<joint name="{escape(name)}" type="fixed">'
        f'<parent link="{escape(parent)}"/><child link="{escape(child)}"/>'
        f'<origin xyz="{xyz_text}" rpy="{rpy_text}"/></joint>'
    )


def render_urdf(values: dict[str, float]) -> str:
    links = (
        "base_footprint",
        "base_link",
        "imu_link",
        "front_camera_link",
        "front_camera_optical_frame",
        "utlidar_lidar",
        "utlidar_imu",
    )
    elements = ["<?xml version=\"1.0\"?>", '<robot name="unitree_go2w_measured">']
    elements.extend(f'<link name="{name}"/>' for name in links)
    elements.append(
        _fixed_joint(
            "base_footprint_to_base_link",
            "base_footprint",
            "base_link",
            (0.0, 0.0, values["base_link_ground_height_m"]),
            (0.0, 0.0, 0.0),
        )
    )
    mappings = (
        ("front_camera", "front_camera_link"),
        ("lidar", "utlidar_lidar"),
        ("lidar_imu", "utlidar_imu"),
    )
    for prefix, child in mappings:
        elements.append(
            _fixed_joint(
                f"base_link_to_{child}",
                "base_link",
                child,
                tuple(values[f"{prefix}_{axis}_m"] for axis in "xyz"),
                tuple(values[f"{prefix}_{axis}_rad"] for axis in ("roll", "pitch", "yaw")),
            )
        )
    # REP-103 optical convention is a coordinate convention, not a guessed pose.
    elements.append(
        _fixed_joint(
            "front_camera_to_optical",
            "front_camera_link",
            "front_camera_optical_frame",
            (0.0, 0.0, 0.0),
            (-math.pi / 2.0, 0.0, -math.pi / 2.0),
        )
    )
    elements.append("</robot>")
    return "".join(elements)


def render_official_sensor_urdf(reference: dict) -> str:
    """Render only the two fixed transforms published by Unitree."""

    frames = reference["frames"]
    lidar = frames["base_to_lidar"]
    lidar_imu = frames["lidar_to_lidar_imu"]
    elements = [
        '<?xml version="1.0"?>',
        '<robot name="unitree_go2w_official_sensor_frames">',
        '<link name="base_link"/>',
        '<link name="utlidar_lidar"/>',
        '<link name="utlidar_imu"/>',
        _fixed_joint(
            "base_link_to_utlidar_lidar",
            "base_link",
            "utlidar_lidar",
            lidar["translation_m"],
            lidar["rotation_rpy_rad"],
        ),
        _fixed_joint(
            "utlidar_lidar_to_utlidar_imu",
            "utlidar_lidar",
            "utlidar_imu",
            lidar_imu["translation_m"],
            lidar_imu["rotation_rpy_rad"],
        ),
        "</robot>",
    ]
    return "".join(elements)
