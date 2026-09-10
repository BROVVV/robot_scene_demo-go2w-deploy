#!/usr/bin/env python3
"""Fail-fast LiDAR observability gate for a new extrinsic calibration target.

Reads read-only rosbag2 bags recorded by record_extrinsic_calibration.sh. If a
bag contains /go2w/lidar/obstacles (already filtered, base_link), it uses that
topic. Otherwise it falls back to /go2w/sensors/cloud and applies the exact
pinned official base_link transform plus the same height/range/self filters
used by go2w_lidar_preprocessor.

For each pose the script voxelizes sampled clouds, keeps voxels persistent
across scenes, removes voxels shared with other poses (the stationary
background), and reports the remaining unique voxels, their largest clusters,
and plane fits. A real matte board at least 0.6 x 0.6 m should leave a compact
planar cluster of hundreds of unique voxels; the rejected paper-on-box target
left only tens of voxels.

The script never connects to the robot, never subscribes to live topics, and
never sends motion commands. Output is written atomically to a JSON report.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def atomic_text(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def rpy_to_quaternion(rpy) -> tuple[float, float, float, float]:
    roll, pitch, yaw = (float(value) for value in rpy)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_matrix(quaternion: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise RuntimeError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def load_geometry() -> tuple[np.ndarray, np.ndarray, dict]:
    reference = yaml.safe_load(
        (PROJECT_ROOT / "configs/go2w/official_reference.yaml").read_text(
            encoding="utf-8"
        )
    ) or {}
    entry = reference["frames"]["base_to_lidar"]
    translation = np.asarray(entry["translation_m"], dtype=np.float64)
    rotation = quaternion_matrix(rpy_to_quaternion(entry["rotation_rpy_rad"]))
    lidar = yaml.safe_load(
        (PROJECT_ROOT / "configs/go2w/lidar_preprocess.yaml").read_text(
            encoding="utf-8"
        )
    ) or {}
    envelope = reference["dimensions"]["standing_envelope_m"]
    parameters = {
        "minimum_range": float(lidar["range_m"]["minimum"]),
        "maximum_range": float(lidar["range_m"]["maximum"]),
        "minimum_height": float(lidar["height_m"]["minimum"]),
        "maximum_height": float(lidar["height_m"]["maximum"]),
        "ground_height": float(lidar["ground_separation_height_m"]),
        "self_half_length": float(envelope["length"]) / 2.0,
        "self_half_width": float(envelope["width"]) / 2.0,
        "self_filter_margin": float(lidar["self_filter_margin_m"]),
    }
    return rotation, translation, parameters


def filter_points(points: np.ndarray, p: dict) -> np.ndarray:
    values = np.asarray(points, dtype=np.float64)
    finite = np.isfinite(values).all(axis=1)
    ranges = np.linalg.norm(values, axis=1)
    bounded = (
        finite
        & (ranges >= p["minimum_range"])
        & (ranges <= p["maximum_range"])
        & (values[:, 2] >= p["minimum_height"])
        & (values[:, 2] <= p["maximum_height"])
    )
    self_points = (
        (np.abs(values[:, 0]) <= p["self_half_length"] + p["self_filter_margin"])
        & (np.abs(values[:, 1]) <= p["self_half_width"] + p["self_filter_margin"])
    )
    filtered = values[bounded & ~self_points]
    return filtered[filtered[:, 2] > p["ground_height"]]


def storage_identifier(bag: Path) -> str:
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise RuntimeError(f"ROS bag metadata is absent: {metadata_path}")
    payload = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = payload.get("rosbag2_bagfile_information") or payload
    value = str(info.get("storage_identifier") or "").strip()
    if not value:
        raise RuntimeError("ROS bag storage identifier is absent")
    return value


def bag_topic_types(bag: Path) -> dict:
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=storage_identifier(bag)
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    return {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }


def load_clouds(
    bag: Path,
    topic: str,
    maximum_samples: int,
    rotation: np.ndarray,
    translation: np.ndarray,
    parameters: dict,
    fallback_raw: bool,
) -> tuple[list[np.ndarray], str]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    types = bag_topic_types(bag)
    raw_topic = "/go2w/sensors/cloud"
    if topic not in types:
        if fallback_raw and raw_topic in types:
            topic = raw_topic
        else:
            raise RuntimeError(
                f"bag {bag} has neither {topic} nor fallback {raw_topic}"
            )
    message_type = get_message(types[topic])
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=storage_identifier(bag)
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    clouds = []
    while reader.has_next():
        name, serialized, _ = reader.read_next()
        if name != topic:
            continue
        message = deserialize_message(serialized, message_type)
        records = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        values = np.column_stack(
            tuple(
                np.asarray(records[field], dtype=np.float64)
                for field in ("x", "y", "z")
            )
        )
        if topic == raw_topic:
            values = values @ rotation.T + translation
            values = filter_points(values, parameters)
        clouds.append(values)
    if not clouds:
        raise RuntimeError(f"bag {bag} has no {topic} samples")
    if len(clouds) > maximum_samples:
        indices = np.linspace(0, len(clouds) - 1, maximum_samples, dtype=np.int64)
        clouds = [clouds[int(index)] for index in indices]
    return clouds, topic


def persistent_voxel_keys(
    clouds: list[np.ndarray], voxel_size: float, minimum_fraction: float
) -> set[tuple[int, int, int]]:
    counts: dict[tuple[int, int, int], int] = {}
    for cloud in clouds:
        indices = np.floor(cloud / voxel_size).astype(np.int64)
        for row in set(map(tuple, indices)):
            counts[row] = counts.get(row, 0) + 1
    minimum = max(1, int(math.ceil(len(clouds) * minimum_fraction)))
    return {key for key, count in counts.items() if count >= minimum}


def roi_keys(
    keys: set[tuple[int, int, int]], voxel_size: float, roi: dict
) -> set[tuple[int, int, int]]:
    return {
        key
        for key in keys
        if roi["x_min"] <= (key[0] + 0.5) * voxel_size <= roi["x_max"]
        and roi["y_min"] <= (key[1] + 0.5) * voxel_size <= roi["y_max"]
        and roi["z_min"] <= (key[2] + 0.5) * voxel_size <= roi["z_max"]
    }


def voxel_centers(keys: set[tuple[int, int, int]], voxel_size: float) -> np.ndarray:
    if not keys:
        return np.empty((0, 3), dtype=np.float64)
    return (np.asarray(sorted(keys), dtype=np.float64) + 0.5) * voxel_size


def cluster_centers(centers: np.ndarray, radius: float) -> list[np.ndarray]:
    if len(centers) == 0:
        return []
    remaining = list(range(len(centers)))
    clusters: list[np.ndarray] = []
    while remaining:
        seed = remaining.pop(0)
        members = [seed]
        while True:
            seed_point = centers[members[0]]
            keep = []
            grew = False
            for index in remaining:
                if float(np.linalg.norm(centers[index] - seed_point)) <= radius:
                    members.append(index)
                    grew = True
                else:
                    keep.append(index)
            remaining = keep
            if not grew:
                break
        clusters.append(centers[members])
    return clusters


def plane_fit(points: np.ndarray) -> dict:
    if len(points) < 3:
        return {
            "count": int(len(points)),
            "normal_base": None,
            "rho_m": None,
            "rms_m": None,
            "span_m": None,
            "plane_fit_ok": False,
        }
    centroid = points.mean(axis=0)
    _, singular_values, vh = np.linalg.svd(points - centroid)
    normal = vh[-1]
    distances = np.abs((points - centroid) @ normal)
    return {
        "count": int(len(points)),
        "normal_base": [float(value) for value in normal],
        "rho_m": float(np.dot(normal, centroid)),
        "rms_m": float(np.sqrt(np.mean(distances**2))),
        "span_m": [float(value) for value in singular_values],
        "plane_fit_ok": bool(len(points) >= 8),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", action="append", required=True, type=Path)
    parser.add_argument("--label", action="append", default=[])
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--topic", default="/go2w/lidar/obstacles")
    parser.add_argument("--fallback-raw", action="store_true")
    parser.add_argument("--voxel-size-m", type=float, default=0.02)
    parser.add_argument("--minimum-scene-fraction", type=float, default=0.6)
    parser.add_argument("--cluster-radius-m", type=float, default=0.08)
    parser.add_argument("--maximum-samples", type=int, default=20)
    args = parser.parse_args()
    if len(args.bag) < 2:
        raise SystemExit("at least two poses are required for cross-pose differencing")
    labels = args.label or [bag.name for bag in args.bag]
    if len(labels) != len(args.bag):
        raise SystemExit("--label count must match --bag count")
    if not 0.0 < args.voxel_size_m <= 0.1:
        raise SystemExit("voxel size must be in (0, 0.1] m")

    rotation, translation, parameters = load_geometry()
    roi = {
        "x_min": 0.25,
        "x_max": 2.5,
        "y_min": -1.5,
        "y_max": 1.5,
        "z_min": -0.5,
        "z_max": 1.2,
    }
    pose_keys: list[set[tuple[int, int, int]]] = []
    pose_info: list[dict] = []
    used_topics: list[str] = []
    for bag, label in zip(args.bag, labels):
        bag = bag.expanduser().resolve()
        if not (bag / "metadata.yaml").is_file():
            raise SystemExit(f"not a rosbag2 directory: {bag}")
        clouds, topic = load_clouds(
            bag,
            args.topic,
            args.maximum_samples,
            rotation,
            translation,
            parameters,
            args.fallback_raw,
        )
        persistent = persistent_voxel_keys(
            clouds, args.voxel_size_m, args.minimum_scene_fraction
        )
        pose_keys.append(roi_keys(persistent, args.voxel_size_m, roi))
        pose_info.append(
            {
                "label": label,
                "bag": str(bag),
                "topic": topic,
                "cloud_samples": len(clouds),
                "persistent_voxel_count_roi": len(pose_keys[-1]),
            }
        )
        used_topics.append(topic)

    pose_results = []
    for index, keys in enumerate(pose_keys):
        others = set()
        for other_index, other_keys in enumerate(pose_keys):
            if other_index != index:
                others |= other_keys
        unique = keys - others
        centers = voxel_centers(unique, args.voxel_size_m)
        clusters = cluster_centers(centers, args.cluster_radius_m)
        fits = [
            plane_fit(cluster)
            for cluster in sorted(clusters, key=len, reverse=True)[:10]
        ]
        best = fits[0] if fits else None
        pose_results.append(
            {
                **pose_info[index],
                "unique_voxel_count": len(unique),
                "best_cluster": best,
                "clusters": fits,
            }
        )

    promising_poses = [
        result
        for result in pose_results
        if result["unique_voxel_count"] >= 150
        and result["best_cluster"] is not None
        and result["best_cluster"]["count"] >= 100
        and result["best_cluster"]["plane_fit_ok"]
        and result["best_cluster"]["rms_m"] is not None
        and result["best_cluster"]["rms_m"] < 0.05
    ]
    summary = {
        "schema_version": "1.0",
        "robot_motion_commanded": False,
        "voxel_size_m": args.voxel_size_m,
        "minimum_scene_fraction": args.minimum_scene_fraction,
        "cluster_radius_m": args.cluster_radius_m,
        "roi_base_link": roi,
        "pose_results": pose_results,
        "observability_promising": len(promising_poses) >= 2,
        "promising_poses": [result["label"] for result in promising_poses],
        "gate_note": (
            "pre-solve observability gate only; the authoritative acceptance is "
            "held-out overlay edge error <= 5 px after solving"
        ),
        "note": (
            "this gate does not authorize extrinsics, fusion, or motion"
        ),
    }
    atomic_text(
        args.output,
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
