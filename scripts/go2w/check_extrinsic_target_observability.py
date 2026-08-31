#!/usr/bin/env python3
"""Quick LiDAR observability gate for an RGB-LiDAR extrinsic calibration target.

This is a fail-fast host-side check. It loads already-extracted v2 datasets
(outputs/calibration/<name>_dataset_v2/scene_*/points.npy), transforms every
cloud from the vendor LiDAR frame into base_link with the pinned official
reference, voxelizes, keeps voxels persistent across scenes of one pose, and
then reports how many voxels are unique to each pose after cross-pose
differencing. A real board should produce a compact, planar, board-sized
cluster of unique voxels in the frontal ROI; a paper-on-box target produced
only 17-57 unique voxels in the rejected medium/right poses.

The script never connects to the robot, never subscribes to ROS topics, and
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


def rotation_matrix(rpy: list[float]) -> np.ndarray:
    rx, ry, rz = (float(value) for value in rpy)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array(
        [
            [cy * cz, -cy * sz, sy],
            [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy],
            [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy],
        ]
    )


def load_official_reference() -> tuple[np.ndarray, np.ndarray]:
    """Return R (3x3) and t (3,) of base_link -> utlidar_lidar from YAML."""
    reference_path = PROJECT_ROOT / "configs/go2w/official_reference.yaml"
    payload = yaml.safe_load(reference_path.read_text(encoding="utf-8")) or {}
    frames = payload.get("frames") or {}
    entry = frames.get("base_to_lidar") or {}
    xyz = entry.get("translation_m")
    rpy = entry.get("rotation_rpy_rad")
    if not xyz or not rpy or len(xyz) != 3 or len(rpy) != 3:
        raise RuntimeError("frames.base_to_lidar values are incomplete")
    return rotation_matrix(rpy), np.asarray(xyz, dtype=np.float64)


def points_in_base_link(
    points: np.ndarray, rotation: np.ndarray, translation: np.ndarray
) -> np.ndarray:
    """Invert base_link -> lidar so vendor points are expressed in base_link."""
    inverse_rotation = rotation.T
    return (inverse_rotation @ points.T).T + (-inverse_rotation @ translation)


def load_dataset(dataset: Path) -> list[np.ndarray]:
    scenes = sorted(dataset.glob("scene_*"))
    if not scenes:
        raise RuntimeError(f"dataset has no scene directories: {dataset}")
    clouds = []
    for scene in scenes:
        points_path = scene / "points.npy"
        if not points_path.is_file():
            raise RuntimeError(f"missing points.npy: {points_path}")
        values = np.load(points_path)
        if values.ndim != 2 or values.shape[1] != 3:
            raise RuntimeError(f"unexpected points shape: {values.shape}")
        clouds.append(np.asarray(values, dtype=np.float64))
    return clouds


def persistent_voxel_keys(
    clouds: list[np.ndarray],
    voxel_size: float,
    minimum_fraction: float,
) -> set[tuple[int, int, int]]:
    counts: dict[tuple[int, int, int], int] = {}
    for cloud in clouds:
        indices = np.floor(cloud / voxel_size).astype(np.int64)
        unique = {tuple(int(value) for value in row) for row in indices}
        for key in unique:
            counts[key] = counts.get(key, 0) + 1
    minimum = max(1, int(math.ceil(len(clouds) * minimum_fraction)))
    return {key for key, count in counts.items() if count >= minimum}


def voxel_centers(keys: set[tuple[int, int, int]], voxel_size: float) -> np.ndarray:
    if not keys:
        return np.empty((0, 3), dtype=np.float64)
    return (np.asarray(sorted(keys), dtype=np.float64) + 0.5) * voxel_size


def cluster_centers(centers: np.ndarray, radius: float) -> list[np.ndarray]:
    """Greedy radius clustering for the small unique-voxel set."""
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", action="append", required=True, type=Path)
    parser.add_argument(
        "--label", action="append", default=[], help="one label per --dataset"
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--voxel-size-m", type=float, default=0.02)
    parser.add_argument("--minimum-scene-fraction", type=float, default=0.6)
    parser.add_argument("--cluster-radius-m", type=float, default=0.08)
    args = parser.parse_args()
    if len(args.dataset) < 2:
        raise SystemExit("at least two poses are required for cross-pose differencing")
    labels = args.label or [dataset.name for dataset in args.dataset]
    if len(labels) != len(args.dataset):
        raise SystemExit("--label count must match --dataset count")
    if not 0.0 < args.voxel_size_m <= 0.1:
        raise SystemExit("voxel size must be in (0, 0.1] m")
    if not 0.0 < args.minimum_scene_fraction <= 1.0:
        raise SystemExit("scene fraction must be in (0, 1]")

    rotation, translation = load_official_reference()
    roi = {
        "x_min": 0.25,
        "x_max": 2.5,
        "y_min": -1.5,
        "y_max": 1.5,
        "z_min": -0.5,
        "z_max": 1.2,
    }

    pose_keys: list[set[tuple[int, int, int]]] = []
    scene_counts: list[int] = []
    for dataset in args.dataset:
        clouds = load_dataset(dataset)
        scene_counts.append(len(clouds))
        base_clouds = [
            points_in_base_link(cloud, rotation, translation) for cloud in clouds
        ]
        persistent = persistent_voxel_keys(
            base_clouds, args.voxel_size_m, args.minimum_scene_fraction
        )
        pose_keys.append(roi_keys(persistent, args.voxel_size_m, roi))

    pose_results = []
    for index, keys in enumerate(pose_keys):
        others = set()
        for other_index, other_keys in enumerate(pose_keys):
            if other_index != index:
                others |= other_keys
        unique = keys - others
        centers = voxel_centers(unique, args.voxel_size_m)
        clusters = cluster_centers(centers, args.cluster_radius_m)
        pose_results.append(
            {
                "label": labels[index],
                "dataset": str(args.dataset[index]),
                "scene_count": scene_counts[index],
                "persistent_voxel_count_roi": len(keys),
                "unique_voxel_count": len(unique),
                "clusters": [
                    plane_fit(cluster)
                    for cluster in sorted(clusters, key=len, reverse=True)[:10]
                ],
            }
        )

    summary = {
        "schema_version": "1.0",
        "robot_motion_commanded": False,
        "voxel_size_m": args.voxel_size_m,
        "minimum_scene_fraction": args.minimum_scene_fraction,
        "cluster_radius_m": args.cluster_radius_m,
        "roi_base_link": roi,
        "pose_results": pose_results,
        "note": (
            "observability gate only; it does not authorize fusion, extrinsics, "
            "or motion"
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
