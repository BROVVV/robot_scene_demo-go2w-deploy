#!/usr/bin/env python3
"""Estimate a non-authoritative RGB-LiDAR candidate from 3D-2D edge pairs."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = PROJECT_ROOT / "ros2_ws/src/go2w_rgb_lidar_fusion"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from go2w_rgb_lidar_fusion.overlay_core import (  # noqa: E402
    CalibrationBlocked,
    estimate_lidar_to_camera_pnp,
    load_camera_model,
    render_depth_overlay,
    reprojection_metrics,
    rpy_from_rotation_matrix,
)


def atomic_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_scene(path: Path) -> dict:
    directory = path.expanduser().resolve()
    points_path = directory / "points.npy"
    annotation_path = directory / "correspondences.yaml"
    if not points_path.is_file() or not annotation_path.is_file():
        raise CalibrationBlocked(
            f"scene requires points.npy and correspondences.yaml: {directory}"
        )
    points = np.asarray(np.load(points_path, allow_pickle=False), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise CalibrationBlocked(f"scene point cloud must be finite Nx3: {directory}")
    annotation = yaml.safe_load(annotation_path.read_text(encoding="utf-8")) or {}
    label = str(annotation.get("scene_label") or directory.name).strip()
    band = str(annotation.get("distance_band") or "").strip().lower()
    if band not in {"near", "medium", "far", "moved_recheck"}:
        raise CalibrationBlocked(
            f"scene distance_band must be near/medium/far/moved_recheck: {directory}"
        )
    objects = []
    images = []
    indices = []
    for item in annotation.get("correspondences") or []:
        try:
            index = int(item["point_index"])
            pixel = np.asarray(item["image_px"], dtype=np.float64)
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationBlocked(f"invalid correspondence in {directory}") from exc
        if index < 0 or index >= len(points) or pixel.shape != (2,) or not np.isfinite(pixel).all():
            raise CalibrationBlocked(f"correspondence is out of range in {directory}")
        indices.append(index)
        objects.append(points[index])
        images.append(pixel)
    if not objects:
        raise CalibrationBlocked(f"scene has no 3D-2D correspondences: {directory}")
    return {
        "directory": directory,
        "label": label,
        "distance_band": band,
        "points": points,
        "point_indices": indices,
        "object_points": np.asarray(objects, dtype=np.float64),
        "image_points": np.asarray(images, dtype=np.float64),
        "image_path": directory / "image.jpg",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", required=True, type=Path)
    parser.add_argument("--scene", required=True, action="append", type=Path)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--maximum-candidate-mean-error-px", type=float, default=5.0)
    args = parser.parse_args()
    if not args.operator.strip():
        raise SystemExit("operator must not be empty")
    if args.maximum_candidate_mean_error_px <= 0.0:
        raise SystemExit("maximum candidate error must be positive")
    output = args.output_dir.expanduser().resolve()
    try:
        camera = load_camera_model(args.camera)
        scenes = [load_scene(path) for path in args.scene]
        objects = np.vstack([scene["object_points"] for scene in scenes])
        images = np.vstack([scene["image_points"] for scene in scenes])
        transform, metrics = estimate_lidar_to_camera_pnp(objects, images, camera)
        if metrics.mean_px > args.maximum_candidate_mean_error_px:
            raise CalibrationBlocked(
                f"candidate mean reprojection error {metrics.mean_px:.3f}px exceeds "
                f"{args.maximum_candidate_mean_error_px:.3f}px"
            )
        roll, pitch, yaw = rpy_from_rotation_matrix(transform.rotation)
        per_scene = []
        overlay_root = output / "overlays"
        for scene in scenes:
            scene_metrics = reprojection_metrics(
                scene["object_points"], scene["image_points"], camera, transform
            )
            overlay_path = None
            if scene["image_path"].is_file():
                image = cv2.imread(str(scene["image_path"]), cv2.IMREAD_COLOR)
                if image is None:
                    raise CalibrationBlocked(
                        f"scene image cannot be decoded: {scene['image_path']}"
                    )
                overlay, overlay_summary = render_depth_overlay(
                    image, scene["points"], camera, transform
                )
                overlay_root.mkdir(parents=True, exist_ok=True)
                overlay_path = overlay_root / f"{scene['label']}.jpg"
                if not cv2.imwrite(str(overlay_path), overlay):
                    raise RuntimeError(f"failed to write overlay: {overlay_path}")
            else:
                overlay_summary = None
            per_scene.append(
                {
                    "scene_label": scene["label"],
                    "distance_band": scene["distance_band"],
                    "correspondence_count": scene_metrics.count,
                    "mean_reprojection_error_px": scene_metrics.mean_px,
                    "p95_reprojection_error_px": scene_metrics.p95_px,
                    "maximum_reprojection_error_px": scene_metrics.maximum_px,
                    "overlay_path": str(overlay_path) if overlay_path else None,
                    "overlay_summary": overlay_summary,
                }
            )
        timestamp = dt.datetime.now(dt.timezone.utc).isoformat()
        candidate = {
            "schema_version": 1,
            "robot_model": "Unitree Go2-W",
            "calibration_status": "candidate_unvalidated",
            "confirmed": False,
            "source": "multi_scene_3d_2d_pnp_candidate",
            "operator": args.operator.strip(),
            "timestamp": timestamp,
            "transform_parent": "front_camera_optical_frame",
            "transform_child": "utlidar_lidar",
            "translation_m": {
                "x": float(transform.translation[0]),
                "y": float(transform.translation[1]),
                "z": float(transform.translation[2]),
            },
            "rotation_rpy_rad": {
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            },
            "estimation": {
                "scene_count": len(scenes),
                "correspondence_count": metrics.count,
                "mean_reprojection_error_px": metrics.mean_px,
                "median_reprojection_error_px": metrics.median_px,
                "p95_reprojection_error_px": metrics.p95_px,
                "maximum_reprojection_error_px": metrics.maximum_px,
                "camera_calibration_source": camera.source,
            },
            "validation": {
                "required_scene_count": 5,
                "completed_scene_count": 0,
                "mean_edge_error_px": None,
                "maximum_allowed_mean_edge_error_px": 5.0,
                "distance_bands_tested": [],
                "moved_position_recheck_passed": False,
                "report_path": None,
            },
            "authorizes_fusion": False,
            "authorizes_motion": False,
        }
        report = {
            "schema_version": "1.0",
            "passed_candidate_estimation": True,
            "installed_or_confirmed": False,
            "authorizes_fusion": False,
            "authorizes_motion": False,
            "candidate_path": str(output / "sensor_extrinsics_candidate.yaml"),
            "camera_source": camera.source,
            "aggregate_metrics": candidate["estimation"],
            "scenes": per_scene,
            "next_gate": (
                "operator overlay validation in at least five near/medium/far scenes "
                "plus a moved-position recheck"
            ),
        }
        atomic_text(
            output / "sensor_extrinsics_candidate.yaml",
            yaml.safe_dump(candidate, sort_keys=False, allow_unicode=True),
        )
        atomic_text(
            output / "pnp_report.json",
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except CalibrationBlocked as exc:
        result = {
            "schema_version": "1.0",
            "passed_candidate_estimation": False,
            "installed_or_confirmed": False,
            "authorizes_fusion": False,
            "authorizes_motion": False,
            "blocker": str(exc),
        }
        atomic_text(
            output / "pnp_report.json",
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        )
        print("BLOCKED: " + str(exc), file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
