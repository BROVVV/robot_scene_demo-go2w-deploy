#!/usr/bin/env python3
"""Validate the latest complete frame bundle from the Conda side."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.live_robot.frame_bundle_reader import FrameBundleReader


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--spool-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    bundle = FrameBundleReader(args.spool_root).read_latest(timeout_seconds=5.0)
    image = cv2.imread(str(bundle.image_path), cv2.IMREAD_COLOR)
    payload = bundle.payload
    errors = []
    camera_info = payload.get("camera_info") or {}
    expected_shape = (
        int(camera_info.get("height", 0)),
        int(camera_info.get("width", 0)),
    )
    if (
        image is None
        or expected_shape[0] < 480
        or expected_shape[1] < 640
        or image.shape[:2] != expected_shape
    ):
        errors.append("image_and_camera_info_resolution_invalid_or_mismatched")
    solid_green_fraction = None
    if image is not None:
        blue = image[:, :, 0].astype(np.int16)
        green = image[:, :, 1].astype(np.int16)
        red = image[:, :, 2].astype(np.int16)
        solid_green_fraction = float(
            np.mean(
                (green >= 80)
                & (green >= blue * 3)
                & (green >= red * 3)
                & (blue <= 12)
                & (red <= 12)
            )
        )
        if solid_green_fraction >= 0.95:
            errors.append("solid_green_transport_corruption_detected")
    if payload.get("image_capture_time_trusted") is not False:
        errors.append("camera_capture_time_incorrectly_claimed_trusted")
    if payload.get("camera_frame") != "front_camera_optical_frame":
        errors.append("unexpected_camera_frame")
    health = payload.get("sensor_health") or {}
    if health.get("camera") is not True:
        errors.append("camera_health_false")
    # Intrinsics and diagnostic overlay are usable; navigation-grade geometry,
    # LIO and the full camera TF chain remain fail-closed.
    if health.get("camera_info_calibrated") is not True:
        errors.append("camera_intrinsics_health_false")
    if health.get("rgb_lidar_overlay") is not True:
        errors.append("rgb_lidar_diagnostic_overlay_health_false")
    for key in (
        "rgb_lidar_extrinsics",
        "rgb_lidar_fusion",
        "lio",
        "tf",
    ):
        if health.get(key, False) is not False:
            errors.append(f"expected_fail_closed_health:{key}")
    if health.get("lidar") is not True:
        errors.append("validated_lidar_health_false")
    if (payload.get("robot_pose") or {}).get("available") is not False:
        errors.append("unavailable_lio_pose_was_fabricated")
    clearance = payload.get("clearance") or {}
    if clearance.get("lidar_fresh") is not True:
        errors.append("validated_lidar_clearance_not_fresh")
    for key in ("front_m", "left_m", "right_m"):
        value = clearance.get(key)
        if value is not None and not isinstance(value, (int, float)):
            errors.append(f"invalid_clearance_value:{key}")
    expected_statuses = {
        "front_status": {"measured", "no_return"},
        "left_status": {"measured", "no_return", "unknown"},
        "right_status": {"measured", "no_return", "unknown"},
    }
    for key, allowed in expected_statuses.items():
        if clearance.get(key) not in allowed:
            errors.append(f"invalid_clearance_status:{key}")
    if clearance.get("rotation_clearance_valid") is not False:
        errors.append("rotation_clearance_should_be_fail_closed")
    if clearance.get("left_status") != "unknown" or clearance.get(
        "right_status"
    ) != "unknown":
        errors.append("unvalidated_side_clearance_was_not_marked_unknown")
    result = {
        "passed": not errors,
        "errors": errors,
        "bundle_directory": str(bundle.directory),
        "frame_id": bundle.frame_id,
        "image_shape": list(image.shape) if image is not None else None,
        "solid_green_fraction": solid_green_fraction,
        "sensor_health": health,
        "motion_commands_sent": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
