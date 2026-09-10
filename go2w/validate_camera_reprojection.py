#!/usr/bin/env python3
"""Sanity-check an installed camera calibration on a newly captured board frame."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def values(payload: dict, key: str) -> np.ndarray:
    record = payload.get(key) or {}
    return np.asarray(record.get("data", []), dtype=np.float64)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--camera", type=Path, required=True)
    parser.add_argument("--board", default="9x6")
    parser.add_argument("--square-m", type=float, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    cols, rows = (int(item) for item in args.board.lower().split("x", 1))
    if cols < 2 or rows < 2 or args.square_m <= 0.0:
        raise SystemExit("invalid board geometry")

    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"cannot read image: {args.image}")
    camera = yaml.safe_load(args.camera.read_text(encoding="utf-8")) or {}
    matrix = values(camera, "camera_matrix").reshape(3, 3)
    distortion = values(camera, "distortion_coefficients").reshape(-1)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray,
        (cols, rows),
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    payload = {
        "schema_version": "1.0",
        "validation_type": "new_frame_camera_reprojection_sanity_check",
        "robot_motion_commanded": False,
        "board_inner_corners": args.board,
        "square_size_m": args.square_m,
        "image_path": str(args.image.resolve()),
        "camera_path": str(args.camera.resolve()),
        "corner_detection_passed": bool(found),
        "note": "single-frame PnP residual is a sanity check, not independent multi-view calibration proof",
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    annotated = image.copy()
    if found:
        object_points = np.zeros((rows * cols, 3), dtype=np.float32)
        object_points[:, :2] = (
            np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square_m
        )
        solved, rvec, tvec = cv2.solvePnP(
            object_points,
            corners.reshape(-1, 2),
            matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, matrix, distortion
        )
        errors = np.linalg.norm(
            projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1
        )
        mean_px = float(np.mean(errors))
        rmse_px = float(np.sqrt(np.mean(errors**2)))
        max_px = float(np.max(errors))
        passed = bool(solved and mean_px <= 1.5 and max_px <= 4.0)
        payload.update(
            solve_pnp_passed=bool(solved),
            corner_count=int(len(corners)),
            mean_reprojection_error_px=mean_px,
            rmse_reprojection_error_px=rmse_px,
            maximum_reprojection_error_px=max_px,
            board_translation_camera_m=[float(item) for item in tvec.reshape(-1)],
            passed=passed,
        )
        cv2.drawChessboardCorners(annotated, (cols, rows), corners, True)
    else:
        payload.update(solve_pnp_passed=False, passed=False)

    cv2.imwrite(str(args.output_dir / "detected_corners.jpg"), annotated)
    undistorted = cv2.undistort(image, matrix, distortion)
    cv2.imwrite(str(args.output_dir / "undistorted.jpg"), undistorted)
    (args.output_dir / "result.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if payload["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
