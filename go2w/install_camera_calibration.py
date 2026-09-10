#!/usr/bin/env python3
"""Validate and atomically install an operator-produced ROS calibration."""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import re
import tempfile
from pathlib import Path

import yaml


def matrix(data: dict, key: str, count: int) -> list[float]:
    value = data.get(key, {})
    raw = value.get("data", []) if isinstance(value, dict) else value
    numbers = [float(item) for item in raw]
    if len(numbers) != count:
        raise ValueError(f"{key} must have {count} values")
    return numbers


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--board", required=True, help="inner corners, e.g. 8x6")
    parser.add_argument("--square-m", required=True, type=float)
    args = parser.parse_args()
    if not args.operator.strip():
        raise SystemExit("operator must not be empty")
    if re.fullmatch(r"[1-9][0-9]*x[1-9][0-9]*", args.board) is None:
        raise SystemExit("board must be inner-corner dimensions such as 8x6")
    if args.square_m <= 0.0:
        raise SystemExit("square-m must be positive")

    payload = yaml.safe_load(args.input.read_text(encoding="utf-8")) or {}
    width = int(payload.get("image_width", 0))
    height = int(payload.get("image_height", 0))
    k = matrix(payload, "camera_matrix", 9)
    d = payload.get("distortion_coefficients", {})
    d = d.get("data", []) if isinstance(d, dict) else d
    p = matrix(payload, "projection_matrix", 12)
    matrix(payload, "rectification_matrix", 9)
    numeric_values = k + [float(item) for item in d] + p
    if width <= 0 or height <= 0 or not d or not all(
        math.isfinite(item) for item in numeric_values
    ):
        raise SystemExit("calibration has invalid resolution or coefficients")
    if k[0] <= 0.0 or k[4] <= 0.0 or p[0] <= 0.0 or p[5] <= 0.0:
        raise SystemExit("calibration focal lengths are not positive")

    payload.update(
        calibration_status="calibrated",
        calibration_source="ros2_camera_calibration_operator_measurement",
        operator=args.operator.strip(),
        calibrated_at=dt.datetime.now(dt.timezone.utc).isoformat(),
        board_inner_corners=args.board,
        square_size_m=float(args.square_m),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=f".{args.output.name}.", dir=args.output.parent
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(payload, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, args.output)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
