#!/usr/bin/env python3
"""Interactive, operator-confirmed collection of physical Go2-W measurements."""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import tempfile
from pathlib import Path

import yaml


LENGTHS = (
    ("body_max_length_m", "机身最大长度 (m)"),
    ("body_max_width_m", "机身最大宽度 (m)"),
    ("stationary_standing_height_m", "静止站立最大高度 (m)"),
    ("wheel_outer_envelope_length_m", "轮胎外缘总长度 (m)"),
    ("wheel_outer_envelope_width_m", "轮胎外缘总宽度 (m)"),
    ("base_link_ground_height_m", "base_link 原点离地高度 (m)"),
)
POSES = tuple(
    (f"{sensor}_{axis}_{unit}", f"{sensor} 相对 base_link 的 {axis} ({unit})")
    for sensor in ("front_camera", "lidar", "lidar_imu")
    for axis, unit in (
        ("x", "m"),
        ("y", "m"),
        ("z", "m"),
        ("roll", "rad"),
        ("pitch", "rad"),
        ("yaw", "rad"),
    )
)


def ask_finite(label: str, *, positive: bool = False) -> float:
    while True:
        try:
            value = float(input(f"{label}: ").strip())
            if not math.isfinite(value) or (positive and value <= 0.0):
                raise ValueError
            return value
        except ValueError:
            print("请输入有限数值；长度必须大于 0。")


def atomic_write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(payload, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--method", required=True)
    parser.add_argument(
        "--confirm-go2w",
        action="store_true",
        help="Confirm the measured robot is physically a Unitree Go2-W",
    )
    args = parser.parse_args()
    if not args.confirm_go2w:
        raise SystemExit("refusing to record: pass --confirm-go2w after checking the model")
    timestamp = dt.datetime.now(dt.timezone.utc).isoformat()
    records = {}
    for key, label in LENGTHS + POSES:
        value = ask_finite(label, positive=key in {item[0] for item in LENGTHS})
        uncertainty = ask_finite(f"{label} 的绝对不确定度", positive=True)
        records[key] = {
            "value": value,
            "measurement_method": args.method,
            "operator": args.operator,
            "timestamp": timestamp,
            "uncertainty": uncertainty,
        }
    confirmation = input("确认以上数值均为现场实测？请输入 MEASURED: ").strip()
    if confirmation != "MEASURED":
        raise SystemExit("operator did not confirm; no file written")
    atomic_write(
        args.output,
        {
            "schema_version": 1,
            "robot_model": "Unitree Go2-W",
            "measurement_status": "measured",
            "confirmed": True,
            "notes": "Operator-confirmed physical measurements; angles use REP-103 RPY.",
            "measurements": records,
        },
    )
    print(f"wrote confirmed measurements to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
