#!/usr/bin/env python3
"""Validate synchronized camera bridge outputs without publishing anything."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


class Validator(Node):
    def __init__(self, required_frames: int) -> None:
        super().__init__("go2w_camera_bridge_validator")
        self.required_frames = required_frames
        self.raw = {}
        self.compressed = {}
        self.info = {}
        self.capture_time_trusted_values = []
        self.diagnostics = []
        self.create_subscription(
            Image, "/camera/front/image_raw", self._raw, qos_profile_sensor_data
        )
        self.create_subscription(
            CompressedImage,
            "/camera/front/image_raw/compressed",
            self._compressed,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/front/camera_info",
            self._info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray, "/camera/front/status", self._diagnostic, 10
        )

    def _raw(self, message):
        self.raw[stamp_ns(message)] = message

    def _compressed(self, message):
        self.compressed[stamp_ns(message)] = message

    def _info(self, message):
        self.info[stamp_ns(message)] = message

    def _diagnostic(self, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            if "capture_time_trusted" in values:
                self.capture_time_trusted_values.append(
                    values["capture_time_trusted"].lower()
                )
            self.diagnostics.append(
                {
                    "level": (
                        status.level[0]
                        if isinstance(status.level, (bytes, bytearray))
                        else int(status.level)
                    ),
                    "message": status.message,
                    "error": values.get("error"),
                }
            )

    def matching(self):
        return sorted(set(self.raw) & set(self.compressed) & set(self.info))


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    rclpy.init()
    node = Validator(args.frames)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and len(node.matching()) < args.frames:
            rclpy.spin_once(node, timeout_sec=0.1)
        matches = node.matching()
        errors = []
        samples = []
        for value in matches[: args.frames]:
            raw = node.raw[value]
            compressed = node.compressed[value]
            info = node.info[value]
            if raw.encoding != "bgr8":
                errors.append(f"encoding={raw.encoding}")
            if raw.header.frame_id != "front_camera_optical_frame":
                errors.append(f"frame={raw.header.frame_id}")
            if compressed.format.lower() != "jpeg":
                errors.append(f"compressed_format={compressed.format}")
            if raw.width != info.width or raw.height != info.height:
                errors.append("Image/CameraInfo resolution mismatch")
            expected_size = int(raw.step) * int(raw.height)
            if len(raw.data) != expected_size or raw.step < raw.width * 3:
                errors.append("raw_image_layout_invalid")
                solid_green_fraction = None
            else:
                rows = np.frombuffer(raw.data, dtype=np.uint8).reshape(
                    raw.height, raw.step
                )
                bgr = rows[:, : raw.width * 3].reshape(raw.height, raw.width, 3)
                blue = bgr[:, :, 0].astype(np.int16)
                green = bgr[:, :, 1].astype(np.int16)
                red = bgr[:, :, 2].astype(np.int16)
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
            samples.append(
                {
                    "stamp_ns": value,
                    "width": raw.width,
                    "height": raw.height,
                    "encoding": raw.encoding,
                    "compressed_bytes": len(compressed.data),
                    "camera_k_nonzero": any(abs(item) > 0.0 for item in info.k),
                    "solid_green_fraction": solid_green_fraction,
                }
            )
        if len(matches) < args.frames:
            errors.append(
                f"only {len(matches)} synchronized frames; need {args.frames}"
            )
        if not node.capture_time_trusted_values:
            errors.append("no timestamp diagnostic received")
        elif any(value != "false" for value in node.capture_time_trusted_values):
            errors.append("capture time was incorrectly marked trusted")
        result = {
            "passed": not errors,
            "required_frames": args.frames,
            "matching_frames": len(matches),
            "received_counts": {
                "raw": len(node.raw),
                "compressed": len(node.compressed),
                "camera_info": len(node.info),
            },
            "recent_stamps": {
                "raw": sorted(node.raw)[-5:],
                "compressed": sorted(node.compressed)[-5:],
                "camera_info": sorted(node.info)[-5:],
            },
            "samples": samples,
            "capture_time_trusted_values": node.capture_time_trusted_values,
            "diagnostics": node.diagnostics[-10:],
            "errors": errors,
        }
        atomic_json(args.output, result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if not errors else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
