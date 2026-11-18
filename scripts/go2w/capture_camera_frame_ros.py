#!/usr/bin/env python3
"""Capture one read-only ROS camera frame for calibration-target inspection."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FrameCapture(Node):
    def __init__(self) -> None:
        super().__init__("go2w_camera_frame_capture")
        self.bridge = CvBridge()
        self.image = None
        self.header = None
        self.create_subscription(
            Image,
            "/camera/front/image_raw",
            self._receive,
            qos_profile_sensor_data,
        )

    def _receive(self, message: Image) -> None:
        if self.image is None:
            self.image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            self.header = message.header


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = FrameCapture()
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while rclpy.ok() and node.image is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.image is None:
            payload = {
                "passed": False,
                "robot_motion_commanded": False,
                "error": "camera frame timeout",
            }
            return_code = 2
        else:
            args.image.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.image), node.image):
                raise RuntimeError(f"failed to write {args.image}")
            payload = {
                "passed": True,
                "robot_motion_commanded": False,
                "image_path": str(args.image.resolve()),
                "width": int(node.image.shape[1]),
                "height": int(node.image.shape[0]),
                "frame_id": node.header.frame_id,
                "stamp": {
                    "sec": int(node.header.stamp.sec),
                    "nanosec": int(node.header.stamp.nanosec),
                },
            }
            return_code = 0
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
        return return_code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
