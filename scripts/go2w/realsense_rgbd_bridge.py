#!/usr/bin/env python3
"""Publish D435 HTTP atomic RGB-D frames as ROS2 topics for RTAB-Map.

Topics:
  /go2w/d435/color/image_raw        sensor_msgs/Image (bgr8)
  /go2w/d435/depth/image_rect_raw   sensor_msgs/Image (32FC1, meters)
  /go2w/d435/color/camera_info      sensor_msgs/CameraInfo
  /go2w/d435/rgbd_health            std_msgs/String

Run under the ROS2 system Python (Humble):
  /usr/bin/python3 scripts/go2w/realsense_rgbd_bridge.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource


class D435RGBDBridge(Node):
    def __init__(self, base_url: str, rate_hz: float = 10.0) -> None:
        super().__init__("go2w_d435_rgbd_bridge")
        self.source = RealSenseHTTPRGBDSource(
            base_url,
            cache_dir=str(PROJECT_ROOT / "runtime/go2w/rgbd_bridge_cache"),
        )
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, "/go2w/d435/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/go2w/d435/depth/image_rect_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/go2w/d435/color/camera_info", 10)
        # The WebUI and the live ROS worker use the canonical front-camera
        # topics. Mirror the D435 color stream there as a read-only alias so
        # the HTTP RGB-D deployment and the existing UI share one source.
        self.front_color_pub = self.create_publisher(Image, "/camera/front/image_raw", 10)
        self.front_compressed_pub = self.create_publisher(
            CompressedImage, "/camera/front/image_raw/compressed", 10
        )
        self.front_info_pub = self.create_publisher(CameraInfo, "/camera/front/camera_info", 10)
        self.health_pub = self.create_publisher(String, "/go2w/d435/rgbd_health", 10)
        self.rate_hz = max(1.0, float(rate_hz))
        self._last_frame_id: str | None = None

    def spin_once(self) -> None:
        try:
            frame = self.source.get_latest(timeout_seconds=2.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"RGB-D unavailable: {exc}")
            self.health_pub.publish(String(data=f"error: {exc}"))
            return
        if frame.frame_id == self._last_frame_id:
            return
        self._last_frame_id = frame.frame_id

        color = cv2.imread(frame.color_ref, cv2.IMREAD_COLOR)
        depth_mm = cv2.imread(frame.depth_ref, cv2.IMREAD_UNCHANGED)
        if color is None or depth_mm is None:
            self.get_logger().warn(f"frame {frame.frame_id} image read failed")
            return
        depth_m = depth_mm.astype(np.float32) * float(frame.depth_unit_m or 0.001)

        stamp = self.get_clock().now().to_msg()
        color_msg = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = "d435_color_optical_frame"
        depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "d435_depth_optical_frame"

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "d435_color_optical_frame"
        info.width = frame.width
        info.height = frame.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [frame.fx, 0.0, frame.cx, 0.0, frame.fy, frame.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [frame.fx, 0.0, frame.cx, 0.0, 0.0, frame.fy, frame.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)
        self.front_color_pub.publish(color_msg)
        compressed = CompressedImage()
        compressed.header = color_msg.header
        compressed.format = "jpeg"
        ok, encoded = cv2.imencode(".jpg", color)
        if ok:
            compressed.data = encoded.tobytes()
            self.front_compressed_pub.publish(compressed)
        self.front_info_pub.publish(info)
        self.health_pub.publish(String(data=f"frame={frame.frame_id} age={frame.health.get('age_s', 0.0)}"))
        self.get_logger().info(
            f"published D435 frame {frame.frame_id} color={frame.width}x{frame.height}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://192.168.123.18:8080")
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = D435RGBDBridge(args.base_url, args.rate)
    try:
        while rclpy.ok():
            node.spin_once()
            time.sleep(1.0 / node.rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
