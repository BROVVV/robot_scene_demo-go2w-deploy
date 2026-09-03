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


# 计划书 §11.1：host_timestamp 合法性窗口（Unix 秒，2014~2038 之外视为无效）。
_HOST_TIMESTAMP_MIN_S = 1.4e9
_HOST_TIMESTAMP_MAX_S = 2.2e9


def _capture_stamp_seconds(frame) -> tuple[float, str]:
    """优先使用服务端采集时间 host_timestamp；非法/缺失才回退接收时间。

    返回 (stamp_seconds, timestamp_quality)。
    """
    host_ts = frame.host_timestamp
    try:
        host_value = float(host_ts)
    except (TypeError, ValueError):
        host_value = 0.0
    if (
        host_value >= _HOST_TIMESTAMP_MIN_S
        and host_value <= _HOST_TIMESTAMP_MAX_S
    ):
        return host_value, "host_timestamp"
    return time.time(), "receive_time"


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

        # 计划书 §11.1：用真实采集时间（host_timestamp）打 ROS 时间戳；
        # 不可用时回退接收时间并显式标记 timestamp_quality。
        stamp_seconds, timestamp_quality = _capture_stamp_seconds(frame)
        stamp = self.get_clock().now().to_msg()
        stamp.sec = int(stamp_seconds)
        stamp.nanosec = int(round((stamp_seconds - int(stamp_seconds)) * 1e9))
        if stamp.nanosec >= 1_000_000_000:
            stamp.sec += 1
            stamp.nanosec -= 1_000_000_000
        color_msg = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = "d435_color_optical_frame"
        depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
        depth_msg.header.stamp = stamp
        # 计划书 §11.2：aligned depth 实际处于 color camera geometry，因此
        # frame_id 使用 color optical frame（depth↔color 无独立 TF 时不能
        # 假装是 depth optical frame）。
        depth_msg.header.frame_id = (
            "d435_color_optical_frame"
            if frame.depth_aligned_to_color
            else "d435_depth_optical_frame"
        )

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
        self.health_pub.publish(
            String(
                data=(
                    f"frame={frame.frame_id} age={frame.health.get('age_s', 0.0)} "
                    f"timestamp_quality={timestamp_quality} "
                    f"depth_aligned_to_color={frame.depth_aligned_to_color}"
                )
            )
        )
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
