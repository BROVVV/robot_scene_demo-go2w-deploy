#!/usr/bin/env python3
"""Show the read-only Go2-W camera with chessboard alignment guides."""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class AlignmentViewer(Node):
    def __init__(self, board: tuple[int, int], display_width: int) -> None:
        super().__init__("go2w_camera_alignment_viewer")
        self.board = board
        self.display_width = display_width
        self.bridge = CvBridge()
        self.last_detection_time = 0.0
        self.last_corners = None
        self.last_frame_shape = None
        self.create_subscription(
            Image,
            "/camera/front/image_raw",
            self._image,
            qos_profile_sensor_data,
        )
        cv2.namedWindow("Go2-W Camera - Q or Esc to close", cv2.WINDOW_NORMAL)

    def _detect(self, frame: np.ndarray) -> None:
        now = time.monotonic()
        if now - self.last_detection_time < 0.20:
            return
        self.last_detection_time = now
        scale = min(1.0, 960.0 / frame.shape[1])
        reduced = cv2.resize(frame, None, fx=scale, fy=scale)
        gray = cv2.cvtColor(reduced, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray,
            self.board,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
        )
        self.last_corners = corners / scale if found else None
        self.last_frame_shape = frame.shape[:2]

    def _image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self._detect(frame)
        canvas = frame.copy()
        height, width = canvas.shape[:2]
        guide_color = (0, 220, 255)
        for fraction in (1 / 3, 2 / 3):
            cv2.line(canvas, (int(width * fraction), 0), (int(width * fraction), height), guide_color, 1)
            cv2.line(canvas, (0, int(height * fraction)), (width, int(height * fraction)), guide_color, 1)
        cv2.drawMarker(
            canvas,
            (width // 2, height // 2),
            (0, 0, 255),
            cv2.MARKER_CROSS,
            60,
            3,
        )

        if self.last_corners is not None and self.last_frame_shape == (height, width):
            corners = self.last_corners.astype(np.float32)
            cv2.drawChessboardCorners(canvas, self.board, corners, True)
            center = corners.reshape(-1, 2).mean(axis=0)
            center_px = (int(round(center[0])), int(round(center[1])))
            cv2.drawMarker(canvas, center_px, (0, 255, 0), cv2.MARKER_TILTED_CROSS, 50, 3)
            dx = int(round(center[0] - width / 2))
            dy = int(round(center[1] - height / 2))
            text = f"BOARD {self.board[0]}x{self.board[1]} DETECTED  center offset: x={dx:+d}px y={dy:+d}px"
            color = (0, 255, 0)
        else:
            text = f"BOARD {self.board[0]}x{self.board[1]} NOT DETECTED - keep all corners visible"
            color = (0, 0, 255)
        cv2.rectangle(canvas, (0, 0), (width, 54), (0, 0, 0), -1)
        cv2.putText(canvas, text, (18, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        display_height = int(round(height * self.display_width / width))
        shown = cv2.resize(canvas, (self.display_width, display_height))
        cv2.imshow("Go2-W Camera - Q or Esc to close", shown)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            rclpy.shutdown()

    def close(self) -> None:
        cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", default="9x6")
    parser.add_argument("--display-width", type=int, default=1280)
    args = parser.parse_args()
    cols, rows = (int(item) for item in args.board.lower().split("x", 1))
    rclpy.init()
    node = AlignmentViewer((cols, rows), args.display_width)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
