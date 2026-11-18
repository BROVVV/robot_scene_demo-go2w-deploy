#!/usr/bin/env python3
"""Publish file-based SDK executor health into the ROS 2 control graph."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Bool, String, UInt64


def _read(path: Path, default: str = "") -> str:
    try:
        return path.read_text(encoding="ascii").strip()
    except OSError:
        return default


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--status-dir",
        default=os.environ.get("GO2W_LEASE_STATUS_DIR", "/tmp/go2w_lease_status"),
    )
    parser.add_argument("--heartbeat-timeout", type=float, default=1.5)
    args = parser.parse_args()

    status_dir = Path(args.status_dir)
    id_file = status_dir / "id"
    alive_file = status_dir / "alive"
    heartbeat_file = status_dir / "heartbeat"
    motion_name_file = status_dir / "motion_name"
    robot_form_file = status_dir / "robot_form"

    rclpy.init(args=[])
    node = rclpy.create_node("go2w_sport_lease_bridge")
    latched_qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    id_pub = node.create_publisher(UInt64, "/go2w/sport_lease/id", latched_qos)
    alive_pub = node.create_publisher(Bool, "/go2w/sport_lease/alive", 10)
    name_pub = node.create_publisher(
        String, "/go2w/motion_mode/name", latched_qos
    )
    form_pub = node.create_publisher(
        String, "/go2w/motion_mode/form", latched_qos
    )

    last_report: tuple[int, bool, str, str] | None = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                heartbeat_age = time.time() - heartbeat_file.stat().st_mtime
            except OSError:
                heartbeat_age = float("inf")
            try:
                lease_id = int(_read(id_file, "0"))
            except ValueError:
                lease_id = 0
            alive = (
                _read(alive_file, "0") == "1"
                and lease_id > 0
                and heartbeat_age <= args.heartbeat_timeout
            )
            motion_name = _read(motion_name_file)
            robot_form = _read(robot_form_file)

            id_pub.publish(UInt64(data=max(0, lease_id if alive else 0)))
            alive_pub.publish(Bool(data=alive))
            name_pub.publish(String(data=motion_name))
            form_pub.publish(String(data=robot_form))
            report = (lease_id, alive, motion_name, robot_form)
            if report != last_report:
                print(
                    "lease bridge: "
                    f"id={lease_id} alive={alive} mode={motion_name} "
                    f"form={robot_form}",
                    flush=True,
                )
                last_report = report
            time.sleep(0.15)
    finally:
        for _ in range(3):
            alive_pub.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
