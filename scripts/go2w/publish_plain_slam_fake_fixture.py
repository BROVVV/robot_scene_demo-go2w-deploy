#!/usr/bin/env python3
# Copyright 2026 robot_scene_demo maintainers

"""Offline fake ROS fixture for the plain_slam bridge smoke test.

Publishes synthetic but realistic data so the *bridge* pipeline can be
validated without a robot or LiDAR:

    /hesai/pandarxt16/points_raw  (Pandar-like cloud + per-point timestamps)
    /utlidar/imu                  (100 Hz synthetic IMU, gravity in +z)
    /go2w/slam/imu_odom_raw       (slow static-ish trajectory in pslam_imu)
    /go2w/slam/aligned_scan       (world-frame scan, same geometry)

Expected downstream behaviour (no LIO needed):
  pandar_slam_adapter -> /go2w/slam/pandar_points (24-byte schema)
  plain_slam_odom_adapter -> /go2w/slam/odom_base (pslam_odom)
  pointcloud_to_occupancy -> /go2w/slam/map_2d (free/occupied/unknown)
  plain_slam_health_monitor -> /go2w/slam/health + /go2w/slam/ready

Usage:  python3 scripts/go2w/publish_plain_slam_fake_fixture.py
"""

from __future__ import annotations

import argparse
import math
import struct
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header

HEADER_S = time.time()


def build_cloud(frame_id: str, stamp: rclpy.time.Time, world: bool = False) -> PointCloud2:
    """Synthetic 16-ring hemispherical scan with a wall at x = 3 m."""
    n_az = 120
    n_rings = 16
    points = []
    for ring in range(n_rings):
        elevation = math.radians(-12.0 + 24.0 * ring / (n_rings - 1))
        for az_i in range(n_az):
            azimuth = 2.0 * math.pi * az_i / n_az
            # Ground + free space out to ~2.9 m, then a wall slice at 3 m.
            range_m = 2.0 + 0.7 * ((ring + az_i) % 5) / 4.0
            if abs(azimuth) < 0.35 and ring > n_rings // 2:
                range_m = 3.0  # wall
            x = range_m * math.cos(elevation) * math.cos(azimuth)
            y = range_m * math.cos(elevation) * math.sin(azimuth)
            z = range_m * math.sin(elevation)
            if world:
                # world frame: same geometry around the origin
                points.append((x, y, z))
            else:
                points.append((x, y, z))
    msg = PointCloud2()
    msg.header = Header(frame_id=frame_id, stamp=stamp.to_msg())
    msg.height = 1
    msg.width = len(points)
    msg.fields = []
    for name, offset, datatype in (
        ("x", 0, PointField.FLOAT32),
        ("y", 4, PointField.FLOAT32),
        ("z", 8, PointField.FLOAT32),
        ("intensity", 12, PointField.FLOAT32),
        ("timestamp", 16, PointField.FLOAT64),
        ("ring", 24, PointField.UINT16),
    ):
        field = PointField(name=name, offset=offset, datatype=datatype, count=1)
        msg.fields.append(field)
    msg.point_step = 26
    msg.row_step = msg.point_step * msg.width
    msg.is_bigendian = False
    msg.is_dense = True
    msg.data = bytearray(msg.row_step)
    stamp_sec = float(stamp.nanoseconds) * 1e-9
    # struct.pack_into keeps this compatible with every numpy/rclpy version
    # (no numpy dependency at all).
    for i, (x, y, z) in enumerate(points):
        base = i * msg.point_step
        struct.pack_into("<fff", msg.data, base, x, y, z)
        struct.pack_into("<f", msg.data, base + 12, 0.5)
        struct.pack_into("<d", msg.data, base + 16, stamp_sec + 0.01 * i / max(1, len(points)))
        struct.pack_into("<H", msg.data, base + 24, 0)
    return msg


def build_imu(stamp: rclpy.time.Time) -> Imu:
    msg = Imu()
    msg.header = Header(frame_id="pslam_imu", stamp=stamp)
    msg.linear_acceleration.x = 0.0
    msg.linear_acceleration.y = 0.0
    msg.linear_acceleration.z = 9.81
    msg.angular_velocity.x = 0.0
    msg.angular_velocity.y = 0.0
    msg.angular_velocity.z = 0.0
    msg.orientation.w = 1.0
    return msg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hz", type=float, default=10.0, help="cloud rate (Hz)")
    parser.add_argument("--seconds", type=float, default=60.0, help="run duration")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("plain_slam_fake_fixture")
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    cloud_pub = node.create_publisher(PointCloud2, "/hesai/pandarxt16/points_raw", sensor_qos)
    scan_pub = node.create_publisher(
        PointCloud2, "/go2w/slam/aligned_scan", sensor_qos)
    imu_pub = node.create_publisher(Imu, "/utlidar/imu", sensor_qos)
    odom_pub = node.create_publisher(Odometry, "/go2w/slam/imu_odom_raw", sensor_qos)

    node.get_logger().info(
        "publishing fake fixture for 60s (cloud 10 Hz, imu 100 Hz) — "
        "this moves NOTHING"
    )

    period = 1.0 / args.hz
    start = time.monotonic()
    imu_period = 0.01
    next_imu_at = time.monotonic()
    pose_yaw = 0.0

    while rclpy.ok() and time.monotonic() - start < args.seconds:
        stamp = node.get_clock().now().to_msg()
        # Slow, bounded motion so odometry stays finite and monotone.
        pose_yaw = 0.05 * math.sin(time.monotonic() - start)
        cloud = build_cloud(
            "pandarxt16_link_unvalidated", rclpy.time.Time.from_msg(stamp))
        cloud_pub.publish(cloud)

        scan = build_cloud("pslam_odom", rclpy.time.Time.from_msg(stamp), world=True)
        scan_pub.publish(scan)

        odom = Odometry()
        odom.header = Header(frame_id="pslam_imu", stamp=stamp)
        odom.child_frame_id = "pslam_imu"
        odom.pose.pose.position.x = 0.02 * math.sin(time.monotonic() - start)
        odom.pose.pose.position.y = 0.01 * math.cos(time.monotonic() - start)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(pose_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(pose_yaw / 2.0)
        odom_pub.publish(odom)

        # IMU paced at a steady 100 Hz so the health monitor sees a sane rate.
        while time.monotonic() >= next_imu_at:
            imu_stamp = node.get_clock().now().to_msg()
            imu_pub.publish(build_imu(imu_stamp))
            next_imu_at += imu_period
        if next_imu_at < time.monotonic():
            next_imu_at = time.monotonic() + imu_period

        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    node.get_logger().info("fake fixture finished")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
