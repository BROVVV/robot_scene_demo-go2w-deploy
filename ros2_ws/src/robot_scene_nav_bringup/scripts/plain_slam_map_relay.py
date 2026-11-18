#!/usr/bin/env python3
# Copyright 2026 robot_scene_demo maintainers

"""Level-B Nav2 interface (plan §13): relay /go2w/slam/map_2d -> /map.

When enabled via ``use_plain_slam_map:=true`` on
``robot_scene_nav2.launch.py``, Nav2's static costmap layer can consume the
plain_slam mapping-assist OccupancyGrid as an additional planning input.

DEFAULT IS OFF. The Pandar extrinsic is ``candidate_unconfirmed`` and the
plain_slam odom never takes motion authority, so this relay must NOT be used
as collision/safety authority.
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("plain_slam_map_relay")

    source_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    target_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    publisher = node.create_publisher(OccupancyGrid, "/map", target_qos)

    def on_map(msg: OccupancyGrid) -> None:
        publisher.publish(msg)

    node.create_subscription(
        OccupancyGrid, "/go2w/slam/map_2d", on_map, source_qos
    )
    node.get_logger().warn(
        "plain_slam map relay ENABLED: /go2w/slam/map_2d -> /map. "
        "MAPPING_ASSIST ONLY: this map must not act as collision authority "
        "(pandar extrinsic candidate_unconfirmed)."
    )
    rclpy.spin(node)
    publisher.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
