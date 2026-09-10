#!/usr/bin/env python3
"""GO2-W 里程计桥：/lf/sportmodestate（主控融合里程）→ /go2w/odom/fused。

GO2-W（轮腿）不发布 /lf/lowstate，wheel_odom.py 无输入可用；主控在
SportModeState.position/velocity 里发布融合里程。本节点将其转发为
nav_msgs/Odometry，保持探索栈的 odom 契约（frame: odom_fused →
base_link，单发布者）。
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import SportModeState


class SportOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("sport_odom_bridge")
        self._odom_pub = self.create_publisher(Odometry, "/go2w/odom/fused", 10)
        # 主控 sportmodestate 用 best_effort 发布（ros2 topic hz 默认 sensor QoS
        # 可收到、reliable 收不到）——用 sensor data QoS 订阅。
        self.create_subscription(
            SportModeState, "/lf/sportmodestate", self._on_sport,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "sport_odom_bridge: /lf/sportmodestate -> /go2w/odom/fused"
        )

    def _on_sport(self, msg: SportModeState) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom_fused"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])
        # The ROS message stores attitude in the nested IMUState.  There is
        # no top-level ``euler`` field in the Unitree SportModeState message;
        # using one makes the bridge die on its first real callback and leaves
        # the whole navigation stack without odometry.
        roll = float(msg.imu_state.rpy[0])
        pitch = float(msg.imu_state.rpy[1])
        yaw = float(msg.imu_state.rpy[2])
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        odom.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        odom.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        odom.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        odom.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        self._odom_pub.publish(odom)


def main() -> None:
    rclpy.init()
    node = SportOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
