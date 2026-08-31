from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .arbiter_core import CommandSample, Twist2D, select_command


class ControlArbiter(Node):
    def __init__(self) -> None:
        super().__init__("go2w_control_arbiter")
        self.declare_parameter("software_control_armed", False)
        self.declare_parameter("remote_override_verified", False)
        self.declare_parameter("command_timeout_sec", 0.3)
        self._commands = {}
        self._emergency = False
        self._remote_override = True
        self._remote_seen = False
        self._selected_pub = self.create_publisher(Twist, "/go2w/cmd_vel_selected", 10)
        self._source_pub = self.create_publisher(String, "/go2w/control_source", 10)
        self._status_pub = self.create_publisher(String, "/go2w/control_status", 10)
        for topic, source in (
            ("/go2w/manual_cmd_vel", "manual"),
            ("/go2w/nav2_cmd_vel", "nav2"),
            ("/go2w/search_cmd_vel", "search"),
        ):
            self.create_subscription(
                Twist, topic, lambda message, source=source: self._command(source, message), 10
            )
        self.create_subscription(Bool, "/go2w/emergency_stop", self._emergency_callback, 10)
        self.create_subscription(Bool, "/go2w/remote_override", self._remote_callback, 10)
        self.create_timer(0.02, self._select)

    def _command(self, source: str, message: Twist) -> None:
        self._commands[source] = CommandSample(
            Twist2D(message.linear.x, message.linear.y, message.angular.z), time.monotonic()
        )

    def _emergency_callback(self, message: Bool) -> None:
        self._emergency = self._emergency or bool(message.data)

    def _remote_callback(self, message: Bool) -> None:
        self._remote_seen = True
        self._remote_override = bool(message.data)

    def _select(self) -> None:
        remote_verified = bool(self.get_parameter("remote_override_verified").value)
        remote_override = self._remote_override if remote_verified and self._remote_seen else True
        selected = select_command(
            self._commands,
            now=time.monotonic(),
            timeout=float(self.get_parameter("command_timeout_sec").value),
            software_armed=bool(self.get_parameter("software_control_armed").value),
            emergency_stop=self._emergency,
            remote_override=remote_override,
        )
        message = Twist()
        message.linear.x = selected.command.linear_x
        message.linear.y = 0.0
        message.angular.z = selected.command.angular_z
        self._selected_pub.publish(message)
        self._source_pub.publish(String(data=selected.source))
        self._status_pub.publish(String(data=selected.blocked_reason or "selected"))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
