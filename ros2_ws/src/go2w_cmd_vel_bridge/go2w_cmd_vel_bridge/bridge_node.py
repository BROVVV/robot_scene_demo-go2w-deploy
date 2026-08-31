from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from go2w_motion_interfaces.action import MotionCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .bridge_core import Limits, SafetyState, Velocity, decide_velocity


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2w_cmd_vel_bridge")
        self.declare_parameter("execution_enabled", False)
        self.declare_parameter("action_name", "/go2w/motion")
        self.declare_parameter("action_slice_duration_sec", 0.25)
        self.declare_parameter("maximum_linear_x_mps", 0.15)
        self.declare_parameter("maximum_angular_z_radps", 0.20)
        self.declare_parameter("maximum_linear_acceleration_mps2", 0.20)
        self.declare_parameter("maximum_angular_acceleration_radps2", 0.40)
        self.declare_parameter("watchdog_sec", 0.30)
        self._limits = Limits(
            maximum_linear_x=float(self.get_parameter("maximum_linear_x_mps").value),
            maximum_angular_z=float(self.get_parameter("maximum_angular_z_radps").value),
            maximum_linear_acceleration=float(
                self.get_parameter("maximum_linear_acceleration_mps2").value
            ),
            maximum_angular_acceleration=float(
                self.get_parameter("maximum_angular_acceleration_radps2").value
            ),
            watchdog_seconds=float(self.get_parameter("watchdog_sec").value),
        )
        self._requested = Velocity()
        self._previous = Velocity()
        self._command_time = None
        self._source = "unknown"
        self._last_tick = time.monotonic()
        self._operator_armed = False
        self._lease_alive = False
        self._lidar_fresh = False
        self._rotation_clearance_valid = False
        self._lio_fresh = False
        self._robot_error_zero = False
        self._emergency = True
        self._remote_override = True
        self._goal_handle = None
        self._goal_pending = False
        self._cancel_pending = False
        self._status_pub = self.create_publisher(String, "/go2w/cmd_vel_bridge/status", 10)
        self.create_subscription(Twist, "/go2w/cmd_vel_selected", self._command, 10)
        self.create_subscription(String, "/go2w/control_source", self._source_callback, 10)
        for topic, attribute in (
            ("/go2w/software_arm", "_operator_armed"),
            ("/go2w/sport_lease/alive", "_lease_alive"),
            ("/go2w/safety/lidar_fresh", "_lidar_fresh"),
            (
                "/go2w/safety/rotation_clearance_valid",
                "_rotation_clearance_valid",
            ),
            ("/lio/valid", "_lio_fresh"),
            ("/go2w/robot_error_zero", "_robot_error_zero"),
            ("/go2w/emergency_stop", "_emergency"),
            ("/go2w/remote_override", "_remote_override"),
        ):
            self.create_subscription(
                Bool, topic, lambda message, attribute=attribute: setattr(self, attribute, bool(message.data)), 10
            )
        self._execution_enabled = bool(self.get_parameter("execution_enabled").value)
        self._client = (
            ActionClient(self, MotionCommand, str(self.get_parameter("action_name").value))
            if self._execution_enabled
            else None
        )
        self.create_timer(0.05, self._tick)

    def _command(self, message: Twist) -> None:
        self._requested = Velocity(float(message.linear.x), float(message.angular.z))
        self._command_time = time.monotonic()

    def _source_callback(self, message: String) -> None:
        self._source = message.data

    def _tick(self) -> None:
        now = time.monotonic()
        age = float("inf") if self._command_time is None else now - self._command_time
        decision = decide_velocity(
            self._requested,
            self._previous,
            command_age_seconds=age,
            dt_seconds=now - self._last_tick,
            source=self._source,
            safety=SafetyState(
                execution_enabled=self._execution_enabled,
                operator_armed=self._operator_armed,
                lease_alive=self._lease_alive,
                lidar_fresh=self._lidar_fresh,
                rotation_clearance_valid=self._rotation_clearance_valid,
                lio_fresh=self._lio_fresh,
                robot_error_zero=self._robot_error_zero,
                emergency_stop=self._emergency,
                remote_override=self._remote_override,
            ),
            limits=self._limits,
        )
        self._last_tick = now
        self._previous = decision.velocity if decision.allowed else Velocity()
        self._status_pub.publish(String(data=decision.reason))
        if decision.cancel_active_action:
            self._cancel_active()
        elif decision.allowed and not decision.velocity.is_zero:
            self._dispatch(decision.velocity)

    def _dispatch(self, velocity: Velocity) -> None:
        if self._client is None or self._goal_pending or self._goal_handle is not None:
            return
        if not self._client.server_is_ready():
            self._status_pub.publish(String(data="leased_action_server_unavailable"))
            return
        goal = MotionCommand.Goal()
        goal.mode = MotionCommand.Goal.MODE_TIMED_VELOCITY
        goal.vx = velocity.linear_x
        goal.vy = 0.0
        goal.yaw_rate = velocity.angular_z
        goal.duration_sec = float(self.get_parameter("action_slice_duration_sec").value)
        goal.timeout_sec = max(1.0, goal.duration_sec + 0.5)
        self._goal_pending = True
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        self._goal_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self._status_pub.publish(String(data="leased_action_goal_rejected"))
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, _future) -> None:
        self._goal_handle = None
        self._cancel_pending = False

    def _cancel_active(self) -> None:
        if self._goal_handle is None or self._cancel_pending:
            return
        self._cancel_pending = True
        self._goal_handle.cancel_goal_async()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
