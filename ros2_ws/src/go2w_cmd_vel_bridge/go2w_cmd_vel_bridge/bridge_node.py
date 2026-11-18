from __future__ import annotations

import time
import json
import math
import uuid

import rclpy
from geometry_msgs.msg import Twist
from go2w_motion_interfaces.action import MotionCommand
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .bridge_core import BridgeDecision, Limits, SafetyState, Velocity, decide_velocity, geometry_execution_allowed
from .trial_guard import TrialVelocityGuard
from .goal_travel_guard import GoalTravelGuard

SCAN_RECEIVE_TIMEOUT_SECONDS = .25
SCAN_STAMP_TIMEOUT_SECONDS = .32


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2w_cmd_vel_bridge")
        self.declare_parameter("execution_enabled", False)
        self.declare_parameter("action_name", "/go2w/motion")
        self.declare_parameter("action_slice_duration_sec", 0.25)
        self.declare_parameter("maximum_linear_x_mps", 0.30)
        self.declare_parameter("maximum_angular_z_radps", 0.20)
        self.declare_parameter("maximum_linear_acceleration_mps2", 0.20)
        self.declare_parameter("maximum_angular_acceleration_radps2", 0.40)
        self.declare_parameter("watchdog_sec", 0.30)
        # 操作员在场的转向放行（2026-10-10 用户批准）。**刻意做成启动参数而不是
        # 话题订阅**：话题任何进程都能发，参数只有启动这条命令的人能给，于是
        # "谁开的谁负责"是结构性成立的，不靠约定。默认 False ⇒ 不传时行为不变。
        self.declare_parameter("operator_supervised_rotation", False)
        self.declare_parameter("trial_ledger", "")
        self.declare_parameter("geometry_execution_scope", "")
        self.declare_parameter("geometry_profile_sha256", "")
        trial_ledger = str(self.get_parameter("trial_ledger").value)
        self._trial_guard = TrialVelocityGuard(trial_ledger) if trial_ledger else None
        self._goal_travel_guard = None if trial_ledger else GoalTravelGuard()
        self._limits = Limits(
            maximum_linear_x=min(self._trial_guard.maximum_speed_mps if trial_ledger else .30,
                                 float(self.get_parameter("maximum_linear_x_mps").value)),
            maximum_angular_z=min(.10 if trial_ledger else float('inf'),
                                  float(self.get_parameter("maximum_angular_z_radps").value)),
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
        # 计划书 §6.5：Nav2 执行的定位新鲜度改看坐标桥发布的
        # /go2w/nav/localization_valid，不再依赖 /lio/valid。
        self._navigation_localization_fresh = False
        self._robot_error_zero = False
        self._emergency = True
        self._remote_override = True
        self._goal_handle = None
        self._safety_received = {}
        self._dispatch_permitted = False
        self._stream_token = None
        self._motion_fault = ''
        self._stream_pub = self.create_publisher(String, '/go2w/motion/velocity_update',
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self._scan_stamp = 0.0
        self._localization_stamp = 0.0
        self._dated_localization_valid = False
        self._raw_nav_received = None
        self._geometry_scope='';self._geometry_wall_time=0.;self._geometry_sha256=''
        self._expected_geometry_scope=str(self.get_parameter('geometry_execution_scope').value)
        self._expected_geometry_sha256=str(self.get_parameter('geometry_profile_sha256').value)
        self._goal_pending = False
        self._cancel_pending = False
        self._status_pub = self.create_publisher(String, "/go2w/cmd_vel_bridge/status", 10)
        self._travel_status_pub = self.create_publisher(String, "/go2w/nav/goal_travel/status", 1)
        if self._goal_travel_guard is not None:
            self.create_subscription(String, "/go2w/nav/goal_travel/request", self._travel_request, 10)
        self.create_subscription(Twist, "/go2w/cmd_vel_selected", self._command, 10)
        self.create_subscription(String, "/go2w/control_source", self._source_callback, 10)
        self.create_subscription(LaserScan, "/go2w/lidar/scan", self._scan_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, "/go2w/nav/localization_status", self._localization_callback, 1)
        self.create_subscription(Twist, "/cmd_vel_nav_raw", self._raw_nav_callback, 1)
        self.create_subscription(String,'/go2w/lidar/geometry_status',self._geometry_status,1)
        self.create_subscription(Odometry, "/go2w/odom/fused", self._trial_odometry,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        for topic, attribute in (
            ("/go2w/software_arm", "_operator_armed"),
            ("/go2w/sport_lease/alive", "_lease_alive"),
            ("/go2w/safety/lidar_fresh", "_lidar_fresh"),
            (
                "/go2w/safety/rotation_clearance_valid",
                "_rotation_clearance_valid",
            ),
            ("/go2w/nav/localization_valid", "_navigation_localization_fresh"),
            ("/go2w/robot_error_zero", "_robot_error_zero"),
            ("/go2w/emergency_stop", "_emergency"),
            ("/go2w/remote_override", "_remote_override"),
        ):
            self.create_subscription(
                Bool, topic, lambda message, attribute=attribute: self._safety_update(attribute, message), 10
            )
        self._execution_enabled = bool(self.get_parameter("execution_enabled").value)
        self._operator_supervised_rotation = bool(
            self.get_parameter("operator_supervised_rotation").value
        )
        self._client = (
            ActionClient(self, MotionCommand, str(self.get_parameter("action_name").value))
            if self._execution_enabled
            else None
        )
        # These values select the physical execution topology and are read to
        # construct guards/clients above.  ROS parameters are mutable by
        # default, which previously let ``ros2 param set`` report a new value
        # while the bridge kept using the old in-memory value.  Runtime may
        # only tighten execution; opening it or rebinding a guard requires an
        # explicit process restart with the complete launch-time contract.
        self.add_on_set_parameters_callback(self._parameters_update)
        self.create_timer(0.05, self._tick)

    def _parameters_update(self, parameters):
        startup_only = {
            "trial_ledger",
            "geometry_execution_scope",
            "geometry_profile_sha256",
            "operator_supervised_rotation",
        }
        for parameter in parameters:
            if parameter.name in startup_only:
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} is startup-only; restart the bridge",
                )
            if (
                parameter.name == "execution_enabled"
                and bool(parameter.value)
                and not self._execution_enabled
            ):
                return SetParametersResult(
                    successful=False,
                    reason="runtime execution enable is forbidden; restart the bridge explicitly",
                )

        disable_execution = any(
            parameter.name == "execution_enabled" and not bool(parameter.value)
            for parameter in parameters
        )
        if disable_execution:
            # Publish an immediate zero on an active stream before canceling;
            # the motion server watchdog remains the independent fallback.
            if self._stream_token is not None:
                self._publish_stream(Velocity())
            self._execution_enabled = False
            self._requested = Velocity()
            self._previous = Velocity()
            self._dispatch_permitted = False
            self._cancel_active()
        return SetParametersResult(successful=True)

    def _command(self, message: Twist) -> None:
        self._requested = Velocity(float(message.linear.x), float(message.angular.z))
        self._command_time = time.monotonic()

    def _source_callback(self, message: String) -> None:
        self._source = message.data

    def _raw_nav_callback(self, _message):
        self._raw_nav_received = time.monotonic()

    def _geometry_status(self,message):
        try:
            data=json.loads(message.data)
            self._geometry_scope=data['execution_scope'] if data.get('posture_valid') is True else ''
            self._geometry_sha256=data.get('profile_sha256','')
            self._geometry_wall_time=float(data['wall_time'])
        except (ValueError,TypeError,KeyError):
            self._geometry_scope=''

    def _trial_odometry(self, message):
        p,q = message.pose.pose.position,message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        values = dict(x=p.x,y=p.y,yaw=yaw,frame=message.header.frame_id,
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,received=time.monotonic())
        if self._trial_guard is not None:
            self._trial_guard.odometry(**values)
        if self._goal_travel_guard is not None:
            norm = sum(v*v for v in (q.x,q.y,q.z,q.w))
            if not math.isfinite(norm) or abs(norm-1.) > .01:
                values['yaw'] = float('nan')
            self._goal_travel_guard.odometry(**values,child=message.child_frame_id)

    def _travel_request(self, message):
        try:
            data = json.loads(message.data)
        except (ValueError,TypeError):
            data = {}
        self._goal_travel_guard.request(data,
            now=self.get_clock().now().nanoseconds/1e9,received_now=time.monotonic(),
            can_begin=not (self._goal_pending or self._goal_handle is not None))

    def _scan_callback(self, message):
        if message.header.frame_id == "base_link" and len(message.ranges):
            self._scan_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def _localization_callback(self, message):
        try:
            status = json.loads(message.data)
            self._localization_stamp = float(status["wall_time"])
            self._dated_localization_valid = status.get("valid") is True
        except (TypeError, ValueError, KeyError):
            self._dated_localization_valid = False

    def _safety_update(self, attribute, message):
        setattr(self, attribute, bool(message.data))
        self._safety_received[attribute] = time.monotonic()
        if attribute == '_operator_armed' and not message.data:
            self._motion_fault = ''

    def _live_flag(self, attribute, now, *, timeout=1.0, missing=False):
        received = self._safety_received.get(attribute)
        if received is None or not 0 <= now - received <= timeout:
            return missing
        return getattr(self, attribute)

    def _tick(self) -> None:
        now = time.monotonic()
        scan_age = self.get_clock().now().nanoseconds / 1e9 - self._scan_stamp
        localization_age = time.time() - self._localization_stamp
        age = float("inf") if self._command_time is None else now - self._command_time
        if self._source == "nav2":
            raw_age = float("inf") if self._raw_nav_received is None else now - self._raw_nav_received
            age = max(age, raw_age)
        decision = decide_velocity(
            self._requested,
            self._previous,
            command_age_seconds=age,
            dt_seconds=now - self._last_tick,
            source=self._source,
            safety=SafetyState(
                execution_enabled=self._execution_enabled,
                operator_armed=self._live_flag("_operator_armed", now),
                lease_alive=self._live_flag("_lease_alive", now),
                lidar_fresh=(self._live_flag("_lidar_fresh", now, timeout=.3)
                             and -.1 <= scan_age <= SCAN_STAMP_TIMEOUT_SECONDS),
                rotation_clearance_valid=self._live_flag("_rotation_clearance_valid", now, timeout=.3),
                operator_supervised_rotation=self._operator_supervised_rotation,
                navigation_localization_fresh=self._live_flag("_navigation_localization_fresh", now, timeout=.3)
                and self._dated_localization_valid and -.1 <= localization_age <= .3,
                robot_error_zero=self._live_flag("_robot_error_zero", now),
                emergency_stop=self._live_flag("_emergency", now, missing=True),
                remote_override=self._live_flag("_remote_override", now, missing=True),
            ),
            limits=self._limits,
        )
        if self._trial_guard is not None:
            trial_reason = self._trial_guard.check(
                now=self.get_clock().now().nanoseconds/1e9, received_now=now,
                linear=decision.velocity.linear_x, angular=decision.velocity.angular_z,
                duration=float(self.get_parameter("action_slice_duration_sec").value),
            )
            if trial_reason:
                decision = BridgeDecision(Velocity(), False,
                    decision.reason + ',' + trial_reason, True)
        if self._goal_travel_guard is not None:
            travel_now = self.get_clock().now().nanoseconds/1e9
            travel_reason = self._goal_travel_guard.check(now=travel_now,received_now=now,
                linear=decision.velocity.linear_x,angular=decision.velocity.angular_z,
                duration=float(self.get_parameter("action_slice_duration_sec").value))
            if travel_reason:
                decision = BridgeDecision(Velocity(),False,decision.reason+','+travel_reason,True)
            travel_status = self._goal_travel_guard.status(now=travel_now,received_now=now)
            travel_status['ready'] = (travel_status['ready']
                and not self._goal_pending and self._goal_handle is None)
            self._travel_status_pub.publish(String(data=json.dumps(travel_status,allow_nan=False)))
        geometry_valid=geometry_execution_allowed(scope=self._geometry_scope,
            expected_scope=self._expected_geometry_scope,profile_sha256=self._geometry_sha256,
            expected_sha256=self._expected_geometry_sha256,
            age_seconds=time.time()-self._geometry_wall_time,
            trial_guard=self._trial_guard is not None,travel_guard=self._goal_travel_guard is not None,
            operator_supervised=self._operator_supervised_rotation)
        if self._execution_enabled and not geometry_valid:
            decision=BridgeDecision(Velocity(),False,decision.reason+',geometry_not_valid_for_execution',True)
        if self._motion_fault:
            decision=BridgeDecision(Velocity(),False,decision.reason+','+self._motion_fault,True)
        self._last_tick = now
        self._dispatch_permitted = decision.allowed and not decision.velocity.is_zero
        self._previous = decision.velocity if decision.allowed else Velocity()
        self._status_pub.publish(String(data=decision.reason))
        if decision.cancel_active_action:
            self._cancel_active()
        elif decision.allowed and not decision.velocity.is_zero:
            self._dispatch(decision.velocity)

    def _dispatch(self, velocity: Velocity) -> None:
        if self._client is None:
            return
        if self._stream_token is not None:
            self._publish_stream(velocity)
            return
        if self._goal_pending or self._goal_handle is not None:
            return
        if not self._client.server_is_ready():
            self._status_pub.publish(String(data="leased_action_server_unavailable"))
            return
        goal = MotionCommand.Goal()
        goal.mode = MotionCommand.Goal.MODE_STREAMING_VELOCITY
        goal.vx = velocity.linear_x
        goal.vy = 0.0
        goal.yaw_rate = velocity.angular_z
        goal.duration_sec = float(self.get_parameter("action_slice_duration_sec").value)
        goal.timeout_sec = 30.0
        self._stream_token = uuid.uuid4().hex
        goal.velocity_stream_token = self._stream_token
        self._goal_pending = True
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self._publish_stream(velocity)

    def _publish_stream(self, velocity):
        if self._stream_token is None or self._cancel_pending:
            return
        self._stream_pub.publish(String(data=json.dumps(dict(version=1,
            token=self._stream_token,stamp=self.get_clock().now().nanoseconds/1e9,
            vx=velocity.linear_x,vy=0.,yaw_rate=velocity.angular_z),allow_nan=False)))

    def _goal_response(self, future) -> None:
        self._goal_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self._stream_token = None
            self._motion_fault = 'leased_action_goal_rejected'
            self._status_pub.publish(String(data="leased_action_goal_rejected"))
            return
        self._goal_handle = handle
        if not self._dispatch_permitted:
            self._cancel_active()
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, _future) -> None:
        try:
            result = _future.result().result
            if not result.success and result.error_code != MotionCommand.Result.ERROR_CANCELED:
                self._motion_fault = 'leased_action_failed_' + str(result.error_code)
                self.get_logger().error(self._motion_fault + ': ' + result.message)
        except Exception as exc:
            self._motion_fault = 'leased_action_result_unavailable'
            self.get_logger().error(str(exc))
        self._stream_token = None
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
