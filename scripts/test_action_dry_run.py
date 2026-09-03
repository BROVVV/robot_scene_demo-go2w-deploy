#!/usr/bin/env python3
"""Offline integration test using synthetic lease and robot state topics."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from go2w_motion_interfaces.action import MotionCommand
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import SetBool, Trigger
from unitree_go.msg import LowState, SportModeState


class Fixture:
    def __init__(self) -> None:
        self.node = rclpy.create_node("go2w_action_dry_run_fixture")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Match the real Go2-W state publishers used by the motion monitor.
        state_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.lease_id_pub = self.node.create_publisher(
            UInt64, "/go2w/sport_lease/id", latched
        )
        self.alive_pub = self.node.create_publisher(
            Bool, "/go2w/sport_lease/alive", 10
        )
        self.name_pub = self.node.create_publisher(
            String, "/go2w/motion_mode/name", latched
        )
        self.sport_pub = self.node.create_publisher(
            SportModeState, "/dry_run/sportmodestate", state_qos
        )
        self.low_pub = self.node.create_publisher(
            LowState, "/dry_run/lowstate", state_qos
        )
        self.action = ActionClient(self.node, MotionCommand, "/go2w/motion")
        self.arm = self.node.create_client(SetBool, "/go2w/arm")
        self.stop = self.node.create_client(Trigger, "/go2w/emergency_stop")
        self.publish_state = True
        self.lease_alive = True
        self.yaw = 0.0
        self.timer = self.node.create_timer(0.02, self.publish)

    def publish(self) -> None:
        lease_id = UInt64()
        lease_id.data = 123456789
        self.lease_id_pub.publish(lease_id)
        alive = Bool()
        alive.data = self.lease_alive
        self.alive_pub.publish(alive)
        name = String()
        name.data = "ai-w"
        self.name_pub.publish(name)
        if not self.publish_state:
            return
        sport = SportModeState()
        sport.error_code = 0
        sport.mode = 1
        sport.velocity = [0.0, 0.0, 0.0]
        sport.yaw_speed = 0.0
        sport.imu_state.rpy[2] = self.yaw
        self.sport_pub.publish(sport)
        low = LowState()
        for index in range(12, 16):
            low.motor_state[index].q = 0.0
            low.motor_state[index].dq = 0.0
        self.low_pub.publish(low)


def wait(future: object, timeout: float = 10.0) -> object:
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    if not future.done():
        raise TimeoutError("future timed out")
    return future.result()


def arm(fixture: Fixture, enabled: bool) -> None:
    deadline = time.monotonic() + 6.0
    response = None
    while time.monotonic() < deadline:
        request = SetBool.Request()
        request.data = enabled
        response = wait(fixture.arm.call_async(request), 8.0)
        if response is not None and response.success:
            return
        time.sleep(0.2)
    raise AssertionError(f"arm={enabled} failed: {response}")


def timed_goal(seconds: float = 0.3) -> MotionCommand.Goal:
    goal = MotionCommand.Goal()
    goal.mode = MotionCommand.Goal.MODE_TIMED_VELOCITY
    goal.vx = 0.05
    goal.duration_sec = seconds
    goal.timeout_sec = seconds + 3.0
    return goal


def send(fixture: Fixture, goal: MotionCommand.Goal):
    handle = wait(fixture.action.send_goal_async(goal), 5.0)
    return handle


def emit(name: str, passed: bool, **details: object) -> None:
    print(json.dumps({"test": name, "pass": passed, **details}), flush=True)
    if not passed:
        raise AssertionError(name)


def main() -> int:
    rclpy.init(args=[])
    fixture = Fixture()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(fixture.node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        emit("server_available", fixture.action.wait_for_server(8.0))
        emit(
            "services_available",
            fixture.arm.wait_for_service(5.0) and fixture.stop.wait_for_service(5.0),
        )
        time.sleep(0.7)
        arm(fixture, True)

        handle = send(fixture, timed_goal())
        emit("timed_goal_accepted", handle is not None and handle.accepted)
        wrapped = wait(handle.get_result_async(), 10.0)
        emit(
            "timed_goal_success",
            wrapped.result.success and wrapped.result.error_code == 0,
            error_code=wrapped.result.error_code,
        )

        invalid = timed_goal()
        invalid.vy = 0.1
        invalid_handle = send(fixture, invalid)
        emit("invalid_goal_rejected", not invalid_handle.accepted)

        first = send(fixture, timed_goal(2.0))
        second = send(fixture, timed_goal(0.2))
        emit("concurrent_goal_rejected", first.accepted and not second.accepted)
        wait(first.cancel_goal_async(), 3.0)
        canceled = wait(first.get_result_async(), 10.0)
        emit(
            "cancel_stops_goal",
            canceled.result.error_code == MotionCommand.Result.ERROR_CANCELED,
            status=int(canceled.status),
        )

        handle = send(fixture, timed_goal(2.0))
        fixture.lease_alive = False
        lease_result = wait(handle.get_result_async(), 10.0)
        emit(
            "lease_loss_aborts",
            lease_result.result.error_code
            == MotionCommand.Result.ERROR_LEASE_UNAVAILABLE,
            error_code=lease_result.result.error_code,
        )
        fixture.lease_alive = True
        time.sleep(0.7)

        handle = send(fixture, timed_goal(2.0))
        time.sleep(0.7)
        fixture.publish_state = False
        stale_result = wait(handle.get_result_async(), 12.0)
        emit(
            "state_stale_aborts",
            stale_result.result.error_code == MotionCommand.Result.ERROR_STATE_STALE,
            error_code=stale_result.result.error_code,
        )
        fixture.publish_state = True
        time.sleep(0.7)

        arm(fixture, True)
        handle = send(fixture, timed_goal(2.0))
        stop_response = wait(fixture.stop.call_async(Trigger.Request()), 8.0)
        emergency_result = wait(handle.get_result_async(), 10.0)
        emit(
            "emergency_stop",
            stop_response.success
            and emergency_result.result.error_code
            == MotionCommand.Result.ERROR_CANCELED,
        )

        arm(fixture, True)
        settled_yaw_goal = MotionCommand.Goal()
        settled_yaw_goal.mode = MotionCommand.Goal.MODE_RELATIVE_YAW
        settled_yaw_goal.relative_yaw_deg = 10.0
        settled_yaw_goal.max_yaw_rate = 0.08
        settled_yaw_goal.timeout_sec = 5.0
        settled_yaw_handle = send(fixture, settled_yaw_goal)
        time.sleep(0.6)
        fixture.yaw = math.radians(10.0)
        settled_yaw_result = wait(settled_yaw_handle.get_result_async(), 12.0)
        emit(
            "relative_yaw_zero_velocity_hold_success",
            settled_yaw_result.result.success
            and settled_yaw_result.result.error_code == 0,
            error_code=settled_yaw_result.result.error_code,
        )
        fixture.yaw = 0.0
        time.sleep(0.5)

        arm(fixture, True)
        yaw_goal = MotionCommand.Goal()
        yaw_goal.mode = MotionCommand.Goal.MODE_RELATIVE_YAW
        yaw_goal.relative_yaw_deg = 10.0
        yaw_goal.max_yaw_rate = 0.08
        yaw_goal.timeout_sec = 0.6
        yaw_handle = send(fixture, yaw_goal)
        yaw_result = wait(yaw_handle.get_result_async(), 10.0)
        emit(
            "relative_yaw_timeout_is_bounded",
            yaw_result.result.error_code == MotionCommand.Result.ERROR_TIMEOUT,
            error_code=yaw_result.result.error_code,
        )
        arm(fixture, False)
    finally:
        executor.shutdown(timeout_sec=2.0)
        fixture.node.destroy_node()
        rclpy.shutdown()
        spin.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
