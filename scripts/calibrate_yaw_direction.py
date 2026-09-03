#!/usr/bin/env python3
"""Calibrate the sign from a robot yaw-rate command to IMU yaw change."""

from __future__ import annotations

import argparse
import math
import os
import tempfile
import time

import rclpy
from go2w_motion_interfaces.action import MotionCommand
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from unitree_go.msg import SportModeState


def normalize(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value <= -math.pi:
        value += 2.0 * math.pi
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--no-write",
        action="store_true",
        help="measure and print the sign without changing the YAML",
    )
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node("go2w_yaw_direction_calibrator")
    yaw = {"value": None}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(
        SportModeState,
        "/lf/sportmodestate",
        lambda msg: yaw.update(value=float(msg.imu_state.rpy[2])),
        qos,
    )
    action = ActionClient(node, MotionCommand, "/go2w/motion")
    arm = node.create_client(SetBool, "/go2w/arm")
    if not action.wait_for_server(timeout_sec=5.0) or not arm.wait_for_service(
        timeout_sec=5.0
    ):
        raise SystemExit("controller services unavailable")

    deadline = time.monotonic() + 5.0
    while yaw["value"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if yaw["value"] is None:
        raise SystemExit("no IMU yaw state")

    arm_request = SetBool.Request()
    arm_request.data = True
    future = arm.call_async(arm_request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None or not future.result().success:
        raise SystemExit(f"arm failed: {future.result()}")

    def run(rate: float) -> tuple[float, float]:
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        before = float(yaw["value"])
        goal = MotionCommand.Goal()
        goal.mode = MotionCommand.Goal.MODE_TIMED_VELOCITY
        goal.yaw_rate = rate
        goal.duration_sec = 0.3
        goal.timeout_sec = 5.0
        send = action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=5.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("calibration goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
        wrapped = result_future.result()
        if wrapped is None or not wrapped.result.success:
            raise RuntimeError(f"calibration motion failed: {wrapped}")
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        after = float(yaw["value"])
        return before, normalize(after - before)

    try:
        positive_before, positive_delta = run(0.05)
        negative_before, negative_delta = run(-0.05)
        if (
            abs(positive_delta) < 0.002
            or abs(negative_delta) < 0.002
            or positive_delta * negative_delta >= 0.0
        ):
            raise RuntimeError(
                "yaw changes were too small or did not have opposite directions"
            )
        sign = 1 if positive_delta > 0.0 else -1
        if not args.no_write:
            with open(args.config, encoding="utf-8") as source:
                lines = source.readlines()
            replaced = False
            for index, line in enumerate(lines):
                if line.lstrip().startswith("yaw_command_sign:"):
                    indent = line[: len(line) - len(line.lstrip())]
                    lines[index] = f"{indent}yaw_command_sign: {sign}\n"
                    replaced = True
                    break
            if not replaced:
                raise RuntimeError("yaw_command_sign is missing from config")
            descriptor, temporary = tempfile.mkstemp(
                prefix="yaw_calibration_",
                dir=os.path.dirname(args.config),
                text=True,
            )
            with os.fdopen(descriptor, "w", encoding="utf-8") as output:
                output.writelines(lines)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, args.config)
        print(
            f"yaw_command_sign={sign} positive_before={positive_before:.6f} "
            f"positive_delta={positive_delta:.6f} negative_before={negative_before:.6f} "
            f"negative_delta={negative_delta:.6f}"
        )
    finally:
        disarm = SetBool.Request()
        disarm.data = False
        future = arm.call_async(disarm)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
