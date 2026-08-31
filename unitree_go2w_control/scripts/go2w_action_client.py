#!/usr/bin/env python3
"""Send one MotionCommand goal and cancel it safely on SIGINT or on request."""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time

import rclpy
from go2w_motion_interfaces.action import MotionCommand
from rclpy.action import ActionClient


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("timed", "yaw"), required=True)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--degrees", type=float, default=0.0)
    parser.add_argument("--max-yaw-rate", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=0.0)
    parser.add_argument("--cancel-after", type=float, default=0.0)
    args = parser.parse_args()

    cancel_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: cancel_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: cancel_requested.set())
    rclpy.init(args=[])
    node = rclpy.create_node("go2w_motion_cli")
    client = ActionClient(node, MotionCommand, "/go2w/motion")

    def shutdown() -> None:
        # Foxy requires ActionClient handles to be destroyed before their node.
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()

    if not client.wait_for_server(timeout_sec=5.0):
        print(json.dumps({"event": "error", "message": "Action server unavailable"}))
        shutdown()
        return 2

    goal = MotionCommand.Goal()
    goal.mode = (
        MotionCommand.Goal.MODE_TIMED_VELOCITY
        if args.mode == "timed"
        else MotionCommand.Goal.MODE_RELATIVE_YAW
    )
    goal.vx = args.vx
    goal.vy = args.vy
    goal.yaw_rate = args.yaw_rate
    goal.duration_sec = args.seconds
    goal.relative_yaw_deg = args.degrees
    goal.max_yaw_rate = args.max_yaw_rate
    goal.timeout_sec = args.timeout

    def feedback_callback(message: object) -> None:
        feedback = message.feedback
        print(
            json.dumps(
                {
                    "event": "feedback",
                    "elapsed_sec": feedback.elapsed_sec,
                    "current_vx": feedback.current_vx,
                    "current_vy": feedback.current_vy,
                    "current_yaw_rate": feedback.current_yaw_rate,
                    "estimated_distance_m": feedback.estimated_distance_m,
                    "target_relative_yaw_deg": feedback.target_relative_yaw_deg,
                    "current_relative_yaw_deg": feedback.current_relative_yaw_deg,
                    "yaw_error_deg": feedback.yaw_error_deg,
                    "robot_mode": feedback.robot_mode,
                    "robot_error_code": feedback.robot_error_code,
                    "lease_alive": feedback.lease_alive,
                    "state_fresh": feedback.state_fresh,
                }
            ),
            flush=True,
        )

    send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
    while rclpy.ok() and not send_future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    if not send_future.done() or send_future.result() is None:
        shutdown()
        return 2
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        print(json.dumps({"event": "goal", "accepted": False}), flush=True)
        shutdown()
        return 3
    print(json.dumps({"event": "goal", "accepted": True}), flush=True)
    result_future = goal_handle.get_result_async()
    started = time.monotonic()
    cancel_sent = False
    while rclpy.ok() and not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
        scheduled_cancel = args.cancel_after > 0 and (
            time.monotonic() - started >= args.cancel_after
        )
        if (cancel_requested.is_set() or scheduled_cancel) and not cancel_sent:
            cancel_future = goal_handle.cancel_goal_async()
            while rclpy.ok() and not cancel_future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
            cancel_sent = True
            print(json.dumps({"event": "cancel_requested"}), flush=True)

    wrapped = result_future.result()
    if wrapped is None:
        shutdown()
        return 2
    result = wrapped.result
    output = {
        "event": "result",
        "status": int(wrapped.status),
        "success": result.success,
        "error_code": result.error_code,
        "message": result.message,
        "elapsed_sec": result.elapsed_sec,
        "estimated_distance_m": result.estimated_distance_m,
        "actual_relative_yaw_deg": result.actual_relative_yaw_deg,
        "last_move_status_code": result.last_move_status_code,
        "last_stop_status_code": result.last_stop_status_code,
    }
    print(json.dumps(output), flush=True)
    shutdown()
    return 0 if result.success or (cancel_sent and result.error_code == 9) else 1


if __name__ == "__main__":
    raise SystemExit(main())
