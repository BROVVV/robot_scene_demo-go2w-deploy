"""Publish a ros2_motion_plan.json file to ROS2 /cmd_vel.

By default this script only prints the commands. Use --execute in a ROS2
environment to publish geometry_msgs/msg/Twist messages.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.schemas import Ros2MotionCommand, Ros2MotionPlan


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay outputs/ros2_motion_plan.json as ROS2 Twist commands."
    )
    parser.add_argument(
        "plan",
        nargs="?",
        default="outputs/ros2_motion_plan.json",
        help="Path to ros2_motion_plan.json.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Publish to ROS2 instead of printing a dry-run preview.",
    )
    parser.add_argument(
        "--allow-dry-run-plan",
        action="store_true",
        help="Allow publishing a plan whose dry_run flag is true.",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="Override the topic in the plan, for example /cmd_vel.",
    )
    parser.add_argument(
        "--force-legacy-while-nav2-inactive",
        action="store_true",
        help="Explicitly permit legacy publishing only when no Nav2 execute job is active.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    plan = _load_plan(Path(args.plan))
    topic = args.topic or plan.topic

    if not args.execute:
        _print_preview(plan, topic)
        return 0

    if plan.dry_run and not args.allow_dry_run_plan:
        print(
            "Refusing to publish a dry_run plan. "
            "Pass --allow-dry-run-plan only after operator approval.",
            file=sys.stderr,
        )
        return 2
    conflict = _active_nav2_execute_job(ROOT / "outputs" / "nav2_status.json")
    if conflict:
        print(
            "NAV2_CMD_VEL_CONFLICT: 活动的 Nav2 execute 任务正在发布速度；"
            "双发布会造成速度竞争，已拒绝 legacy /cmd_vel。",
            file=sys.stderr,
        )
        return 4
    if not args.force_legacy_while_nav2_inactive:
        print(
            "Legacy /cmd_vel 发布需要 --force-legacy-while-nav2-inactive；"
            "该参数也不能绕过活动 Nav2 任务互斥。",
            file=sys.stderr,
        )
        return 4

    return _publish_to_ros2(plan, topic)


def _active_nav2_execute_job(path: Path) -> bool:
    if not path.exists():
        return False
    try:
        status = json.loads(path.read_text(encoding="utf-8"))
        request_path = ROOT / "outputs" / "nav2_request.json"
        request = json.loads(request_path.read_text(encoding="utf-8")) if request_path.exists() else {}
        return request.get("mode") == "execute" and status.get("state") in {
            "created", "validating", "ready", "planning", "planned", "executing", "canceling"
        }
    except (OSError, json.JSONDecodeError):
        return True


def _load_plan(path: Path) -> Ros2MotionPlan:
    if not path.is_file():
        raise FileNotFoundError(f"Motion plan file not found: {path}")
    return Ros2MotionPlan.model_validate(
        json.loads(path.read_text(encoding="utf-8"))
    )


def _print_preview(plan: Ros2MotionPlan, topic: str) -> None:
    print(f"dry_run={plan.dry_run} topic={topic} rate={plan.command_rate_hz:g}Hz")
    print(f"commands={len(plan.commands)}")
    for command in plan.commands:
        print(
            f"{command.command_id} step={command.route_step_id} "
            f"action={command.source_action} duration={command.duration_sec:g}s "
            f"linear.x={command.twist.linear.x:g} angular.z={command.twist.angular.z:g}"
        )


def _publish_to_ros2(plan: Ros2MotionPlan, topic: str) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
    except ImportError as exc:
        print(
            "ROS2 Python packages are not available. "
            "Source your ROS2 environment before using --execute.",
            file=sys.stderr,
        )
        print(f"Original import error: {exc}", file=sys.stderr)
        return 3

    rclpy.init()
    node = rclpy.create_node("robot_scene_demo_motion_player")
    publisher = node.create_publisher(Twist, topic, 10)
    rate = node.create_rate(plan.command_rate_hz)
    try:
        for command in plan.commands:
            message = _to_twist_msg(command, Twist)
            for _ in range(_ticks(command.duration_sec, plan.command_rate_hz)):
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.0)
                rate.sleep()
        publisher.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _to_twist_msg(command: Ros2MotionCommand, twist_cls):
    message = twist_cls()
    message.linear.x = command.twist.linear.x
    message.linear.y = command.twist.linear.y
    message.linear.z = command.twist.linear.z
    message.angular.x = command.twist.angular.x
    message.angular.y = command.twist.angular.y
    message.angular.z = command.twist.angular.z
    return message


def _ticks(duration_sec: float, command_rate_hz: float) -> int:
    return max(1, math.ceil(duration_sec * command_rate_hz))


if __name__ == "__main__":
    raise SystemExit(main())
