#!/usr/bin/env python3
"""Read-only ROS/Nav2 graph health check for the Go2-W navigation gates."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", dest="json_path")
    parser.add_argument("--namespace", default=os.getenv("NAV2_NAMESPACE", ""))
    parser.add_argument("--map-frame", default=os.getenv("NAV2_MAP_FRAME", "map"))
    parser.add_argument("--base-frame", default=os.getenv("NAV2_BASE_FRAME", "base_link"))
    parser.add_argument("--scan-topic", default="/go2w/lidar/scan")
    parser.add_argument("--odom-topic", default="/lio/odom")
    parser.add_argument("--nav2-cmd-topic", default="/go2w/nav2_cmd_vel")
    args = parser.parse_args()
    checks: list[dict] = []

    def add(name, ok, message, *, plan=False, execute=False):
        checks.append(
            {
                "name": name,
                "ok": bool(ok),
                "message": str(message)[-500:],
                "required_for_plan": bool(plan),
                "required_for_execute": bool(execute),
            }
        )

    def command_check(name, command, *, plan=False, execute=False, timeout=8, success=None):
        try:
            result = subprocess.run(command, text=True, capture_output=True, timeout=timeout)
            message = (result.stdout or result.stderr).strip()
            ok = result.returncode == 0 and (success(message) if success else True)
            add(name, ok, message, plan=plan, execute=execute)
        except subprocess.TimeoutExpired:
            add(name, False, f"{timeout} 秒内没有响应", plan=plan, execute=execute)

    add(
        "ros_distro",
        os.getenv("ROS_DISTRO") == "humble",
        os.getenv("ROS_DISTRO", "未设置"),
        plan=True,
        execute=True,
    )
    try:
        import rclpy  # noqa: F401

        add("rclpy_import", True, "可导入", plan=True, execute=True)
    except ImportError as exc:
        add("rclpy_import", False, exc, plan=True, execute=True)
    try:
        from nav2_simple_commander.robot_navigator import BasicNavigator  # noqa: F401

        add("commander_import", True, "可导入", plan=True, execute=True)
    except ImportError as exc:
        add("commander_import", False, exc, plan=True, execute=True)

    prefix = args.namespace.rstrip("/")
    has_server = lambda message: "Action servers: 0" not in message
    has_publisher = lambda message: "Publisher count: 0" not in message
    command_check(
        "compute_path_to_pose_action",
        ["ros2", "action", "info", f"{prefix}/compute_path_to_pose"],
        plan=True,
        execute=True,
        success=has_server,
    )
    command_check(
        "navigate_to_pose_action",
        ["ros2", "action", "info", f"{prefix}/navigate_to_pose"],
        execute=True,
        success=has_server,
    )
    _check_tf(args.map_frame, args.base_frame, add)
    for name, topic in (
        ("scan_topic", args.scan_topic),
        ("odom_topic", args.odom_topic),
        ("map_topic", "/map"),
    ):
        command_check(
            name,
            ["ros2", "topic", "info", topic],
            plan=True,
            execute=True,
            success=has_publisher,
        )
    command_check(
        "nav2_cmd_vel_topic",
        ["ros2", "topic", "info", args.nav2_cmd_topic],
        execute=True,
        success=has_publisher,
    )
    for name, node in (
        ("collision_monitor", "/collision_monitor"),
        ("velocity_smoother", "/velocity_smoother"),
        ("control_arbiter", "/go2w_control_arbiter"),
        ("cmd_vel_bridge", "/go2w_cmd_vel_bridge"),
    ):
        command_check(name, ["ros2", "node", "info", node], execute=True)

    plan_bad = [item["name"] for item in checks if item["required_for_plan"] and not item["ok"]]
    execute_bad = [
        item["name"] for item in checks if item["required_for_execute"] and not item["ok"]
    ]
    payload = {
        "healthy_for_plan": not plan_bad,
        "healthy_for_execute": not execute_bad,
        "checks": checks,
        "plan_blocking_errors": plan_bad,
        "execute_blocking_errors": execute_bad,
        "note": "Graph health alone never authorizes execution; apply the 21-condition capability gate.",
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    print(text)
    if args.json_path:
        target = Path(args.json_path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text + "\n", encoding="utf-8")
    return 0 if payload["healthy_for_plan"] else 2


def _check_tf(map_frame, base_frame, add):
    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener

        rclpy.init(args=None)
        node = rclpy.create_node("robot_scene_nav2_health_check")
        buffer = Buffer()
        listener = TransformListener(buffer, node)  # noqa: F841
        deadline = __import__("time").monotonic() + 5.0
        ok = False
        while __import__("time").monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if buffer.can_transform(map_frame, base_frame, Time(), Duration(seconds=0.1)):
                ok = True
                break
        node.destroy_node()
        rclpy.shutdown()
        add(
            "map_to_base_tf",
            ok,
            f"{map_frame} -> {base_frame} " + ("可用" if ok else "5 秒内不可用"),
            plan=True,
            execute=True,
        )
    except Exception as exc:
        add("map_to_base_tf", False, exc, plan=True, execute=True)


if __name__ == "__main__":
    raise SystemExit(main())
