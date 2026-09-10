from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _detect_interface() -> str:
    result = subprocess.run(
        ["ip", "route", "get", "192.168.123.18"],
        check=True,
        capture_output=True,
        text=True,
    )
    fields = result.stdout.split()
    interface = fields[fields.index("dev") + 1] if "dev" in fields else ""
    if not interface or interface == "lo":
        # On the robot itself a route to its own address resolves through lo.
        result = subprocess.run(
            ["ip", "-4", "-o", "address", "show"],
            check=True,
            capture_output=True,
            text=True,
        )
        for line in result.stdout.splitlines():
            if "192.168.123.18/" in line:
                fields = line.split()
                if len(fields) >= 2:
                    return fields[1]
    return interface


def generate_launch_description() -> LaunchDescription:
    root = os.environ.get(
        "GO2W_CONTROL_ROOT", str(Path(__file__).resolve().parents[4])
    )
    python_executable = os.environ.get("GO2W_CONTROL_PYTHON", sys.executable)
    # SDK 进程专用环境：python cyclonedds 0.10.2 需要匹配的 libddsc
    # （系统 ros-foxy-cyclonedds 0.7.0 缺 ddsi_sertype_v0 符号）。ROS 层
    # rmw_cyclonedds_cpp 0.7.11 与 0.10.2 不兼容，所以只对 SDK 进程注入。
    sdk_env = dict(os.environ)
    cyclone_candidates = [
        os.path.expanduser("~/cyclonedds_0.10.2/lib"),
        os.path.join(os.path.dirname(root), "external", "cyclonedds_0.10.2", "install", "lib"),
        "/home/mxt/robotscene/external/cyclonedds_0.10.2/install/lib",
    ]
    for cyclone_lib in cyclone_candidates:
        if os.path.isdir(cyclone_lib):
            sdk_env["LD_LIBRARY_PATH"] = (
                cyclone_lib + ":" + sdk_env.get("LD_LIBRARY_PATH", "")
            )
            break
    interface = LaunchConfiguration("interface")
    config_file = LaunchConfiguration("config_file")
    robot_ip = LaunchConfiguration("robot_ip")
    require_arm = LaunchConfiguration("require_arm")
    dry_run = LaunchConfiguration("dry_run")
    log_root = LaunchConfiguration("log_root")
    sdk_command_socket = LaunchConfiguration("sdk_command_socket")
    lease_status_dir = LaunchConfiguration("lease_status_dir")
    turn_longitudinal_compensation_vx = LaunchConfiguration(
        "turn_longitudinal_compensation_vx"
    )
    post_turn_zero_velocity_hold_sec = LaunchConfiguration(
        "post_turn_zero_velocity_hold_sec"
    )

    lease_holder = ExecuteProcess(
        cmd=[
            python_executable,
            f"{root}/scripts/hold_sport_lease.py",
            "--interface",
            interface,
            "--socket-path",
            sdk_command_socket,
            "--status-dir",
            lease_status_dir,
        ],
        env=sdk_env,
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="3",
    )

    lease_bridge = ExecuteProcess(
        cmd=[
            python_executable,
            f"{root}/scripts/lease_status_bridge.py",
            "--status-dir",
            lease_status_dir,
        ],
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="3",
    )

    action_server = Node(
        package="go2w_motion_control",
        executable="go2w_motion_action_server",
        name="go2w_motion_action_server",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            config_file,
            {
                "robot_ip": robot_ip,
                "require_arm": require_arm,
                "dry_run": dry_run,
                "log_root": log_root,
                "sdk_command_socket": sdk_command_socket,
                "turn_longitudinal_compensation_vx": (
                    turn_longitudinal_compensation_vx
                ),
                "post_turn_zero_velocity_hold_sec": (
                    post_turn_zero_velocity_hold_sec
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("interface", default_value=_detect_interface()),
            DeclareLaunchArgument(
                "config_file",
                default_value=f"{root}/ros2_ws/install/go2w_motion_control/share/"
                "go2w_motion_control/config/motion_control.yaml",
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.123.18"),
            DeclareLaunchArgument("require_arm", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument(
                "sdk_command_socket", default_value="/tmp/go2w_sdk_motion.sock"
            ),
            DeclareLaunchArgument(
                "lease_status_dir", default_value="/tmp/go2w_lease_status"
            ),
            DeclareLaunchArgument(
                "turn_longitudinal_compensation_vx", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "post_turn_zero_velocity_hold_sec", default_value="1.0"
            ),
            DeclareLaunchArgument("log_root", default_value=f"{root}/logs"),
            lease_holder,
            lease_bridge,
            action_server,
        ]
    )
