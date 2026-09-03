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
    project_root = str(Path(root).parent)
    motion_gateway_socket = "/tmp/go2w_motion_gateway.sock"
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
    sdk_state_file = LaunchConfiguration("sdk_state_file")
    network_low_state_file = LaunchConfiguration("network_low_state_file")
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

    # Foxy ROS on the Jetson stays localhost-only; the SDK-only capture reads
    # the DCU bare-DDS state over eth0 and this system-Python relay republishes
    # fresh state into the local graph. Keeping SDK and rclpy in separate
    # processes avoids loading CycloneDDS 0.10 and ROS Foxy CycloneDDS 0.7 in
    # one interpreter.
    sdk_state_capture = ExecuteProcess(
        cmd=[
            python_executable,
            f"{root}/scripts/sport_state_sdk_capture.py",
            "--interface",
            interface,
            "--output",
            sdk_state_file,
        ],
        env=sdk_env,
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="3",
    )

    ros_python_executable = os.environ.get("GO2W_ROS_PYTHON", "/usr/bin/python3")
    local_state_relay = ExecuteProcess(
        cmd=[
            ros_python_executable,
            f"{root}/scripts/local_sport_state_ros_relay.py",
            "--input",
            sdk_state_file,
            "--output-topic",
            "/go2w/motion/local_sportmodestate",
        ],
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="3",
    )

    local_low_state_relay = ExecuteProcess(
        cmd=[
            ros_python_executable,
            f"{project_root}/scripts/go2w/local_low_state_ros_relay.py",
            "--input",
            network_low_state_file,
            "--output-topic",
            "/go2w/motion/local_lowstate",
        ],
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="3",
    )

    # The Foxy action server and its state relay stay on the robot-local ROS
    # participant.  These two thin proxy processes keep the camera/SLAM ROS
    # graph on the robot-facing network while forwarding only arm, stop and
    # MotionCommand through a permission-600 Unix socket.
    local_motion_gateway = ExecuteProcess(
        cmd=[
            ros_python_executable,
            f"{project_root}/scripts/go2w/motion_local_gateway.py",
            "--socket",
            motion_gateway_socket,
        ],
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="3",
    )
    network_motion_proxy = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-lc",
            "export ROS_LOCALHOST_ONLY=0; "
            f"source {project_root}/scripts/go2w/setup_environment.sh; "
            f"exec {ros_python_executable} {project_root}/scripts/go2w/"
            f"motion_network_proxy.py --socket {motion_gateway_socket}",
        ],
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="3",
    )
    network_low_state_capture = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-lc",
            "export ROS_LOCALHOST_ONLY=0; "
            f"source {project_root}/scripts/go2w/setup_environment.sh; "
            f"exec {ros_python_executable} {project_root}/scripts/go2w/"
            "network_low_state_capture.py --output \"$1\"",
            "network_low_state_capture",
            network_low_state_file,
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
                "sport_state_topic": "/go2w/motion/local_sportmodestate",
                "low_state_topic": "/go2w/motion/local_lowstate",
                # Keep the safety action and services on the robot-local ROS
                # graph. The public names are owned exclusively by the
                # network proxy below, avoiding duplicate DDS endpoints.
                "action_name": "/go2w/local_motion",
                "arm_service": "/go2w/local_arm",
                "emergency_stop_service": "/go2w/local_emergency_stop",
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
                "sdk_state_file", default_value="/tmp/go2w_sdk_sport_state.json"
            ),
            DeclareLaunchArgument(
                "network_low_state_file",
                default_value="/tmp/go2w_network_low_state.json",
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
            sdk_state_capture,
            local_state_relay,
            local_low_state_relay,
            local_motion_gateway,
            action_server,
            network_motion_proxy,
            network_low_state_capture,
        ]
    )
