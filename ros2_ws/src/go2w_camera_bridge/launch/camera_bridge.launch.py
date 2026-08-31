from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("source", default_value="rpc"),
        DeclareLaunchArgument("interface", default_value="enp6s0"),
        DeclareLaunchArgument("calibration_file", default_value=""),
    ]
    bridge = Node(
        package="go2w_camera_bridge",
        executable="camera_bridge",
        name="go2w_camera_bridge",
        output="screen",
        parameters=[{
            "source": LaunchConfiguration("source"),
            "interface": LaunchConfiguration("interface"),
            "calibration_file": LaunchConfiguration("calibration_file"),
        }],
    )
    return LaunchDescription(arguments + [bridge])
