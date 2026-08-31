from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("allow_unstable_alignment", default_value="false"),
        Node(
            package="go2w_sensor_time_bridge",
            executable="time_bridge",
            name="go2w_sensor_time_bridge",
            output="screen",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
                "allow_unstable_alignment": LaunchConfiguration("allow_unstable_alignment"),
            }],
        ),
    ])
