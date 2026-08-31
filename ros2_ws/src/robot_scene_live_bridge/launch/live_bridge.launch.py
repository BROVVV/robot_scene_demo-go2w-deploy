from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("spool_root"),
            DeclareLaunchArgument("session_id"),
            DeclareLaunchArgument("sensor_timeout_seconds", default_value="0.3"),
            Node(
                package="robot_scene_live_bridge",
                executable="live_bridge",
                name="robot_scene_live_bridge",
                output="screen",
                parameters=[
                    {
                        "spool_root": LaunchConfiguration("spool_root"),
                        "session_id": LaunchConfiguration("session_id"),
                        "sensor_timeout_seconds": LaunchConfiguration(
                            "sensor_timeout_seconds"
                        ),
                    }
                ],
            ),
        ]
    )
