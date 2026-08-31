from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file"),
            DeclareLaunchArgument("geometry_file"),
            Node(
                package="go2w_lidar_preprocessor",
                executable="lidar_preprocessor",
                name="go2w_lidar_preprocessor",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("config_file"),
                        "geometry_file": LaunchConfiguration("geometry_file"),
                    }
                ],
            ),
        ]
    )
