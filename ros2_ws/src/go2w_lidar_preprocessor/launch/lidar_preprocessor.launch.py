from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file"),
            DeclareLaunchArgument("geometry_file"),
            DeclareLaunchArgument("sensor_profile", default_value=""),
            DeclareLaunchArgument("mapping_accumulation", default_value="true"),
            DeclareLaunchArgument(
                "mapping_keyframe_minimum_translation_m", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "mapping_keyframe_minimum_rotation_rad", default_value="0.05"
            ),
            Node(package="go2w_lidar_preprocessor", executable="lidar_freshness_watchdog",
                 name="go2w_lidar_freshness_watchdog", output="screen"),
            Node(
                package="go2w_lidar_preprocessor",
                executable="lidar_preprocessor",
                name="go2w_lidar_preprocessor",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration("config_file"),
                        "geometry_file": LaunchConfiguration("geometry_file"),
                        "sensor_profile": LaunchConfiguration("sensor_profile"),
                        "mapping_accumulation": LaunchConfiguration("mapping_accumulation"),
                        "mapping_keyframe_minimum_translation_m": LaunchConfiguration(
                            "mapping_keyframe_minimum_translation_m"
                        ),
                        "mapping_keyframe_minimum_rotation_rad": LaunchConfiguration(
                            "mapping_keyframe_minimum_rotation_rad"
                        ),
                    }
                ],
            ),
        ]
    )
