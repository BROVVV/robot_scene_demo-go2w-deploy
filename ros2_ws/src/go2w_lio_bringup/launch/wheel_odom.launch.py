from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("wheel_radius_m", default_value="0.089"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("lio_enabled", default_value="true"),
            DeclareLaunchArgument("lio_yaw_weight", default_value="0.35"),
            DeclareLaunchArgument(
                "lio_max_position_m", default_value="5.0"
            ),
            Node(
                package="go2w_lio_bringup",
                executable="go2w_wheel_odom",
                name="go2w_wheel_odom",
                output="screen",
                parameters=[
                    {
                        "wheel_radius_m": LaunchConfiguration("wheel_radius_m"),
                        "publish_tf": LaunchConfiguration("publish_tf"),
                        "lio_enabled": LaunchConfiguration("lio_enabled"),
                        "lio_yaw_weight": LaunchConfiguration(
                            "lio_yaw_weight"
                        ),
                        "lio_max_position_m": LaunchConfiguration(
                            "lio_max_position_m"
                        ),
                    }
                ],
            ),
        ]
    )
