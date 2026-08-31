from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("fusion_config"),
        DeclareLaunchArgument("camera_config"),
        DeclareLaunchArgument("extrinsics_config"),
    ]
    node = Node(
        package="go2w_rgb_lidar_fusion",
        executable="fusion_node",
        name="go2w_rgb_lidar_fusion",
        output="screen",
        parameters=[
            {
                "fusion_config": LaunchConfiguration("fusion_config"),
                "camera_config": LaunchConfiguration("camera_config"),
                "extrinsics_config": LaunchConfiguration("extrinsics_config"),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
