from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2w_description.description_config import (
    load_official_reference,
    render_official_sensor_urdf,
)
from go2w_description.sensor_profile import load_sensor_profile


def _create_robot_state_publisher(context):
    path = LaunchConfiguration("reference_file").perform(context)
    reference = load_official_reference(path)
    profile_file = LaunchConfiguration('sensor_profile').perform(context)
    profile = load_sensor_profile(profile_file) if profile_file else None
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2w_official_sensor_frame_publisher",
            output="screen",
            parameters=[{"robot_description": render_official_sensor_urdf(reference, profile)}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("reference_file"),
            DeclareLaunchArgument('sensor_profile',default_value=''),
            OpaqueFunction(function=_create_robot_state_publisher),
        ]
    )
