from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2w_description.description_config import load_confirmed_measurements, render_urdf


def _create_robot_state_publisher(context):
    path = LaunchConfiguration("measurements_file").perform(context)
    values = load_confirmed_measurements(path)
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="go2w_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": render_urdf(values)}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("measurements_file"),
            OpaqueFunction(function=_create_robot_state_publisher),
        ]
    )
