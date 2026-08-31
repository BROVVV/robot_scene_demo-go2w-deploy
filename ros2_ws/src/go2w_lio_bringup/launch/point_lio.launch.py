from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2w_lio_bringup.config_gate import load_point_lio_gate


def _launch_nodes(context):
    lio = load_point_lio_gate(
        LaunchConfiguration("lio_config").perform(context),
        LaunchConfiguration("reference_config").perform(context),
        LaunchConfiguration("time_config").perform(context),
    )
    return [
        Node(
            package="go2w_lio_bringup",
            executable="point_lio_bridge",
            name="go2w_point_lio_readonly_bridge",
            output="screen",
            parameters=[
                {
                    "input_timeout_seconds": lio["safety"]["input_timeout_seconds"],
                    "extrinsic_imu2base_quat_xyzw_xyz": list(
                        lio["resolved_extrinsics"]["imu2base_quat_xyzw_xyz"]
                    ),
                    "gyro_sign_correction": list(lio["imu_frame"]["gyro_sign"]),
                    "yaw_reflect": bool(lio["imu_frame"]["yaw_reflect"]),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("lio_config"),
            DeclareLaunchArgument("reference_config"),
            DeclareLaunchArgument("time_config"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
