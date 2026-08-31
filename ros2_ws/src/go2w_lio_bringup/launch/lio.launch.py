from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2w_lio_bringup.config_gate import load_lio_gate


def _launch_nodes(context):
    lio = load_lio_gate(
        LaunchConfiguration("lio_config").perform(context),
        LaunchConfiguration("reference_config").perform(context),
        LaunchConfiguration("time_config").perform(context),
    )
    parameters = lio["parameters"]
    resolved = lio["resolved_extrinsics"]
    rko_parameters = {
        "lidar_topic": lio["lidar_topic"],
        "imu_topic": lio["imu_topic"],
        "lidar_frame": "utlidar_lidar",
        "imu_frame": "utlidar_imu",
        "base_frame": lio["base_frame"],
        "odom_frame": lio["odom_frame"],
        "odom_topic": "/rko_lio/odom",
        "invert_odom_tf": False,
        "publish_deskewed_scan": True,
        "deskewed_scan_topic": "/rko_lio/frame",
        "publish_local_map": False,
        "publish_lidar_acceleration": False,
        "initialization_phase": parameters["initialization_phase"],
        "deskew": parameters["deskew"],
        "voxel_size": parameters["voxel_size_m"],
        "min_range": parameters["minimum_range_m"],
        "max_range": parameters["maximum_range_m"],
        "max_correspondence_distance": parameters[
            "max_correspondence_distance_m"
        ],
        "double_downsample": parameters["double_downsample"],
        "lidar_timestamps.multiplier_to_seconds": lio["point_time"][
            "multiplier_to_seconds"
        ],
        "lidar_timestamps.force_relative": True,
        "lidar_timestamps.force_absolute": False,
        "extrinsic_lidar2base_quat_xyzw_xyz": list(
            resolved["lidar2base_quat_xyzw_xyz"]
        ),
        "extrinsic_imu2base_quat_xyzw_xyz": list(
            resolved["imu2base_quat_xyzw_xyz"]
        ),
        "dump_results": False,
    }
    lio_node = Node(
        package="rko_lio",
        executable="online_node",
        name="rko_lio_online_node",
        output="screen",
        parameters=[rko_parameters],
    )
    adapter = Node(
        package="go2w_lio_bringup",
        executable="output_adapter",
        name="go2w_lio_output_adapter",
        output="screen",
        parameters=[
            {
                "input_timeout_seconds": lio["safety"]["input_timeout_seconds"],
                "expected_odom_frame": lio["odom_frame"],
                "expected_base_frame": lio["base_frame"],
                "expected_lidar_frame": "utlidar_lidar",
                "extrinsic_lidar2base_quat_xyzw_xyz": list(
                    resolved["lidar2base_quat_xyzw_xyz"]
                ),
            }
        ],
    )
    return [lio_node, adapter]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("lio_config"),
            DeclareLaunchArgument("reference_config"),
            DeclareLaunchArgument("time_config"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
