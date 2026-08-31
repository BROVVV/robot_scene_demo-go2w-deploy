#!/usr/bin/env python3
# Copyright 2026 robot_scene_demo maintainers
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Go2-W plain_slam mapping-assist bring-up (plan §8 / §4.1).

Starts, in the isolated /go2w/slam/* + pslam_* namespaces:

  * plain_slam_ros2 lio_3d_node (go2w_plain_slam_lio)
  * plain_slam_ros2 slam_3d_node (go2w_plain_slam_slam)
  * go2w_plain_slam_bridge: pandar_slam_adapter, plain_slam_odom_adapter,
    pointcloud_to_occupancy, plain_slam_health_monitor

The runtime configuration is generated synchronously here (idempotent) so the
nodes can never start with stale extrinsics.  This launch is mapping-only:
/go2w/odom/fused and the motion / safety chain are never touched.
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROJECT_ROOT = Path(__file__).resolve().parents[4]
SCRIPTS_GO2W = PROJECT_ROOT / "scripts" / "go2w"
if str(SCRIPTS_GO2W) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_GO2W))

# Synchronously (re)generate the runtime config from the YAML sources.
import generate_plain_slam_pandar_config as plain_slam_generator  # noqa: E402

plain_slam_generator.write_runtime_files()

RUNTIME = PROJECT_ROOT / "runtime" / "go2w" / "plain_slam"
BRIDGE_CONFIG_FILE = PROJECT_ROOT / "configs" / "go2w" / "plain_slam_bridge.yaml"
OFFICIAL_REFERENCE = PROJECT_ROOT / "configs" / "go2w" / "official_reference.yaml"
PANDAR_EXTRINSICS = PROJECT_ROOT / "configs" / "go2w" / "hesai_pandarxt16_extrinsics.yaml"
MASTER_CONFIG_FILE = PROJECT_ROOT / "configs" / "go2w" / "plain_slam.yaml"

with BRIDGE_CONFIG_FILE.open("r", encoding="utf-8") as handle:
    _BRIDGE_YAML = yaml.safe_load(handle)
with MASTER_CONFIG_FILE.open("r", encoding="utf-8") as handle:
    _MASTER_YAML = yaml.safe_load(handle)


def segment(name: str) -> dict:
    """Flat ros__parameters dict for one bridge node segment."""
    return dict(_BRIDGE_YAML[name]["ros__parameters"])


def generate_launch_description() -> LaunchDescription:
    # Upstream nodes are renamed (go2w_plain_slam_lio / _slam), so the
    # generated YAML's node-name key (lio_3d_node / slam_3d_node) would NOT
    # match parameter-file selection.  Pass the parameters as flat dicts.
    with (RUNTIME / "generated_lio_3d_config.yaml").open("r", encoding="utf-8") as handle:
        lio_cfg = yaml.safe_load(handle)["lio_3d_node"]["ros__parameters"]
    with (RUNTIME / "generated_slam_3d_config.yaml").open("r", encoding="utf-8") as handle:
        slam_cfg = yaml.safe_load(handle)["slam_3d_node"]["ros__parameters"]
    lio_config = [dict(lio_cfg), {"param_files_dir": str(RUNTIME)}]
    slam_config = [dict(slam_cfg), {"param_files_dir": str(RUNTIME)}]
    raw_imu_topic = str(_MASTER_YAML["imu_source_default"])
    slam_imu_topic = str(_MASTER_YAML["topics"]["imu"])

    pandar_params = segment("pandar_adapter")
    pandar_params.update(
        {
            "input_topic": "/hesai/pandarxt16/points_raw",
            "output_topic": "/go2w/slam/pandar_points",
            "point_status_topic": "/go2w/slam/point_status",
        }
    )

    odom_params = segment("plain_slam_odom_adapter")
    odom_params.update(
        {
            "imu_pose_topic": "/go2w/slam/imu_pose_raw",
            "imu_odom_topic": "/go2w/slam/imu_odom_raw",
            "odom_base_topic": "/go2w/slam/odom_base",
            "base_pose_topic": "/go2w/slam/base_pose",
            "odom_frame": "pslam_odom",
            "child_frame": "base_link_mapping_assist",
            "publish_tf": False,
            "official_reference_file": str(OFFICIAL_REFERENCE),
        }
    )

    occupancy_params = segment("pointcloud_to_occupancy")
    occupancy_params.update(
        {
            "scan_topic": "/go2w/slam/aligned_scan",
            "odom_topic": "/go2w/slam/odom_base",
            "map_topic": "/go2w/slam/map_2d",
            "map_frame": "pslam_odom",
            "occupancy_status_topic": "/go2w/slam/occupancy_status",
            "extrinsics_file": str(PANDAR_EXTRINSICS),
        }
    )

    health_params = segment("plain_slam_health_monitor")
    health_params.update(
        {
            "adapted_cloud_topic": "/go2w/slam/pandar_points",
            "imu_topic": slam_imu_topic,
            "odom_topic": "/go2w/slam/odom_base",
            "aligned_scan_topic": "/go2w/slam/aligned_scan",
            "map_2d_topic": "/go2w/slam/map_2d",
            "map_3d_topic": "/go2w/slam/map_3d",
            "point_status_topic": "/go2w/slam/point_status",
            "occupancy_status_topic": "/go2w/slam/occupancy_status",
            "health_topic": "/go2w/slam/health",
            "ready_topic": "/go2w/slam/ready",
        }
    )

    return LaunchDescription(
        [
            Node(
                package="go2w_plain_slam_bridge",
                executable="imu_fallback_adapter.py",
                name="imu_fallback_adapter",
                output="screen",
                arguments=[
                    "--imu-topic", raw_imu_topic,
                    "--output-topic", slam_imu_topic,
                ],
            ),
            DeclareLaunchArgument(
                "start_upstream",
                default_value="true",
                description="start plain_slam_ros2 lio+slam nodes (false = "
                            "bridge-only smoke test with fake data)",
            ),
            Node(
                package="plain_slam_ros2",
                executable="lio_3d_node",
                name="go2w_plain_slam_lio",
                output="screen",
                parameters=lio_config,
                condition=IfCondition(LaunchConfiguration("start_upstream")),
            ),
            Node(
                package="plain_slam_ros2",
                executable="slam_3d_node",
                name="go2w_plain_slam_slam",
                output="screen",
                parameters=slam_config,
                condition=IfCondition(LaunchConfiguration("start_upstream")),
            ),
            Node(
                package="go2w_plain_slam_bridge",
                executable="pandar_slam_adapter",
                name="pandar_slam_adapter",
                output="screen",
                parameters=[pandar_params],
            ),
            Node(
                package="go2w_plain_slam_bridge",
                executable="plain_slam_odom_adapter",
                name="plain_slam_odom_adapter",
                output="screen",
                parameters=[odom_params],
            ),
            Node(
                package="go2w_plain_slam_bridge",
                executable="pointcloud_to_occupancy",
                name="pointcloud_to_occupancy",
                output="screen",
                parameters=[occupancy_params],
            ),
            Node(
                package="go2w_plain_slam_bridge",
                executable="plain_slam_health_monitor",
                name="plain_slam_health_monitor",
                output="screen",
                parameters=[health_params],
            ),
        ]
    )
