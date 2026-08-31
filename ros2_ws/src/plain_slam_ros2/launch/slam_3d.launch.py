from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  config_dir = os.path.join(
    get_package_share_directory('plain_slam_ros2'),
    'config'
  )

  lio_config_yaml = os.path.join(config_dir, 'lio_3d_config.yaml')
  slam_config_yaml = os.path.join(config_dir, 'slam_3d_config.yaml')

  return LaunchDescription([
    Node(
      package='plain_slam_ros2',
      executable='lio_3d_node',
      name='lio_3d_node',
      parameters=[
        lio_config_yaml,
        {'param_files_dir': config_dir}
      ],
      output='log'
    ),
    Node(
      package='plain_slam_ros2',
      executable='slam_3d_node',
      name='slam_3d_node',
      output='screen',
      parameters=[
        slam_config_yaml,
        {'param_files_dir': config_dir}
      ]
    )
  ])
