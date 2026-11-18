"""Offline velocity/collision test topics, disconnected from the physical arbiter."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parents[2]/'ros2_ws/src/robot_scene_nav_bringup/config'
    smoother=yaml.safe_load((root/'velocity_smoother.yaml').read_text())['velocity_smoother']['ros__parameters']
    collision=yaml.safe_load((root/'collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
    collision.update(cmd_vel_in_topic='/go2w/acceptance/smoothed',cmd_vel_out_topic='/go2w/acceptance/collision_checked')
    collision['scan']['topic']='/go2w/acceptance/synthetic_scan'
    return LaunchDescription([
        Node(package='nav2_velocity_smoother',executable='velocity_smoother',name='acceptance_velocity_smoother',
             parameters=[smoother],remappings=[('cmd_vel','/go2w/acceptance/raw'),('cmd_vel_smoothed','/go2w/acceptance/smoothed')]),
        Node(package='go2w_cmd_vel_bridge',executable='velocity_guard',name='acceptance_velocity_guard',
             parameters=[{'input_topic':'/go2w/acceptance/collision_checked','raw_topic':'/go2w/acceptance/raw',
                          'output_topic':'/go2w/acceptance/filtered','scan_topic':'/go2w/acceptance/synthetic_scan',
                          'require_localization':False}]),
        Node(package='nav2_collision_monitor' ,executable='collision_monitor',name='acceptance_collision_monitor',parameters=[collision]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='acceptance_lifecycle',
             parameters=[{'autostart':True,'node_names':['acceptance_velocity_smoother','acceptance_collision_monitor']}])
    ])
