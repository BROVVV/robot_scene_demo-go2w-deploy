from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description():
    return LaunchDescription([Node(package="nav2_collision_monitor",executable="collision_monitor",
        name="collision_monitor",output="screen",parameters=[PathJoinSubstitution([FindPackageShare("robot_scene_nav_bringup"),"config","collision_monitor.yaml"])])])
