"""碰撞监视器（计划书 §6.3）。

单独一个文件是因为 Nav2 官方也是这么组织的；它由 robot_scene_nav2.launch.py 在
``execution_enabled=true`` 时 include，并由同一个 lifecycle manager 一起拉起。

输入 ``/cmd_vel_nav_smoothed``，输出 ``/cmd_vel_nav_collision_checked``。
下游速度保护独立检查新鲜度和定位健康，作为最终速度话题的唯一发布者。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _parameters(config_path, use_sim_time, output_topic):
    from pathlib import Path
    import yaml
    parameters = yaml.safe_load(Path(config_path).read_text())["collision_monitor"]["ros__parameters"]
    parameters["use_sim_time"] = use_sim_time
    parameters["cmd_vel_out_topic"] = output_topic
    return parameters


def _launch_monitor(context):
    from pathlib import Path
    # Foxy gives node-specific YAML precedence over the wildcard file produced
    # by a launch dictionary. Merge first so the requested output cannot be
    # silently replaced by the file's physical-topic default.
    parameters = _parameters(Path(__file__).resolve().parents[1]/"config/collision_monitor.yaml",
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true",
        LaunchConfiguration("cmd_vel_out_topic").perform(context))
    return [Node(package="nav2_collision_monitor", executable="collision_monitor",
                 name="collision_monitor", output="screen", parameters=[parameters])]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("cmd_vel_out_topic", default_value="/cmd_vel_nav_collision_checked"),
            OpaqueFunction(function=_launch_monitor),
        ]
    )
