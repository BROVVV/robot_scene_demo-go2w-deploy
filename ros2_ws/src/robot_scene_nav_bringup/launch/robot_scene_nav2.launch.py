from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params=LaunchConfiguration("params_file"); map_yaml=LaunchConfiguration("map")
    execution_enabled=LaunchConfiguration("execution_enabled")
    use_plain_slam_map=LaunchConfiguration("use_plain_slam_map")
    return LaunchDescription([
        DeclareLaunchArgument("map"), DeclareLaunchArgument("use_sim_time",default_value="false"),
        DeclareLaunchArgument("execution_enabled", default_value="false"),
        # Level-B interface (plan §13): mapping-assist map for Nav2 planning.
        # DEFAULTS TO OFF: the Pandar extrinsic is candidate_unconfirmed and
        # plain_slam is not motion/safety authority.
        DeclareLaunchArgument("use_plain_slam_map", default_value="false"),
        DeclareLaunchArgument("params_file",default_value=PathJoinSubstitution([FindPackageShare("robot_scene_nav_bringup"),"config","nav2_params_humble.yaml"])),
        LogInfo(
            condition=UnlessCondition(execution_enabled),
            msg="Go2-W Nav2 execute bringup blocked: execution_enabled is false. Use the plan-only launch for planning.",
        ),
        LogInfo(
            condition=IfCondition(use_plain_slam_map),
            msg="use_plain_slam_map=true: relaying /go2w/slam/map_2d -> /map "
                "(MAPPING_ASSIST ONLY, not collision/safety authority).",
        ),
        Node(
            package="robot_scene_nav_bringup",
            executable="plain_slam_map_relay.py",
            name="plain_slam_map_relay",
            output="screen",
            condition=IfCondition(use_plain_slam_map),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("nav2_bringup"),"launch","bringup_launch.py"])),
            condition=IfCondition(execution_enabled),
            launch_arguments={"map":map_yaml,"params_file":params,"use_sim_time":LaunchConfiguration("use_sim_time")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("robot_scene_nav_bringup"),"launch","collision_monitor.launch.py"])),
            condition=IfCondition(execution_enabled),
        ),
    ])
