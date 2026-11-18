"""Independent live 2D mapping or saved-map AMCL; never starts motion nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('robot_scene_nav_bringup')
    mode = LaunchConfiguration('mode')
    mapping = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    localizing = IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localization']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('map_authorized', default_value='false'),
        # Bind every keyframe graph and every downstream semantic snapshot to
        # the exact geometry profile used by the live LiDAR preprocessor.
        # The production launcher passes the SHA-256; ``unknown`` remains only
        # for direct developer launches and is rejected by acceptance checks.
        DeclareLaunchArgument('sensor_profile_id', default_value='unknown'),
        # 初始位姿。amcl_2d.yaml 里 set_initial_pose 是 false，而 AMCL 在
        # "set_initial_pose=false 且没人发 /initialpose" 时**一个位姿都不发、
        # 连 map->odom_fused 都不发**（publishAmclPose / calculateMaptoOdomTransform /
        # sendMapToOdomTransform 三处都在 !initial_pose_is_known_ 时直接 return），
        # 而 initial_pose_is_known_ 只有三条路能变 true：/initialpose 话题、
        # /set_initial_pose 服务、以及 set_initial_pose=true 这个参数。
        # 前两条仓里没人发，所以走第三条：把操作员提供的当前初始估计传进去。
        # 存档时的位姿可能已因遥控/搬动而失效，不自动当作当前位姿。
        #
        # 默认 false + 0.0：不给值就等价于原来的模板，不会因为漏传参数而"假装在
        # 原点定位"。调用方（start_2d_localization.sh）在 localization 模式下必须
        # 显式给值，给不出来就拒绝启动。
        DeclareLaunchArgument('use_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        Node(package='robot_scene_nav_bringup', executable='keyframe_slam_toolbox', name='slam_toolbox',
             output='screen', condition=mapping,
             remappings=[('map','/go2w/slam/map_raw')],
             parameters=[PathJoinSubstitution([share, 'config', 'slam_2d_mapping.yaml']),
                         {'sensor_profile_id': LaunchConfiguration('sensor_profile_id')}]),
        Node(package='robot_scene_nav_bringup',executable='observed_floor_map.py',
             name='go2w_observed_floor_map',output='screen',condition=mapping),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', condition=localizing,
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'frame_id': 'map', 'use_sim_time': False}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', condition=localizing,
             # 这里的字典**在后**，所以覆盖 amcl_2d.yaml 里的 set_initial_pose: false。
             # 注意 launch_ros 会对 substitution 的值做 yaml 求值
             # （evaluate_parameters.py：yaml.safe_load 之后只在结果是
             # bool/int/float/str/bytes 时才用），所以 'false' 到这里是真 bool，
             # 不会撞上 "STRING 覆盖 BOOL 参数" 的 InvalidParameterTypeException。
             parameters=[PathJoinSubstitution([share, 'config', 'amcl_2d.yaml']),
                         {'set_initial_pose': LaunchConfiguration('use_initial_pose'),
                          'initial_pose.x': LaunchConfiguration('initial_pose_x'),
                          'initial_pose.y': LaunchConfiguration('initial_pose_y'),
                          'initial_pose.yaw': LaunchConfiguration('initial_pose_yaw')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization_2d', output='screen', condition=localizing,
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl'], 'use_sim_time': False}]),
        Node(package='robot_scene_nav_bringup', executable='tf_authority_monitor', output='screen'),
        Node(package='robot_scene_nav_bringup', executable='localization_health_monitor.py',
             name='go2w_localization_health', output='screen',
             parameters=[{'mode': mode, 'map_authorized': LaunchConfiguration('map_authorized')}]),
    ])
