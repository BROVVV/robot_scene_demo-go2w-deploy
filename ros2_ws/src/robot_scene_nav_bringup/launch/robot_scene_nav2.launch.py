"""Nav2 planning/control, consuming the independent 2D map/TF chain by default.

external_2d: SLAM Toolbox owns map->odom_fused; wheel odometry owns
odom_fused->base_link; the health monitor owns localization_valid.
legacy_plain_slam explicitly opts into the historical 3D frame bridge.
execution_enabled=false starts planner/BT only. The output topic can be
isolated for control-chain validation without connecting the physical bridge.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


EXECUTE_NODES = [
    "planner_server",
    "controller_server",
    "bt_navigator",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
]
# bt_navigator 不产生速度（它只把目标派给 controller_server），但它是
# /navigate_to_pose 的宿主。后端 _readiness() 要求它在线才肯发 ComputePathToPose，
# 所以只规划模式也必须起它，否则「只规划」退化成「什么都不做」。
PLAN_ONLY_NODES = ["planner_server", "bt_navigator"]


def _foxy_execution_gate(context):
    if (os.environ.get("ROS_DISTRO") == "foxy"
            and LaunchConfiguration("execution_enabled").perform(context).lower() == "true"):
        if LaunchConfiguration("foxy_isolated_validation").perform(context).lower() == "true":
            import xml.etree.ElementTree as ET
            uri = os.environ.get("CYCLONEDDS_URI", "")
            output = LaunchConfiguration("nav2_cmd_vel_topic").perform(context)
            if (os.environ.get("ROS_DOMAIN_ID") != "99" or
                    os.environ.get("ROS_LOCALHOST_ONLY") != "1" or
                    os.environ.get("RMW_IMPLEMENTATION") != "rmw_cyclonedds_cpp" or
                    not output.startswith("/go2w/acceptance/") or not uri.startswith("file://")):
                raise RuntimeError("Foxy validation requires domain 99, loopback DDS and an isolated output")
            tree = ET.parse(uri[7:])
            if ([n.text for n in tree.iter("NetworkInterfaceAddress")] != ["127.0.0.1"] or
                    [n.text for n in tree.iter("AllowMulticast")] != ["false"] or
                    any(n.get("Address") != "127.0.0.1" for n in tree.iter("Peer"))):
                raise RuntimeError("Foxy validation DDS must bind only loopback")
            return []
        # Normal Foxy production execution stays closed. A new one-use ledger
        # permits only the separately supervised 20 cm commissioning envelope;
        # the physical bridge independently checks and consumes that same file.
        trial = LaunchConfiguration("foxy_supervised_trial_ledger").perform(context)
        if trial:
            import platform
            import sys
            if (platform.machine() != "aarch64" or not Path('/etc/nv_tegra_release').is_file()
                    or os.environ.get('GO2W_AREA_CLEARED') != 'I_HAVE_CLEARED_THE_AREA'
                    or os.environ.get('ROS_DOMAIN_ID') != '0'
                    or os.environ.get('ROS_LOCALHOST_ONLY', '0') != '0'
                    or os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp'
                    or LaunchConfiguration('nav2_cmd_vel_topic').perform(context) != '/go2w/nav2_cmd_vel'):
                raise RuntimeError('Foxy supervised commissioning requires the native robot and field confirmation')
            root = Path(os.environ.get('GO2W_RELEASE_ROOT', '')).resolve()
            ledger = Path(trial)
            if (root.parent != Path('/home/unitree/robotscene_releases')
                    or not ledger.is_absolute()
                    or Path('/home/unitree/robotscene_runtime') not in ledger.resolve().parents):
                raise RuntimeError('Foxy supervised commissioning paths are not robot-local')
            sys.path.insert(0, str(root))
            from app.navigation.foxy_supervised_trial import validate_supervised_trial
            validate_supervised_trial(ledger, boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
            return []
        raise RuntimeError(
            "Foxy execution is disabled: velocity smoother, collision monitor, "
            "and motion safety equivalence have not been validated"
        )
    return []


def generate_launch_description():
    share = FindPackageShare("robot_scene_nav_bringup")
    params_file = LaunchConfiguration("params_file")
    smoother_params = LaunchConfiguration("velocity_smoother_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    execution_enabled = LaunchConfiguration("execution_enabled")
    bounded_trial = LaunchConfiguration("bounded_trial")
    localization_source = LaunchConfiguration("localization_source")
    # 两棵树按模式选，理由见 behavior_trees/navigate_to_pose_plan_only.xml：
    # Foxy 的 BtActionNode 构造时就 wait_for_action_server()，树里带 FollowPath
    # 而 controller_server 没起 ⇒ bt_navigator 的 on_configure 永久阻塞。
    # launch 里没有 IfElseSubstitution（Foxy 上 ImportError），用 PythonExpression。
    navigation_bt = PathJoinSubstitution(
        [
            share,
            "behavior_trees",
            PythonExpression(
                [
                    "'navigate_to_pose_bounded_trial.xml' if '",
                    bounded_trial,
                    "'.lower() == 'true' else ('navigate_to_pose_robot_scene.xml' if '",
                    execution_enabled,
                    "'.lower() == 'true' else 'navigate_to_pose_plan_only.xml')",
                ]
            ),
        ]
    )
    # 同一个 XML 也填给 default_nav_through_poses_bt_xml，**必须填**。
    # Humble 的 bt_navigator 在 on_activate 里会把两棵默认树都先解析一遍（为了尽早
    # 暴露错误）。只填 to_pose、把 through_poses 留在库存默认值上，那棵库存树就会
    # 成为真正的要求：它含 BackUp，而 behavior_plugins 为了不让机器人在走廊分析之外
    # 产生位移，刻意没有注册 backup。实测（2026-10-08，执行模式）：
    #   [bt_navigator]: "backup" action server not available after waiting for 1.00s
    #   Exception when loading BT: Action server backup not available
    #   Error loading XML file: .../navigate_through_poses_w_replanning_and_recovery.xml
    #   [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
    # 后果不止 bt_navigator：manager 遇错即中止，velocity_smoother / collision_monitor /
    # behavior_server 全停在 inactive，而 planner_server 与 controller_server 已是 active。
    # 本项目从不发 NavigateThroughPoses，这棵树只需要能被构造出来，所以复用同一棵按
    # 模式选出的树即可——两棵树都不含 BackUp / Spin 恢复分支。behavior_server
    # 仍由经过扫掠检查的后端显式调用 Spin，绝不由 BT 自主触发。
    # 只规划模式同理：那棵 navigate_to_pose_plan_only.xml 不带 FollowPath，正好也不会
    # 去等没起来的 controller_server。
    through_poses_bt = navigation_bt
    # Foxy's BT action nodes wait for Wait's server while constructing the
    # tree. Configure Recovery before BT; Humble keeps its existing sequence.
    execution_nodes = (["planner_server", "controller_server", "behavior_server",
                       "bt_navigator", "velocity_smoother", "collision_monitor"]
                       if os.environ.get("ROS_DISTRO") == "foxy" else EXECUTE_NODES)
    common = {"use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("execution_enabled", default_value="false"),
            DeclareLaunchArgument("bounded_trial", default_value="false"),
            DeclareLaunchArgument("nav2_cmd_vel_topic", default_value="/go2w/nav2_cmd_vel"),
            DeclareLaunchArgument("foxy_isolated_validation", default_value="false"),
            DeclareLaunchArgument("foxy_supervised_trial_ledger", default_value=""),
            OpaqueFunction(function=_foxy_execution_gate),
            DeclareLaunchArgument("localization_source", default_value="external_2d",
                                  choices=["external_2d", "legacy_plain_slam"]),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [share, "config", PythonExpression([
                        "'nav2_params_foxy_execution.yaml' if '", execution_enabled,
                        "'.lower() == 'true' else 'nav2_params_foxy.yaml'"])
                     if os.environ.get("ROS_DISTRO") == "foxy"
                     else "nav2_params_humble.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "velocity_smoother_params_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "velocity_smoother.yaml"]
                ),
            ),
            LogInfo(
                condition=UnlessCondition(execution_enabled),
                msg="GO2-W Nav2 只规划：execution_enabled=false。规划器与"
                    "bt_navigator 已启动（规划链路可查），控制器/平滑器/碰撞监视器"
                    "未启动，不会有速度进入机器人。",
            ),
            LogInfo(
                condition=IfCondition(execution_enabled),
                msg="GO2-W Nav2 执行链路已启动（execution_enabled=true）。真正的运动"
                    "授权仍在 go2w_cmd_vel_bridge 与 navigation_gate.yaml 里单独把关。",
            ),
            # Historical source only; never co-publish into the external 2D chain.
            Node(
                package="robot_scene_nav_bringup",
                executable="plain_slam_nav_frame_bridge.py",
                condition=IfCondition(PythonExpression(["'", localization_source, "' == 'legacy_plain_slam'"])),
                name="plain_slam_nav_frame_bridge",
                output="screen",
                parameters=[common],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file, common],
            ),
            Node(
                package=("go2w_nav2_controller_compat" if os.environ.get("ROS_DISTRO") == "foxy"
                         else "nav2_controller"),
                executable="controller_server",
                name="controller_server",
                output="screen",
                condition=IfCondition(execution_enabled),
                parameters=[params_file, common],
                remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                condition=IfCondition(execution_enabled),
                parameters=[smoother_params, common],
                remappings=[
                    ("cmd_vel", "/cmd_vel_nav_raw"),
                    ("cmd_vel_smoothed", "/cmd_vel_nav_smoothed"),
                ],
            ),
            Node(
                package="go2w_cmd_vel_bridge", executable="velocity_guard",
                name="go2w_navigation_velocity_guard", output="screen",
                condition=IfCondition(execution_enabled),
                # TF lookup in collision processing can block its executor.
                # Apply freshness/health/controller-stop checks AFTER it so
                # that a pending nonzero callback cannot delay final zero.
                parameters=[common, {
                    "input_topic": "/cmd_vel_nav_collision_checked",
                    "output_topic": LaunchConfiguration("nav2_cmd_vel_topic"),
                }],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                # 刻意不挂 execution_enabled：bt_navigator 自身不产生速度，
                # 只提供 /navigate_to_pose，两种模式都要在（见 PLAN_ONLY_NODES）。
                parameters=[
                    params_file,
                    common,
                    # BT XML 的绝对路径。这个参数在 Foxy 0.4.x 和 Humble 1.x 之间
                    # 改过名，两个都传：节点不认的那个只会成为一条未被声明的
                    # override 被忽略，各自认自己那一个。
                    # 传不中的后果不是报错，而是**静默回落到内建的
                    # navigate_to_pose_w_replanning_and_recovery.xml**——那份带
                    # BackUp / Spin 恢复分支，正是计划书 §3.1 要避开的。
                    # 只规划模式拿到的是不带 FollowPath 的那棵树，见上面 navigation_bt。
                    {
                        "default_bt_xml_filename": navigation_bt,            # Foxy 0.4.7
                        "default_nav_to_pose_bt_xml": navigation_bt,         # Humble 1.x
                        # 同上，理由见 through_poses_bt 上面那段注释。
                        "default_nav_through_poses_bt_xml": through_poses_bt,  # Humble 1.x
                    },
                ],
            ),
            Node(
                package="nav2_recoveries" if os.environ.get("ROS_DISTRO") == "foxy" else "nav2_behaviors",
                executable="recoveries_server" if os.environ.get("ROS_DISTRO") == "foxy" else "behavior_server",
                name="behavior_server",
                output="screen",
                condition=IfCondition(execution_enabled),
                parameters=[params_file, common],
                # Spin 默认发布 /cmd_vel；若不接入这条受保护链，它会运行到
                # time_allowance 后 abort，而机器人始终收不到角速度。
                remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(__file__).resolve().with_name("collision_monitor.launch.py"))
                ),
                condition=IfCondition(execution_enabled),
                launch_arguments={"use_sim_time": use_sim_time,
                                  "cmd_vel_out_topic": "/cmd_vel_nav_collision_checked"}.items(),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation_2d",
                output="screen",
                condition=IfCondition(execution_enabled),
                parameters=[common, {"autostart": True, "node_names": execution_nodes}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_plan_only",
                output="screen",
                condition=UnlessCondition(execution_enabled),
                parameters=[common, {"autostart": True, "node_names": PLAN_ONLY_NODES}],
            ),
        ]
    )
