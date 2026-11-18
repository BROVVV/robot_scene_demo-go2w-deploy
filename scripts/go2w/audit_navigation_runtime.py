#!/usr/bin/python3
"""Read-only map/sensor/TF/lifecycle/action audit; never authorizes motion."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault('GO2W_PROJECT_ROOT', str(ROOT))
sys.path.insert(0, str(ROOT))
from app.navigation.localization_health import (
    audit_expectations,
    authoritative_map_authorization,
    canonical_node_name,
    lifecycle_manager_owners,
)
spec = importlib.util.spec_from_file_location('go2w_localization_monitor', ROOT/'ros2_ws/src/robot_scene_nav_bringup/scripts/localization_health_monitor.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
import rclpy
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
try:
    from rclpy.parameter import parameter_value_to_python
except ImportError:  # Foxy exposes the conversion through Parameter only.
    from rcl_interfaces.msg import Parameter as ParameterMessage
    from rclpy.parameter import Parameter

    def parameter_value_to_python(value):
        result = Parameter.from_parameter_msg(
            ParameterMessage(name="audit", value=value)
        ).value
        return list(result) if value.type >= Parameter.Type.BYTE_ARRAY.value else result
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin as NativeSpin
if os.environ.get('ROS_DISTRO') == 'foxy':
    # The pinned Foxy recovery compatibility plugin exposes the same /spin
    # action name with the duration-capable project message.  Querying it with
    # nav2_msgs/Spin never matches DDS types and falsely reports no server.
    from go2w_nav2_compat_msgs.action import Spin as RuntimeSpin
else:
    RuntimeSpin = NativeSpin
from rclpy.action import ActionClient
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String

# 速度链每一级的输出话题与**唯一预期发布者**。只数 count==1 证明不了是谁发的：
# 上一轮遗留的孤儿节点、或者另一套并行起的 bringup，都会让 count 变成 2 而被
# 当成"链路不健康"，反过来一个冒名顶替的发布者也会让 count 恰好等于 1。
# 所以这里既报 count 也报节点名，并逐级核对是不是预期的那个节点。
# 链路：controller_server -> /cmd_vel_nav_raw -> velocity_smoother
#       -> /cmd_vel_nav_smoothed -> collision_monitor
#       -> /cmd_vel_nav_collision_checked -> go2w_navigation_velocity_guard -> nav2_cmd_vel_topic
VELOCITY_STAGES = {
    # Nav2 的 controller 与显式 Spin 行为共用标准速度入口；两者由 action
    # 状态机互斥调用。只核对身份集合，不把同一 behavior_server 内多个插件
    # 建出的同名 publisher endpoint 误报成第二个节点。
    '/cmd_vel_nav_raw': ('controller_server', 'behavior_server'),
    '/cmd_vel_nav_smoothed': ('velocity_smoother',),
    '/cmd_vel_nav_collision_checked': ('collision_monitor',),
    '/go2w/cmd_vel_selected': ('go2w_control_arbiter',),
}
# 物理入口话题是写死的常量，不随 --nav2-cmd-topic 变：验收用的隔离链路会把
# 最终速度保护的输出改到 /go2w/acceptance/... 上，那时**物理入口必须
# 一个发布者都没有**。这两件事要分开报，否则"隔离跑通"会被读成"物理链已通"。
PHYSICAL_CMD_VEL_TOPIC = '/go2w/nav2_cmd_vel'
BRIDGE_STATUS_TOPIC = '/go2w/cmd_vel_bridge/status'


def _receive_live_status(node, samples):
    """Refresh after graph/file checks, before applying the existing age gate."""
    previous=len(samples)
    deadline=time.monotonic()+1.5
    while len(samples)==previous and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
    return samples[-1] if samples else {}

# bridge 的八条闸门输入。桥自己 fail-closed（每个标志的初值都是"拒绝"），所以
# 只看 status 串里的 blocker 名字分不清两种完全不同的处境：
#   (1) 有人**发布**了拒绝值（例如急停真的被按下）；
#   (2) 根本**没人发布**，桥停在初始的拒绝值上。
# 这两种的现场处置完全不同（去按/去放 vs 去把发布者起起来），所以这里既读值
# 也报发布者身份。
SAFETY_FLAG_TOPICS = [
    '/go2w/software_arm', '/go2w/sport_lease/alive', '/go2w/safety/lidar_fresh',
    '/go2w/safety/rotation_clearance_valid', '/go2w/nav/localization_valid',
    '/go2w/robot_error_zero', '/go2w/emergency_stop', '/go2w/remote_override',
]
#: 期望是 True 才算"允许"的标志。其余（software_arm / remote_override）在这个
#: 项目里默认就是 False，True 反而是有人接管了控制权。
SAFETY_FLAG_EXPECTED_TRUE = frozenset({
    '/go2w/sport_lease/alive', '/go2w/safety/lidar_fresh',
    '/go2w/nav/localization_valid', '/go2w/robot_error_zero',
})


def _topic_publishers(node, topic):
    """返回 {topic, count, publishers:[节点名], expected, matches_expected}。

    `get_publishers_info_by_topic` 在话题类型还没发现时会抛，按"没发现"处理。
    """
    try:
        infos = node.get_publishers_info_by_topic(topic)
    except Exception:
        infos = []
    names = sorted({(info.node_namespace.rstrip('/') + '/' + info.node_name) for info in infos})
    return {'topic': topic, 'count': len(infos), 'publishers': names}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--json', required=True)
    parser.add_argument('--require-execution', action='store_true')
    parser.add_argument('--trial-ledger', default='')
    parser.add_argument('--nav2-cmd-topic', default='/go2w/nav2_cmd_vel')
    # 计划书 §P4-5：审计原来把 mode 写死在 mapping（LocalizationHealthMonitor
    # 的 mode 参数默认值），于是拿它出的报告去声称"localization 模式验收过了"
    # 是**口径错**的：判据整套按 mapping 跑（不看 /amcl_pose、不看协方差、
    # 不看图授权），生命周期也查的是 planner/bt 而不是 map_server/amcl。
    # 所以 mode 必须是调用方显式给的，且**落进报告本身**（见下面的 audit_mode），
    # 让"这份报告验的是哪个模式"没法靠记忆蒙。
    parser.add_argument('--mode', choices=('mapping', 'localization'), default='mapping')
    args = parser.parse_args()
    rclpy.init()
    node = module.LocalizationHealthMonitor(publish=False, mode=args.mode)
    # 口径本身放在 app/navigation/localization_health.py（纯函数，离线可测）：
    # 这个脚本 import 时就 import rclpy，没起 ROS 的机器上进不来。
    plan = audit_expectations(args.mode, require_execution=args.require_execution)
    expected = plan['lifecycle_nodes']
    futures, clients = {}, {}
    for name in expected:
        clients[name] = node.create_client(GetState, f'/{name}/get_state')
    parameter_clients, parameter_futures = {}, {}
    mapping_parameters = {
        '/slam_toolbox': ['scan_topic', 'continuous_scan_topic', 'minimum_travel_distance',
                          'mapping_use_keyframes', 'throttle_scans',
                          'direct_scan_minimum_interval_s', 'minimum_time_interval',
                          'map_update_interval'],
        '/go2w_lidar_preprocessor': ['mapping_accumulation'],
    } if args.mode == 'mapping' else {}
    profile_path=ROOT/'configs/go2w/navigation_sensor_profile.yaml'
    mapping_parameters.setdefault('/go2w_cmd_vel_bridge',[]).extend([
        'geometry_execution_scope','geometry_profile_sha256','operator_supervised_rotation'])
    geometry_status=[]
    floor_status=[]
    if profile_path.exists():
        mapping_parameters.setdefault('/go2w_lidar_preprocessor',[]).append('sensor_profile')
        mapping_parameters['/go2w_official_sensor_frame_publisher']=['robot_description']
        def on_geometry(message):
            try:geometry_status.append(json.loads(message.data))
            except (ValueError,TypeError):pass
        node.create_subscription(String,'/go2w/lidar/geometry_status',on_geometry,1)
        def on_floor_status(message):
            try:floor_status.append(json.loads(message.data))
            except (ValueError,TypeError):pass
        node.create_subscription(String,'/go2w/nav/floor_visibility_status',on_floor_status,1)
    if args.trial_ledger:
        mapping_parameters['/go2w_cmd_vel_bridge'].extend(['trial_ledger', 'execution_enabled'])
        if args.require_execution:
            mapping_parameters['/controller_server'] = [
                'general_goal_checker.xy_goal_tolerance',
                'general_goal_checker.yaw_goal_tolerance',
                'general_goal_checker.stateful',
                'progress_checker.required_movement_radius',
            ]
    for name in mapping_parameters:
        parameter_clients[name] = node.create_client(GetParameters, name+'/get_parameters')
    actions = {'compute_path_to_pose': ActionClient(node, ComputePathToPose, '/compute_path_to_pose'),
               'navigate_to_pose': ActionClient(node, NavigateToPose, '/navigate_to_pose')}
    # 精确原地转向不再伪装成零长度 NavigateToPose。执行态必须把原生 Spin
    # 服务器也纳入启动审计；只读规划态不启动 behavior_server，因此不强求。
    if args.require_execution:
        actions['spin'] = ActionClient(node, RuntimeSpin, '/spin')
    # 桥的状态串是"物理执行到底开没开"最直接的证据：它按 tick 发最近一次的
    # 闸门结论，执行没开时里面一定有 execution_disabled。只看图上"物理 cmd_vel
    # 没有发布者"是不够的 —— 那种写法在 bringup 没起来时同样是 0，看着一样。
    bridge_reasons = []
    travel_status = []
    localization_status = []
    def on_localization_status(message):
        try:
            item=json.loads(message.data)
            if isinstance(item,dict):localization_status.append(item)
        except (ValueError,TypeError):pass
    node.create_subscription(String,'/go2w/nav/localization_status',on_localization_status,1)
    def on_travel_status(message):
        try:
            item=json.loads(message.data)
            if isinstance(item,dict):travel_status.append(item)
        except (ValueError,TypeError):pass
    node.create_subscription(String,'/go2w/nav/goal_travel/status',on_travel_status,1)
    node.create_subscription(String, BRIDGE_STATUS_TOPIC,
                             lambda message: bridge_reasons.append(message.data),
                             QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                                        reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.VOLATILE))
    flag_values = {topic: [] for topic in SAFETY_FLAG_TOPICS}
    for topic in SAFETY_FLAG_TOPICS:
        # durability 用 VOLATILE 而不是 TRANSIENT_LOCAL。DDS 要求订阅端申请的
        # durability **不高于**发布端提供的：申请 TRANSIENT_LOCAL 就只配得上
        # TRANSIENT_LOCAL 的发布者，一旦某个标志的发布者用 VOLATILE（连续值，
        # 实测 /go2w/sport_lease/alive 等四条就是），两边根本配不上对，
        # 症状是**静默收不到任何消息**（value=None、samples=0），而不是报错。
        # 申请 VOLATILE 则两种发布者都配得上；代价是拿不到"锁存值"的补发，
        # 只能等下一次发布 —— 所以下面等满整个 duration 再收尾。
        node.create_subscription(
            Bool, topic,
            (lambda name: lambda message: flag_values[name].append(bool(message.data)))(topic),
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
    end = time.monotonic() + max(3, args.duration)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
        for name, client in clients.items():
            if name not in futures and client.service_is_ready():
                futures[name] = client.call_async(GetState.Request())
        for name, client in parameter_clients.items():
            if name not in parameter_futures and client.service_is_ready():
                request = GetParameters.Request(names=mapping_parameters[name])
                parameter_futures[name] = client.call_async(request)
    report = node.report()
    localization_status_publishers = _topic_publishers(
        node, '/go2w/nav/localization_status')['publishers']
    authoritative_status = localization_status[-1] if localization_status else {}
    map_authorized = authoritative_map_authorization(
        authoritative_status, localization_status_publishers, mode=args.mode)
    if args.mode == 'localization' and map_authorized:
        # The audit sampler's own parameter is deliberately false; replace
        # only this one check with evidence from the unique live health owner.
        report.setdefault('checks', {})['map_authorized'] = True
        report.setdefault('evidence', {})['map_authorized'] = True
        report['blocking'] = [reason for reason in report.get('blocking', [])
                              if reason != 'map_authorized']
        report['valid'] = not report['blocking']
    lifecycle = {}
    for name in expected:
        future = futures.get(name)
        try:
            lifecycle[name] = future.result().current_state.label if future and future.done() else 'unavailable'
        except Exception:
            lifecycle[name] = 'error'
    pub_counts = {topic: node.count_publishers(topic) for topic in (
        args.nav2_cmd_topic,
        '/go2w/nav/localization_valid', '/go2w/nav/tf_authorities')}
    stages = {}
    for topic, expected_publishers in VELOCITY_STAGES.items():
        stages[topic] = _topic_publishers(node, topic)
        expected = sorted('/' + name for name in expected_publishers)
        stages[topic]['expected'] = expected
        stages[topic]['matches_expected'] = stages[topic]['publishers'] == expected
    isolated_output = _topic_publishers(node, args.nav2_cmd_topic)
    isolated_output['expected'] = 'go2w_navigation_velocity_guard'
    isolated_output['matches_expected'] = isolated_output['publishers'] == ['/go2w_navigation_velocity_guard']
    physical = _topic_publishers(node, PHYSICAL_CMD_VEL_TOPIC)
    # 物理执行关没关：物理入口没有发布者是一个**充分不必要**的证据 —— 充分是
    # 因为没人发速度就不可能动；不必要是因为即使在 enable-motion 下正常起，
    # 物理入口也会有最终速度保护这一个发布者，实体执行闸门仍在 bridge 里。
    # 所以三条一起报，谁都不能单独被当成"已授权"。
    physical_disabled = physical['count'] == 0
    bridge_blocks = sorted({reason for item in bridge_reasons for reason in item.split(',') if reason})
    physical_disabled_by_bridge = 'execution_disabled' in bridge_blocks
    safety_flags = {}
    for topic in SAFETY_FLAG_TOPICS:
        values = flag_values[topic]
        publishers = _topic_publishers(node, topic)['publishers']
        safety_flags[topic] = {
            'value': values[-1] if values else None,
            'samples': len(values),
            'publishers': publishers,
            'expected_true': topic in SAFETY_FLAG_EXPECTED_TRUE,
            # 只报事实，不下"正常/异常"的判断：software_arm=false 是常态，
            # rotation_clearance_valid=false 也是常态（软件上开不了持久门）。
            'unpublished': not publishers,
        }
    # 同名节点出现两次是这套链路最典型的坏法，而上面所有按**名字**做的检查
    # （get_state 服务、action server、生命周期）对它完全免疫：两个
    # lifecycle_manager 管同一批节点时，服务名只有一个，怎么查都显示"唯一"。
    # 实测：2026-10-08 起就一直挂着一个 stock 之外遗留的
    # `lifecycle_manager_navigation`（由当时那版 launch 起），和新的
    # `lifecycle_manager_navigation_2d` 并存，所有按名字的检查一路绿灯。
    # 所以这里直接枚举图上的节点，把重名的点出来。
    node_names = [canonical_node_name(name, ns)
                  for name, ns in node.get_node_names_and_namespaces()]
    seen, duplicates = set(), {}
    for full in sorted(node_names):
        if full in seen:
            duplicates.setdefault(full, 1)
            duplicates[full] += 1
        seen.add(full)
    lifecycle_managers = lifecycle_manager_owners(node_names)
    servers = {name: client.server_is_ready() for name, client in actions.items()}
    blockers = list(report.get('blocking', ['no_samples']))
    mapping_frontend = {'parameters': {}, 'publishers': {}}
    if args.mode == 'mapping':
        for name, names in mapping_parameters.items():
            future = parameter_futures.get(name)
            try:
                values = future.result().values if future and future.done() else []
                mapping_frontend['parameters'][name] = dict(zip(names, map(parameter_value_to_python, values)))
            except Exception:
                mapping_frontend['parameters'][name] = {}
        for topic in ('/go2w/lidar/mapping_scan', '/go2w/lidar/mapping_keyframe_scan'):
            info = _topic_publishers(node, topic)
            mapping_frontend['publishers'][topic] = info
            if (topic == '/go2w/lidar/mapping_scan'
                    and (info['count'] != 1
                         or info['publishers'] != ['/go2w_lidar_preprocessor'])):
                blockers.append('mapping_frontend:publisher:'+topic)
        slam = mapping_frontend['parameters'].get('/slam_toolbox', {})
        frontend = mapping_frontend['parameters'].get('/go2w_lidar_preprocessor', {})
        mapping_frontend['configuration_valid'] = bool(
            slam.get('scan_topic') == '/go2w/lidar/mapping_scan'
            and slam.get('continuous_scan_topic') == '/go2w/lidar/mapping_scan'
            and slam.get('mapping_use_keyframes') is False
            and slam.get('throttle_scans') == 1
            # The GO2-W mapping host cannot sustain scan matching at the raw
            # ~17 Hz LiDAR cadence: it accumulated 1--12 s of source-time lag
            # and made the localization gate flap.  The validated 4 Hz
            # admission interval keeps TF below the 0.3 s freshness limit
            # while leaving the high-rate safety scan untouched.
            and slam.get('direct_scan_minimum_interval_s') == .25
            and slam.get('minimum_time_interval') == 0.
            and slam.get('map_update_interval') == .2
            and slam.get('minimum_travel_distance') == 0.
            and frontend.get('mapping_accumulation') is True)
        if not mapping_frontend['configuration_valid']:
            blockers.append('mapping_frontend:configuration')
    blockers += ['lifecycle:'+name for name, state in lifecycle.items() if state != 'active']
    blockers += ['action:'+name for name, active in servers.items() if not active]
    # 一条链路只该有一个 lifecycle_manager。多于一个就是上面说的"按名字查不出来"
    # 的那类问题，必须让 healthy_for_plan 变红，而不是静静躺在报告里。
    # localization 模式下正常就是**两个**（localization_2d 自己的 + 导航栈的
    # plan_only），所以口径按模式分开（见 audit_expectations）；同时点名要求
    # localization 那个必须在，否则"两个"可能只是两套导航栈并存 —— 那正是这条
    # 规则要抓的坏法。
    for manager in plan['required_managers']:
        if manager not in lifecycle_managers:
            blockers += ['lifecycle_manager_missing:' + manager]
    if not lifecycle_managers:
        blockers += ['lifecycle_manager_count:0']
    elif len(lifecycle_managers) > plan['max_lifecycle_managers']:
        blockers += ['lifecycle_manager_count:%d' % len(lifecycle_managers)]
    if args.require_execution:
        motion_nodes = {'go2w_motion_action_server','go2w_motion_state_monitor',
                        'go2w_motion_local_gateway','go2w_motion_network_proxy'}
        blockers += ['duplicate_motion_node:'+name for name in duplicates
                     if name.rsplit('/',1)[-1] in motion_nodes]
        # 逐级核对发布者**身份**，不再只看 count==1（见 VELOCITY_STAGES 注释）。
        blockers += ['publisher:'+topic for topic, item in stages.items()
                     if not item['matches_expected']]
        blockers += ['publisher:'+args.nav2_cmd_topic] if not isolated_output['matches_expected'] else []
    services=dict(node.get_service_names_and_types())
    arbiter_reset_available='std_srvs/srv/Trigger' in services.get(
        '/go2w/control_arbiter/reset_emergency_stop',[])
    if args.require_execution and not arbiter_reset_available:
        blockers.append('service:control_arbiter_explicit_reset_missing')
    travel_publishers = _topic_publishers(node,'/go2w/nav/goal_travel/status')
    travel = travel_status[-1] if travel_status else {}
    travel_valid = (travel_publishers['count']==1
        and travel_publishers['publishers']==['/go2w_cmd_vel_bridge']
        and travel.get('version')==1 and bool(travel.get('epoch'))
        and travel.get('ready') is True and travel.get('token') is None
        and travel.get('phase')=='idle' and not travel.get('reason')
        and -.05<=time.time()-float(travel.get('stamp',0))<=.5)
    if args.require_execution and not args.trial_ledger and not travel_valid:
        blockers.append('goal_travel:independent_wheel_guard_not_ready')
    report.update(
        goal_travel=dict(valid=travel_valid,status=travel,publishers=travel_publishers),
        mapping_frontend=mapping_frontend,
        lifecycle=lifecycle, action_servers=servers, velocity_publishers=pub_counts,
        velocity_stages=stages, nav2_cmd_vel_output=isolated_output,
        lifecycle_managers=lifecycle_managers, duplicate_node_names=sorted(duplicates),
        localization_authorization={
            'accepted': map_authorized,
            'publishers': localization_status_publishers,
            'status_mode': authoritative_status.get('mode'),
            'status_valid': authoritative_status.get('valid'),
        },
        physical_cmd_vel=physical,
        arbiter_explicit_reset_service_available=arbiter_reset_available,
        physical_execution={
            'topic': PHYSICAL_CMD_VEL_TOPIC,
            'no_publisher': physical_disabled,
            'publishers': physical['publishers'],
            'isolated_output_topic': args.nav2_cmd_topic,
            'isolated_output_is_separate': args.nav2_cmd_topic != PHYSICAL_CMD_VEL_TOPIC,
            'bridge_status_reasons': bridge_blocks,
            'bridge_reports_execution_disabled': physical_disabled_by_bridge,
            'safety_flags': safety_flags,
            'bridge_status_topic': BRIDGE_STATUS_TOPIC,
            'verdict': ('disabled' if (physical_disabled or physical_disabled_by_bridge) else 'unknown'),
            'note': ('物理入口无发布者 与 桥自报 execution_disabled 都是"关着"的证据，'
                     '但两者都不等于"已授权执行"：本审计只读，从不授权运动。'),
        },
        # `healthy_for_plan` 只描述**图上的规划链路**是否齐备，与能否执行无关。
        # 旧名字容易被读成"可以动了"，这里把口径写进 JSON 本身。
        healthy_for_plan_scope='planning graph only; never an execution permission',
        healthy_for_plan=not blockers, plan_blocking_errors=blockers,
        # 这份报告验的是哪个模式，写进报告里。消费方（存图器的 provenance、
        # 验收报告）要引用它，就必须引用这一个字段，而不是自己记。
        audit_mode=args.mode,
        motion_authorized=False)
    if args.trial_ledger:
        trial_parameters = {}
        for name in ('/go2w_cmd_vel_bridge', '/controller_server'):
            future = parameter_futures.get(name)
            try:
                values = future.result().values if future is not None and future.done() and future.result() else []
            except Exception:
                values = []
            trial_parameters[name] = dict(zip(mapping_parameters.get(name, []),
                map(parameter_value_to_python, values)))
        bridge = trial_parameters['/go2w_cmd_vel_bridge']
        controller = trial_parameters['/controller_server']
        valid = (bridge.get('trial_ledger') == str(Path(args.trial_ledger).resolve())
            and bridge.get('execution_enabled') is args.require_execution)
        if args.require_execution:
            valid = valid and all(controller.get(key) == expected for key, expected in (
                ('general_goal_checker.xy_goal_tolerance', .03),
                ('general_goal_checker.yaw_goal_tolerance', .05),
                ('general_goal_checker.stateful', False),
                ('progress_checker.required_movement_radius', .02)))
        report['trial_budget_profile'] = dict(configuration_valid=valid, parameters=trial_parameters,
            motion_authorized=False)
        if not valid:
            blockers.append('trial_budget:configuration')
            report['healthy_for_plan'] = False
    if profile_path.exists():
        from go2w_description.sensor_profile import load_sensor_profile
        from go2w_description.description_config import load_official_reference,render_official_sensor_urdf
        try:
            profile=load_sensor_profile(profile_path)
            reference=load_official_reference(str(ROOT/'configs/go2w/official_reference.yaml'))
            expected_xml=render_official_sensor_urdf(reference,profile)
            runtime={}
            for name in ('/go2w_lidar_preprocessor','/go2w_official_sensor_frame_publisher'):
                future=parameter_futures.get(name)
                values=future.result().values if future and future.done() else []
                runtime[name]=dict(zip(mapping_parameters[name],map(parameter_value_to_python,values)))
            status=_receive_live_status(node,geometry_status)
            valid=(runtime['/go2w_lidar_preprocessor'].get('sensor_profile')==str(profile_path)
                and runtime['/go2w_official_sensor_frame_publisher'].get('robot_description')==expected_xml
                and status.get('profile_sha256')==profile['profile_sha256']
                and status.get('posture_valid') is True
                and -.1<=time.time()-float(status.get('wall_time',0))<=.3)
            report['sensor_geometry']=dict(valid=valid,profile_sha256=profile['profile_sha256'],
                execution_scope=profile['execution_scope'],status=status)
            if not valid:blockers.append('sensor_geometry:runtime_mismatch')
            future=parameter_futures.get('/go2w_cmd_vel_bridge')
            values=future.result().values if future and future.done() and future.result() else []
            bridge_geometry=dict(zip(mapping_parameters['/go2w_cmd_vel_bridge'],map(parameter_value_to_python,values)))
            report['sensor_geometry']['bridge_binding']=bridge_geometry
            if args.require_execution:
                if (bridge_geometry.get('geometry_execution_scope') != profile['execution_scope']
                        or bridge_geometry.get('geometry_profile_sha256') != profile['profile_sha256']):
                    blockers.append('sensor_geometry:bridge_binding_mismatch')
                if not args.trial_ledger and (profile['execution_scope']!='operator_supervised_field'
                        or bridge_geometry.get('operator_supervised_rotation') is not True):
                    blockers.append('sensor_geometry:supervised_field_required')
            if args.mode=='mapping' and profile.get('raw_floor_plane'):
                rays_2d=os.environ.get('GO2W_PERCEPTION_MODE')=='2d'
                floor_topic=('/go2w/lidar/floor_visibility_2d' if rays_2d
                             else '/go2w/lidar/floor_observations')
                owners={topic:_topic_publishers(node,topic) for topic in (
                    '/map','/go2w/slam/map_raw',floor_topic)}
                expected={'/map':['/go2w_observed_floor_map'],
                    '/go2w/slam/map_raw':['/slam_toolbox'],
                    floor_topic:['/go2w_lidar_preprocessor']}
                fs=_receive_live_status(node,floor_status)
                floor_ok=(all(owners[k]['count']==1 and owners[k]['publishers']==v for k,v in expected.items())
                    and (not rays_2d or fs.get('input_mode')=='rays_2d')
                    and fs.get('rays',0)>0 and fs.get('occupied_cells_removed')==0
                    and -.1<=time.time()-float(fs.get('wall_time',0))<=2.)
                report['floor_visibility']=dict(valid=floor_ok,publishers=owners,status=fs)
                if not floor_ok:blockers.append('floor_visibility:runtime_mismatch')
        except Exception as exc:
            blockers.append('sensor_geometry:'+str(exc))
        report['healthy_for_plan']=not blockers
    path = Path(args.json)
    report['audit_completed_at']=time.time()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: report[k] for k in (
        'healthy_for_plan', 'healthy_for_plan_scope', 'plan_blocking_errors', 'lifecycle',
        'lifecycle_managers', 'duplicate_node_names', 'action_servers', 'velocity_stages',
        'nav2_cmd_vel_output', 'physical_execution', 'audit_mode')}))
    for action_client in actions.values():
        action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not blockers else 2

if __name__ == '__main__':
    raise SystemExit(main())
