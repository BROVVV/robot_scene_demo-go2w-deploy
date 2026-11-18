#!/usr/bin/python3
"""Synthetic velocity-chain test, confined to domain 99 and loopback DDS.

No physical bridge, SDK, lease holder, controller or navigation action is
started. Output silence after a process fault requires a downstream watchdog;
this probe does not claim physical stopping acceptance.
"""
from pathlib import Path
import argparse
import atexit
import json
import math
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET


def isolated_environment():
    if os.environ.get('ROS_DOMAIN_ID') != '99' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('domain 99 and localhost-only are required')
    uri = os.environ.get('CYCLONEDDS_URI', '')
    if not uri.startswith('file://'):
        raise RuntimeError('explicit loopback CycloneDDS configuration required')
    tree = ET.parse(uri[7:])
    addresses = [node.text for node in tree.iter('NetworkInterfaceAddress')]
    if (addresses != ['127.0.0.1']
            or [n.text for n in tree.iter('AllowMulticast')] != ['false']
            or any(n.get('Address') != '127.0.0.1' for n in tree.iter('Peer'))):
        raise RuntimeError('DDS must bind only 127.0.0.1')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        raise RuntimeError('the inspected CycloneDDS configuration must be used')


def main():
    isolated_environment()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--with-controller', action='store_true')
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[3]
    import yaml
    import rclpy
    from rclpy.duration import Duration
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.msg import Transition

    config = root / 'ros2_ws/src/robot_scene_nav_bringup/config'
    smoother = yaml.safe_load((config/'velocity_smoother.yaml').read_text())['velocity_smoother']['ros__parameters']
    collision = yaml.safe_load((config/'collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
    prefix = '/go2w/acceptance/'
    collision.update(cmd_vel_in_topic=prefix+'smoothed', cmd_vel_out_topic=prefix+'collision_checked')
    collision['scan']['topic'] = prefix+'synthetic_scan'
    params = out/'parameters.yaml'
    params.write_text(yaml.safe_dump({
        'acceptance_velocity_smoother': {'ros__parameters': smoother},
        'acceptance_collision_monitor': {'ros__parameters': collision}}))
    processes = {}
    executables = {}
    logs = []
    def cleanup_processes():
        for process in processes.values():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes.values():
            try: process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5.)
        for stream in logs: stream.close()
    # Also covers dependency/configuration errors before the ROS fixture is
    # initialized; a failed probe must not leave test publishers behind.
    atexit.register(cleanup_processes)
    def spawn(key, package, executable, extra):
        binary = Path(get_package_prefix(package))/'lib'/package/executable
        stream = (out/(key+'.log')).open('w'); logs.append(stream)
        processes[key] = subprocess.Popen([str(binary), '--ros-args', '-r', '__node:=acceptance_'+key,
            *extra], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        executables[key] = {'package':package,'binary':str(binary),'pid':processes[key].pid}
    spawn('velocity_smoother', 'nav2_velocity_smoother', 'velocity_smoother',
          ['--params-file', str(params), '-r', 'cmd_vel:='+prefix+'raw', '-r', 'cmd_vel_smoothed:='+prefix+'smoothed'])
    spawn('collision_monitor', 'nav2_collision_monitor', 'collision_monitor', ['--params-file', str(params)])
    spawn('velocity_guard', 'go2w_cmd_vel_bridge', 'velocity_guard', [
        '-p', 'input_topic:='+prefix+'collision_checked', '-p', 'raw_topic:='+prefix+'raw',
        '-p', 'output_topic:='+prefix+'filtered', '-p', 'scan_topic:='+prefix+'synthetic_scan',
        '-p', 'require_localization:=true', '-r', '/go2w/nav/localization_status:='+prefix+'localization_status'])
    rclpy.init()
    node = rclpy.create_node('go2w_foxy_velocity_chain_probe')
    raw = node.create_publisher(Twist, prefix+'raw', 10)
    scans = node.create_publisher(LaserScan, prefix+'synthetic_scan', 10)
    health = node.create_publisher(String, prefix+'localization_status', 10)
    tf = TransformBroadcaster(node)
    odom = node.create_publisher(Odometry, prefix+'synthetic_odom', 10)
    received = []
    node.create_subscription(Twist, prefix+'filtered',
        lambda m: received.append((time.monotonic(), float(m.linear.x), float(m.angular.z))), 100)
    report = {'scope': 'synthetic controller chain' if args.with_controller else 'synthetic component chain only', 'physical_motion_enabled': False,
              'domain_id': 99, 'localization_guard_required': True,
              'executables':executables, 'cases': {}, 'passed': False}
    def phase(duration, *, obstacle=None, raw_enabled=True, scans_enabled=True,
              health_enabled=True, valid=True, speed=.12, scan_offset=0.):
        started = time.monotonic(); next_send = started
        while time.monotonic()-started < duration:
            if time.monotonic() >= next_send:
                stamp = node.get_clock().now().to_msg()
                transform = TransformStamped(); transform.header.stamp = stamp
                transform.header.frame_id = 'odom_fused'; transform.child_frame_id = 'base_link'
                transform.transform.rotation.w = 1.; tf.sendTransform(transform)
                state = Odometry(); state.header.stamp = stamp
                state.header.frame_id = 'odom_fused'; state.child_frame_id = 'base_link'
                state.pose.pose.orientation.w = 1.; odom.publish(state)
                if raw_enabled and raw is not None:
                    command = Twist(); command.linear.x = speed; raw.publish(command)
                if health_enabled:
                    health.publish(String(data=json.dumps({'valid': valid, 'wall_time': time.time()})))
                if scans_enabled:
                    scan = LaserScan(); scan.header.frame_id = 'base_link'
                    scan.header.stamp = (node.get_clock().now()-Duration(seconds=scan_offset)).to_msg()
                    scan.angle_min = -math.pi; scan.angle_increment = math.pi/360
                    scan.angle_max = scan.angle_min+719*scan.angle_increment
                    scan.range_min = .1; scan.range_max = 6.2; scan.ranges = [6.]*720
                    if obstacle is not None:
                        for index in range(352,369): scan.ranges[index] = obstacle
                    scans.publish(scan)
                next_send = time.monotonic()+.025
            rclpy.spin_once(node, timeout_sec=.003)
        tail = [row for row in received if row[0] > started+max(0., duration-.25)]
        zero_delays = [row[0]-started for row in received
                       if row[0] >= started and abs(row[1])+abs(row[2]) < 1e-8]
        return {'samples': len(tail), 'tail_x': [row[1] for row in tail],
                'tail_w': [row[2] for row in tail],
                'first_zero_s': min(zero_delays) if zero_delays else None}
    def stopped(result):
        return result['samples'] > 0 and all(abs(v)<1e-8 for v in result['tail_x']+result['tail_w'])
    def lifecycle(name, transition):
        client = node.create_client(ChangeState, '/acceptance_'+name+'/change_state')
        try:
            assert client.wait_for_service(timeout_sec=10.), 'lifecycle unavailable: '+name
            request = ChangeState.Request(); request.transition.id = transition
            future = client.call_async(request)
            end = time.monotonic()+10.
            while not future.done() and time.monotonic()<end:
                phase(.04, speed=0.)
            assert future.done() and future.result().success, 'lifecycle failed: '+name
        finally: node.destroy_client(client)
    try:
        for name in ('velocity_smoother','collision_monitor'):
            lifecycle(name, Transition.TRANSITION_CONFIGURE)
            lifecycle(name, Transition.TRANSITION_ACTIVATE)
        phase(.6, speed=0.)
        for topic in ('/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected','/go2w/software_arm'):
            assert node.count_publishers(topic)==0, 'Unexpected physical topic: '+topic
        if args.with_controller:
            from rclpy.action import ActionClient
            from nav2_msgs.action import FollowPath
            from nav_msgs.msg import Path as RosPath
            from geometry_msgs.msg import PoseStamped
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            # No synthetic raw publisher coexists with the real controller.
            node.destroy_publisher(raw); raw = None
            # Match the actual Foxy production compatibility controller and
            # parameter schema; the stock Foxy server is a different binary.
            controller_config = yaml.safe_load((config/'nav2_params_foxy_execution.yaml').read_text())
            controller = controller_config['controller_server']['ros__parameters']
            controller['odom_topic'] = prefix+'synthetic_odom'
            follow = controller['FollowPath']
            for key in ('desired_linear_vel','min_approach_linear_velocity','regulated_linear_scaling_min_speed'):
                follow[key] = .12
            local = controller_config['local_costmap']
            local['local_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = prefix+'synthetic_scan'
            controller_params = out/'controller_parameters.yaml'
            controller_params.write_text(yaml.safe_dump({
                'acceptance_controller': {'ros__parameters': controller}, 'local_costmap':local}))
            spawn('controller', 'go2w_nav2_controller_compat', 'controller_server',
                  ['--params-file', str(controller_params), '-r', 'cmd_vel:='+prefix+'raw'])
            lifecycle('controller', Transition.TRANSITION_CONFIGURE)
            lifecycle('controller', Transition.TRANSITION_ACTIVATE)
            action = ActionClient(node, FollowPath, '/follow_path')
            parameters = node.create_client(SetParameters, '/acceptance_controller/set_parameters')
            def await_future(future, timeout=5.):
                deadline = time.monotonic()+timeout
                while not future.done() and time.monotonic()<deadline:
                    phase(.04, raw_enabled=False)
                assert future.done(), 'controller response timeout'
                return future.result()
            def start_path():
                deadline = time.monotonic()+5
                while not action.server_is_ready() and time.monotonic()<deadline:
                    phase(.05, raw_enabled=False)
                assert action.server_is_ready()
                goal = FollowPath.Goal(); goal.controller_id = 'FollowPath'
                goal.path = RosPath(); goal.path.header.frame_id = 'odom_fused'
                goal.path.header.stamp = node.get_clock().now().to_msg()
                for index in range(21):
                    pose = PoseStamped(); pose.header = goal.path.header
                    pose.pose.position.x = index*.05; pose.pose.orientation.w = 1.
                    goal.path.poses.append(pose)
                handle = await_future(action.send_goal_async(goal))
                assert handle and handle.accepted
                return handle
            handle = start_path()
            result = phase(1.5, raw_enabled=False)
            report['cases']['controller_output'] = result
            assert result['samples'] and min(result['tail_x'])>.11, result
            for speed in (.07, .12):
                request = SetParameters.Request()
                request.parameters = [Parameter(name='FollowPath.desired_linear_vel',
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=speed))]
                reply = await_future(parameters.call_async(request))
                assert len(reply.results)==1 and reply.results[0].successful
                result = phase(.8, raw_enabled=False)
                report['cases']['controller_speed_'+str(speed)] = result
                assert result['samples'] and all(abs(v-speed)<.005 for v in result['tail_x']), result
            cancel_started = time.monotonic()
            await_future(handle.cancel_goal_async())
            result = phase(.6, raw_enabled=False)
            zeros = [row[0]-cancel_started for row in received if row[0]>=cancel_started
                     and abs(row[1])+abs(row[2])<1e-8]
            result['first_zero_s'] = min(zeros) if zeros else None
            outcome = await_future(handle.get_result_async())
            report['cases']['controller_cancel'] = {**result, 'action_status':outcome.status}
            assert outcome.status==5 and stopped(result), (outcome.status,result)
            assert result['first_zero_s'] is not None and result['first_zero_s']<=.3, result
            handle = start_path()
            result = phase(1., raw_enabled=False)
            assert result['samples'] and min(result['tail_x'])>.11, result
            fault_started = time.monotonic()
            processes['controller'].kill(); processes['controller'].wait(timeout=5.)
            result = phase(.75, raw_enabled=False)
            zeros = [row[0]-fault_started for row in received if row[0]>=fault_started
                     and abs(row[1])+abs(row[2])<1e-8]
            result['first_zero_s'] = min(zeros) if zeros else None
            report['cases']['controller_exit'] = result
            assert stopped(result) and result['first_zero_s'] is not None and result['first_zero_s']<=.4, result
            action.destroy(); node.destroy_client(parameters)
            raw = node.create_publisher(Twist, prefix+'raw', 10)
        free = phase(1.5); report['cases']['free'] = free
        assert free['samples'] and min(free['tail_x'])>.11, free
        caution = phase(.6, obstacle=.8); report['cases']['outer_zone'] = caution
        assert caution['samples'] and min(caution['tail_x'])>.11, caution
        result = phase(.6, obstacle=.45); report['cases']['stop_zone'] = result
        assert stopped(result), result
        for name, kwargs, deadline in (
            ('raw_stop', {'speed':0.}, .2),
            ('scan_dropout', {'scans_enabled':False}, .4),
            ('scan_stamp_stale', {'scan_offset':1.}, .2),
            ('localization_invalid', {'valid':False}, .2),
            ('localization_dropout', {'health_enabled':False}, .4),
            ('raw_dropout', {'raw_enabled':False}, .4)):
            phase(1.)
            result = phase(.75, **kwargs); report['cases'][name] = result
            assert stopped(result), (name, result)
            # Check the sustained final zero as well as the first zero: a
            # single transient zero must not hide resumed nonzero output.
            assert result['first_zero_s'] is not None and result['first_zero_s']<=deadline, (name,result)
        phase(1.)
        processes['velocity_smoother'].kill(); processes['velocity_smoother'].wait(timeout=5.)
        result = phase(.75); report['cases']['smoother_exit'] = result
        assert stopped(result) and result['first_zero_s'] is not None and result['first_zero_s']<=.4, result
        report['passed'] = True
    finally:
        if rclpy.ok(): phase(.15, speed=0., valid=False)
        cleanup_processes()
        report['process_exit_codes'] = {key:p.returncode for key,p in processes.items()}
        report['cleanup_passed'] = all(
            code == (-signal.SIGKILL if ((name == 'velocity_smoother' and
                     'smoother_exit' in report['cases']) or (name == 'controller' and
                     'controller_exit' in report['cases'])) else 0)
            for name, code in report['process_exit_codes'].items())
        report['passed'] = report['passed'] and report['cleanup_passed']
        (out/'samples.json').write_text(json.dumps(received))
        (out/'report.json').write_text(json.dumps(report, indent=2))
        for stream in logs: stream.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
