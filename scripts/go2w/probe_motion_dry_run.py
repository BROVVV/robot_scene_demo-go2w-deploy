#!/usr/bin/env python3
"""Test only the C++ controller in an isolated domain with synthetic inputs.

Never starts the SDK, lease holder, network proxy or normal control launch.
A decoy Unix socket proves that dry-run requests never reach the transport.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if not 90 <= domain <= 100 or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Requires ROS_DOMAIN_ID=90..100 and ROS_LOCALHOST_ONLY=1')
    if args.binary.name != 'go2w_motion_action_server' or not args.binary.is_file():
        parser.error('Pass the built C++ action server executable')
    if not args.config.is_file():
        parser.error('Controller config does not exist')
    args.output.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import UInt64, Bool, String
    from std_srvs.srv import SetBool, Trigger
    from rcl_interfaces.srv import GetParameters
    from unitree_go.msg import SportModeState, LowState
    from go2w_motion_interfaces.action import MotionCommand
    import yaml

    rclpy.init()
    node = rclpy.create_node('motion_acceptance_synthetic_inputs')
    prefix = '/acceptance/motion/'
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {
        'sport': node.create_publisher(SportModeState, prefix + 'sport', 20),
        'low': node.create_publisher(LowState, prefix + 'low', 20),
        'id': node.create_publisher(UInt64, prefix + 'id', latched),
        'alive': node.create_publisher(Bool, prefix + 'alive', 10),
        'mode': node.create_publisher(String, prefix + 'mode', latched),
    }
    state = {'enabled': False, 'publish_state': True, 'lease': True, 'mode': 1}

    def publish():
        if not state['enabled']:
            return
        if state['publish_state']:
            sport = SportModeState()
            sport.mode = state['mode']
            publishers['sport'].publish(sport)
            publishers['low'].publish(LowState())
        if state['lease']:
            publishers['id'].publish(UInt64(data=424242))
            publishers['alive'].publish(Bool(data=True))
        publishers['mode'].publish(String(data='ai-w'))

    node.create_timer(.02, publish)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def wait(future, timeout=12):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        if not future.done():
            raise TimeoutError('Isolated controller response timed out')
        return future.result()

    report = {'domain': domain, 'dry_run': True, 'cases': {}}
    process = None
    action = None
    try:
        with tempfile.TemporaryDirectory(prefix='motion_acceptance_') as temporary:
            socket_path = str(Path(temporary) / 'decoy.sock')
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as decoy:
                decoy.bind(socket_path)
                decoy.listen(8)
                decoy.setblocking(False)
                overrides = {
                    'dry_run': 'true', 'require_arm': 'true',
                    'sport_state_topic': prefix + 'sport',
                    'low_state_topic': prefix + 'low', 'require_low_state': 'true',
                    'lease_id_topic': prefix + 'id', 'lease_alive_topic': prefix + 'alive',
                    'motion_name_topic': prefix + 'mode',
                    'action_name': prefix + 'command', 'arm_service': prefix + 'arm',
                    'emergency_stop_service': prefix + 'stop',
                    'sdk_command_socket': socket_path,
                    'log_root': str(args.output.resolve() / 'controller_logs'),
                    'final_stationary_timeout_sec': '1.0',
                }
                # Foxy node-specific YAML can outrank unqualified CLI -p.
                # Use exactly one node-specific file, then read back its values.
                config = yaml.safe_load(args.config.read_text())
                parameters = config['go2w_motion_action_server']['ros__parameters']
                parameters.update({key: yaml.safe_load(value) for key, value in overrides.items()})
                private_config = Path(temporary) / 'isolated.yaml'
                private_config.write_text(yaml.safe_dump(config))
                command = [str(args.binary), '--ros-args', '--params-file', str(private_config)]
                command.extend(['-r', '/go2w/motion/velocity_update:=' + prefix + 'velocity_update'])
                with (args.output / 'server.log').open('x') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True)
                    action = ActionClient(node, MotionCommand, prefix + 'command')
                    arm = node.create_client(SetBool, prefix + 'arm')
                    stop = node.create_client(Trigger, prefix + 'stop')
                    read_parameters = node.create_client(GetParameters,
                        '/go2w_motion_action_server/get_parameters')
                    deadline = time.monotonic() + 20
                    while not (action.server_is_ready() and arm.service_is_ready()
                               and stop.service_is_ready() and read_parameters.service_is_ready()):
                        assert process.poll() is None, 'Controller exited during startup'
                        if time.monotonic() > deadline:
                            raise TimeoutError('Isolated action/service discovery failed')
                        spin(.1)
                    actual = wait(read_parameters.call_async(GetParameters.Request(
                        names=['dry_run', 'sdk_command_socket', 'action_name'])))
                    assert actual.values[0].bool_value is True
                    assert actual.values[1].string_value == socket_path
                    assert actual.values[2].string_value == prefix + 'command'
                    report['effective_parameters_verified'] = True
                    state['enabled'] = True
                    spin(1)

                    def goal(duration=.15, vx=.05):
                        message = MotionCommand.Goal()
                        message.vx = vx
                        message.duration_sec = duration
                        message.timeout_sec = duration + 3.
                        return message

                    def send(message):
                        return wait(action.send_goal_async(message))

                    def set_arm(expected):
                        response = wait(arm.call_async(SetBool.Request(data=True)))
                        assert response.success == expected, response.message
                        return response.message

                    assert not send(goal()).accepted
                    report['cases']['disarmed_goal_rejected'] = True
                    set_arm(True)
                    assert not send(goal(vx=10.)).accepted
                    report['cases']['over_limit_goal_rejected'] = True
                    handle = send(goal())
                    assert handle.accepted
                    result = wait(handle.get_result_async())
                    assert result.status == 4 and result.result.success, result.result.message
                    assert result.result.last_move_status_code == 0
                    assert result.result.last_stop_status_code == 0
                    report['cases']['timed_goal_and_final_stop'] = True

                    set_arm(True)
                    handle = send(goal(duration=2.))
                    assert handle.accepted
                    spin(.7)
                    assert wait(handle.cancel_goal_async()).goals_canceling
                    result = wait(handle.get_result_async())
                    assert result.status == 5 and result.result.error_code == 9
                    report['cases']['cancel_stops_goal'] = True

                    spin(.2)
                    set_arm(True)
                    handle = send(goal(duration=3.))
                    assert handle.accepted
                    spin(.7)
                    state['publish_state'] = False
                    result = wait(handle.get_result_async())
                    assert not result.result.success and result.result.error_code == 4
                    assert not send(goal()).accepted
                    report['cases']['state_timeout_aborts_and_rejects'] = True
                    state['publish_state'] = True
                    spin(.7)
                    state['mode'] = 14
                    spin(.2)
                    set_arm(False)
                    report['cases']['unvalidated_robot_mode_rejects_arm'] = True
                    state['mode'] = 1
                    spin(.2)
                    set_arm(True)
                    handle = send(goal(duration=3.))
                    assert handle.accepted
                    spin(.7)
                    state['lease'] = False
                    result = wait(handle.get_result_async())
                    assert not result.result.success and result.result.error_code == 3
                    report['cases']['lease_timeout_aborts'] = True
                    state['lease'] = True
                    spin(.3)
                    response = wait(stop.call_async(Trigger.Request()))
                    assert response.success, response.message
                    assert not send(goal()).accepted
                    report['cases']['emergency_stop_disarms'] = True

                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=8)
                    assert process.returncode == 0, 'Controller did not shut down cleanly'
                    try:
                        connection, _ = decoy.accept()
                    except BlockingIOError:
                        report['sdk_transport_connections'] = 0
                    else:
                        connection.close()
                        raise AssertionError('Dry-run connected to the SDK transport')
                    responses = [json.loads(line)
                        for path in (args.output / 'controller_logs').rglob('responses.jsonl')
                        for line in path.read_text().splitlines()]
                    assert responses, 'No controller request evidence was recorded'
                    assert all(json.loads(row['raw_response']).get('dry_run') is True
                               for row in responses)
                    report['simulated_rpc_responses'] = len(responses)
                    report['passed'] = True
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        if action is not None:
            action.destroy()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report))


if __name__ == '__main__':
    main()
