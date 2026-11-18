#!/usr/bin/python3
"""Gracefully shut down the Foxy Nav2 nodes owned by one verified launcher.

Uses that launcher's ROS environment, never the stop caller's DDS domain.
No SDK, velocity publisher, or motion service is used.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

MANAGERS = ('lifecycle_manager_navigation_2d', 'lifecycle_manager_plan_only')
NODES = ('planner_server', 'controller_server', 'behavior_server',
         'bt_navigator', 'velocity_smoother', 'collision_monitor')


def inspect_launch(pid, proc=Path('/proc')):
    directory = proc / str(pid)
    argv = directory.joinpath('cmdline').read_bytes().split(b'\0')
    argv = [a.decode() for a in argv if a]
    if not any(Path(a).name == 'robot_scene_nav2.launch.py' for a in argv):
        raise ValueError('PID is not the project Nav2 launcher')
    ros2_index = 1 if argv and Path(argv[0]).name in ('python3', 'python3.8') else 0
    if len(argv) <= ros2_index+1 or Path(argv[ros2_index]).name != 'ros2' or argv[ros2_index+1] != 'launch':
        raise ValueError('Expected ros2 launch owner')
    environment = dict(item.decode().split('=', 1) for item in
                       directory.joinpath('environ').read_bytes().split(b'\0')
                       if b'=' in item)
    if environment.get('ROS_DISTRO') != 'foxy':
        return environment, None
    parents, managers = {}, []
    for child in proc.iterdir():
        if not child.name.isdigit():
            continue
        try:
            status = child.joinpath('status').read_text()
            parents[int(child.name)] = int(next(line.split(':', 1)[1] for line in
                                               status.splitlines() if line.startswith('PPid:')))
            args = child.joinpath('cmdline').read_bytes().split(b'\0')
            if args and Path(args[0].decode()).name == 'lifecycle_manager':
                for name in MANAGERS:
                    if ('__node:=' + name).encode() in args:
                        managers.append((int(child.name), name))
        except (OSError, ValueError, StopIteration):
            continue
    owned = []
    for manager in managers:
        ancestor, seen = manager[0], set()
        while ancestor and ancestor not in seen:
            if ancestor == pid:
                owned.append(manager)
                break
            seen.add(ancestor)
            ancestor = parents.get(ancestor, 0)
    if len(owned) != 1 or sum(m[1] == owned[0][1] for m in managers) != 1:
        raise ValueError('Expected exactly one local Nav2 manager owned by this launcher')
    return environment, owned[0]


def shutdown_worker(pid):
    # Recheck ownership immediately before creating the ROS service client.
    _, manager = inspect_launch(pid)
    if manager is None:
        return {'success': True, 'skipped': 'Not a Foxy launcher'}
    manager_pid, manager_name = manager
    import rclpy
    from nav2_msgs.srv import ManageLifecycleNodes
    from lifecycle_msgs.srv import GetState
    rclpy.init()
    node = rclpy.create_node('go2w_owned_nav2_shutdown')
    report = {'success': False, 'launch_pid': pid, 'manager_pid': manager_pid,
              'final_states': {}, 'physical_command_sent': False}

    def call(kind, service, request, timeout):
        client = node.create_client(kind, service)
        try:
            if not client.wait_for_service(timeout_sec=2.):
                raise RuntimeError('Service unavailable: ' + service)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError('Response timeout: ' + service)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        # A second graph owner can be on another machine in this DDS domain.
        end = time.monotonic() + 2.
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        owners = [(name, namespace) for name, namespace in
                  node.get_node_names_and_namespaces() if name == manager_name]
        if owners != [(manager_name, '/')]:
            raise RuntimeError('Nav2 manager graph owner is not unique')
        request = ManageLifecycleNodes.Request()
        request.command = request.SHUTDOWN
        started = time.monotonic()
        result = call(ManageLifecycleNodes, '/' + manager_name + '/manage_nodes', request, 10.)
        if not result.success:
            raise RuntimeError('Lifecycle manager rejected shutdown')
        expected_nodes = NODES if manager_name == MANAGERS[0] else ('planner_server', 'bt_navigator')
        for name in expected_nodes:
            state = call(GetState, '/' + name + '/get_state', GetState.Request(), 2.).current_state
            report['final_states'][name] = {'id': state.id, 'label': state.label}
            if state.id != 4:
                raise RuntimeError('Node did not finalize: ' + name)
        report['elapsed_s'] = time.monotonic() - started
        report['success'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch-pid', required=True, type=int)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        if args.worker:
            report = shutdown_worker(args.launch_pid)
            print(json.dumps(report))
            return 0 if report['success'] else 1
        environment, manager = inspect_launch(args.launch_pid)
        if manager is None:
            print(json.dumps({'success': True, 'skipped': 'Not a Foxy launcher'}))
            return 0
        # Native ROS Python, with the exact owner's setup and DDS configuration.
        return subprocess.run(['/usr/bin/python3', str(Path(__file__).resolve()),
                               '--worker', '--launch-pid', str(args.launch_pid)],
                              env=environment, timeout=30).returncode
    except Exception as error:
        print(json.dumps({'success': False, 'error': str(error)}))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
