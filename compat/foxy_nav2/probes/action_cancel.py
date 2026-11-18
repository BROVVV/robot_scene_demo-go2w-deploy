#!/usr/bin/python3
"""Exercise the real rclpy action port against a synthetic planner only."""
from pathlib import Path
import argparse
import json
import threading
import time

from velocity_chain import isolated_environment


def main():
    isolated_environment()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from nav2_msgs.action import ComputePathToPose
    from app.navigation.nav2_search_backend import (
        RosActionClientPort, Nav2GoalRequest, ACTION_CANCELED, ACTION_SUCCEEDED)

    rclpy.init()
    server_node = rclpy.create_node('acceptance_delayed_planner')
    client_node = rclpy.create_node('acceptance_cancel_client')
    requests = []; cancellations = []; executor_errors = []

    def accept(request):
        # Force the operator command to precede server acceptance.
        time.sleep(.25)
        requests.append(float(request.pose.pose.position.x))
        return GoalResponse.ACCEPT

    def cancel(handle):
        cancellations.append(float(handle.request.pose.pose.position.x))
        return CancelResponse.ACCEPT

    def execute(handle):
        deadline = time.monotonic() + (4. if handle.request.pose.pose.position.x == 1. else .2)
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                return ComputePathToPose.Result()
            time.sleep(.01)
        handle.succeed()
        return ComputePathToPose.Result()

    action_name = '/go2w/acceptance/delayed_compute_path'
    server = ActionServer(server_node, ComputePathToPose, action_name,
        execute_callback=execute, goal_callback=accept, cancel_callback=cancel,
        callback_group=ReentrantCallbackGroup())
    port = RosActionClientPort(client_node, action_name, ComputePathToPose)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node); executor.add_node(client_node)
    def spin():
        try: executor.spin()
        except Exception as error: executor_errors.append(repr(error))
    thread = threading.Thread(target=spin)
    thread.start()
    report = {'domain_id':99,'physical_motion_enabled':False,'passed':False}
    def wait(status):
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            if port.poll().status == status: return
            assert not executor_errors,executor_errors
            time.sleep(.01)
        raise AssertionError((status,port.poll()))
    try:
        assert port._client.wait_for_server(timeout_sec=5)
        port.send(Nav2GoalRequest(1.,0.,0.))
        assert port.cancel()
        wait(ACTION_CANCELED)
        report['pending_cancel'] = {'status':port.poll().status,'server_cancel_requests':list(cancellations)}
        assert cancellations == [1.]
        port.send(Nav2GoalRequest(2.,0.,0.))
        wait(ACTION_SUCCEEDED)
        assert port.cancel() is False
        report['next_goal'] = {'status':port.poll().status,'server_requests':list(requests)}
        assert requests == [1.,2.]
        report['passed'] = True
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(3)
        report['executor_exited'] = not thread.is_alive()
        report['executor_errors'] = executor_errors
        port._client.destroy(); server.destroy()
        client_node.destroy_node(); server_node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
    assert report['executor_exited'] and not executor_errors
    print(json.dumps(report))


if __name__ == '__main__':
    main()
