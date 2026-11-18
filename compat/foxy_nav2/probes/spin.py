#!/usr/bin/python3
"""Duration-capable native Spin against synthetic pose/costmap inputs only."""
from pathlib import Path
import argparse
import json
import math
import os
import signal
import subprocess
import time

from velocity_chain import isolated_environment


def main():
    isolated_environment()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir.resolve(); out.mkdir(parents=True,exist_ok=False)
    import yaml
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist, TransformStamped, PolygonStamped, Point32
    from nav2_msgs.msg import Costmap
    from go2w_nav2_compat_msgs.action import Spin
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.msg import Transition
    from tf2_ros import TransformBroadcaster
    from ament_index_python.packages import get_package_prefix

    root = Path(__file__).resolve().parents[3]
    params = yaml.safe_load((root/'ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml').read_text())['behavior_server']['ros__parameters']
    prefix = '/go2w/acceptance/spin/'
    params.pop('behavior_plugins')
    params['recovery_plugins'] = ['spin']
    params['spin']['plugin'] = 'go2w_foxy_spin::Spin'
    params.pop('wait')
    params['costmap_topic'] = prefix+'costmap'
    params['footprint_topic'] = prefix+'footprint'
    config = out/'parameters.yaml'
    config.write_text(yaml.safe_dump({'acceptance_spin_server':{'ros__parameters':params}}))
    binary = Path(get_package_prefix('nav2_recoveries'))/'lib/nav2_recoveries/recoveries_server'
    log = (out/'server.log').open('w')
    process = subprocess.Popen([str(binary),'--ros-args','-r','__node:=acceptance_spin_server',
        '--params-file',str(config),'-r','cmd_vel:='+prefix+'raw'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(); node = rclpy.create_node('acceptance_spin_probe')
    tf = TransformBroadcaster(node)
    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    costmaps = node.create_publisher(Costmap,prefix+'costmap',qos)
    footprints = node.create_publisher(PolygonStamped,prefix+'footprint',10)
    commands = []; pose_yaw = 0.; angular_velocity = 0.; last_pump = time.monotonic()
    def command(message):
        nonlocal angular_velocity
        angular_velocity = float(message.angular.z)
        commands.append((time.monotonic(),float(message.linear.x),angular_velocity))
    node.create_subscription(Twist,prefix+'raw',command,100)
    report = {'scope':'synthetic native Spin only','domain_id':99,
              'physical_motion_enabled':False,'cases':{},'passed':False}
    def pump(duration, *, moving=False, blocked=False):
        nonlocal pose_yaw,last_pump
        end = time.monotonic()+duration
        while time.monotonic()<end:
            now = time.monotonic()
            if moving: pose_yaw += angular_velocity*min(.05,now-last_pump)
            last_pump = now
            stamp = node.get_clock().now().to_msg()
            transform = TransformStamped(); transform.header.stamp = stamp
            transform.header.frame_id = 'odom_fused'; transform.child_frame_id = 'base_link'
            transform.transform.rotation.z = math.sin(pose_yaw/2)
            transform.transform.rotation.w = math.cos(pose_yaw/2)
            tf.sendTransform(transform)
            costmap = Costmap(); costmap.header.stamp = stamp; costmap.header.frame_id = 'odom_fused'
            costmap.metadata.map_load_time = stamp; costmap.metadata.update_time = stamp
            costmap.metadata.resolution = .05; costmap.metadata.size_x = 100; costmap.metadata.size_y = 100
            costmap.metadata.origin.position.x = -2.5; costmap.metadata.origin.position.y = -2.5
            costmap.metadata.origin.orientation.w = 1.
            costmap.data = [254 if blocked else 0]*10000; costmaps.publish(costmap)
            footprint = PolygonStamped(); footprint.header.stamp = stamp
            footprint.header.frame_id = 'base_link'
            footprint.polygon.points = [Point32(x=x,y=y,z=0.) for x,y in
                ((.35,.215),(.35,-.215),(-.35,-.215),(-.35,.215))]
            footprints.publish(footprint)
            rclpy.spin_once(node,timeout_sec=.01)
            assert process.poll() is None, 'Spin server exited'
    def wait(future, *, moving=False, blocked=False, timeout=5.):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:
            pump(.02,moving=moving,blocked=blocked)
        assert future.done(),'Spin action/service timeout'
        return future.result()
    def lifecycle(transition):
        client = node.create_client(ChangeState,'/acceptance_spin_server/change_state')
        try:
            assert client.wait_for_service(timeout_sec=10.)
            request = ChangeState.Request(); request.transition.id = transition
            assert wait(client.call_async(request)).success
        finally: node.destroy_client(client)
    def start(yaw, seconds):
        goal = Spin.Goal(); goal.target_yaw = yaw
        goal.time_allowance.sec = int(seconds)
        goal.time_allowance.nanosec = round((seconds-int(seconds))*1e9)
        handle = wait(action.send_goal_async(goal))
        assert handle and handle.accepted
        return handle
    def record(name, started, result, expected):
        pump(.15)
        rows = [row for row in commands if row[0]>=started]
        assert rows and any(abs(row[2])>.05 for row in rows), (name,rows)
        assert result.status==expected,(name,result.status)
        assert abs(rows[-1][1])+abs(rows[-1][2])<1e-8,(name,rows[-1])
        assert all(abs(row[1])<1e-8 and abs(row[2])<=params['max_rotational_vel']+1e-6 for row in rows)
        report['cases'][name] = {'status':result.status,'samples':len(rows),
            'elapsed_s':time.monotonic()-started,'max_angular_rps':max(abs(row[2]) for row in rows),
            'final_zero':True}
    action = None
    try:
        pump(.3)
        for topic in ('/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected','/go2w/software_arm'):
            assert node.count_publishers(topic)==0,'Unexpected physical topic: '+topic
        lifecycle(Transition.TRANSITION_CONFIGURE)
        lifecycle(Transition.TRANSITION_ACTIVATE)
        action = ActionClient(node,Spin,'/spin')
        end = time.monotonic()+5
        while not action.server_is_ready() and time.monotonic()<end:pump(.05)
        assert action.server_is_ready()
        for angle in (.25,-.25):
            started = time.monotonic(); initial = pose_yaw
            handle = start(angle,3.)
            result = wait(handle.get_result_async(),moving=True)
            name = 'turn_positive' if angle>0 else 'turn_negative'
            record(name,started,result,4)
            assert abs((pose_yaw-initial)-angle)<.04,(name,pose_yaw-initial)
        started = time.monotonic(); handle = start(1.,.35)
        result = wait(handle.get_result_async())
        record('server_timeout',started,result,6)
        assert report['cases']['server_timeout']['elapsed_s']<.8
        handle = start(2.,5.); pump(.2)
        canceled = time.monotonic(); wait(handle.cancel_goal_async())
        result = wait(handle.get_result_async())
        pump(.2)
        zeros = [row[0]-canceled for row in commands if row[0]>=canceled and abs(row[1])+abs(row[2])<1e-8]
        assert result.status==5 and zeros and min(zeros)<=.3,(result.status,zeros)
        report['cases']['cancel'] = {'status':result.status,'cancel_to_zero_s':min(zeros)}
        started = time.monotonic(); handle = start(2.,3.)
        pump(.2,moving=True)
        result = wait(handle.get_result_async(),moving=True,blocked=True)
        record('collision',started,result,6)
        report['passed'] = True
    finally:
        if action is not None: action.destroy()
        os.killpg(process.pid,signal.SIGINT) if process.poll() is None else None
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3.)
        report['server_exit_code'] = process.returncode
        report['passed'] = report['passed'] and process.returncode==0
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        (out/'commands.json').write_text(json.dumps(commands))
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
