#!/usr/bin/python3
"""Real NavFn on a synthetic map in a loopback-only DDS domain.

Starts only planner_server. No controller, navigation BT, bridge or actuator
client is created. Requires an explicit isolated environment before ROS init.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    if (os.environ.get('ROS_LOCALHOST_ONLY') != '1'
            or os.environ.get('ROS_DOMAIN_ID') != '179'
            or os.environ.get('CYCLONEDDS_URI')):
        parser.error('requires ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID=179, no CYCLONEDDS_URI')
    import rclpy
    import yaml
    from geometry_msgs.msg import TransformStamped, PoseStamped
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster

    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((root/'ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml').read_text())
    # Copy only the deployed planner/global costmap parameters. The production
    # configuration is unchanged; no motion node is launched by this probe.
    config = {key:params[key] for key in ('planner_server', 'global_costmap')}
    config_path = output/'planner_params.yaml'
    config_path.write_text(yaml.safe_dump(config))
    rclpy.init()
    node = rclpy.create_node('synthetic_large_map_planner_probe')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(OccupancyGrid, '/map', qos)
    scan_pub = node.create_publisher(LaserScan, '/go2w/lidar/scan', 10)
    tf = TransformBroadcaster(node)
    size, resolution = 2000, .05
    data = [0]*(size*size)
    for i in range(size):
        data[i] = data[(size-1)*size+i] = data[i*size] = data[i*size+size-1] = 100
    for y in range(1400):
        for x in range(997,1003):data[y*size+x]=100
    for y in range(1600,1800):
        data[y*size+1600:y*size+1800]=[-1]*200
    message = OccupancyGrid()
    message.header.frame_id='map'
    message.info.width=message.info.height=size
    message.info.resolution=resolution
    message.info.origin.orientation.w=1.
    message.data=data
    message.header.stamp=node.get_clock().now().to_msg()
    publisher.publish(message)

    def tick():
        stamp=node.get_clock().now().to_msg()
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_link'
        transform.header.stamp=stamp;transform.transform.translation.x=25.;transform.transform.translation.y=25.
        transform.transform.rotation.w=1.;tf.sendTransform(transform)
        scan=LaserScan();scan.header.frame_id='base_link';scan.header.stamp=stamp
        scan.angle_min=-math.pi;scan.angle_max=math.pi;scan.angle_increment=math.pi/180
        scan.range_min=.1;scan.range_max=100.;scan.ranges=[float('inf')]*361
        scan_pub.publish(scan)
    timer=node.create_timer(.1,tick)
    log=(output/'planner.log').open('w')
    process=None
    result={'source':'fixture_synthetic_large_map','real_robot_motion':False,
            'ros_domain_id':179,'localhost_only':True,'map_size_m':[100,100],
            'resolution_m':resolution,'map_cells':size*size,'results':[]}
    def await_future(future, timeout):
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError('isolated planner request timed out')
        return future.result()
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def pose(x,y):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
        p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.
        return p
    try:
        process=subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server',
            '--ros-args','--params-file',str(config_path)],stdout=log,stderr=subprocess.STDOUT,
            start_new_session=True)
        service=node.create_client(ChangeState,'/planner_server/change_state')
        if not service.wait_for_service(timeout_sec=15):raise RuntimeError('planner lifecycle unavailable')
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition
            response=await_future(service.call_async(request),20)
            if not response or not response.success:raise RuntimeError('planner lifecycle failed')
        spin(3.)
        action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        if not action.wait_for_server(timeout_sec=10):raise RuntimeError('planner action unavailable')
        for name,xy,expected in [('wall_detour',(75.,25.),True),('unknown',(85.,85.),False),
                                 ('occupied',(50.,25.),False),('outside',(150.,25.),False)]:
            goal=ComputePathToPose.Goal();goal.start=pose(25,25);goal.goal=pose(*xy)
            goal.use_start=True;goal.planner_id='GridBased'
            t=time.perf_counter();handle=await_future(action.send_goal_async(goal),10)
            if not handle or not handle.accepted:raise RuntimeError('planner goal handle missing')
            answer=await_future(handle.get_result_async(),30)
            path=[(p.pose.position.x,p.pose.position.y) for p in answer.result.path.poses]
            item={'case':name,'target':xy,'status':answer.status,'poses':len(path),
                  'length_m':sum(math.dist(a,b) for a,b in zip(path,path[1:])),
                  'maximum_y':max((p[1] for p in path),default=None),
                  'seconds':time.perf_counter()-t}
            item['passed']=(answer.status==4 and bool(path)) is expected
            if expected:
                (output/'detour_path.json').write_text(json.dumps(path))
                item['passed']=item['passed'] and item['maximum_y']>70 and item['length_m']>100
            result['results'].append(item)
        result['passed']=all(item['passed'] for item in result['results'])
    except Exception as exc:
        result.update(passed=False,error=str(exc))
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
            result['planner_exit_code']=process.returncode
        log.close();node.destroy_timer(timer);node.destroy_node();rclpy.shutdown()
        (output/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
