#!/usr/bin/python3
"""Isolated SLAM contract probe using captured geometry and SYNTHETIC poses.

Checks static duplicate rejection, scan-time TF freshness, pure-yaw processing
and dropout. Never connects to domain 0 or sends motion commands.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot', required=True)
    p.add_argument('--scan-topic', required=True)
    p.add_argument('--output-dir', required=True)
    p.add_argument('--domain-id', type=int, default=78)
    p.add_argument('--continuous-bootstrap', action='store_true',
        help='withhold initial keyframes to reproduce a lost startup scan')
    args = p.parse_args()
    if not 1 <= args.domain_id <= 100:
        raise ValueError('requires isolated nonzero domain 1..100')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    import numpy as np
    import yaml
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster, Buffer, TransformListener
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    root = Path(__file__).resolve().parents[2]
    out = Path(args.output_dir).resolve();out.mkdir(parents=True,exist_ok=True)
    frame = json.loads(Path(args.snapshot).read_text())['scans'][args.scan_topic][0]
    params = yaml.safe_load((root/'ros2_ws/src/robot_scene_nav_bringup/config/slam_2d_mapping.yaml').read_text())
    params['slam_toolbox']['ros__parameters'].update(scan_topic='/replay/keyframe',
        continuous_scan_topic='/replay/continuous', base_frame='replay_base',
        odom_frame='replay_odom', map_frame='replay_map',
        mapping_use_keyframes=True, throttle_scans=1)
    cfg=out/'params.yaml';cfg.write_text(yaml.safe_dump(params))
    executable=Path(get_package_prefix('robot_scene_nav_bringup'))/'lib/robot_scene_nav_bringup/keyframe_slam_toolbox'
    log=(out/'slam.log').open('w')
    child=subprocess.Popen([str(executable),'--ros-args','--params-file',str(cfg)],stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('keyframe_slam_isolated_probe')
    broadcaster=TransformBroadcaster(node);buffer=Buffer();listener=TransformListener(buffer,node)
    key=node.create_publisher(LaserScan,'/replay/keyframe',10)
    continuous=node.create_publisher(LaserScan,'/replay/continuous',10)
    matched=[];maps=[]
    node.create_subscription(PoseWithCovarianceStamped,'/pose',lambda m:matched.append(
        {'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,
         'yaw':2*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w)}),10)
    node.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(
        {'received':time.time(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
         'data':list(m.data)}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE))
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.01)
            if child.poll() is not None:raise RuntimeError('isolated mapper exited')
    def rotated(yaw):
        # Rotate observed endpoints into a simulated rotated base. Missing
        # directions remain missing, without introducing extra free rays.
        ranges=np.array([math.inf if v is None else v for v in frame['ranges']])
        angles=frame['angle_min']+np.arange(len(ranges))*frame['angle_increment']
        bearing=(angles-yaw-frame['angle_min'])%(2*math.pi)+frame['angle_min']
        bins=np.rint((bearing-frame['angle_min'])/frame['angle_increment']).astype(int)%len(ranges)
        result=np.full(len(ranges),math.inf);np.minimum.at(result,bins,ranges)
        return result.tolist()
    def publish(yaw, *, keyframe=False, scans=True):
        now=node.get_clock().now()
        for offset in (-.02,.02):
            t=TransformStamped();t.header.stamp=rclpy.time.Time(nanoseconds=now.nanoseconds+int(offset*1e9)).to_msg()
            t.header.frame_id='replay_odom';t.child_frame_id='replay_base'
            t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
            broadcaster.sendTransform(t)
        if not scans:return
        m=LaserScan();m.header.stamp=now.to_msg();m.header.frame_id='replay_base'
        for name in ('angle_min','angle_increment','range_min','range_max'):setattr(m,name,float(frame[name]))
        m.angle_max=m.angle_min+(len(frame['ranges'])-1)*m.angle_increment
        m.ranges=rotated(yaw);continuous.publish(m)
        if keyframe:key.publish(m)
    result={'scope':'isolated real mapper with captured scan geometry and synthetic odometry; NOT physical motion validation',
            'domain_id':args.domain_id,'motion_sent':False}
    try:
        deadline=time.monotonic()+12
        while time.monotonic()<deadline and (key.get_subscription_count()!=1 or continuous.get_subscription_count()!=1):spin(.1)
        if key.get_subscription_count()!=1 or continuous.get_subscription_count()!=1:raise RuntimeError('isolated subscriptions unavailable')
        # Establish a TF buffer and let all subscriber endpoints discover.
        for _ in range(20):publish(0,scans=False);spin(.05)
        for _ in range(60):publish(0,keyframe=not args.continuous_bootstrap);spin(.065)
        result['continuous_bootstrap'] = args.continuous_bootstrap
        if not maps:
            result['passed'] = False
            result['failure'] = 'no map after fresh continuous scans and valid acquisition-time TF'
            print(json.dumps(result),flush=True)
            return 2
        transform=buffer.lookup_transform('replay_map','replay_base',rclpy.time.Time())
        result['stationary']={'matched_scans':len(matched),
            'unique_map_contents':len({tuple(v['data']) for v in maps}),
            'latest_map_age_s':time.time()-maps[-1]['stamp'] if maps else None,
            'latest_tf_age_s':time.time()-(transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9),
            'max_translation_m':max((math.hypot(v['x'],v['y']) for v in matched),default=None),
            'max_yaw_rad':max((abs(v['yaw']) for v in matched),default=None)}
        for _ in range(50):publish(.15,keyframe=True);spin(.065)
        result['pure_yaw']={'matched_scans_total':len(matched),'last_pose':matched[-1] if matched else None}
        for _ in range(50):publish(.15,scans=False);spin(.065)
        t=buffer.lookup_transform('replay_map','replay_odom',rclpy.time.Time())
        result['dropout']={'latest_map_age_s':time.time()-maps[-1]['stamp'] if maps else None,
                           'latest_tf_age_s':time.time()-(t.header.stamp.sec+t.header.stamp.nanosec*1e-9)}
        s,y,d=result['stationary'],result['pure_yaw'],result['dropout']
        result['passed']=(s['matched_scans']==1 and s['unique_map_contents']==1
            and s['latest_map_age_s']<1.5 and -.3<s['latest_tf_age_s']<.3
            and s['max_translation_m']<.001 and s['max_yaw_rad']<.001
            and y['matched_scans_total']==2 and abs(y['last_pose']['yaw']-.15)<.05
            and d['latest_map_age_s']>2 and d['latest_tf_age_s']>2)
    finally:
        node.destroy_node();rclpy.shutdown();child.terminate()
        try:child.wait(timeout=6)
        except subprocess.TimeoutExpired:child.kill();child.wait()
        log.close();(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if result.get('passed') else 2


if __name__=='__main__':raise SystemExit(main())
