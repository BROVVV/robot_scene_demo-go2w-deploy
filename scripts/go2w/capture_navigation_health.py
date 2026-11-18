#!/usr/bin/env python3
"""Read-only navigation diagnostics; preserves full blocking reasons on disk."""
import argparse
import json
import math
from pathlib import Path
import signal
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--jsonl',required=True)
    p.add_argument('--duration',type=float,default=30.)
    args=p.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=600:
        p.error('--duration must be 0..600 seconds')
    import rclpy
    try:
        from rclpy.signals import SignalHandlerOptions
    except ImportError:  # Foxy does not expose SignalHandlerOptions.
        SignalHandlerOptions = None
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import LaserScan,PointCloud2
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from unitree_go.msg import LowState
    import yaml
    root=Path(__file__).resolve().parents[2]
    profile=yaml.safe_load((root/'configs/go2w/navigation_sensor_profile.yaml').read_text())
    target=Path(args.jsonl);target.parent.mkdir(parents=True,exist_ok=True)
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    if SignalHandlerOptions is None:
        rclpy.init()
    else:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    node=rclpy.create_node('go2w_navigation_health_capture')
    counts={};last={};errors={}
    with target.open('x',buffering=1) as stream:
        def write(topic,payload):
            row={'received':time.time(),'topic':topic,'payload':payload}
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
            counts[topic]=counts.get(topic,0)+1
        def status(topic,message):
            try:data=json.loads(message.data)
            except ValueError:data=message.data
            write(topic,data)
            if isinstance(data,dict):
                for key in data.get('blocking') or []:
                    errors[key]=errors.get(key,0)+1
                if data.get('floor_check_error'):
                    key=data['floor_check_error'];errors[key]=errors.get(key,0)+1
        for topic in ('/go2w/nav/localization_status','/go2w/lidar/geometry_status',
                      '/go2w/lidar/mapping_scan_status',
                      '/go2w/nav/floor_visibility_status',
                      '/go2w/cmd_vel_bridge/status','/go2w_navigation_velocity_guard/status'):
            node.create_subscription(String,topic,lambda m,t=topic:status(t,m),10)
        for topic in ('/cmd_vel_nav_raw','/cmd_vel_nav_smoothed','/cmd_vel_nav_collision_checked',
                      '/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected'):
            node.create_subscription(Twist,topic,lambda m,t=topic:write(t,
                {'linear_x':float(m.linear.x),'angular_z':float(m.angular.z)}),10)
        def sensor(topic,message):
            now=time.monotonic()
            if now-last.get(topic,0)<.05:return
            last[topic]=now
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            data={'stamp':stamp,'age_s':time.time()-stamp,'frame':message.header.frame_id}
            if isinstance(message,Odometry):
                pose=message.pose.pose;data.update(x=pose.position.x,y=pose.position.y)
            write(topic,data)
        for topic,kind in (('/go2w/lidar/scan',LaserScan),('/go2w/lidar/mapping_scan',LaserScan),
                           ('/go2w/sensors/cloud',PointCloud2),('/go2w/odom/fused',Odometry)):
            node.create_subscription(kind,topic,lambda m,t=topic:sensor(t,m),qos_profile_sensor_data)
        def posture(message):
            now=time.monotonic()
            if now-last.get('posture',0)<.1:return
            last['posture']=now
            expected=profile['posture']['joint_q']
            delta=[float(message.motor_state[i].q)-expected[i] for i in range(12)]
            write('/lf/lowstate',{'joint_delta_rad':delta,
                'roll_pitch':[float(v) for v in message.imu_state.rpy[:2]],
                'battery_soc':int(message.bms_state.soc)})
        node.create_subscription(LowState,'/lf/lowstate',posture,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        end=time.monotonic()+args.duration
        try:
            while not stopped and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        finally:
            node.destroy_node();rclpy.shutdown()
    print(json.dumps({'output':str(target),'counts':counts,'blocking_counts':errors},ensure_ascii=False))


if __name__=='__main__':main()
