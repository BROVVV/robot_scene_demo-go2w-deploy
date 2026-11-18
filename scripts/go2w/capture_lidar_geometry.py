#!/usr/bin/python3
"""Capture raw/processed geometry plus atomic RGB-D, without motion outputs."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',required=True);p.add_argument('--duration',type=float,default=5)
    p.add_argument('--raw-only',action='store_true',help='calibration capture when posture guard correctly blocks processed clouds')
    args=p.parse_args()
    if not 1<=args.duration<=30:raise ValueError('capture duration must be 1..30 seconds')
    out=Path(args.output_dir).resolve();out.mkdir(parents=True,exist_ok=True)
    if (out/'metadata.json').exists():raise ValueError('use a new capture directory')
    from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource
    from sensor_msgs.msg import PointCloud2,Imu
    from sensor_msgs_py import point_cloud2
    from nav_msgs.msg import Odometry
    from unitree_go.msg import SportModeState
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    frame=RealSenseHTTPRGBDSource(cache_dir=out/'rgbd').get_latest(timeout_seconds=10)
    (out/'rgbd_materialized.json').write_text(json.dumps(asdict(frame),indent=2)+'\n')
    rclpy.init();node=rclpy.create_node('go2w_geometry_readonly_capture')
    clouds={};meta={'captured_at_unix':time.time(),'motion_sent':False,'raw_only':args.raw_only,'clouds':{},'imu':[],'poses':[],'sport':[]}
    def cloud(m,topic):
        points=point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True)
        if getattr(getattr(points,'dtype',None),'names',None):
            xyz=np.column_stack([points[k] for k in ('x','y','z')]).astype(np.float32)
        else:xyz=np.asarray(list(points),dtype=np.float32).reshape(-1,3)
        clouds.setdefault(topic,[]).append(xyz)
        meta['clouds'].setdefault(topic,[]).append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            'frame':m.header.frame_id,'points':len(xyz)})
    topics=(('/go2w/sensors/cloud',) if args.raw_only else
            ('/go2w/sensors/cloud','/go2w/lidar/obstacles','/go2w/lidar/collision_obstacles'))
    for topic in topics:
        node.create_subscription(PointCloud2,topic,lambda m,t=topic:cloud(m,t),qos_profile_sensor_data)
    node.create_subscription(Imu,'/go2w/sensors/lidar_imu',lambda m:meta['imu'].append({
        'accel':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],
        'quaternion':[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w]}),qos_profile_sensor_data)
    node.create_subscription(Odometry,'/go2w/odom/fused',lambda m:meta['poses'].append({
        'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,
        'q':[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w]}),qos_profile_sensor_data)
    node.create_subscription(SportModeState,'/lf/sportmodestate',lambda m:meta['sport'].append({
        'rpy':[float(v) for v in m.imu_state.rpy],'mode':int(m.mode),'error_code':int(m.error_code)}),
        QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE))
    end=time.monotonic()+args.duration
    try:
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        node.destroy_node();rclpy.shutdown()
    if len(clouds)!=len(topics) or not meta['poses']:raise RuntimeError('incomplete geometry capture')
    for topic,frames in clouds.items():
        np.savez_compressed(out/(topic.rsplit('/',1)[-1]+'.npz'),points=np.concatenate(frames),sizes=[len(v) for v in frames])
    (out/'metadata.json').write_text(json.dumps(meta)+'\n')
    print(json.dumps({'output_dir':str(out),'motion_sent':False,'rgb':frame.color_ref,
                      'frames':{k:len(v) for k,v in clouds.items()},'last_pose':meta['poses'][-1]}))


if __name__=='__main__':main()
