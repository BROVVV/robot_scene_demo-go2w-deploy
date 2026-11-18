#!/usr/bin/env python3
"""Passively record raw LiDAR and nearest received leg posture for later review.

No publishers, services, actions or robot RPCs are created. Receive timestamps
come from one monotonic clock; a near receive time is not sensor clock sync.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np


def finite_or_none(value):
    number=float(value)
    return number if math.isfinite(number) else None


def pair_clouds_to_states(cloud_received_ns, state_received_ns, *, tolerance_ns=30_000_000):
    """Nearest receive-time association; -1 means there is no close state."""
    clouds=np.asarray(cloud_received_ns,dtype=np.int64)
    states=np.asarray(state_received_ns,dtype=np.int64)
    if clouds.ndim!=1 or states.ndim!=1 or tolerance_ns<0:
        raise ValueError('invalid capture timestamps')
    if len(states)==0:
        return np.full(len(clouds),-1,dtype=np.int64)
    if np.any(np.diff(states)<0):
        raise ValueError('state receive timestamps must be ordered')
    right=np.searchsorted(states,clouds,side='left')
    left=np.clip(right-1,0,len(states)-1)
    right=np.clip(right,0,len(states)-1)
    nearest=np.where(np.abs(states[left]-clouds)<=np.abs(states[right]-clouds),left,right)
    nearest[np.abs(states[nearest]-clouds)>tolerance_ns]=-1
    return nearest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--duration',type=float,default=15.)
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 1<=args.duration<=30:
        parser.error('--duration must be 1..30 seconds')
    output=Path(args.output_dir).resolve()
    output.mkdir(parents=True,exist_ok=False)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from unitree_go.msg import LowState

    clouds=[];cloud_meta=[];states=[];state_received=[]
    start_wall_ns=time.time_ns()
    rclpy.init()
    node=rclpy.create_node('go2w_motion_geometry_passive_capture')

    def receive_cloud(message):
        received=time.monotonic_ns()
        if len(clouds)>=600:
            raise RuntimeError('raw cloud frame limit reached')
        points=point_cloud2.read_points(message,field_names=('x','y','z'),skip_nans=True)
        if getattr(getattr(points,'dtype',None),'names',None):
            xyz=np.column_stack([points[key] for key in ('x','y','z')]).astype(np.float32)
        else:
            xyz=np.asarray(list(points),dtype=np.float32).reshape(-1,3)
        if len(xyz)>250_000:
            raise RuntimeError('raw cloud point limit reached')
        clouds.append(xyz)
        cloud_meta.append(dict(received_monotonic_ns=received,
            header_stamp_ns=int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec),
            frame_id=message.header.frame_id,points=len(xyz)))

    def receive_state(message, source_topic):
        if len(states)>=20_000:
            raise RuntimeError('lowstate sample limit reached')
        received=time.monotonic_ns()
        q=[finite_or_none(message.motor_state[i].q) for i in range(12)]
        rpy=[finite_or_none(message.imu_state.rpy[i]) for i in range(3)]
        state_received.append(received)
        states.append(dict(source_topic=source_topic,
                           q=q,rpy=rpy,state_finite=all(v is not None for v in q+rpy),
                           battery_soc=int(message.bms_state.soc)))

    node.create_subscription(PointCloud2,'/go2w/sensors/cloud',receive_cloud,qos_profile_sensor_data)
    node.create_subscription(
        LowState, '/lf/lowstate',
        lambda message: receive_state(message, '/lf/lowstate'),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        LowState, '/go2w/motion/local_lowstate',
        lambda message: receive_state(message, '/go2w/motion/local_lowstate'),
        QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE),
    )
    try:
        end=time.monotonic()+args.duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
    finally:
        node.destroy_node();rclpy.shutdown()

    matches=pair_clouds_to_states([c['received_monotonic_ns'] for c in cloud_meta],state_received)
    for item,index in zip(cloud_meta,matches):
        item['nearest_state_index']=int(index)
        item['state_receive_delta_ms']=(
            (state_received[index]-item['received_monotonic_ns'])/1e6 if index>=0 else None)
    metadata=dict(schema='go2w.passive_motion_geometry.v1',start_wall_ns=start_wall_ns,
        duration_requested_s=args.duration,motion_command_sent=False,
        association='nearest_local_receipt_time; not hardware synchronized',
        association_tolerance_ms=30,clouds=cloud_meta,
        states=[dict(received_monotonic_ns=received,**state)
                for received,state in zip(state_received,states)])
    (output/'metadata.json').write_text(json.dumps(metadata,allow_nan=False)+'\n')
    if clouds:
        np.savez_compressed(output/'raw_cloud.npz',
            points=np.concatenate(clouds),sizes=np.asarray([len(c) for c in clouds],dtype=np.int32))
    print(json.dumps(dict(output_dir=str(output),cloud_frames=len(clouds),
        state_samples=len(states),paired_frames=int(np.count_nonzero(matches>=0)),
        motion_command_sent=False)))


if __name__=='__main__':
    main()
