#!/usr/bin/python3
"""Isolated real-ROS map fusion/dropout checks; synthetic inputs, no robot IO."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import threading
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--json',required=True);args=p.parse_args()
    os.environ['ROS_DOMAIN_ID']='90'
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster
    root=Path(__file__).resolve().parents[2]
    spec=importlib.util.spec_from_file_location('observed_floor_map_probe',root/'ros2_ws/src/robot_scene_nav_bringup/scripts/observed_floor_map.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rclpy.init();fusion=module.ObservedFloorMap();node=rclpy.create_node('isolated_floor_visibility_probe')
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(fusion);executor.add_node(node)
    thread=threading.Thread(target=executor.spin);thread.start()
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    raw=node.create_publisher(OccupancyGrid,'/go2w/slam/map_raw',latched)
    floor=node.create_publisher(PointCloud2,'/go2w/lidar/floor_observations',qos_profile_sensor_data)
    received=[];node.create_subscription(OccupancyGrid,'/map',lambda m:received.append(m),latched)
    broadcaster=StaticTransformBroadcaster(node);transforms=[]
    for parent,child,z in [('map','odom_fused',0.),('odom_fused','synthetic_lidar',.4)]:
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
        t.transform.rotation.w=1.;t.transform.translation.z=z;transforms.append(t)
    broadcaster.sendTransform(transforms)
    def observe():
        h=Header();h.frame_id='synthetic_lidar';h.stamp=node.get_clock().now().to_msg()
        floor.publish(point_cloud2.create_cloud_xyz32(h,[(.9,0.,-.4)]))
    def wait_map(previous):
        end=time.monotonic()+2
        while len(received)<=previous and time.monotonic()<end:time.sleep(.01)
        assert len(received)>previous
        return received[-1]
    result=dict(domain_id=90,synthetic_inputs=True,motion_dispatch=False)
    try:
        time.sleep(.5)
        assert sorted(node.get_node_names())==['go2w_observed_floor_map','isolated_floor_visibility_probe']
        source=OccupancyGrid();source.header.frame_id='map';source.header.stamp=node.get_clock().now().to_msg()
        source.info.width=source.info.height=30;source.info.resolution=.1
        source.info.origin.position.x=source.info.origin.position.y=-1.5;source.info.origin.orientation.w=1.
        source.data=[-1]*900;source.data[15*30+20]=100
        for _ in range(4):observe();time.sleep(.03)
        old=len(received);raw.publish(source);out=wait_map(old)
        assert sum(v==0 for v in out.data)>0
        assert out.data[15*30+20]==100 and out.data[15*30+21]==-1
        assert out.header.stamp==source.header.stamp
        result.update(finite_observation_clears_near_unknown=True,known_wall_preserved=True,
            behind_wall_stays_unknown=True,source_stamp_preserved=True)
        time.sleep(.9);old=len(received);raw.publish(source);out=wait_map(old)
        assert list(out.data)==list(source.data);result['expired_floor_cannot_clear']=True
        time.sleep(.1);old=len(received)
        for _ in range(8):observe();time.sleep(.03)
        assert len(received)==old;result['floor_alone_cannot_republish_dead_mapper']=True
        Path(args.json).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        executor.shutdown();thread.join();node.destroy_node();fusion.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
