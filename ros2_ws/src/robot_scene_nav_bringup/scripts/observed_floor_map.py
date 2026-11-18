#!/usr/bin/python3
"""Add actual floor visibility to SLAM's map without inventing max-range rays."""
import copy
from array import array
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0,os.environ.get('GO2W_PROJECT_ROOT',str(Path(__file__).resolve().parents[4])))
from app.navigation.floor_visibility import (FloorRayMemory, add_floor_visibility,
                                             measured_floor_endpoints)
from app.navigation.corridor_analyzer import clear_current_physical_footprint_data
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener


def matrix(transform):
    p,q=transform.transform.translation,transform.transform.rotation
    x,y,z,w=q.x,q.y,q.z,q.w
    if not all(math.isfinite(v) for v in (p.x,p.y,p.z,x,y,z,w)) or abs(x*x+y*y+z*z+w*w-1)>.001:
        raise ValueError('invalid floor observation transform')
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return R,np.array([p.x,p.y,p.z])


class ObservedFloorMap(Node):
    def __init__(self):
        super().__init__('go2w_observed_floor_map')
        self.buffer=Buffer(cache_time=Duration(seconds=10.))
        self.listener=TransformListener(
            self.buffer,self,
            qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        # Keep measured ground evidence for this mapping epoch.  A short
        # time-only deque made cells flicker back to unknown after 0.8 s and
        # could make a dry-run pass immediately before execution rejected the
        # identical footprint.  The memory is spatially/angularly bounded and
        # dies with this node; known occupied cells still win in
        # add_floor_visibility().
        self.rays=FloorRayMemory()
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(OccupancyGrid,'/map',latched)
        self.status=self.create_publisher(String,'/go2w/nav/floor_visibility_status',10)
        self.create_subscription(OccupancyGrid,'/go2w/slam/map_raw',self.on_map,latched)
        default_mode='rays_2d' if os.environ.get('GO2W_PERCEPTION_MODE')=='2d' else 'pointcloud'
        self.declare_parameter('floor_input_mode',default_mode)
        self.floor_input_mode=str(self.get_parameter('floor_input_mode').value)
        if self.floor_input_mode=='rays_2d':
            from go2w_mapping_interfaces.msg import FloorVisibility2D
            self.create_subscription(FloorVisibility2D,'/go2w/lidar/floor_visibility_2d',
                self.on_floor_2d,qos_profile_sensor_data)
        elif self.floor_input_mode=='pointcloud':
            self.create_subscription(PointCloud2,'/go2w/lidar/floor_observations',self.on_floor,qos_profile_sensor_data)
        else:
            raise ValueError('floor_input_mode must be rays_2d or pointcloud')

    def on_floor_2d(self,message):
        stamp=message.header.stamp;seconds=stamp.sec+stamp.nanosec*1e-9
        if (message.header.frame_id!='odom_fused'
                or not -.05<=self.get_clock().now().nanoseconds/1e9-seconds<=.2
                or len(message.endpoints_x)!=len(message.endpoints_y)):
            return
        try:
            origin=np.array([message.origin_x,message.origin_y,0.])
            points=np.column_stack((message.endpoints_x,message.endpoints_y,
                                    np.zeros(len(message.endpoints_x))))
            ends=measured_floor_endpoints(points,origin)
            if len(ends):self.rays.add(origin,ends)
        except (ValueError,TypeError):return

    def on_floor(self,message):
        stamp=message.header.stamp;seconds=stamp.sec+stamp.nanosec*1e-9
        if not -.05<=self.get_clock().now().nanoseconds/1e9-seconds<=.2:return
        try:
            tf=self.buffer.lookup_transform('odom_fused',message.header.frame_id,rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=.04))
            R,t=matrix(tf)
            records=point_cloud2.read_points(message,field_names=('x','y','z'),skip_nans=True)
            if getattr(getattr(records,'dtype',None),'names',None):
                points=np.column_stack([records[k] for k in ('x','y','z')])
            else:points=np.asarray(list(records)).reshape(-1,3)
            if not len(points):return
            observed=points@R.T+t
            # Quantize the REAL horizontal projection around the acquisition
            # origin, not the tilted raw sensor XY plane. Keep actual returns
            # at cell-scale angular spacing; no synthesized/max-range rays.
            ends=measured_floor_endpoints(observed,t)
            if not len(ends):return
            self.rays.add(t,ends)
        except Exception:return

    def on_map(self,message):
        result=copy.deepcopy(message);used=0
        published_data=message.data
        floor_data=None
        footprint_clear=dict(published_cells_cleared=0,
            published_occupied_cells_cleared=0,published_unknown_cells_cleared=0,
            footprint_padding_masked=False)
        if message.header.frame_id!='map':return
        try:
            tf=self.buffer.lookup_transform('map','odom_fused',rclpy.time.Time())
            R,t=matrix(tf);rays=[]
            for start,end in self.rays.rays():
                a=R@start+t;b=R@end+t
                rays.append((a[:2],b[:2]))
            used=len(rays)
            floor_data=add_floor_visibility(message.data,width=message.info.width,height=message.info.height,
                resolution=message.info.resolution,origin=np.array([message.info.origin.position.x,message.info.origin.position.y]),rays=rays)
            # Keep measured floor evidence even if the independent current
            # base-pose lookup below is temporarily unavailable.
            published_data=floor_data
            # The valid current base pose proves that cells centred under the
            # physical body cannot be a fixed wall. Clear only that rectangle;
            # the 4 cm padding ring, future swept area and live collision scan
            # remain untouched. This prevents stale self-returns from trapping
            # NavFn inside its own static layer.
            base_tf=self.buffer.lookup_transform('map','base_link',rclpy.time.Time())
            base_R,base_t=matrix(base_tf)
            map_stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            base_stamp=base_tf.header.stamp.sec+base_tf.header.stamp.nanosec*1e-9
            if not -.30 <= map_stamp-base_stamp <= .30:
                raise ValueError('current footprint transform is not aligned with source map')
            published_data,footprint_clear=clear_current_physical_footprint_data(
                floor_data,width=message.info.width,height=message.info.height,
                resolution=message.info.resolution,
                origin_x=message.info.origin.position.x,
                origin_y=message.info.origin.position.y,
                robot_x=base_t[0],robot_y=base_t[1],
                robot_yaw=math.atan2(base_R[1,0],base_R[0,0]))
        except Exception:pass
        # ROS's generated list setter validates every element in Python.
        # An int8 array is the same wire representation and avoids validating
        # the whole grid twice. Assign only once after all reconciliation.
        result.data=array('b',published_data)
        original=np.asarray(message.data,dtype=np.int8)
        published=np.asarray(result.data,dtype=np.int8)
        unknown_cleared=int(np.count_nonzero((original<0)&(published==0)))
        occupied_removed=(int(np.count_nonzero((original>=65)&
            (np.asarray(floor_data,dtype=np.int8)<65))) if floor_data is not None else 0)
        # Preserve SLAM's source timestamp: continuing floor observations
        # must never keep a dead mapper looking fresh.
        self.pub.publish(result)
        self.status.publish(String(data=json.dumps(dict(wall_time=time.time(),rays=used,
            input_mode=self.floor_input_mode,
            evidence_origins=self.rays.origin_count,persistent_measured_evidence=True,
            observed_unknown_cells_cleared=unknown_cleared,
            # Preserve the old safety field's meaning: floor rays never erase
            # occupied cells. Current-body reconciliation is reported below.
            occupied_cells_removed=occupied_removed,
            current_physical_footprint=footprint_clear,
            source_stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9))))


def floor_executor_class(input_mode, selection):
    if selection not in ('multi','single'):
        raise ValueError('GO2W_FLOOR_MAP_EXECUTOR must be multi or single')
    # The legacy point-cloud callback waits for TF and needs another executor
    # thread to receive it. Planar callbacks use nonblocking lookups only.
    if selection=='single' and input_mode!='rays_2d':
        raise ValueError('single floor executor requires rays_2d input')
    return SingleThreadedExecutor if selection=='single' else MultiThreadedExecutor


def main():
    selection=os.environ.get('GO2W_FLOOR_MAP_EXECUTOR','multi')
    if selection not in ('multi','single'):
        raise SystemExit('GO2W_FLOOR_MAP_EXECUTOR must be multi or single')
    rclpy.init();n=ObservedFloorMap();executor=None
    try:
        executor_type=floor_executor_class(n.floor_input_mode,selection)
        executor=(executor_type() if selection=='single' else executor_type(num_threads=2))
        executor.add_node(n);executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        if executor is not None:executor.shutdown()
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
