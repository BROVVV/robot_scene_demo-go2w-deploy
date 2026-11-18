from __future__ import annotations

import copy
import json
import math
import os
import signal
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
try:
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # Foxy has no signal_handler_options argument.
    SignalHandlerOptions = None
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String
from unitree_go.msg import LowState
from tf2_ros import Buffer, TransformException, TransformListener

from .config import load_safety_ready_config
from .pointcloud_xyz import read_xyz
from .executor_shutdown import shutdown_executor
from go2w_description.description_config import _rpy_to_quaternion_xyzw
from .mapping_scan_accumulator import MappingScanAccumulator, MappingKeyframeSelector
from .self_returns import RawSelfReturnFilter, measured_low_self_band
from .floor_attitude import (estimate_floor, joint_angles_within_limits,
                             standing_height, rpy_matrix)
from .preprocess_core import (
    collision_obstacles,
    directional_clearance,
    filter_points_base_link,
    laser_scan_ranges,
    sensor_header_fresh,
    transform_points,
)


class LidarPreprocessor(Node):
    def __init__(self) -> None:
        super().__init__("go2w_lidar_preprocessor")
        self.declare_parameter("config_file", "")
        self.declare_parameter("geometry_file", "")
        self.declare_parameter('sensor_profile','')
        self.declare_parameter("mapping_accumulation", True)
        self.declare_parameter("publish_pointclouds", True)
        publish_pointclouds = bool(self.get_parameter("publish_pointclouds").value)
        self.declare_parameter("publish_floor_visibility_2d", False)
        self._floor_2d_pub = None
        if self.get_parameter("publish_floor_visibility_2d").value:
            from go2w_mapping_interfaces.msg import FloorVisibility2D
            self._floor_2d_type = FloorVisibility2D
            self._floor_2d_pub = self.create_publisher(
                FloorVisibility2D, '/go2w/lidar/floor_visibility_2d', qos_profile_sensor_data
            )
        self.declare_parameter("mapping_keyframe_minimum_translation_m", 0.05)
        self.declare_parameter("mapping_keyframe_minimum_rotation_rad", 0.05)
        self._mapping_accumulator = MappingScanAccumulator() if self.get_parameter('mapping_accumulation').value else None
        self._keyframe_selector = MappingKeyframeSelector(
            minimum_translation_m=float(
                self.get_parameter("mapping_keyframe_minimum_translation_m").value
            ),
            minimum_rotation_rad=float(
                self.get_parameter("mapping_keyframe_minimum_rotation_rad").value
            ),
        )
        self._mapper_gids = set()
        self._ready = False
        self._last_valid_ns = None
        self._timestamp_rejected = False
        self._layout_rejected = False
        self._raw_self_filter=None;self._self_removed=0
        self._mapping_self_removed=0
        self._floor_compensation=None;self._floor_urdf=None;self._reference_height=None
        self._body_roll_pitch=None;self._height_delta=0.
        self._floor_check_at=0.;self._floor_check_error='not_checked'
        self._posture_blocking=['posture_not_received']
        self._posture_warnings=[]
        self._posture_max_joint_deviation_rad=None
        self._posture_max_joint_deviation_index=None
        self._static_self_model_eligible=False
        try:
            self._config, self._geometry = load_safety_ready_config(
                str(self.get_parameter("config_file").value),
                str(self.get_parameter("geometry_file").value),
                str(self.get_parameter('sensor_profile').value),
            )
            model=self._config.get('sensor_profile',{}).get('raw_self_return_model')
            if model:
                points=np.load(model['absolute_path'],allow_pickle=False)['points']
                self._raw_self_filter=RawSelfReturnFilter(points,model['radius_m'])
            profile=self._config.get('sensor_profile',{})
            self._floor_compensation=profile.get('floor_attitude_compensation')
            if self._floor_compensation:
                from pathlib import Path
                self._floor_urdf=Path(self._floor_compensation['urdf_absolute_path']).read_bytes()
                self._reference_height=standing_height(self._floor_urdf,
                    profile['posture']['joint_q'],profile['posture']['body_roll_pitch_rad'])
            self._ready = True
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(f"safety gate closed: {exc}")

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self,
            qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._posture_at=0.;self._posture_valid=False
        self._geometry_status_pub=self.create_publisher(String,'/go2w/lidar/geometry_status',10)
        self._floor_pub=(self.create_publisher(PointCloud2,'/go2w/lidar/floor_observations',qos_profile_sensor_data)
                         if publish_pointclouds else None)
        self.create_subscription(LowState,'/lf/lowstate',self._posture,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        self._filtered_pub = self.create_publisher(
            PointCloud2, "/go2w/lidar/cloud_filtered", qos_profile_sensor_data
        ) if publish_pointclouds else None
        self._obstacles_pub = self.create_publisher(
            PointCloud2, "/go2w/lidar/obstacles", qos_profile_sensor_data
        ) if publish_pointclouds else None
        self._collision_obstacles_pub = self.create_publisher(
            PointCloud2, "/go2w/lidar/collision_obstacles", qos_profile_sensor_data
        ) if publish_pointclouds else None
        self._scan_pub = self.create_publisher(
            LaserScan, "/go2w/lidar/scan", qos_profile_sensor_data
        )
        # Nav2's obstacle layer marks even one scan return as lethal, while
        # Collision Monitor intentionally requires >4 points.  Publish an
        # unaccumulated navigation scan with the measured wheel/leg band
        # removed so an intermittent self return cannot deadlock RPP/Spin.
        # The hard collision path continues to consume the conservative raw
        # /go2w/lidar/scan below.
        self._navigation_scan_pub = self.create_publisher(
            LaserScan, "/go2w/lidar/navigation_scan", qos_profile_sensor_data
        )
        # Structural mapping needs wall/furniture returns above the low
        # collision band. Missing returns remain infinity (unknown).
        self._mapping_scan_pub = self.create_publisher(
            LaserScan, "/go2w/lidar/mapping_scan", qos_profile_sensor_data
        )
        self._mapping_status_pub = self.create_publisher(String, '/go2w/lidar/mapping_scan_status', 10)
        self._mapping_keyframe_pub = self.create_publisher(LaserScan, '/go2w/lidar/mapping_keyframe_scan', qos_profile_sensor_data)
        self._clearance_pub = self.create_publisher(
            Vector3Stamped, "/go2w/lidar/clearance", 10
        )
        self._diagnostic_clearance_pub = self.create_publisher(
            Vector3Stamped, "/go2w/diagnostics/lidar_clearance_raw", 10
        )
        self._front_pub = self.create_publisher(
            Float32, "/go2w/safety/front_clearance", 10
        )
        self._left_pub = self.create_publisher(
            Float32, "/go2w/safety/left_clearance", 10
        )
        self._right_pub = self.create_publisher(
            Float32, "/go2w/safety/right_clearance", 10
        )
        self._rotation_clearance_valid_pub = self.create_publisher(
            Bool, "/go2w/safety/rotation_clearance_valid", 10
        )
        self._fresh_pub = self.create_publisher(
            Bool, "/go2w/lidar/preprocess_fresh", 10
        )
        cloud_queue_depth = int(os.environ.get("GO2W_LIDAR_CLOUD_QUEUE_DEPTH", "5"))
        if not 1 <= cloud_queue_depth <= 10:
            raise ValueError("GO2W_LIDAR_CLOUD_QUEUE_DEPTH must be 1..10")
        reliable_cloud_qos = QoSProfile(
            # Jetson uses depth 1 so slow callbacks discard clouds that would
            # fail the 0.3 s safety freshness gate; local default stays 5.
            depth=cloud_queue_depth,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            PointCloud2,
            "/go2w/sensors/cloud",
            self._cloud,
            reliable_cloud_qos,
        )
        self.create_timer(0.1, self._publish_freshness)
        self.create_timer(.25, self._check_mapper_subscribers)

    def _check_mapper_subscribers(self):
        # A new SLAM process needs a fresh initial scan, even if the robot has
        # stayed still since the previous process stopped. Observer subscribers
        # must not trigger another static scan in the mapper.
        gids = {tuple(info.endpoint_gid) for info in self.get_subscriptions_info_by_topic(
            self._mapping_keyframe_pub.topic_name) if info.node_name == 'slam_toolbox'}
        if gids-self._mapper_gids:
            self._keyframe_selector.request_initial_scan()
        self._mapper_gids = gids

    def _cloud(self, message: PointCloud2) -> None:
        if not self._ready:
            self._last_valid_ns = None
            self._fresh_pub.publish(Bool(data=False))
            return
        if self._config.get('sensor_profile') and (not self._posture_valid or time.monotonic()-self._posture_at>.3):
            self._last_valid_ns=None
            return
        if not message.header.frame_id:
            self.get_logger().error("cloud has no frame_id")
            self._last_valid_ns = None
            self._fresh_pub.publish(Bool(data=False))
            return
        now_ns = self.get_clock().now().nanoseconds
        stamp = message.header.stamp
        if not sensor_header_fresh(
            now_ns=now_ns,
            stamp_sec=stamp.sec,
            stamp_nanosec=stamp.nanosec,
            # Source stamps are taken near acquisition start and include the
            # full cloud assembly/transport time.  Keep that independently
            # configurable (0.3 s on Go2-W); the receive-side dropout watchdog
            # below remains capped at 0.2 s, so this does not hide a dead topic.
            timeout_seconds=float(self._config["safety"]["lidar_timeout_seconds"]),
        ):
            if not self._timestamp_rejected:
                self.get_logger().warning(
                    "cloud header timestamp is stale or in the future; "
                    "LiDAR safety freshness revoked"
                )
            self._timestamp_rejected = True
            self._last_valid_ns = None
            self._fresh_pub.publish(Bool(data=False))
            return
        self._timestamp_rejected = False
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(f"base_link TF unavailable: {exc}")
            self._last_valid_ns = None
            self._fresh_pub.publish(Bool(data=False))
            return
        # Foxy's read_points creates one Python tuple per point. Direct byte
        # decoding also preserves Humble's named-field interpretation.
        # Read the same named scalars directly, including organized-cloud row
        # padding. No return is dropped and the packet buffer stays unchanged.
        try:
            source_xyz = read_xyz(message)
        except ValueError as error:
            self._last_valid_ns = None
            self._fresh_pub.publish(Bool(data=False))
            if not self._layout_rejected:
                self.get_logger().warning('invalid PointCloud2 layout: '+str(error))
            self._layout_rejected = True
            return
        self._layout_rejected = False
        # Static posture can use the reviewed 3 cm surface before floor fitting.
        # Moving posture uses the measured gait extension later, after the live
        # floor check and base-frame transform restrict it to the low leg band.
        if self._raw_self_filter is not None and self._static_self_model_eligible:
            keep=self._raw_self_filter.keep(source_xyz)
            self._self_removed=int((~keep).sum())
            source_xyz=source_xyz[keep]
        else:
            self._self_removed=0
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        profile=self._config.get('sensor_profile')
        if profile:
            expected=profile['frames']['base_to_cloud']
            q=np.array([rotation.x,rotation.y,rotation.z,rotation.w])
            eq=np.array(_rpy_to_quaternion_xyzw(tuple(expected['rotation_rpy_rad'])))
            if (message.header.frame_id != expected['child']
                    or np.linalg.norm(np.array([translation.x,translation.y,translation.z])-expected['translation_m'])>1e-5
                    or min(np.linalg.norm(q-eq),np.linalg.norm(q+eq))>1e-5):
                self._last_valid_ns=None
                self._fresh_pub.publish(Bool(data=False))
                return
        floor_estimate=None
        floor_points=None
        need_floor_output=(self._floor_pub is not None
                           or getattr(self,'_floor_2d_pub',None) is not None)
        if self._floor_compensation:
            try:
                floor_estimate=estimate_floor(source_xyz,profile,self._body_roll_pitch,
                                              height_delta_m=self._height_delta)
            except ValueError as exc:
                self._floor_check_error=str(exc);self._floor_check_at=0.
                self._last_valid_ns=None;self._fresh_pub.publish(Bool(data=False))
                return
            self._floor_check_at=time.monotonic();self._floor_check_error=''
            if need_floor_output:
                tolerance=profile['raw_floor_plane']['tolerance_m']
                floor=abs(source_xyz@floor_estimate.normal-floor_estimate.distance_m)<=tolerance
                floor_points=source_xyz[floor]
        elif profile and profile.get('raw_floor_plane') and need_floor_output:
            plane=profile['raw_floor_plane']
            valid=np.isfinite(source_xyz).all(axis=1)
            floor=valid&(abs(source_xyz@np.array(plane['normal'])-plane['distance_m'])<=plane['tolerance_m'])
            floor_points=source_xyz[floor]
        xyz = transform_points(
            source_xyz,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        if self._raw_self_filter is not None and not self._static_self_model_eligible:
            model=profile.get('raw_self_return_model') or {}
            keep=self._raw_self_filter.keep_moving(
                source_xyz,xyz,
                radius=float(model.get('dynamic_radius_m',self._raw_self_filter.radius)),
                region=model.get('dynamic_region_base') or {})
            self._self_removed=int((~keep).sum())
            source_xyz=source_xyz[keep]
            xyz=xyz[keep]
        filtered, obstacles = filter_points_base_link(xyz, self._geometry)
        if floor_estimate is not None:
            extr=profile['frames']['base_to_cloud']
            normal=rpy_matrix(*extr['rotation_rpy_rad'])@floor_estimate.normal
            distance=floor_estimate.distance_m+normal@np.array(extr['translation_m'])
            # Preserve the existing 8cm above-ground obstacle policy, now
            # relative to the verified current plane rather than fixed Z.
            obstacles=filtered[(distance-filtered@normal)>
                               self._floor_compensation['minimum_obstacle_height_m']]
        collision_points = collision_obstacles(obstacles, self._geometry)
        # Structural SLAM and Nav2's single-point obstacle layer must not turn
        # pose-varying wheel/leg returns into permanent/lethal occupied cells.
        # The band is measured and tightly bounded by the sensor profile.  The
        # hard collision scan and clearance path retain every point, so a real
        # nearby low obstacle still reaches Collision Monitor and can be mapped
        # later when observed outside the robot-relative band.
        mapping_obstacles = obstacles
        model=(profile or {}).get('raw_self_return_model') or {}
        region=model.get('dynamic_region_base')
        if region:
            mapping_band=measured_low_self_band(obstacles,region)
            self._mapping_self_removed=int(mapping_band.sum())
            mapping_obstacles=obstacles[~mapping_band]
        else:
            self._mapping_self_removed=0
        header = copy.deepcopy(message.header)
        header.frame_id = "base_link"
        clearance = directional_clearance(collision_points, self._geometry)
        rotation_validation = self._config.get("rotation_clearance_validation") or {}
        rotation_clearance_valid = bool(rotation_validation.get("valid", False))
        clearance_message = Vector3Stamped()
        clearance_message.header = header
        clearance_message.vector.x = clearance.front
        clearance_message.vector.y = (
            clearance.left if rotation_clearance_valid else math.nan
        )
        clearance_message.vector.z = (
            clearance.right if rotation_clearance_valid else math.nan
        )
        self._clearance_pub.publish(clearance_message)
        diagnostic_clearance = Vector3Stamped()
        diagnostic_clearance.header = header
        diagnostic_clearance.vector.x = clearance.front
        diagnostic_clearance.vector.y = clearance.left
        diagnostic_clearance.vector.z = clearance.right
        self._diagnostic_clearance_pub.publish(diagnostic_clearance)
        self._front_pub.publish(Float32(data=clearance.front))
        self._left_pub.publish(
            Float32(data=clearance.left if rotation_clearance_valid else math.nan)
        )
        self._right_pub.publish(
            Float32(data=clearance.right if rotation_clearance_valid else math.nan)
        )
        self._rotation_clearance_valid_pub.publish(
            Bool(data=rotation_clearance_valid)
        )
        self._publish_scan(header, collision_points)
        self._publish_scan(
            header, mapping_obstacles, publisher=self._navigation_scan_pub
        )
        self._last_valid_ns = self.get_clock().now().nanoseconds
        self._fresh_pub.publish(Bool(data=True))
        # Safety scans and clearance use the already validated current cloud.
        # Supplementary floor odometry can wait 40ms, and accumulated mapping
        # can wait another 60ms. Preserve their data and acquisition stamp,
        # while publishing safety outputs before entering either wait.
        if floor_points is not None:
            if self._floor_pub is not None:
                self._floor_pub.publish(point_cloud2.create_cloud_xyz32(message.header,floor_points.tolist()))
            if getattr(self,'_floor_2d_pub',None) is not None:
                self._publish_floor_visibility_2d(message.header,floor_points)
        for publisher, points in ((self._filtered_pub, filtered),
                                  (self._obstacles_pub, obstacles),
                                  (self._collision_obstacles_pub, collision_points)):
            if publisher is not None:
                publisher.publish(point_cloud2.create_cloud_xyz32(header, points.tolist()))
        self._publish_scan(header, mapping_obstacles, publisher=self._mapping_scan_pub)

    def _publish_floor_visibility_2d(self, header, points) -> None:
        try:
            transform=self._tf_buffer.lookup_transform('odom_fused',header.frame_id,
                Time.from_msg(header.stamp),timeout=Duration(seconds=.04))
        except TransformException:
            return
        t,q=transform.transform.translation,transform.transform.rotation
        observed=transform_points(points,(t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
        message=self._floor_2d_type()
        message.header=copy.deepcopy(header)
        message.header.frame_id='odom_fused'
        message.origin_x=float(t.x);message.origin_y=float(t.y)
        message.endpoints_x=observed[:,0].astype(np.float32).tolist()
        message.endpoints_y=observed[:,1].astype(np.float32).tolist()
        self._floor_2d_pub.publish(message)

    def _publish_scan(self, header, obstacles: np.ndarray, *, publisher=None) -> None:
        scan_config = self._config["scan"]
        angle_min = float(scan_config["angle_min_rad"])
        angle_max = float(scan_config["angle_max_rad"])
        angle_increment = float(scan_config["angle_increment_rad"])
        ranges = laser_scan_ranges(
            obstacles,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            range_min=self._geometry.minimum_range,
            range_max=self._geometry.maximum_range,
        )
        if publisher is self._mapping_scan_pub and self._mapping_accumulator is not None:
            try:
                t = self._tf_buffer.lookup_transform('odom_fused', 'base_link',
                    Time.from_msg(header.stamp), timeout=Duration(seconds=.06))
            except TransformException:
                # No unstamped/fallback pose may seed an accumulated map scan.
                self._mapping_accumulator.reset()
                self._mapping_status_pub.publish(String(data=json.dumps(
                    {'mode':'waiting_for_odometry','wall_time':time.time()})))
                return
            p,q = t.transform.translation,t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            ranges = self._mapping_accumulator.add(ranges,
                stamp=header.stamp.sec+header.stamp.nanosec*1e-9,
                pose=(p.x,p.y,yaw), angle_min=angle_min, angle_increment=angle_increment,
                range_min=self._geometry.minimum_range, range_max=self._geometry.maximum_range)
            self._mapping_status_pub.publish(String(data=json.dumps(
                {**self._mapping_accumulator.status,'wall_time':time.time()},allow_nan=False)))
            if ranges is None:
                return
        scan = LaserScan()
        scan.header = header
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (len(ranges) - 1) * angle_increment
        scan.angle_increment = angle_increment
        scan.range_min = self._geometry.minimum_range
        scan.range_max = self._geometry.maximum_range
        scan.ranges = ranges.tolist()
        (publisher or self._scan_pub).publish(scan)
        if publisher is self._mapping_scan_pub and self._mapping_accumulator is not None:
            stable = self._mapping_accumulator.status['mode']=='accumulated_stationary'
            if self._keyframe_selector.select((p.x,p.y,yaw), stable=stable):
                self._mapping_keyframe_pub.publish(scan)

    def _publish_freshness(self) -> None:
        profile=self._config.get('sensor_profile',{}) if self._ready else {}
        posture_blocking=list(getattr(self,'_posture_blocking',[]))
        if time.monotonic()-self._posture_at>.3:
            posture_blocking.append('posture_state_stale')
        if self._floor_compensation and time.monotonic()-self._floor_check_at>.3:
            posture_blocking.append('observed_floor_stale')
        self._geometry_status_pub.publish(String(data=json.dumps(dict(
            wall_time=time.time(),profile_sha256=profile.get('profile_sha256',''),
            execution_scope=profile.get('execution_scope','legacy_reference_unrevalidated'),
            self_surface_points_removed=self._self_removed,
            mapping_low_self_band_points_removed=getattr(self,'_mapping_self_removed',0),
            posture_valid=(self._posture_valid and time.monotonic()-self._posture_at<=.3
                and (not self._floor_compensation or time.monotonic()-self._floor_check_at<=.3)),
            posture_blocking=posture_blocking,
            posture_warnings=list(getattr(self,'_posture_warnings',[])),
            geometry_mode=('static_self_model' if getattr(self,'_static_self_model_eligible',False)
                else 'dynamic_measured_self_surface'),
            maximum_joint_deviation_rad=getattr(self,'_posture_max_joint_deviation_rad',None),
            maximum_joint_deviation_index=getattr(self,'_posture_max_joint_deviation_index',None),
            static_self_model_eligible=bool(getattr(self,'_static_self_model_eligible',False)),
            raw_self_filter_active=bool(getattr(self,'_raw_self_filter',None) is not None),
            allowed_joint_deviation_rad=profile.get('posture',{}).get('maximum_joint_deviation_rad'),
            floor_attitude_compensated=bool(self._floor_compensation),
            floor_check_error=self._floor_check_error if self._floor_compensation else '',
            manufacturer_calibration=False))))
        if not self._ready or self._last_valid_ns is None:
            self._fresh_pub.publish(Bool(data=False))
            return
        timeout_ns = int(
            min(.2, float(self._config["safety"]["lidar_timeout_seconds"])) * 1e9
        )
        age = self.get_clock().now().nanoseconds - self._last_valid_ns
        self._fresh_pub.publish(Bool(data=0 <= age <= timeout_ns))

    def _posture(self,message):
        self._posture_at=time.monotonic()
        profile=self._config.get('sensor_profile') if self._ready else None
        if profile:
            expected=profile['posture']['joint_q']
            deviations=[abs(float(message.motor_state[i].q)-expected[i]) for i in range(12)]
            finite=all(math.isfinite(v) for v in deviations)
            self._posture_blocking=[]
            self._posture_warnings=[]
            self._posture_max_joint_deviation_rad=max(deviations) if finite else None
            self._posture_max_joint_deviation_index=int(np.argmax(deviations)) if finite else None
            self._static_self_model_eligible=finite and max(deviations)<=profile['posture']['maximum_joint_deviation_rad']
            # A small deviation bound certifies the learned raw self-return
            # surface, not the whole LiDAR geometry.  With the independent
            # live-floor + official-URDF path below, ordinary wheel gait can
            # use a conservative fixed-mask fallback without dropping scans.
            self._posture_valid=finite
            if not finite:
                self._posture_blocking.append('nonfinite_joint_state')
            elif not self._static_self_model_eligible:
                if self._floor_compensation:
                    self._posture_warnings.append('static_self_model_outside_envelope')
                else:
                    self._posture_valid=False
                    self._posture_blocking.append('joint_reference_deviation')
            self._body_roll_pitch=[float(v) for v in message.imu_state.rpy[:2]]
            if self._floor_compensation:
                # The strict four-wheel-same-height assumption describes a
                # parked pose.  Normal rolling gait briefly articulates the
                # four wheel legs by >4 cm even on a level floor.  Outside the
                # stationary model envelope, keep official joint limits and
                # IMU bounds as gates and let estimate_floor() independently
                # verify the calibrated body height on every cloud.
                try:
                    joint_q=[float(v.q) for v in message.motor_state[:12]]
                    if self._static_self_model_eligible:
                        self._height_delta=standing_height(
                            self._floor_urdf,joint_q,self._body_roll_pitch)-self._reference_height
                    else:
                        if (not joint_angles_within_limits(self._floor_urdf,joint_q)
                                or not all(math.isfinite(v) for v in self._body_roll_pitch)
                                or max(abs(v) for v in self._body_roll_pitch)>math.radians(10)):
                            raise ValueError('invalid dynamic kinematics')
                        self._height_delta=0.
                except ValueError as exc:
                    self._posture_valid=False
                    self._posture_blocking.append('standing_kinematics: '+str(exc))
            else:
                attitude_valid=all(
                math.isfinite(float(message.imu_state.rpy[i])) and
                abs(float(message.imu_state.rpy[i])-profile['posture']['body_roll_pitch_rad'][i])
                    <=profile['posture']['maximum_attitude_deviation_rad'] for i in range(2))
                self._posture_valid=self._posture_valid and attitude_valid
                if not attitude_valid:self._posture_blocking.append('body_attitude_deviation')


def main(args=None) -> None:
    # Finish worker callbacks before invalidating their publisher context.
    if SignalHandlerOptions is None:
        rclpy.init(args=args)
    else:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopped = threading.Event()
    previous_handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    for s in previous_handlers:
        signal.signal(s, lambda *_: stopped.set())
    node = LidarPreprocessor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not stopped.is_set():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown_executor(executor)
        node.destroy_node()
        for s, handler in previous_handlers.items():
            signal.signal(s, handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
