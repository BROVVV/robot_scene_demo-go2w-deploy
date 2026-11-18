"""2D navigation map/TF adapter. Missing health never falls back to wheel coordinates."""
from __future__ import annotations

import math
import json
import time

import numpy as np

from app.navigation.localization_health import map_statistics
from app.mapping.map_quality_2d import Grid2D, compare_world_aligned, occupancy_counts
from app.spatial.models import SPATIAL_QUALITY_METRIC_LIDAR, SPATIAL_QUALITY_NO_GLOBAL_POSE, SpatialPose
from app.spatial.occupancy_grid_2d import OccupancyGrid2D
from app.spatial.plain_slam_spatial_provider import PlainSlamSpatialProvider
from app.slam.slam_bridge import SLAMBridge
from app.slam.keyframe_graph import KeyframeGraphStore


class Nav2SpatialProvider(PlainSlamSpatialProvider):
    _PASSIVE_SPARSE_SCAN_BLOCKERS = frozenset({
        'scan_returns', 'mapping_scan_returns',
    })
    _PASSIVE_SPATIAL_REQUIRED_CHECKS = frozenset({
        'map_unique', 'map_valid', 'map_current',
        'scan_unique', 'scan_fresh', 'scan_monotonic',
        'odom_unique', 'odom_fresh', 'odom_frames', 'pose_finite',
        'localizer_active', 'nodes_unique', 'no_pose_jump',
        'map->odom_fused:unique', 'map->odom_fused:fresh',
        'map->odom_fused:finite', 'odom_fused->base_link:unique',
        'odom_fused->base_link:fresh', 'odom_fused->base_link:finite',
        'mapping_scan_unique', 'mapping_scan_fresh',
        'mapping_scan_monotonic',
    })

    def __init__(self, *, enable_ros=False,
                 allow_passive_mapping_sparse_scans=False, **kwargs):
        self._valid = False
        self._valid_at = 0.
        self._pose_diagnostic = {'reason': 'not_queried'}
        # Passive semantic mapping never authorizes motion.  In an open outdoor
        # scene the collision/mapping scans can legitimately contain fewer than
        # ten returns even while map, odometry and both TF edges are healthy.
        # Keep the global localization safety verdict untouched and opt in only
        # at the read-only consumer that needs a projection pose.
        self._allow_passive_mapping_sparse_scans = bool(
            allow_passive_mapping_sparse_scans
        )
        self._valid_basis = 'unavailable'
        self._keyframe_graph = KeyframeGraphStore()
        self._quality_grid = None
        self._spatial_grid = None
        self._map_quality = {"state": "UNAVAILABLE", "reason": "no_map"}
        super().__init__(enable_ros=enable_ros, map_topic='/map', odom_topic='/go2w/odom/fused',
                         map_frame='map', odom_frame='odom_fused', pose_stale_s=.3, **kwargs)
        # Long-term frontiers may be far away. Per-goal travel and clearance
        # remain bounded by the executor/backend, not this discovery radius.
        self.frontier_extractor.max_distance_m = math.inf
        self._transform_source = 'unavailable'

    def _enable_ros(self, *, map_topic, odom_topic):
        import rclpy
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener
        from rclpy.duration import Duration
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node('go2w_nav2_spatial_provider')
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=600.))
        # The listener's default depth-100 dynamic queue can retain seconds
        # of old poses while health (depth 1) has already caught up. Match the
        # bounded sensor inputs; keep the 600-second TF cache and the default
        # latched static-TF subscription unchanged. Missing history still
        # fails the timestamped lookup rather than using a latest-pose fallback.
        self._tf_listener = TransformListener(self._tf_buffer, self._node, qos=sensor)
        self._map_bridge = SLAMBridge(map_frame='map', odom_frame='odom_fused', base_frame='base_link',
                                     camera_frame=self.camera_frame, camera_translation_m=self.nominal_translation_m,
                                     buffer_seconds=600, max_samples=12000)
        self._subs = [self._node.create_subscription(OccupancyGrid, map_topic, self._on_occupancy_grid, latched),
                      self._node.create_subscription(Odometry, odom_topic, self._on_odometry, sensor),
                      self._node.create_subscription(TFMessage, '/tf', self._on_tf, sensor),
                      self._node.create_subscription(String, '/go2w/nav/localization_status', self._on_valid, 1),
                      self._node.create_subscription(String, '/go2w/slam/keyframe_graph',
                                                     self._on_keyframe_graph, latched)]
        # Keep dated TF/map/health inputs flowing during long vision calls.
        import threading
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True,
                                             name='nav2-spatial-inputs')
        self._spin_thread.start()

    def spin_once(self):
        # The provider owns its executor; never add it to a second executor.
        return None

    def close(self):
        executor = getattr(self, '_executor', None)
        if executor is not None:
            self._executor = None
            executor.shutdown(timeout_sec=2.)
            self._spin_thread.join(timeout=2.)
        super().close()

    def _on_valid(self, message):
        try:
            status = json.loads(message.data)
            current = -.1 <= time.time()-float(status['wall_time']) <= .3
            globally_valid = status.get('valid') is True
            passive_sparse_valid = (
                self._allow_passive_mapping_sparse_scans
                and self._passive_sparse_mapping_health(status)
            )
            self._valid = current and (globally_valid or passive_sparse_valid)
            self._valid_basis = (
                'navigation_health' if current and globally_valid
                else 'passive_mapping_sparse_scan' if current and passive_sparse_valid
                else 'unavailable'
            )
        except (ValueError, KeyError, TypeError):
            self._valid = False
            self._valid_basis = 'unavailable'
        self._valid_at = time.monotonic()

    @classmethod
    def _passive_sparse_mapping_health(cls, status):
        """Accept only the sparse-return exception for read-only map projection.

        This is deliberately stricter than looking at ``blocking`` alone: all
        map/odom/TF ownership, freshness and finite-pose checks must be present
        and true.  The published Nav2 health remains false, so no motion consumer
        can inherit this passive-only exception.
        """
        if status.get('mode') != 'mapping':
            return False
        blocking = status.get('blocking')
        checks = status.get('checks')
        if not isinstance(blocking, list) or not isinstance(checks, dict):
            return False
        blockers = set(blocking)
        if not blockers or not blockers <= cls._PASSIVE_SPARSE_SCAN_BLOCKERS:
            return False
        return all(checks.get(name) is True
                   for name in cls._PASSIVE_SPATIAL_REQUIRED_CHECKS)

    def _on_keyframe_graph(self, message):
        try:
            accepted = self._keyframe_graph.update_json(message.data)
            snapshot = self._keyframe_graph.snapshot()
            if accepted and snapshot is not None and self._map is not None:
                self._map.provenance = {
                    **dict(self._map.provenance or {}),
                    **self._keyframe_provenance(snapshot),
                }
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._last_error = 'invalid_keyframe_graph: ' + str(exc)

    @staticmethod
    def _keyframe_provenance(snapshot):
        return {
            'map_id': snapshot.map_id,
            'map_epoch': snapshot.map_epoch,
            'graph_revision': snapshot.graph_revision,
            'sensor_profile_id': snapshot.sensor_profile_id,
        }

    def _healthy(self):
        return self._valid and time.monotonic() - self._valid_at <= .3

    def _on_occupancy_grid(self, message):
        if message.header.frame_id != 'map' or not map_statistics(
                message.info.width, message.info.height, message.info.resolution, message.data)['valid']:
            self._last_error = 'invalid_navigation_map'
            self._map = None
            return
        graph = self._keyframe_graph.snapshot()
        q = message.info.origin.orientation
        origin_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        self._spatial_grid = OccupancyGrid2D(
            width=int(message.info.width), height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y), origin_yaw=origin_yaw,
            data=tuple(int(value) for value in message.data),
            map_epoch=(graph.map_epoch if graph is not None else None),
        )
        quality_grid = Grid2D(
            data=np.asarray(message.data, dtype=np.int16).reshape(
                int(message.info.height), int(message.info.width)
            ),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            frame_id=str(message.header.frame_id),
            map_epoch=(graph.map_epoch if graph is not None else None),
        )
        change = {"reason": "first_map_for_epoch"}
        previous = self._quality_grid
        if (previous is not None and previous.map_epoch is not None
                and quality_grid.map_epoch == previous.map_epoch):
            try:
                change = compare_world_aligned(previous, quality_grid)
            except ValueError as exc:
                change = {"reason": str(exc)}
        self._quality_grid = quality_grid
        self._map_quality = {
            "state": "MEASURED",
            "captured_at": time.time(),
            "map_epoch": quality_grid.map_epoch,
            "graph_revision": graph.graph_revision if graph is not None else None,
            "counts": occupancy_counts(quality_grid),
            "world_aligned_change": change,
        }
        super()._on_occupancy_grid(message)
        self._map.source = 'slam_toolbox_2d'
        self._map.provenance = {'frame_id': 'map', 'mapping_mode': 'navigation_2d',
                                'generator': 'slam_toolbox_or_validated_map_server'}
        # Occupancy grids are periodic while keyframe graphs publish only on
        # content changes.  Never let a later map callback erase the identity
        # and revision already attached by the latched graph message.
        snapshot = self._keyframe_graph.snapshot()
        if snapshot is not None:
            self._map.provenance.update(self._keyframe_provenance(snapshot))

    def _on_odometry(self, message):
        if (message.header.frame_id != 'odom_fused' or message.child_frame_id != 'base_link'
                or self._map_bridge is None):
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self._map_bridge.note_odom_to_base(message.header.stamp.sec + message.header.stamp.nanosec*1e-9,
                                         (p.x, p.y, yaw))

    def get_pose(self):
        if not self._healthy():
            self._record_pose_diagnostic('localization_health_unavailable')
            return None
        if self._tf_buffer is None:
            self._record_pose_diagnostic('tf_buffer_unavailable')
            return None
        import rclpy
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            age = self._node.get_clock().now().nanoseconds/1e9 - (t.header.stamp.sec+t.header.stamp.nanosec*1e-9)
            if not -.3 <= age <= .3:
                self._record_pose_diagnostic('tf_pose_timestamp_out_of_range',
                                             tf_age_s=age if math.isfinite(age) else None)
                return None
            p, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
                self._record_pose_diagnostic('tf_pose_nonfinite')
                return None
            self._pose = SpatialPose(p.x, p.y, yaw, frame_id='map', quality=SPATIAL_QUALITY_METRIC_LIDAR,
                                     source='tf:map->base_link')
            self._pose_ts = time.monotonic()
            self._record_pose_diagnostic('available', tf_age_s=age)
            return self._pose
        except Exception as exc:
            self._record_pose_diagnostic('tf_pose_lookup_failed', detail=str(exc))
            return None

    def _record_pose_diagnostic(self, reason, **details):
        self._pose_diagnostic = {
            'reason': reason, 'wall_time': time.time(),
            'localization_valid': bool(self._valid),
            'localization_valid_basis': self._valid_basis,
            'localization_receive_age_s': (
                time.monotonic() - self._valid_at if self._valid_at > 0 else None),
            **details,
        }

    def pose_diagnostics(self):
        """Last pose attempt only; this snapshot grants no pose or motion."""
        return dict(self._pose_diagnostic)

    def set_pose(self, pose):
        # The metric pose has exactly one source: TF. Application snapshots cannot replace it.
        if pose is not None and pose.frame_id != 'map':
            from app.spatial.models import SpatialFrameMismatch
            raise SpatialFrameMismatch(pose.frame_id, 'map')

    def get_map(self):
        return self._map if self._healthy() else None

    def quality(self):
        return SPATIAL_QUALITY_METRIC_LIDAR if self.get_map() is not None and self.get_pose() is not None else SPATIAL_QUALITY_NO_GLOBAL_POSE

    def get_frontiers(self):
        grid, pose = self.get_map(), self.get_pose()
        return self.frontier_extractor.extract(grid, pose) if grid is not None and pose is not None else []

    def _camera_transforms(self, timestamp):
        import rclpy
        from rclpy.duration import Duration
        # Use the latest SLAM correction and the acquisition-time camera pose.
        # Looking up old map->camera directly would preserve old loop-closure
        # errors, while latest base pose would project a moving image wrongly.
        correction=self._tf_buffer.lookup_transform('map','odom_fused',rclpy.time.Time())
        camera=self._tf_buffer.lookup_transform('odom_fused',self.camera_frame,
            rclpy.time.Time(seconds=float(timestamp)), timeout=Duration(seconds=.15))
        return correction,camera

    @staticmethod
    def _tf_yaw(transform):
        q = transform.transform.rotation
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def _base_pose_at(self, timestamp):
        """Return acquisition-time ``T_map_base`` using the latest map correction."""
        import rclpy
        from rclpy.duration import Duration
        correction = self._tf_buffer.lookup_transform(
            'map', 'odom_fused', rclpy.time.Time()
        )
        base = self._tf_buffer.lookup_transform(
            'odom_fused', 'base_link', rclpy.time.Time(seconds=float(timestamp)),
            timeout=Duration(seconds=.15),
        )
        position = self._tf_point((0., 0., 0.), base)
        position = self._tf_point(position, correction)
        yaw = self._tf_yaw(correction) + self._tf_yaw(base)
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        return position[0], position[1], yaw

    @staticmethod
    def _tf_point(point, transform):
        t,q=transform.transform.translation,transform.transform.rotation
        values=(*point,t.x,t.y,t.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite camera transform')
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3:
            raise ValueError('invalid camera transform rotation')
        x,y,z=point;qx,qy,qz,qw=q.x,q.y,q.z,q.w
        return (
            t.x+(1-2*(qy*qy+qz*qz))*x+2*(qx*qy-qz*qw)*y+2*(qx*qz+qy*qw)*z,
            t.y+2*(qx*qy+qz*qw)*x+(1-2*(qx*qx+qz*qz))*y+2*(qy*qz-qx*qw)*z,
            t.z+2*(qx*qz-qy*qw)*x+2*(qy*qz+qx*qw)*y+(1-2*(qx*qx+qy*qy))*z,
        )

    @staticmethod
    def _inverse_tf_point(point, transform):
        """Apply the inverse of a geometry_msgs TransformStamped."""
        t, q = transform.transform.translation, transform.transform.rotation
        shifted = (point[0] - t.x, point[1] - t.y, point[2] - t.z)
        inverse = type("InverseTransform", (), {})()
        inverse.transform = type("Transform", (), {})()
        inverse.transform.translation = type("Translation", (), {"x": 0., "y": 0., "z": 0.})()
        inverse.transform.rotation = type(
            "Rotation", (), {"x": -q.x, "y": -q.y, "z": -q.z, "w": q.w}
        )()
        return Nav2SpatialProvider._tf_point(shifted, inverse)

    def camera_point_to_map(self, xyz_camera, timestamp=None):
        if (xyz_camera is None or timestamp is None or not math.isfinite(timestamp)
                or not self._healthy() or self._tf_buffer is None):
            return None
        try:
            correction, camera = self._camera_transforms(timestamp)
            point = self._tf_point(self._tf_point(xyz_camera, camera), correction)
            self._transform_source = 'tf2'
            self._last_error = ''
            return point
        except Exception as exc:
            self._last_error = 'image_time_camera_tf_unavailable: ' + str(exc)
            return None

    def map_point_to_camera(self, xyz_map, timestamp=None):
        """Project a durable map point into the acquisition-time camera frame."""
        if (xyz_map is None or timestamp is None or not math.isfinite(timestamp)
                or not self._healthy() or self._tf_buffer is None):
            return None
        try:
            correction, camera = self._camera_transforms(timestamp)
            point_odom = self._inverse_tf_point(tuple(float(v) for v in xyz_map), correction)
            return self._inverse_tf_point(point_odom, camera)
        except Exception as exc:
            self._last_error = 'map_to_image_camera_tf_unavailable: ' + str(exc)
            return None

    def spatial_pair_evidence(self, source, target, distance):
        """Reject a geometric edge when the 2-D map proves a wall between it."""
        grid = self._spatial_grid
        if grid is None:
            return True, "occupancy_grid_unavailable"
        source_xyz = getattr(source, "map_xyz", None)
        target_xyz = getattr(target, "map_xyz", None)
        if source_xyz is None or target_xyz is None:
            return False, "missing_map_coordinate"
        source_epoch = str((getattr(source, "provenance", None) or {}).get("map_epoch") or "")
        target_epoch = str((getattr(target, "provenance", None) or {}).get("map_epoch") or "")
        if grid.map_epoch and (source_epoch and source_epoch != grid.map_epoch
                               or target_epoch and target_epoch != grid.map_epoch):
            return False, "occupancy_grid_epoch_mismatch"
        return grid.line_evidence(
            (float(source_xyz[0]), float(source_xyz[1])),
            (float(target_xyz[0]), float(target_xyz[1])),
        )

    def camera_point_to_spatial(self, xyz_camera, *, pose=None):
        # The legacy caller tries this after a failed timestamped projection.
        # A nominal camera offset must not silently replace a missing transform.
        return None

    def reproject_camera_point_to_map(self, xyz_camera, timestamp):
        return self.camera_point_to_map(xyz_camera,timestamp)

    def anchor_map_point(self, map_xyz, timestamp):
        if map_xyz is None or timestamp is None:
            return None
        try:
            observer_pose = self._base_pose_at(float(timestamp))
            return self._keyframe_graph.anchor_map_point(
                tuple(float(v) for v in map_xyz), float(timestamp),
                observer_pose=observer_pose,
            )
        except Exception:  # noqa: BLE001 - missing historical TF degrades to unanchored
            return None

    def reproject_anchor_point(self, anchor_id, point_in_anchor, *, map_id=None,
                               map_epoch=None):
        return self._keyframe_graph.reproject(
            str(anchor_id), tuple(float(v) for v in point_in_anchor),
            map_id=map_id, map_epoch=map_epoch,
        )

    def transform_provenance(self):
        value = super().transform_provenance()
        snapshot = self._keyframe_graph.snapshot()
        if snapshot is not None:
            value.update({
                'map_id': snapshot.map_id,
                'map_epoch': snapshot.map_epoch,
                'graph_revision': snapshot.graph_revision,
                'sensor_profile_id': snapshot.sensor_profile_id,
            })
        return value

    def health(self):
        return {'source': 'navigation_2d', 'localization_valid': self._healthy(),
                'localization_valid_basis': self._valid_basis,
                'map_received': self._map is not None, 'quality': self.quality(),
                'mapping_quality': dict(self._map_quality),
                'motion_authorized': False, 'last_error': self._last_error}
