"""plain_slam + PandarXT-16 SpatialProvider adapter.

Consumes the mapping-assist pipeline's ROS2 topics:

    /go2w/slam/map_2d     nav_msgs/OccupancyGrid   (ray-traced free/occ/unknown)
    /go2w/slam/odom_base  nav_msgs/Odometry        (shadow odom in pslam_odom)

and converts them into the project's spatial models with the plain_slam
provenance chain.  When the mapping pipeline is stale/unavailable it degrades
to ``CameraLocalSpatialProvider`` (never crashes the exploration loop).

Safety semantics: this provider is *mapping assistance only*.  Its map/pose
live in the isolated ``pslam_odom`` frame and must never be mixed into
``/go2w/odom/fused`` motion-authoritative coordinates.  The Pandar extrinsic
stays ``candidate_unconfirmed`` and the provenance always records it.
"""

from __future__ import annotations

import math
import time
from typing import Any

from app.spatial.camera_local_spatial_provider import CameraLocalSpatialProvider
from app.spatial.frontier_extractor import FrontierExtractor
from app.spatial.models import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_METRIC_LIDAR,
    SPATIAL_QUALITY_RELATIVE_RGBD,
    SpatialFrameMismatch,
    FrontierCandidate,
    SpatialMapSnapshot,
    SpatialPose,
)
from app.spatial.spatial_transform import (
    DEFAULT_NOMINAL_CAMERA_TRANSLATION_M,
    camera_point_to_map,
)

SOURCE_MAP = "plain_slam_pandarxt16"
SOURCE_POSE = "plain_slam_pandarxt16_odom"
EXTRINSIC_STATUS = "candidate_unconfirmed"


class PlainSlamSpatialProvider:
    def __init__(
        self,
        *,
        fallback: CameraLocalSpatialProvider | None = None,
        enable_ros: bool = False,
        map_topic: str = "/go2w/slam/map_2d",
        odom_topic: str = "/go2w/slam/odom_base",
        map_frame: str = "pslam_odom",
        odom_frame: str = "pslam_odom",
        camera_frame: str = "d435_color_optical_frame",
        base_frame: str = "base_link",
        nominal_translation_m: tuple[float, float, float] = DEFAULT_NOMINAL_CAMERA_TRANSLATION_M,
        map_stale_s: float = 3.0,
        pose_stale_s: float = 3.0,
    ) -> None:
        self.fallback = fallback or CameraLocalSpatialProvider()
        self.frontier_extractor = FrontierExtractor(min_component_size=1)
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        self.camera_frame = camera_frame
        self.base_frame = base_frame
        self.nominal_translation_m = tuple(float(v) for v in nominal_translation_m)
        self.map_stale_s = float(map_stale_s)
        self.pose_stale_s = float(pose_stale_s)
        self._map: SpatialMapSnapshot | None = None
        self._map_ts: float = 0.0
        self._pose: SpatialPose | None = None
        self._pose_ts: float = 0.0
        self._available = False
        self._last_error: str | None = None
        self._node = None
        self._subs: list[Any] = []
        self._tf_buffer = None
        self._tf_listener = None
        self._transform_source = "nominal_extrinsic"
        self._tf_available = False
        if enable_ros:
            self._enable_ros(map_topic=map_topic, odom_topic=odom_topic)

    # ------------------------------------------------------------------ #
    # ROS wiring                                                          #
    # ------------------------------------------------------------------ #
    def _enable_ros(self, *, map_topic: str, odom_topic: str) -> None:
        try:
            import rclpy
            from nav_msgs.msg import OccupancyGrid, Odometry
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )

            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node("go2w_plain_slam_spatial_provider")
            # The mapping pipeline publishes odom_base with SensorDataQoS
            # (BEST_EFFORT); a default RELIABLE subscription would never see
            # it.  BEST_EFFORT also works for the RELIABLE map_2d (matches in
            # the compatible direction).
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self._subs.append(
                self._node.create_subscription(
                    OccupancyGrid, map_topic, self._on_occupancy_grid, sensor_qos
                )
            )
            self._subs.append(
                self._node.create_subscription(
                    Odometry, odom_topic, self._on_odometry, sensor_qos
                )
            )
        except Exception as exc:  # noqa: BLE001 - degrade to fallback
            self._last_error = f"plain_slam ros init failed: {exc}"
            self._node = None

    def spin_once(self) -> None:
        if self._node is not None:
            try:
                import rclpy

                # odom is high-rate while map_2d is low-rate.  A single
                # callback can therefore repeatedly consume odom and leave
                # the occupancy-grid callback queued, making the provider
                # appear metric-unavailable at an observation boundary.
                # Drain a short bounded batch so one refresh services both
                # streams without creating a background executor/thread.
                for _ in range(8):
                    rclpy.spin_once(self._node, timeout_sec=0.01)
            except Exception as exc:  # noqa: BLE001
                self._last_error = str(exc)

    def close(self) -> None:
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            self._node = None
            self._tf_buffer = None

    # ------------------------------------------------------------------ #
    # SpatialProvider protocol                                            #
    # ------------------------------------------------------------------ #
    def quality(self) -> str:
        if self._map_is_fresh() and self.get_pose() is not None:
            return SPATIAL_QUALITY_METRIC_LIDAR
        if self._pose is not None and self.get_pose() is not None:
            return SPATIAL_QUALITY_RELATIVE_RGBD
        return SPATIAL_QUALITY_CAMERA_LOCAL

    def set_pose(self, pose: SpatialPose | None) -> None:
        """Set the robot pose in the plain_slam world frame.

        计划书 §9.3 / 不变量 3：plain_slam 的世界坐标系是 ``pslam_odom``。
        wheel odom（frame ``odom``）等其它 frame 的数值严禁直接写入本
        provider（只能用于相对运动/当前 yaw/运动验证）。frame 不一致时抛出
        :class:`SpatialFrameMismatch`，由调用方 transform 或降级，绝不静默混算。
        """
        if pose is not None and pose.frame_id != self.map_frame:
            self._last_error = (
                f"SPATIAL_FRAME_MISMATCH: pose_frame={pose.frame_id} "
                f"map_frame={self.map_frame}"
            )
            raise SpatialFrameMismatch(
                pose_frame=pose.frame_id,
                map_frame=self.map_frame,
                detail="wheel odom must not be injected into pslam_odom",
            )
        self.fallback.set_pose(pose)
        if pose is not None and self._pose is None:
            self._pose = pose
            self._pose_ts = time.monotonic()

    def get_pose(self) -> SpatialPose | None:
        """plain_slam pose when reasonably fresh, otherwise fallback pose."""
        if self._pose is not None and (
            self._map_is_fresh() or self._pose_is_fresh()
        ):
            return self._pose
        return self.fallback.get_pose()

    def get_map(self) -> SpatialMapSnapshot | None:
        return self._map if self._map_is_fresh() else None

    def get_frontiers(self) -> list[FrontierCandidate]:
        if self._map_is_fresh():
            pose = self.get_pose()
            # 只有 pslam_odom 的 pose 才能与 pslam 地图混算（不变量 3）。
            if pose is not None and pose.frame_id == self.map_frame:
                frontiers = self.frontier_extractor.extract(self._map, pose)
                if frontiers:
                    return frontiers
        return self.fallback.get_frontiers()

    def camera_point_to_spatial(
        self,
        xyz_camera: tuple[float, float, float],
        pose: SpatialPose | None = None,
    ) -> tuple[float, float, float] | None:
        """Camera-optical point -> pslam/odom plane (nominal fallback only).

        TF2 between ``pslam_odom`` and the D435 optical frame is generally not
        published in the first version (plain_slam owns ``pslam_odom ->
        pslam_imu`` only), so the nominal camera-extrinsic + planar pose path
        is the realistic default.  It is always labeled nominal, never claimed
        as a calibrated metric transform.
        """
        if xyz_camera is None:
            return None
        pose = pose or self.get_pose()
        if pose is None:
            return None
        if pose.frame_id != self.map_frame:
            # 严禁 frame A 数值 + frame B 地图直接运算（不变量 3）。
            self._last_error = (
                f"SPATIAL_FRAME_MISMATCH: pose_frame={pose.frame_id} "
                f"map_frame={self.map_frame}"
            )
            raise SpatialFrameMismatch(
                pose_frame=pose.frame_id,
                map_frame=self.map_frame,
                detail="camera_point_to_spatial requires pslam_odom pose",
            )
        self._transform_source = "nominal_extrinsic"
        return camera_point_to_map(
            xyz_camera,
            pose,
            camera_translation_m=self.nominal_translation_m,
        )

    def transform_provenance(self) -> dict[str, Any]:
        pose = self.get_pose()
        return {
            "source_frame": self.camera_frame,
            "target_frame": (
                self._map.provenance.get("frame_id", self.map_frame)
                if self._map is not None
                else self.map_frame
            ),
            "transform_source": self._transform_source,
            "pose_source": pose.source if pose is not None else None,
            "map_revision": self._map.revision if self._map is not None else None,
            "transform_quality": self.quality(),
            "tf_available": self._tf_available,
        }

    def health(self) -> dict[str, Any]:
        return {
            "plain_slam_available": self._available,
            "map_received": self._map is not None,
            "map_fresh": self._map_is_fresh(),
            "pose_received": self._pose is not None,
            "pose_fresh": self._pose_is_fresh(),
            "quality": self.quality(),
            "source": SOURCE_MAP,
            "extrinsic_status": EXTRINSIC_STATUS,
            "mapping_mode": "mapping_assist",
            "motion_authorized": False,
            "safety_authorized": False,
            "last_error": self._last_error,
            "transform_source": self._transform_source,
        }

    # ------------------------------------------------------------------ #
    # Freshness helpers                                                   #
    # ------------------------------------------------------------------ #
    def _map_is_fresh(self) -> bool:
        return self._map is not None and (
            time.monotonic() - self._map_ts <= self.map_stale_s
        )

    def _pose_is_fresh(self) -> bool:
        return self._pose is not None and (
            time.monotonic() - self._pose_ts <= self.pose_stale_s
        )

    # ------------------------------------------------------------------ #
    # ROS callbacks                                                       #
    # ------------------------------------------------------------------ #
    def _on_occupancy_grid(self, msg: Any) -> None:
        self._available = True
        width = int(msg.info.width)
        height = int(msg.info.height)
        res = float(msg.info.resolution)
        origin = (float(msg.info.origin.position.x), float(msg.info.origin.position.y))
        free: list[tuple[int, int]] = []
        occupied: list[tuple[int, int]] = []
        unknown: list[tuple[int, int]] = []
        data = list(msg.data)
        for index, value in enumerate(data):
            x = index % width
            y = index // width
            if value < 0:
                unknown.append((x, y))
            elif value == 0:
                free.append((x, y))
            else:
                occupied.append((x, y))
        self._map = SpatialMapSnapshot(
            revision=getattr(self._map, "revision", 0) + 1,
            resolution_m=res,
            origin=origin,
            width=width,
            height=height,
            free=free,
            occupied=occupied,
            unknown=unknown,
            quality=SPATIAL_QUALITY_METRIC_LIDAR,
            source=SOURCE_MAP,
            provenance={
                "frame_id": msg.header.frame_id,
                "mapping_mode": "mapping_assist",
                "pandar_extrinsic_status": EXTRINSIC_STATUS,
                "generator": "pointcloud_to_occupancy/ray_traced",
            },
        )
        self._map_ts = time.monotonic()

    def _on_odometry(self, msg: Any) -> None:
        self._available = True
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._pose = SpatialPose(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=float(yaw),
            frame_id=msg.header.frame_id or self.odom_frame,
            quality=SPATIAL_QUALITY_METRIC_LIDAR,
            source=SOURCE_POSE,
            provenance={
                "stamp_ns": msg.header.stamp.nanosec,
                "child_frame_id": msg.child_frame_id,
                "extrinsic_status": EXTRINSIC_STATUS,
                "mapping_mode": "mapping_assist",
            },
        )
        self._pose_ts = time.monotonic()
