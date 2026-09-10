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
        # §5.1 / §10：map→odom 只存在于 TF（slam_3d_node 广播），
        # 用它把语义点从 pslam_odom 升到 pslam_map。
        self._map_bridge: Any = None
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
            # 2026-09-09 P2 修复第四步：map_2d 的 publisher 是
            # `rclcpp::QoS(10).transient_local()`（RELIABLE + TRANSIENT_LOCAL）。
            # 之前用 BEST_EFFORT + VOLATILE 订阅时，只要 worker 主循环阻塞几秒、
            # 错过发布时刻，就再也拿不到样本 → quality() 降级 CAMERA_LOCAL。
            # 改成与 publisher 同级的 RELIABLE + TRANSIENT_LOCAL 后，订阅建立时
            # 即可取到最后一份地图，不再依赖"恰好赶上发布时刻"。
            # odom_base / tf 仍是 BEST_EFFORT（它们的 publisher 就是 BEST_EFFORT）。
            map_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self._subs.append(
                self._node.create_subscription(
                    OccupancyGrid, map_topic, self._on_occupancy_grid, map_qos
                )
            )
            self._subs.append(
                self._node.create_subscription(
                    Odometry, odom_topic, self._on_odometry, sensor_qos
                )
            )
            from tf2_msgs.msg import TFMessage

            from app.slam.slam_bridge import SLAMBridge

            self._map_bridge = SLAMBridge(
                map_frame="pslam_map",
                odom_frame=self.odom_frame,
                base_frame=self.base_frame,
                camera_frame=self.camera_frame,
                camera_translation_m=self.nominal_translation_m,
                # 闭环通常超过默认 30 秒；保留十分钟历史 odom→base，
                # 让 timestamp + camera_xyz 锚点能在回环后重算。
                buffer_seconds=600.0,
                max_samples=12000,
            )
            self._subs.append(
                self._node.create_subscription(
                    TFMessage, "/tf", self._on_tf, sensor_qos
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

    def _on_tf(self, message: Any) -> None:
        if self._map_bridge is None:
            return
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if (parent != self._map_bridge.map_frame
                    or child != self._map_bridge.odom_frame):
                continue
            quaternion = transform.transform.rotation
            translation = transform.transform.translation
            stamp = (float(transform.header.stamp.sec)
                     + float(transform.header.stamp.nanosec) * 1e-9)
            self._map_bridge.note_map_to_odom(
                stamp,
                (translation.x, translation.y,
                 math.atan2(2.0 * (quaternion.z * quaternion.w),
                            1.0 - 2.0 * (quaternion.z * quaternion.z))),
            )
            # 2026-09-09 修复：该标志此前只在 __init__ 里置 False、在
            # transform_provenance() 里被读，**从未置 True**，导致 P2 排查时
            # 误判成「TF 缺失」。真正收到 map→odom 后置位。
            self._tf_available = True

    def camera_point_to_map(
        self,
        xyz_camera: tuple[float, float, float],
        timestamp: float | None = None,
    ) -> tuple[float, float, float] | None:
        """§10：camera-optical 点 -> ``pslam_map``（经 SLAMBridge 合成 T_map_base）。

        ``timestamp`` 应为图像拍摄时刻；缺省时退回当前 SLAM 位姿。
        map→odom 尚未收到时返回 ``None``——绝不退回 odom 系冒充 map 系。
        """
        if self._map_bridge is None or xyz_camera is None:
            return None
        if self.get_pose() is None:
            return None
        return self._map_bridge.camera_point_to_map(xyz_camera, timestamp)

    def reproject_camera_point_to_map(
        self,
        xyz_camera: tuple[float, float, float],
        timestamp: float,
    ) -> tuple[float, float, float] | None:
        """用最新 map→odom 校正重算历史图像时刻的语义点。"""
        if self._map_bridge is None or xyz_camera is None:
            return None
        return self._map_bridge.reproject_camera_point_to_map(
            xyz_camera, timestamp
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
            "map_correction_revision": (
                self._map_bridge.map_correction_revision
                if self._map_bridge is not None else None
            ),
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
        if self._map_bridge is not None:
            # 与 /tf 同一 ROS 时基，供 SLAMBridge 合成 T_map_base（§10）。
            self._map_bridge.note_odom_to_base(
                float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
                (float(pose.position.x), float(pose.position.y), float(yaw)),
            )
