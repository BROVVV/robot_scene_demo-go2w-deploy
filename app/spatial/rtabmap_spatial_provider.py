"""RTAB-Map SpatialProvider adapter.

Consumes ROS2 ``/rtabmap/map`` and ``/rtabmap/odom`` when available and
converts them into the project's spatial models.  When RTAB-Map is not
running / not installed it degrades to ``CameraLocalSpatialProvider``.

The provider also implements the crucial ``camera_point_to_spatial`` chain:

    d435_color_optical_frame -> base_link -> map/odom

TF2 is the preferred source for both transforms.  When TF2 is unavailable the
provider falls back to the nominal optical-axis model plus a planar robot pose
(``RELATIVE_RGBD``, never silently claimed as metric).
"""

from __future__ import annotations

import math
from typing import Any

from app.spatial.camera_local_spatial_provider import CameraLocalSpatialProvider
from app.spatial.frontier_extractor import FrontierExtractor
from app.spatial.models import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_METRIC_RGBD,
    SPATIAL_QUALITY_RELATIVE_RGBD,
    FrontierCandidate,
    SpatialMapSnapshot,
    SpatialPose,
)
from app.spatial.spatial_transform import (
    DEFAULT_NOMINAL_CAMERA_TRANSLATION_M,
    camera_optical_to_base,
    camera_point_to_map,
    transform_quality,
)


class RtabmapSpatialProvider:
    def __init__(
        self,
        *,
        fallback: CameraLocalSpatialProvider | None = None,
        enable_ros: bool = False,
        map_topic: str = "/rtabmap/map",
        odom_topic: str = "/rtabmap/odom",
        map_frame: str = "map",
        odom_frame: str = "odom",
        camera_frame: str = "d435_color_optical_frame",
        base_frame: str = "base_link",
        nominal_translation_m: tuple[float, float, float] = DEFAULT_NOMINAL_CAMERA_TRANSLATION_M,
    ) -> None:
        self.fallback = fallback or CameraLocalSpatialProvider()
        self.frontier_extractor = FrontierExtractor(min_component_size=1)
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        self.camera_frame = camera_frame
        self.base_frame = base_frame
        self.nominal_translation_m = tuple(float(v) for v in nominal_translation_m)
        self._map: SpatialMapSnapshot | None = None
        self._pose: SpatialPose | None = None
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

    def _enable_ros(self, *, map_topic: str, odom_topic: str) -> None:
        try:
            import rclpy
            from nav_msgs.msg import OccupancyGrid, Odometry
            from tf2_ros import Buffer, TransformListener

            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node("go2w_rtabmap_spatial_provider")
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self._node)
            self._subs.append(
                self._node.create_subscription(
                    OccupancyGrid, map_topic, self._on_occupancy_grid, 10
                )
            )
            self._subs.append(
                self._node.create_subscription(
                    Odometry, odom_topic, self._on_odometry, 10
                )
            )
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"rtabmap ros init failed: {exc}"
            self._node = None
            self._tf_buffer = None

    def spin_once(self) -> None:
        if self._node is not None:
            try:
                import rclpy

                rclpy.spin_once(self._node, timeout_sec=0.05)
                self._probe_tf()
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

    def quality(self) -> str:
        if self._map is not None and self.get_pose() is not None:
            return transform_quality(
                map_available=True,
                pose_available=True,
                transform_source=self._transform_source,
                has_map_frame=bool(self._map.provenance.get("frame_id")),
            )
        if self.get_pose() is not None:
            return SPATIAL_QUALITY_RELATIVE_RGBD
        return SPATIAL_QUALITY_CAMERA_LOCAL

    def set_pose(self, pose: SpatialPose | None) -> None:
        # RTAB-Map pose comes from /rtabmap/odom; fallback keeps the robot
        # pose for relative frontier mode when RTAB-Map is not available.
        self.fallback.set_pose(pose)
        if pose is not None and self._pose is None:
            self._pose = pose

    def get_pose(self) -> SpatialPose | None:
        return self._pose or self.fallback.get_pose()

    def get_map(self) -> SpatialMapSnapshot | None:
        return self._map

    def get_frontiers(self) -> list[FrontierCandidate]:
        if self._map is not None:
            pose = self.get_pose()
            frontiers = self.frontier_extractor.extract(self._map, pose)
            if frontiers:
                return frontiers
        return self.fallback.get_frontiers()

    def camera_point_to_spatial(
        self,
        xyz_camera: tuple[float, float, float],
        pose: SpatialPose | None = None,
    ) -> tuple[float, float, float] | None:
        """Transform a camera-optical point into the map/odom frame.

        Priority:
        1. TF2 ``map_frame <- camera_frame``.
        2. Nominal optical extrinsic + robot planar pose fallback.

        The caller can inspect :meth:`transform_provenance` / :meth:`quality`
        to know which path was used.
        """
        if xyz_camera is None:
            return None
        pose = pose or self.get_pose()
        # A. Real TF2 path (map_frame <- d435_color_optical_frame)
        tf_xyz = self._tf_point_to_map(xyz_camera)
        if tf_xyz is not None:
            self._transform_source = "tf2"
            self._tf_available = True
            return tf_xyz
        # B. Nominal extrinsic fallback.  A planar pose is required.
        if pose is None:
            return None
        result = camera_point_to_map(
            xyz_camera,
            pose,
            camera_translation_m=self.nominal_translation_m,
        )
        if result is None:
            return None
        self._transform_source = "nominal_extrinsic"
        return result

    def transform_provenance(self) -> dict[str, Any]:
        pose = self.get_pose()
        return {
            "source_frame": self.camera_frame,
            "target_frame": self._map.provenance.get("frame_id", self.map_frame)
            if self._map is not None
            else self.map_frame,
            "transform_source": self._transform_source,
            "pose_source": pose.source if pose is not None else None,
            "map_revision": self._map.revision if self._map is not None else None,
            "transform_quality": self.quality(),
            "tf_available": self._tf_available,
        }

    def health(self) -> dict[str, Any]:
        return {
            "rtabmap_available": self._available,
            "map_received": self._map is not None,
            "pose_received": self._pose is not None,
            "quality": self.quality(),
            "last_error": self._last_error,
            "transform_source": self._transform_source,
            "tf_available": self._tf_available,
            "note": "RTAB-Map ROS2 topics + camera_point_to_spatial",
        }

    # ------------------------------------------------------------------ #
    # Transform helpers                                                   #
    # ------------------------------------------------------------------ #
    def _probe_tf(self) -> None:
        if self._tf_buffer is None or self._node is None:
            return
        try:
            import rclpy

            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
            self._tf_available = bool(transform)
            self._transform_source = "tf2"
        except Exception:  # noqa: BLE001 - TF may be missing; fallback is fine
            return

    def _tf_point_to_map(
        self, xyz_camera: tuple[float, float, float]
    ) -> tuple[float, float, float] | None:
        if self._tf_buffer is None or self._node is None:
            return None
        try:
            import rclpy
            from geometry_msgs.msg import PointStamped

            point = PointStamped()
            point.header.frame_id = self.camera_frame
            point.header.stamp = self._node.get_clock().now().to_msg()
            x, y, z = (float(v) for v in xyz_camera)
            point.point.x = x
            point.point.y = y
            point.point.z = z
            transformed = self._tf_buffer.transform(point, self.map_frame)
            return (
                round(float(transformed.point.x), 4),
                round(float(transformed.point.y), 4),
                round(float(transformed.point.z), 4),
            )
        except Exception:  # noqa: BLE001 - fallback handled by caller
            self._tf_available = False
            return None

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
            quality=SPATIAL_QUALITY_METRIC_RGBD
            if self.quality() == SPATIAL_QUALITY_METRIC_RGBD
            else SPATIAL_QUALITY_RELATIVE_RGBD,
            source="rtabmap",
            provenance={"frame_id": msg.header.frame_id},
        )

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
            quality=SPATIAL_QUALITY_METRIC_RGBD
            if self._map is not None
            else SPATIAL_QUALITY_RELATIVE_RGBD,
            source="rtabmap_odom",
            provenance={"stamp_ns": msg.header.stamp.nanosec},
        )