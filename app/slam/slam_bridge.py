"""SLAMBridge：唯一的权威世界坐标接口（20260908 计划书 §5.1 / §10 / §41）。

职责严格限制为三件事：

1. 维护 ``map → odom`` 与 ``odom → base`` 的**带时间戳**位姿缓冲；
2. 按**图像拍摄时刻**给出 ``T_map_base``（禁止用 ``now()`` 或"检测返回时刻"的位姿）；
3. 复用 ``app/spatial/spatial_transform.camera_point_to_map`` 把 ``camera_xyz`` 投到 map 系。

明确禁止（§5.2）：任何上层模块自己积分 odom 造第二份全局位姿。
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Deque, Iterable, Tuple

from app.spatial.spatial_transform import (
    DEFAULT_NOMINAL_CAMERA_TRANSLATION_M,
    camera_point_to_map as _camera_point_to_world,
)

Pose2D = Tuple[float, float, float]


def compose(first: Pose2D, second: Pose2D) -> Pose2D:
    """``T_a_c = T_a_b ∘ T_b_c``（2D 平面位姿）。"""
    ax, ay, ayaw = (float(v) for v in first)
    bx, by, byaw = (float(v) for v in second)
    return (
        ax + math.cos(ayaw) * bx - math.sin(ayaw) * by,
        ay + math.sin(ayaw) * bx + math.cos(ayaw) * by,
        ayaw + byaw,
    )


def yaw_from_quaternion(quaternion: Any) -> float:
    return math.atan2(2.0 * (quaternion.z * quaternion.w),
                      1.0 - 2.0 * (quaternion.z * quaternion.z))


def wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


class SLAMBridge:
    """把 SLAM 的 map→odom 与 odom→base 合成为带时间戳的 T_map_base。"""

    def __init__(
        self,
        *,
        map_frame: str = "pslam_map",
        odom_frame: str = "pslam_odom",
        base_frame: str = "base_link_mapping_assist",
        camera_frame: str = "d435_color_optical_frame",
        camera_translation_m: Iterable[float] = DEFAULT_NOMINAL_CAMERA_TRANSLATION_M,
        buffer_seconds: float = 30.0,
        max_samples: int = 4000,
        correction_translation_epsilon_m: float = 1e-4,
        correction_yaw_epsilon_rad: float = 1e-4,
    ) -> None:
        self.map_frame = str(map_frame)
        self.odom_frame = str(odom_frame)
        self.base_frame = str(base_frame)
        self.camera_frame = str(camera_frame)
        self.camera_translation_m = tuple(float(v) for v in camera_translation_m)
        self.buffer_seconds = max(1.0, float(buffer_seconds))
        self.max_samples = max(16, int(max_samples))
        self.correction_translation_epsilon_m = float(
            correction_translation_epsilon_m
        )
        self.correction_yaw_epsilon_rad = float(correction_yaw_epsilon_rad)
        self.map_correction_revision = 0
        self._map_odom: Deque[Tuple[float, Pose2D]] = deque(maxlen=self.max_samples)
        self._odom_base: Deque[Tuple[float, Pose2D]] = deque(maxlen=self.max_samples)

    # ---- 输入 ---------------------------------------------------------------

    def note_map_to_odom(self, stamp: float, pose: Pose2D) -> None:
        value = float(stamp)
        if not math.isfinite(value) or value <= 0.0:
            return
        if self._map_odom and value <= self._map_odom[-1][0]:
            return
        normalized = (
            float(pose[0]),
            float(pose[1]),
            wrap_pi(float(pose[2])),
        )
        if self._map_odom:
            previous = self._map_odom[-1][1]
            translation_delta = math.hypot(
                normalized[0] - previous[0], normalized[1] - previous[1]
            )
            yaw_delta = abs(wrap_pi(normalized[2] - previous[2]))
            if (
                translation_delta > self.correction_translation_epsilon_m
                or yaw_delta > self.correction_yaw_epsilon_rad
            ):
                self.map_correction_revision += 1
        self._append(self._map_odom, value, normalized)

    def note_odom_to_base(self, stamp: float, pose: Pose2D) -> None:
        self._append(self._odom_base, stamp, pose)

    def _append(self, buffer: Deque[Tuple[float, Pose2D]], stamp: float,
                pose: Pose2D) -> None:
        value = float(stamp)
        if not math.isfinite(value) or value <= 0.0:
            return
        if buffer and value <= buffer[-1][0]:
            return
        buffer.append((value, (float(pose[0]), float(pose[1]), wrap_pi(pose[2]))))
        horizon = value - self.buffer_seconds
        while buffer and buffer[0][0] < horizon:
            buffer.popleft()

    # ---- 查询 ---------------------------------------------------------------

    @staticmethod
    def _nearest(buffer: Deque[Tuple[float, Pose2D]],
                 timestamp: float | None) -> Pose2D | None:
        if not buffer:
            return None
        if timestamp is None:
            return buffer[-1][1]
        target = float(timestamp)
        best = None
        best_gap = None
        for stamp, pose in buffer:
            gap = abs(stamp - target)
            if best_gap is None or gap < best_gap:
                best, best_gap = pose, gap
        return best

    def robot_pose_map(self, timestamp: float | None = None) -> Pose2D | None:
        """``T_map_base``，按 ``timestamp``（图像拍摄时刻）取最近的 SLAM 位姿。"""
        map_odom = self._nearest(self._map_odom, timestamp)
        odom_base = self._nearest(self._odom_base, timestamp)
        if map_odom is None or odom_base is None:
            return None
        return compose(map_odom, odom_base)

    def robot_pose_map_reprojected(self, timestamp: float) -> Pose2D | None:
        """用最新 ``map→odom`` 修正重算历史时刻的 ``T_map_base``。

        loop closure 改写的是当前 map 对 odom 的全局校正；历史相机观测仍需
        使用拍摄时刻的 odom→base，不能改用当前机器人位姿。
        """
        map_odom = self._nearest(self._map_odom, None)
        odom_base = self._nearest(self._odom_base, timestamp)
        if map_odom is None or odom_base is None:
            return None
        return compose(map_odom, odom_base)

    def camera_pose_map(self, timestamp: float | None = None) -> Pose2D | None:
        """``T_map_camera`` 的平面部分（相机原点在 map 系的位置与朝向）。"""
        base = self.robot_pose_map(timestamp)
        if base is None:
            return None
        camera_in_base = (self.camera_translation_m[0], self.camera_translation_m[1],
                          math.pi / 2.0)
        return compose(base, camera_in_base)

    def camera_point_to_map(self, xyz_camera: Iterable[float],
                            timestamp: float | None = None,
                            ) -> Tuple[float, float, float] | None:
        """把 camera 系 3D 点投到 map 系；位姿缺失时返回 ``None``（不猜）。"""
        pose = self.robot_pose_map(timestamp)
        if pose is None:
            return None
        return _camera_point_to_world(
            xyz_camera,
            {"x": pose[0], "y": pose[1], "yaw_rad": pose[2]},
            camera_translation_m=self.camera_translation_m)

    def reproject_camera_point_to_map(
        self,
        xyz_camera: Iterable[float],
        timestamp: float,
    ) -> Tuple[float, float, float] | None:
        """按历史图像时间和最新 map 校正重投影一个原始相机观测。"""
        pose = self.robot_pose_map_reprojected(timestamp)
        if pose is None:
            return None
        return _camera_point_to_world(
            xyz_camera,
            {"x": pose[0], "y": pose[1], "yaw_rad": pose[2]},
            camera_translation_m=self.camera_translation_m)

    # ---- 诊断 ---------------------------------------------------------------

    def status(self) -> dict[str, Any]:
        return {
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "camera_frame": self.camera_frame,
            "map_to_odom_samples": len(self._map_odom),
            "odom_to_base_samples": len(self._odom_base),
            "latest_map_to_odom_stamp": self._map_odom[-1][0] if self._map_odom else None,
            "latest_odom_to_base_stamp": (
                self._odom_base[-1][0] if self._odom_base else None),
            "map_correction_revision": self.map_correction_revision,
            "camera_extrinsic": "nominal_extrinsic",
        }
