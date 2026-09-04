"""Fixed-world map state for the WebUI SLAM bridge (plan §9 and §10).

The bridge process needs ``rclpy``, which the offline test environment does
not have, so every rule that decides *what the WebUI is allowed to show*
lives here: canonical frame, permanent map source, capacity layers, mapping
session, and the motion-episode drift gate.
"""

from __future__ import annotations

import json
import math
from typing import Any, Iterable

from app.spatial.pointcloud_web_codec import GlobalVoxelCloud

POINTS_PLACEHOLDER = "__GO2W_MAP_POINTS__"

HEALTHY = "HEALTHY"
DEGRADED_LIO_DRIFT = "DEGRADED_LIO_DRIFT"
WAITING_FOR_MAP = "WAITING_FOR_MAP"

# §10.1：原地转向判定阈值。轮式里程计几乎不动而 LIO 平移超过 0.20m 就是假平移。
ROTATION_WHEEL_XY_M = 0.10
ROTATION_PSLAM_XY_M = 0.20


def wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _delta(start: tuple[float, float, float] | None,
           end: tuple[float, float, float] | None) -> tuple[float, float]:
    if start is None or end is None:
        return (0.0, 0.0)
    return (math.hypot(end[0] - start[0], end[1] - start[1]),
            abs(math.degrees(wrap_pi(end[2] - start[2]))))


class MotionEpisodeGate:
    """§10.1：以"整段运动"为单位比较轮式里程计和 LIO 位姿。

    逐帧比较抓不到慢慢累积的假平移，也会被 CPU 抖动骗到；一次运动从起步到
    重新静止只判一次，判的是这段运动的总量。
    """

    def __init__(self, *, speed_max: float = 0.02, yaw_rate_max: float = 0.03,
                 settle_seconds: float = 0.5) -> None:
        self.speed_max = float(speed_max)
        self.yaw_rate_max = float(yaw_rate_max)
        self.settle_seconds = max(0.1, float(settle_seconds))
        self.episode_id = 0
        self.moving = False
        self.stationary_since: float | None = None
        self.wheel_delta = (0.0, 0.0)
        self.lio_delta = (0.0, 0.0)
        self._wheel_start: tuple[float, float, float] | None = None
        self._lio_start: tuple[float, float, float] | None = None
        self._wheel: tuple[float, float, float] | None = None
        self._lio: tuple[float, float, float] | None = None
        self._wheel_time: float | None = None

    def note_lio(self, x: float, y: float, yaw: float) -> None:
        self._lio = (float(x), float(y), float(yaw))

    def note_wheel(self, x: float, y: float, yaw: float, *, speed: float,
                   yaw_rate: float, now: float) -> str:
        """Feed fused wheel odometry; returns ``started``/``finished``/``""``."""
        pose = (float(x), float(y), float(yaw))
        travelled = 0.0
        if self._wheel is not None and self._wheel_time is not None:
            elapsed = max(1e-3, now - self._wheel_time)
            travelled = math.hypot(pose[0] - self._wheel[0],
                                   pose[1] - self._wheel[1]) / elapsed
        self._wheel, self._wheel_time = pose, now
        active = (abs(float(speed)) > self.speed_max
                  or abs(float(yaw_rate)) > self.yaw_rate_max
                  or travelled > self.speed_max * 1.5)
        if active:
            self.stationary_since = None
            if self.moving:
                return ""
            self.moving = True
            self.episode_id += 1
            self._wheel_start, self._lio_start = pose, self._lio
            return "started"
        if self.stationary_since is None:
            self.stationary_since = now
        if self.moving and now - self.stationary_since >= self.settle_seconds:
            self.moving = False
            self.wheel_delta = _delta(self._wheel_start, pose)
            self.lio_delta = _delta(self._lio_start, self._lio)
            return "finished"
        return ""

    def stationary(self, now: float) -> bool:
        if self.moving or self.stationary_since is None:
            return False
        return (now - self.stationary_since) >= self.settle_seconds


def drift_reason(wheel: tuple[float, float], lio: tuple[float, float]) -> str:
    """§10.1：一段运动结束后，判断 LIO 是否造出了轮式里程计没有的平移。"""
    wheel_xy = wheel[0]
    lio_xy = lio[0]
    if wheel_xy <= ROTATION_WHEEL_XY_M and lio_xy > ROTATION_PSLAM_XY_M:
        return f"LIO在原地转向中产生{lio_xy:.2f}米假平移，地图已冻结"
    if lio_xy > 3.0 * wheel_xy + 0.30:
        return (f"LIO这段运动平移{lio_xy:.2f}米，轮式里程计只有{wheel_xy:.2f}米，"
                "地图已冻结")
    return ""


class SlamWebMapState:
    """§9.1/§9.3/§10.3：网页三维地图的唯一事实来源。

    永久地图 = 权威 SLAM 地图（``pslam_map``）整张替换；``aligned_scan``
    只做实时预览，永远不写进永久历史；两个坐标系在没有带时间戳的 TF 之前
    绝不混用，也绝不改写 ``header.frame_id``。
    """

    def __init__(self, *, canonical_frame: str = "pslam_map",
                 permanent_source: str = "/go2w/slam/map_3d",
                 voxel_size_m: float = 0.12,
                 max_global_voxels: int = 300_000,
                 max_web_points: int = 50_000,
                 gate: MotionEpisodeGate | None = None) -> None:
        self.canonical_frame = str(canonical_frame)
        self.permanent_source = str(permanent_source)
        self.cloud = GlobalVoxelCloud(voxel_size_m=voxel_size_m,
                                      max_voxels=max_global_voxels)
        self.max_web_points = max(1_000, int(max_web_points))
        self.gate = gate or MotionEpisodeGate()
        self.session_id = 1
        self.session_start_stamp = 0.0
        self.health = WAITING_FOR_MAP
        self.health_reason = "等待 plain_slam 发布第一张地图"
        self.map_revision = 0
        self.map_stamp = 0.0
        self.map_wall_time = 0.0
        self.source_map_points = 0
        self.accepted_maps = 0
        self.rejected_counts: dict[str, int] = {}
        self.last_rejected_reason = ""
        self.lio_pose: tuple[float, float, float] | None = None
        self.wheel_pose: tuple[float, float, float] | None = None
        self.preview: dict[str, Any] = {
            "frame_id": "", "ros_stamp": 0.0, "wall_time": 0.0,
            "point_count": 0, "points": [],
        }
        self._json = "[]"
        self._json_revision = -1
        self._json_limit = 0
        self._web_count = 0
        self._web_info: dict[str, Any] = {"mode": "empty", "voxel_size_m": 0.0,
                                          "limit": self.max_web_points,
                                          "truncated": False}

    def reject(self, reason: str) -> bool:
        self.rejected_counts[reason] = self.rejected_counts.get(reason, 0) + 1
        self.last_rejected_reason = reason
        return False

    def accept_map(self, points: Iterable[tuple[float, float, float]] | Any, *,
                   frame_id: str, stamp: float, wall_time: float,
                   source_points: int | None = None) -> bool:
        """Replace the permanent cloud with one authoritative map revision."""
        if str(frame_id) != self.canonical_frame:
            return self.reject("map_frame_mismatch")
        if stamp and self.session_start_stamp and stamp < self.session_start_stamp:
            return self.reject("stale_session_map")
        if self.health == DEGRADED_LIO_DRIFT:
            return self.reject("lio_drift_frozen")
        self.cloud.replace(points)
        if len(self.cloud) == 0:
            return self.reject("empty_map")
        self.source_map_points = int(source_points if source_points is not None
                                     else self.cloud.source_point_count)
        self.map_revision += 1
        self.accepted_maps += 1
        self.map_stamp = float(stamp)
        self.map_wall_time = float(wall_time)
        self.health = HEALTHY
        self.health_reason = ""
        self.last_rejected_reason = ""
        return True

    def set_preview(self, points: list[tuple[float, float, float]], *,
                   frame_id: str, stamp: float, wall_time: float) -> None:
        """§9.1：最新 scan 只是预览图层，永远不进永久地图。"""
        self.preview = {
            "frame_id": str(frame_id), "ros_stamp": float(stamp),
            "wall_time": float(wall_time), "point_count": len(points),
            "points": [[round(x, 3), round(y, 3), round(z, 3)]
                       for x, y, z in points],
        }

    def note_lio(self, x: float, y: float, yaw: float) -> None:
        self.lio_pose = (float(x), float(y), float(yaw))
        self.gate.note_lio(x, y, yaw)

    def note_wheel(self, x: float, y: float, yaw: float, *, speed: float,
                   yaw_rate: float, now: float) -> str:
        self.wheel_pose = (float(x), float(y), float(yaw))
        event = self.gate.note_wheel(x, y, yaw, speed=speed,
                                     yaw_rate=yaw_rate, now=now)
        if event == "finished":
            reason = drift_reason(self.gate.wheel_delta, self.gate.lio_delta)
            if reason:
                # 这段运动的 LIO 平移对不上轮式里程计：冻结，拒收这段的地图。
                self.health = DEGRADED_LIO_DRIFT
                self.health_reason = reason
                return "degraded"
            if self.health == DEGRADED_LIO_DRIFT:
                # 下一段真实运动里两者重新一致，说明前端已恢复：解冻继续建图。
                # 仍然只在 finished 时判，安静的 scan 自己不能解冻。
                self.health = HEALTHY if self.accepted_maps else WAITING_FOR_MAP
                self.health_reason = ""
        return event

    def reset_session(self, *, stamp: float, reason: str = "operator_reset") -> None:
        """§9.4：新 mapping session：清空缓存、丢弃旧 session 的迟到消息。"""
        self.session_id += 1
        self.session_start_stamp = float(stamp)
        self.cloud.clear()
        self.map_revision += 1
        self.map_stamp = 0.0
        self.map_wall_time = 0.0
        self.source_map_points = 0
        self.accepted_maps = 0
        self.rejected_counts = {}
        self.last_rejected_reason = ""
        self.health = WAITING_FOR_MAP
        self.health_reason = f"新建图会话（{reason}），等待第一张地图"
        self.preview = {"frame_id": "", "ros_stamp": 0.0, "wall_time": 0.0,
                        "point_count": 0, "points": []}

    def points_json(self) -> str:
        """§9.3.3：同一张地图只编码一次，之后每帧快照直接复用。"""
        if (self._json_revision != self.map_revision
                or self._json_limit != self.max_web_points):
            points, info = self.cloud.sample(self.max_web_points)
            self._json = json.dumps(
                [[round(x, 3), round(y, 3), round(z, 3)] for x, y, z in points],
                separators=(",", ":"),
            )
            self._json_revision = self.map_revision
            self._json_limit = self.max_web_points
            self._web_count = len(points)
            self._web_info = info
        return self._json

    @staticmethod
    def _pose(pose: tuple[float, float, float] | None) -> dict[str, float] | None:
        if pose is None:
            return None
        return {"x": round(pose[0], 4), "y": round(pose[1], 4),
                "yaw": round(pose[2], 4)}

    def snapshot(self, *, now: float, generated_at: float) -> dict[str, Any]:
        """Snapshot with ``points`` left as a placeholder for the cached JSON."""
        self.points_json()
        gate = self.gate
        return {
            "schema_version": "go2w_slam_web_cloud_v2",
            "available": self._web_count > 0,
            "source": (self.permanent_source if self.accepted_maps
                       else "waiting_for_plain_slam"),
            "permanent_source": self.permanent_source,
            "canonical_frame": self.canonical_frame,
            "frame_id": self.canonical_frame,
            "target_map_frame": self.canonical_frame,
            "mapping_session_id": self.session_id,
            "map_revision": self.map_revision,
            "ros_stamp": round(self.map_stamp, 6),
            "last_good_map_stamp": round(self.map_stamp, 6),
            "map_updated_at": self.map_wall_time,
            "generated_at": generated_at,
            "point_count": self._web_count,
            "source_map_points": self.source_map_points,
            "global_cached_voxels": len(self.cloud),
            "web_display_points": self._web_count,
            "global_voxel_size_m": round(self.cloud.effective_voxel_size_m, 4),
            "voxel_size_m": round(self.cloud.effective_voxel_size_m, 4),
            "configured_voxel_size_m": self.cloud.voxel_size_m,
            "max_global_voxels": self.cloud.max_voxels,
            "max_web_points": self.max_web_points,
            "web_sampling": dict(self._web_info),
            "capacity_limited": self.cloud.capacity_limited,
            "map_extent_m": self.cloud.extent_m(),
            "bounds": self.cloud.bounds(),
            "mapping_health": self.health,
            "health_reason": self.health_reason,
            "lio_pose_valid": self.health != DEGRADED_LIO_DRIFT,
            "last_rejected_reason": self.last_rejected_reason,
            "rejected_counts": dict(self.rejected_counts),
            "accepted_maps": self.accepted_maps,
            "motion_episode_id": gate.episode_id,
            "motion_active": gate.moving,
            "stationary": gate.stationary(now),
            "wheel_episode_delta_xy_m": round(gate.wheel_delta[0], 4),
            "wheel_episode_delta_yaw_deg": round(gate.wheel_delta[1], 2),
            "pslam_episode_delta_xy_m": round(gate.lio_delta[0], 4),
            "pslam_episode_delta_yaw_deg": round(gate.lio_delta[1], 2),
            "pose": self._pose(self.lio_pose),
            "wheel_pose": self._pose(self.wheel_pose),
            "preview": dict(self.preview),
            "points": POINTS_PLACEHOLDER,
            "mapping_mode": "mapping_assist",
            "motion_authorized": False,
            "safety_authorized": False,
        }

    def snapshot_json(self, *, now: float, generated_at: float) -> str:
        return self.serialize(self.snapshot(now=now, generated_at=generated_at))

    def serialize(self, payload: dict[str, Any]) -> str:
        """Serialise with the cached point fragment spliced in."""
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        return body.replace(f'"{POINTS_PLACEHOLDER}"', self.points_json(), 1)
