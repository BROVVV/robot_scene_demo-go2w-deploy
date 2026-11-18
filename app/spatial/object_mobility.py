"""短期对象轨迹。只以同一地图坐标系中的深度位置判断运动。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from app.spatial.dynamic_labels import DYNAMIC_LABELS

FIXED_LABELS = frozenset({
    "tree trunk", "树干", "building", "建筑", "lamp post", "路灯",
    "fixed railing", "固定护栏", "pillar", "立柱", "column", "门框",
    "door frame", "fixed cabinet", "固定橱柜", "fixed equipment", "固定设备",
})
HIGH_DYNAMIC_LABELS = DYNAMIC_LABELS | frozenset({
    "animal", "动物", "pet", "宠物", "bird", "鸟", "running robot", "移动机器人",
})

# Open-vocabulary detectors legitimately vary the wording across nearby frames
# (for example "computer chair" -> "chair" or "industrial pipe" -> "pipe").
# Exact-string-only association resets the short track in that case, so an
# object seen while the robot is walking can never collect enough stationary
# evidence to enter the semantic map.  Keep conservative, object-class-specific
# aliases; do not use free token overlap ("door frame" and "picture frame"
# must not become the same track merely because both contain "frame").
_LABEL_FAMILIES = {
    "chair": frozenset({"chair", "office chair", "computer chair", "armchair",
                        "椅子", "办公椅", "电脑椅", "扶手椅"}),
    "pipe": frozenset({"pipe", "pipeline", "industrial pipe",
                       "管道", "管线", "工业管道"}),
    "box": frozenset({"box", "cardboard box", "carton", "纸箱", "纸板盒"}),
    "cabinet": frozenset({"cabinet", "fixed cabinet", "electrical cabinet",
                           "equipment cabinet", "橱柜", "固定橱柜", "电气柜", "设备柜"}),
    "bin": frozenset({"bin", "trash bin", "waste bin", "垃圾桶"}),
    "vehicle": frozenset({"car", "parked car", "vehicle", "汽车", "停放车辆", "车辆"}),
    "tree": frozenset({"tree", "tree trunk", "树", "树干"}),
    "column": frozenset({"pillar", "column", "立柱", "柱子"}),
    "lamp_post": frozenset({"lamp post", "street light", "streetlight", "路灯"}),
    "tank": frozenset({"tank", "storage tank", "储罐"}),
}
_LABEL_TO_FAMILY = {
    alias.casefold(): family
    for family, aliases in _LABEL_FAMILIES.items()
    for alias in aliases
}


def labels_of(item: Any) -> set[str]:
    if isinstance(item, dict):
        values = (item.get("label"), item.get("label_zh"))
    else:
        values = (getattr(item, "label", None), getattr(item, "label_zh", None))
    labels = {str(value).strip().casefold() for value in values if value}
    labels.update(
        f"object_family:{family}"
        for label in tuple(labels)
        if (family := _LABEL_TO_FAMILY.get(label)) is not None
    )
    return labels


@dataclass
class Track:
    track_id: str
    labels: set[str]
    last_xyz: tuple[float, float, float] | None = None
    last_time: float = 0.0
    last_frame_id: str = ""
    stable_start: float = 0.0
    stable_origin: tuple[float, float, float] | None = None
    stable_frames: int = 0
    motion_hits: int = 0
    mobility_class: str | None = None
    mobility_state: str = "pending"
    reason: str = "定位与连续观测不足"
    persistent_id: str | None = None
    bbox: list[float] | None = None
    # Only a confirmed moving episode may authorize the worker to open a new
    # persistent position segment after this track becomes stationary again.
    position_change_pending: bool = False

    def fields(self) -> dict[str, Any]:
        return {
            "track_id": self.track_id,
            "mobility_class": self.mobility_class,
            "mobility_state": self.mobility_state,
            "mobility_reason": self.reason,
            "mobility_confidence": 0.85 if self.mobility_class else 0.0,
            "topology_eligible": self.mobility_class in {"fixed", "semi_static"},
        }


class ObjectMobilityTracker:
    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = config or {}
        self.tracks: dict[str, Track] = {}
        self.next_id = 1
        self.map_epoch = ""

    def reset(self) -> None:
        self.tracks.clear()
        self.next_id = 1
        self.map_epoch = ""

    def update(self, observations: list[Any], *, frame_id: str, timestamp: float,
               map_epoch: str) -> list[Track]:
        """按类别、世界位置及图像框一对一关联；地图 epoch 变化清空轨迹。"""
        if map_epoch and self.map_epoch and map_epoch != self.map_epoch:
            self.reset()
        if map_epoch:
            self.map_epoch = map_epoch
        used: set[str] = set()
        results: list[Track] = []
        for obs in observations:
            labels = labels_of(obs)
            xyz = getattr(obs, "map_xyz", None)
            bbox = getattr(obs, "bbox", None)
            best: Track | None = None
            best_score = float("inf")
            for track in self.tracks.values():
                if track.track_id in used or not labels.intersection(track.labels):
                    continue
                if timestamp - track.last_time > 15 or timestamp < track.last_time:
                    continue
                if xyz is not None and track.last_xyz is not None:
                    score = math.dist(xyz, track.last_xyz)
                    if score > 1.5:
                        continue
                elif bbox and track.bbox:
                    score = 1.0 - _iou(bbox, track.bbox)
                    if score > 0.8:
                        continue
                else:
                    continue
                if score < best_score:
                    best, best_score = track, score
            if best is None:
                best = Track(f"track_{self.next_id:04d}", labels)
                self.tracks[best.track_id] = best
                self.next_id += 1
            used.add(best.track_id)
            self._advance(best, labels, xyz, bbox, frame_id, timestamp)
            results.append(best)
        for track_id, track in list(self.tracks.items()):
            if timestamp - track.last_time > 30:
                del self.tracks[track_id]
        return results

    def _advance(self, track: Track, labels: set[str], xyz: Any, bbox: Any,
                 frame_id: str, timestamp: float) -> None:
        if frame_id == track.last_frame_id:
            return
        previous_xyz, previous_time = track.last_xyz, track.last_time
        track.labels |= labels
        track.last_frame_id = frame_id
        track.last_time = timestamp
        track.bbox = bbox
        if labels.intersection(HIGH_DYNAMIC_LABELS):
            track.mobility_class = "dynamic"
            track.mobility_state = "moving"
            track.reason = "人或动物等高动态倾向类别"
            track.last_xyz = xyz
            return
        if xyz is None or not all(math.isfinite(float(v)) for v in xyz):
            track.reason = "缺少可靠深度或定位，等待观测"
            return
        xyz = tuple(float(v) for v in xyz)
        track.last_xyz = xyz
        if previous_xyz is None or timestamp <= previous_time:
            track.stable_start = timestamp
            track.stable_origin = xyz
            track.stable_frames = 1
            return
        dt = timestamp - previous_time
        distance = math.dist(xyz, previous_xyz)
        moving = (distance > float(self.config.get("moving_distance_m", .25))
                  and distance / dt > float(self.config.get("moving_speed_mps", .15)))
        if moving:
            track.motion_hits += 1
            track.stable_start = timestamp
            track.stable_origin = xyz
            track.stable_frames = 1
            if track.motion_hits >= int(self.config.get("moving_evidence", 2)):
                track.mobility_class = "dynamic"
                track.mobility_state = "moving"
                track.position_change_pending = True
                track.reason = f"位姿补偿后连续 {track.motion_hits} 次运动证据"
            else:
                track.mobility_state = "motion_candidate"
                track.reason = "一次运动候选，暂缓更新位置"
            return
        if track.stable_origin is None or math.dist(xyz, track.stable_origin) > float(self.config.get("stable_radius_m", .15)):
            track.stable_start = timestamp
            track.stable_origin = xyz
            track.stable_frames = 1
            return
        track.stable_frames += 1
        span = float(self.config.get("restable_span_s", 3.0) if track.mobility_class == "dynamic" else self.config.get("stable_span_s", 2.0))
        if track.stable_frames >= int(self.config.get("stable_observations", 3)) and timestamp - track.stable_start >= span:
            track.motion_hits = 0
            track.mobility_class = "fixed" if labels.intersection(FIXED_LABELS) else "semi_static"
            track.mobility_state = "stationary"
            track.reason = "连续定位稳定；固定地标类别" if track.mobility_class == "fixed" else "连续定位稳定；可移动物体"


def _iou(a: list[float], b: list[float]) -> float:
    if len(a) != 4 or len(b) != 4:
        return 0.0
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    overlap = max(0, x2-x1) * max(0, y2-y1)
    aa = max(0, a[2]-a[0]) * max(0, a[3]-a[1])
    bb = max(0, b[2]-b[0]) * max(0, b[3]-b[1])
    return overlap / max(aa+bb-overlap, 1e-9)
