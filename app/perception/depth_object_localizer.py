"""Depth-based object localization from an atomic RGB-D frame.

The localizer intentionally does not claim a global map position.  Its output
is ``CAMERA_LOCAL`` unless the caller later combines it with a SpatialProvider
pose to produce ``RELATIVE_RGBD`` / ``METRIC_RGBD``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from app.detectors.vocabulary import (
    association_label,
    category_for_label,
    normalize_scene_tag,
    resolve_display_label,
)
from app.perception.rgbd_source import RGBDFrame

SPATIAL_QUALITY_RGB_ONLY = "RGB_ONLY"
SPATIAL_QUALITY_CAMERA_LOCAL = "CAMERA_LOCAL"
SPATIAL_QUALITY_RELATIVE_RGBD = "RELATIVE_RGBD"
SPATIAL_QUALITY_METRIC_RGBD = "METRIC_RGBD"


def decode_mask_runs(
    runs: list[list[int]],
    height: int,
    width: int,
) -> np.ndarray:
    """把 [row, start_x, length] 行游程解码为 HxW bool mask（计划书 §5）。"""
    mask = np.zeros((height, width), dtype=bool)
    for row, start_x, length in runs:
        if 0 <= row < height and length > 0:
            x0 = max(0, int(start_x))
            x1 = min(width, int(start_x) + int(length))
            if x1 > x0:
                mask[row, x0:x1] = True
    return mask


def object_labels(item: dict[str, Any]) -> tuple[str, str]:
    """从一个 scene object 取 `(关联用规范标签, 中文显示名)`。

    两个字段各有各的用途，**不能合成一个**：

    * 关联标签喂 `SemanticObjectMap` 的硬门（`entry.label != obs.label`），
      必须是稳定、可区分的规范值。未知英文保持原文 —— 若与显示名共用
      「未分类物体」这类兜底词，bin / barrel / stool 会被并成同一个实体
      （计划书 §3）。
    * 显示名优先使用已有中文；未知词保留检测器英文，避免大量不同类别全部显示
      成「未分类物体」。
    """
    raw = item.get("label") or item.get("name") or "object"
    zh = item.get("label_zh")
    attributes = item.get("attributes") or ()
    is_yoloe = (
        str(item.get("source") or "") == "yoloe_seg"
        or "detected_by_yoloe" in attributes
    )
    return (
        # YOLOE prompt-free has a stable fused English vocabulary.  Preserve
        # that raw label for identity and map filtering; translating it here
        # would collapse ``door handle`` into ``门`` before the non-object gate
        # gets a chance to see the disallowed ``handle`` token.
        normalize_scene_tag(raw) if is_yoloe else association_label(raw, runtime_zh=zh),
        resolve_display_label(raw, runtime_zh=zh),
    )


def object_category(item: dict[str, Any]) -> str:
    """取物体类别；检测器已经算好就用它的，否则按标签查静态表。"""
    raw = item.get("label") or item.get("name") or "object"
    return str(item.get("category") or category_for_label(str(raw)))


@dataclass
class ObjectSpatialObservation:
    object_id: str | None
    label: str
    label_zh: str = ""
    category: str = ""
    bbox: list[float] | None = None
    depth_m: float | None = None
    camera_xyz: tuple[float, float, float] | None = None
    bearing_deg: float | None = None
    elevation_deg: float | None = None
    map_xyz: tuple[float, float, float] | None = None
    spatial_quality: str = SPATIAL_QUALITY_RGB_ONLY
    confidence: float = 0.0
    box_center_camera: tuple[float, float, float] | None = None
    box_size_camera: tuple[float, float, float] | None = None
    box_center_map: tuple[float, float, float] | None = None
    box_size_map: tuple[float, float, float] | None = None
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "label": self.label,
            "label_zh": self.label_zh,
            "category": self.category,
            "bbox": self.bbox,
            "depth_m": self.depth_m,
            "camera_xyz": list(self.camera_xyz) if self.camera_xyz else None,
            "bearing_deg": self.bearing_deg,
            "elevation_deg": self.elevation_deg,
            "map_xyz": list(self.map_xyz) if self.map_xyz else None,
            "spatial_quality": self.spatial_quality,
            "confidence": self.confidence,
            "box_center_camera": (
                list(self.box_center_camera) if self.box_center_camera else None
            ),
            "box_size_camera": (
                list(self.box_size_camera) if self.box_size_camera else None
            ),
            "box_center_map": (
                list(self.box_center_map) if self.box_center_map else None
            ),
            "box_size_map": list(self.box_size_map) if self.box_size_map else None,
            "provenance": self.provenance,
        }


def _source_object_id(item: dict[str, Any]) -> str | None:
    """Return the stable source object id for a scene object dict.

    The real-time semantic observer uses ``frame_object_id`` (e.g.
    ``semantic_obj_001``) as the per-frame identity.  Falling back to the other
    common key names keeps compatibility with older payloads while still
    preserving the frame-object identity through the RGB-D chain.
    """
    value = (
        item.get("id")
        or item.get("object_id")
        or item.get("frame_object_id")
    )
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def enrich_scene_objects_with_spatial(
    scene_objects: list[dict[str, Any]],
    localized: list[ObjectSpatialObservation],
    *,
    image_timestamp: float,
) -> list[dict[str, Any]]:
    """按同帧 source id 把 RGB-D / map 坐标写回语义物体。

    只允许 ``id/object_id/frame_object_id`` 精确匹配；不按 label 或列表下标
    猜测，因此同类多个物体不会互相串位。
    """
    localized_by_source_id = {
        str(item.object_id): item
        for item in localized
        if item.object_id
    }
    enriched: list[dict[str, Any]] = []
    for scene_object in scene_objects:
        item = dict(scene_object)
        source_object_id = _source_object_id(item)
        spatial = localized_by_source_id.get(source_object_id or "")
        if spatial is not None:
            item.update({
                "source_object_id": source_object_id,
                "camera_xyz": (
                    list(spatial.camera_xyz) if spatial.camera_xyz is not None else None
                ),
                "map_xyz": list(spatial.map_xyz) if spatial.map_xyz is not None else None,
                "box_center_camera": (
                    list(spatial.box_center_camera)
                    if spatial.box_center_camera is not None else None
                ),
                "box_size_camera": (
                    list(spatial.box_size_camera)
                    if spatial.box_size_camera is not None else None
                ),
                "box_center_map": (
                    list(spatial.box_center_map)
                    if spatial.box_center_map is not None else None
                ),
                "box_size_map": (
                    list(spatial.box_size_map)
                    if spatial.box_size_map is not None else None
                ),
                "map_frame": (spatial.provenance or {}).get("map_frame"),
                "spatial_quality": spatial.spatial_quality,
                "image_timestamp": float(image_timestamp),
            })
        enriched.append(item)
    return enriched


def resolve_depth_frame(
    semantic_frame_id: str | None,
    current_frame: RGBDFrame | None,
    cache: dict[str, Any] | None = None,
) -> RGBDFrame | None:
    """计划书 §5.2：为语义结果选择“同帧”深度帧做 3D 定位。

    只允许：
    * 语义 source frame == 当前 RGB-D 帧；或
    * 从 RGB-D 缓存中取回语义 source frame 对应的原始深度帧。
    找不到同帧深度 -> 返回 None，调用方必须降级为 SEMANTIC_2D_ONLY，
    严禁用当前 depth 顶替旧帧语义（old bbox + current depth = 错误定位）。
    对带采集身份的HTTP帧使用evidence_id，不将重连后复用的数字计数器
    视为同一采集；缓存键命中后仍检查帧本身的身份。
    """
    if semantic_frame_id is None:
        return current_frame
    if current_frame is not None and current_frame.evidence_id == str(semantic_frame_id):
        return current_frame
    if cache is not None:
        candidate = cache.get(str(semantic_frame_id))
        if isinstance(candidate, RGBDFrame) and candidate.evidence_id == str(semantic_frame_id):
            return candidate
    return None


class DepthObjectLocalizer:
    """Localize detected objects using the dominant depth surface in the bbox."""

    def __init__(
        self,
        *,
        min_depth_m: float = 0.15,
        max_depth_m: float = 8.0,
        bbox_inner_ratio: float = 0.5,
        mad_multiplier: float = 3.5,
        depth_unit_m: float = 0.001,
        max_mask_depth_points: int = 500,
        mask_depth_tolerance_m: float = 0.30,
        pipe_surface_radius_px: int = 18,
        pipe_surface_min_points: int = 8,
        pipe_surface_max_mad_m: float = 0.12,
    ) -> None:
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.bbox_inner_ratio = max(0.1, min(1.0, float(bbox_inner_ratio)))
        self.mad_multiplier = float(mad_multiplier)
        self.depth_unit_m = float(depth_unit_m)
        self.max_mask_depth_points = max(16, int(max_mask_depth_points))
        self.mask_depth_tolerance_m = float(mask_depth_tolerance_m)
        self.pipe_surface_radius_px = max(2, int(pipe_surface_radius_px))
        self.pipe_surface_min_points = max(3, int(pipe_surface_min_points))
        self.pipe_surface_max_mad_m = max(0.01, float(pipe_surface_max_mad_m))
        self._last_sample_count = 0

    def localize(
        self,
        scene_objects: list[dict[str, Any]],
        frame: RGBDFrame,
    ) -> list[ObjectSpatialObservation]:
        depth_m = self._load_depth(frame)
        if depth_m is None:
            return [
                self._rgb_only(item, frame)
                for item in scene_objects
            ]
        result: list[ObjectSpatialObservation] = []
        for item in scene_objects:
            mask = self._decode_mask(item, frame)
            if mask is not None:
                result.append(self._localize_with_mask(item, frame, depth_m, mask))
            else:
                result.append(self._localize_with_bbox(item, frame, depth_m))
        return result

    def _localize_with_bbox(
        self,
        item: dict[str, Any],
        frame: RGBDFrame,
        depth_m: np.ndarray,
    ) -> ObjectSpatialObservation:
        bbox = item.get("bbox_2d") or item.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return self._rgb_only(item, frame)
        bbox_px = self._to_pixels(bbox, frame.width, frame.height)
        if bbox_px is None:
            return self._rgb_only(item, frame)
        depth = self._sample_depth(depth_m, bbox_px)
        if depth is None:
            return self._rgb_only(item, frame)
        u = (bbox_px[0] + bbox_px[2]) / 2.0
        v = (bbox_px[1] + bbox_px[3]) / 2.0
        xyz = self._camera_xyz(u, v, depth, frame)
        bearing, elevation = self._angles(xyz)
        label, label_zh = object_labels(item)
        return ObjectSpatialObservation(
            object_id=_source_object_id(item),
            label=label,
            label_zh=label_zh,
            category=object_category(item),
            bbox=list(bbox),
            depth_m=round(depth, 4),
            camera_xyz=(round(xyz[0], 4), round(xyz[1], 4), round(xyz[2], 4)),
            bearing_deg=round(bearing, 3),
            elevation_deg=round(elevation, 3),
            spatial_quality=SPATIAL_QUALITY_CAMERA_LOCAL,
            confidence=float(item.get("confidence", item.get("score", 0.5))),
            provenance={
                "source": "depth_object_localizer",
                "depth_source": "bbox",
                "location_mode": "direct",
                "depth_valid_pixels": self._last_sample_count,
                "frame_id": frame.evidence_id,
                "sensor_frame_id": frame.frame_id,
                "source_object_id": _source_object_id(item),
            },
        )

    def _localize_with_mask(
        self,
        item: dict[str, Any],
        frame: RGBDFrame,
        depth_m: np.ndarray,
        mask: np.ndarray,
    ) -> ObjectSpatialObservation:
        if mask.shape != depth_m.shape:
            return self._hazard_fallback_or_bbox(item, frame, depth_m, mask=None)
        points = self._mask_camera_points(mask, depth_m, frame)
        if points is None:
            return self._hazard_fallback_or_bbox(item, frame, depth_m, mask=mask)
        center, size = self._aabb(points)
        depth = float(np.median(points[:, 2]))
        bearing, elevation = self._angles(center)
        bbox = item.get("bbox_2d") or item.get("bbox")
        label, label_zh = object_labels(item)
        return ObjectSpatialObservation(
            object_id=_source_object_id(item),
            label=label,
            label_zh=label_zh,
            category=object_category(item),
            bbox=list(bbox) if isinstance(bbox, (list, tuple)) else None,
            depth_m=round(depth, 4),
            camera_xyz=(round(center[0], 4), round(center[1], 4), round(center[2], 4)),
            bearing_deg=round(bearing, 3),
            elevation_deg=round(elevation, 3),
            spatial_quality=SPATIAL_QUALITY_CAMERA_LOCAL,
            confidence=float(item.get("confidence", item.get("score", 0.5))),
            box_center_camera=(
                round(float(center[0]), 4),
                round(float(center[1]), 4),
                round(float(center[2]), 4),
            ),
            box_size_camera=(
                round(float(size[0]), 4),
                round(float(size[1]), 4),
                round(float(size[2]), 4),
            ),
            provenance={
                "source": "depth_object_localizer",
                "depth_source": str(item.get("mask_source") or "sam2_mask"),
                "mask_source": str(item.get("mask_source") or "sam2_mask"),
                "location_mode": "direct",
                "depth_valid_pixels": self._last_sample_count,
                "frame_id": frame.evidence_id,
                "sensor_frame_id": frame.frame_id,
                "source_object_id": _source_object_id(item),
            },
        )

    def _hazard_fallback_or_bbox(
        self,
        item: dict[str, Any],
        frame: RGBDFrame,
        depth_m: np.ndarray,
        *,
        mask: np.ndarray | None,
    ) -> ObjectSpatialObservation:
        """Use local host-surface depth for a mask-backed simulated hazard.

        Ordinary objects keep the legacy bbox fallback.  A hazard must never
        borrow arbitrary background pixels from its rectangle: it either gets
        a local estimate from the associated visible pipe surface, remains
        host-only, or is explicitly unavailable.
        """
        if str(item.get("category") or "") not in {"hazard", "review"}:
            return self._localize_with_bbox(item, frame, depth_m)
        context = self._decode_context_mask(item, frame)
        if context is None or mask is None:
            return self._rgb_only_with_location(
                item, frame, "host_only" if item.get("host_detection_index") is not None
                else "unavailable"
            )
        estimate = self._sample_pipe_surface_depth(mask, context, depth_m)
        if estimate is None:
            return self._rgb_only_with_location(item, frame, "host_only")
        u, v, depth, count, mad = estimate
        xyz = self._camera_xyz(u, v, depth, frame)
        bearing, elevation = self._angles(xyz)
        label, label_zh = object_labels(item)
        bbox = item.get("bbox_2d") or item.get("bbox")
        return ObjectSpatialObservation(
            object_id=_source_object_id(item),
            label=label,
            label_zh=label_zh,
            category=object_category(item),
            bbox=list(bbox) if isinstance(bbox, (list, tuple)) else None,
            depth_m=round(depth, 4),
            camera_xyz=(round(xyz[0], 4), round(xyz[1], 4), round(xyz[2], 4)),
            bearing_deg=round(bearing, 3),
            elevation_deg=round(elevation, 3),
            spatial_quality=SPATIAL_QUALITY_CAMERA_LOCAL,
            confidence=float(item.get("confidence", item.get("score", 0.5))),
            provenance={
                "source": "depth_object_localizer",
                "depth_source": "pipe_surface_estimate",
                "mask_source": str(item.get("mask_source") or "unknown_mask"),
                "location_mode": "pipe_surface_estimate",
                "depth_valid_pixels": count,
                "pipe_surface_depth_mad": round(mad, 4),
                "frame_id": frame.evidence_id,
                "sensor_frame_id": frame.frame_id,
                "source_object_id": _source_object_id(item),
            },
        )

    def _decode_context_mask(
        self, item: dict[str, Any], frame: RGBDFrame
    ) -> np.ndarray | None:
        runs = item.get("context_mask_rle")
        size = item.get("context_mask_size")
        if not isinstance(runs, list) or not runs:
            return None
        if not isinstance(size, (list, tuple)) or len(size) != 2:
            return None
        try:
            height, width = int(size[0]), int(size[1])
        except (TypeError, ValueError):
            return None
        if (height, width) != (frame.height, frame.width):
            return None
        return decode_mask_runs(runs, height, width)

    def _sample_pipe_surface_depth(
        self,
        cue_mask: np.ndarray,
        context_mask: np.ndarray,
        depth_m: np.ndarray,
    ) -> tuple[float, float, float, int, float] | None:
        ys, xs = np.nonzero(cue_mask)
        if ys.size == 0:
            return None
        u = float(np.median(xs))
        v = float(np.median(ys))
        yy, xx = np.ogrid[:depth_m.shape[0], :depth_m.shape[1]]
        local = (xx - u) ** 2 + (yy - v) ** 2 <= self.pipe_surface_radius_px ** 2
        valid = (
            context_mask & ~cue_mask & local & np.isfinite(depth_m)
            & (depth_m > self.min_depth_m) & (depth_m < self.max_depth_m)
        )
        values = depth_m[valid]
        if values.size < self.pipe_surface_min_points:
            return None
        median = float(np.median(values))
        mad = float(np.median(np.abs(values - median)))
        if mad > self.pipe_surface_max_mad_m:
            return None
        keep = np.abs(values - median) <= max(0.03, 3.5 * max(mad, 1e-6))
        values = values[keep]
        if values.size < self.pipe_surface_min_points:
            return None
        return u, v, float(np.median(values)), int(values.size), mad

    def _decode_mask(
        self,
        item: dict[str, Any],
        frame: RGBDFrame,
    ) -> np.ndarray | None:
        mask_rle = item.get("mask_rle")
        mask_size = item.get("mask_size")
        if not isinstance(mask_rle, list) or not mask_rle:
            return None
        if not isinstance(mask_size, (list, tuple)) or len(mask_size) != 2:
            return None
        try:
            height, width = int(mask_size[0]), int(mask_size[1])
        except (TypeError, ValueError):
            return None
        if height <= 0 or width <= 0:
            return None
        return decode_mask_runs(mask_rle, height, width)

    def _mask_camera_points(
        self,
        mask: np.ndarray,
        depth_m: np.ndarray,
        frame: RGBDFrame,
    ) -> np.ndarray | None:
        valid = (
            mask
            & (depth_m > self.min_depth_m)
            & (depth_m < self.max_depth_m)
            & np.isfinite(depth_m)
        )
        ys, xs = np.nonzero(valid)
        if ys.size < 3:
            self._last_sample_count = 0
            return None
        if ys.size > self.max_mask_depth_points:
            indexes = np.linspace(0, ys.size - 1, self.max_mask_depth_points).astype(int)
            ys, xs = ys[indexes], xs[indexes]
        depths = depth_m[ys, xs]
        median = float(np.median(depths))
        keep = np.abs(depths - median) <= self.mask_depth_tolerance_m
        ys, xs, depths = ys[keep], xs[keep], depths[keep]
        if ys.size < 3:
            self._last_sample_count = 0
            return None
        self._last_sample_count = int(ys.size)
        if frame.fx <= 0 or frame.fy <= 0:
            return np.stack([np.zeros_like(depths), np.zeros_like(depths), depths], axis=1)
        x = (xs - frame.cx) / frame.fx * depths
        y = (ys - frame.cy) / frame.fy * depths
        return np.stack([x, y, depths], axis=1)

    @staticmethod
    def _aabb(points: np.ndarray) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """p05/p95 AABB：比 min/max 稳（计划书 §8.3）。"""
        low = np.percentile(points, 5, axis=0)
        high = np.percentile(points, 95, axis=0)
        center = (low + high) / 2.0
        size = high - low
        return (
            (float(center[0]), float(center[1]), float(center[2])),
            (float(size[0]), float(size[1]), float(size[2])),
        )

    def _rgb_only(self, item: dict[str, Any], frame: RGBDFrame) -> ObjectSpatialObservation:
        return self._rgb_only_with_location(item, frame, "unavailable")

    def _rgb_only_with_location(
        self, item: dict[str, Any], frame: RGBDFrame, location_mode: str
    ) -> ObjectSpatialObservation:
        label, label_zh = object_labels(item)
        return ObjectSpatialObservation(
            object_id=_source_object_id(item),
            label=label,
            label_zh=label_zh,
            category=object_category(item),
            bbox=list(item.get("bbox_2d") or item.get("bbox") or []),
            spatial_quality=SPATIAL_QUALITY_RGB_ONLY,
            confidence=float(item.get("confidence", item.get("score", 0.5))),
            provenance={
                "source": "depth_object_localizer_rgb_only",
                "mask_source": item.get("mask_source"),
                "location_mode": location_mode,
                "frame_id": frame.evidence_id,
                "sensor_frame_id": frame.frame_id,
                "source_object_id": _source_object_id(item),
            },
        )

    def _load_depth(self, frame: RGBDFrame) -> np.ndarray | None:
        try:
            import cv2
        except ImportError:
            return None
        path = frame.depth_ref
        if not path:
            return None
        img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if img is None:
            return None
        if img.dtype == np.uint16:
            depth_mm = img.astype(np.float32)
        else:
            depth_mm = img.astype(np.float32)
        return depth_mm * float(frame.depth_unit_m if frame.depth_unit_m else self.depth_unit_m)

    @staticmethod
    def _to_pixels(bbox: list[float], width: int, height: int) -> tuple[int, int, int, int] | None:
        if width <= 0 or height <= 0:
            return None
        x1, y1, x2, y2 = (float(v) for v in bbox)
        normalized = max(abs(x1), abs(y1), abs(x2), abs(y2)) <= 1.5
        if normalized:
            x1, x2 = x1 * width, x2 * width
            y1, y2 = y1 * height, y2 * height
        px = (int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2)))
        if px[2] <= px[0] or px[3] <= px[1]:
            return None
        return (
            max(0, px[0]), max(0, px[1]),
            min(width - 1, px[2]), min(height - 1, px[3]),
        )

    def _sample_depth(self, depth_m: np.ndarray, bbox: tuple[int, int, int, int]) -> float | None:
        x1, y1, x2, y2 = bbox
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)
        inner_w = max(1, int(round(bw * self.bbox_inner_ratio)))
        inner_h = max(1, int(round(bh * self.bbox_inner_ratio)))
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        ix1 = max(x1, int(round(cx - inner_w / 2.0)))
        ix2 = min(x2, ix1 + inner_w)
        iy1 = max(y1, int(round(cy - inner_h / 2.0)))
        iy2 = min(y2, iy1 + inner_h)
        if ix2 <= ix1 or iy2 <= iy1:
            return None
        inner_patch = depth_m[iy1:iy2, ix1:ix2]
        inner_valid = inner_patch[
            (inner_patch > self.min_depth_m)
            & (inner_patch < self.max_depth_m)
            & np.isfinite(inner_patch)
        ]
        if inner_valid.size < 1:
            self._last_sample_count = 0
            return None

        # A large object can contain several real depth layers.  A changing VLM
        # bbox may move the inner median from one layer to another (for example,
        # from a desk's rear surface to its nearer top).  Pick the strongest
        # 5-cm depth cluster in the complete bbox, then require that cluster to
        # also be present in the inner crop.  The inner support check prevents
        # a large far-background region from winning a loose ordinary-object
        # bbox.
        full_patch = depth_m[y1:y2, x1:x2]
        full_valid = full_patch[
            (full_patch > self.min_depth_m)
            & (full_patch < self.max_depth_m)
            & np.isfinite(full_patch)
        ]
        bin_width_m = 0.05
        bin_count = max(
            1,
            int(math.ceil((self.max_depth_m - self.min_depth_m) / bin_width_m)),
        )
        histogram, edges = np.histogram(
            full_valid,
            bins=bin_count,
            range=(self.min_depth_m, self.max_depth_m),
        )
        dominant_index = int(np.argmax(histogram))
        lower = float(edges[dominant_index])
        upper = float(edges[dominant_index + 1])
        full_cluster = full_valid[
            (full_valid >= lower)
            & (full_valid <= upper if dominant_index == bin_count - 1 else full_valid < upper)
        ]
        inner_cluster = inner_valid[
            (inner_valid >= lower)
            & (inner_valid <= upper if dominant_index == bin_count - 1 else inner_valid < upper)
        ]
        inner_support = float(inner_cluster.size) / float(inner_valid.size)
        full_support = float(full_cluster.size) / float(full_valid.size)
        cluster_depth = float(np.median(full_cluster))
        wide_bbox = bw >= 2 * bh
        if (inner_support >= 0.005 or wide_bbox) and (
            full_support <= 0.50
            or inner_support >= 0.80
            or cluster_depth <= float(np.median(inner_valid)) + bin_width_m
        ):
            self._last_sample_count = int(inner_cluster.size)
            return cluster_depth

        self._last_sample_count = int(inner_valid.size)
        median = float(np.median(inner_valid))
        mad = float(np.median(np.abs(inner_valid - median))) if inner_valid.size > 1 else 0.0
        if mad > 1e-6:
            lower = median - self.mad_multiplier * 1.4826 * mad
            upper = median + self.mad_multiplier * 1.4826 * mad
            filtered = inner_valid[(inner_valid >= lower) & (inner_valid <= upper)]
            if filtered.size > 0:
                return float(np.median(filtered))
        return median

    @staticmethod
    def _camera_xyz(u: float, v: float, depth: float, frame: RGBDFrame) -> tuple[float, float, float]:
        if frame.fx <= 0 or frame.fy <= 0:
            return (0.0, 0.0, depth)
        x = (u - frame.cx) / frame.fx * depth
        y = (v - frame.cy) / frame.fy * depth
        return (x, y, depth)

    @staticmethod
    def _angles(xyz: tuple[float, float, float]) -> tuple[float, float]:
        x, y, z = xyz
        bearing = math.degrees(math.atan2(x, z)) if z > 1e-6 else 0.0
        elevation = math.degrees(math.atan2(y, z)) if z > 1e-6 else 0.0
        return bearing, elevation
