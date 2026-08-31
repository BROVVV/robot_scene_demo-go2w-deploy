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

from app.perception.rgbd_source import RGBDFrame

SPATIAL_QUALITY_RGB_ONLY = "RGB_ONLY"
SPATIAL_QUALITY_CAMERA_LOCAL = "CAMERA_LOCAL"
SPATIAL_QUALITY_RELATIVE_RGBD = "RELATIVE_RGBD"
SPATIAL_QUALITY_METRIC_RGBD = "METRIC_RGBD"


@dataclass
class ObjectSpatialObservation:
    object_id: str | None
    label: str
    bbox: list[float] | None = None
    depth_m: float | None = None
    camera_xyz: tuple[float, float, float] | None = None
    bearing_deg: float | None = None
    elevation_deg: float | None = None
    map_xyz: tuple[float, float, float] | None = None
    spatial_quality: str = SPATIAL_QUALITY_RGB_ONLY
    confidence: float = 0.0
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "label": self.label,
            "bbox": self.bbox,
            "depth_m": self.depth_m,
            "camera_xyz": list(self.camera_xyz) if self.camera_xyz else None,
            "bearing_deg": self.bearing_deg,
            "elevation_deg": self.elevation_deg,
            "map_xyz": list(self.map_xyz) if self.map_xyz else None,
            "spatial_quality": self.spatial_quality,
            "confidence": self.confidence,
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


class DepthObjectLocalizer:
    """Localize detected objects in a depth image using robust median sampling."""

    def __init__(
        self,
        *,
        min_depth_m: float = 0.15,
        max_depth_m: float = 8.0,
        bbox_inner_ratio: float = 0.5,
        mad_multiplier: float = 3.5,
        depth_unit_m: float = 0.001,
    ) -> None:
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.bbox_inner_ratio = max(0.1, min(1.0, float(bbox_inner_ratio)))
        self.mad_multiplier = float(mad_multiplier)
        self.depth_unit_m = float(depth_unit_m)
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
            bbox = item.get("bbox_2d") or item.get("bbox")
            if not isinstance(bbox, list) or len(bbox) != 4:
                result.append(self._rgb_only(item, frame))
                continue
            bbox_px = self._to_pixels(bbox, frame.width, frame.height)
            if bbox_px is None:
                result.append(self._rgb_only(item, frame))
                continue
            depth = self._sample_depth(depth_m, bbox_px)
            if depth is None:
                result.append(self._rgb_only(item, frame))
                continue
            u = (bbox_px[0] + bbox_px[2]) / 2.0
            v = (bbox_px[1] + bbox_px[3]) / 2.0
            xyz = self._camera_xyz(u, v, depth, frame)
            bearing, elevation = self._angles(xyz)
            result.append(
                ObjectSpatialObservation(
                    object_id=_source_object_id(item),
                    label=str(
                        item.get("label_zh")
                        or item.get("label")
                        or item.get("name")
                        or "object"
                    ),
                    bbox=list(bbox),
                    depth_m=round(depth, 4),
                    camera_xyz=(round(xyz[0], 4), round(xyz[1], 4), round(xyz[2], 4)),
                    bearing_deg=round(bearing, 3),
                    elevation_deg=round(elevation, 3),
                    spatial_quality=SPATIAL_QUALITY_CAMERA_LOCAL,
                    confidence=float(item.get("confidence", item.get("score", 0.5))),
                    provenance={
                        "source": "depth_object_localizer",
                        "depth_valid_pixels": self._last_sample_count,
                        "frame_id": frame.frame_id,
                        "source_object_id": _source_object_id(item),
                    },
                )
            )
        return result

    def _rgb_only(self, item: dict[str, Any], frame: RGBDFrame) -> ObjectSpatialObservation:
        return ObjectSpatialObservation(
            object_id=_source_object_id(item),
            label=str(
                item.get("label_zh") or item.get("label") or item.get("name") or "object"
            ),
            bbox=list(item.get("bbox_2d") or item.get("bbox") or []),
            spatial_quality=SPATIAL_QUALITY_RGB_ONLY,
            confidence=float(item.get("confidence", item.get("score", 0.5))),
            provenance={
                "source": "depth_object_localizer_rgb_only",
                "frame_id": frame.frame_id,
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
        patch = depth_m[iy1:iy2, ix1:ix2]
        valid = patch[(patch > self.min_depth_m) & (patch < self.max_depth_m) & np.isfinite(patch)]
        self._last_sample_count = int(valid.size)
        if valid.size < 1:
            return None
        median = float(np.median(valid))
        mad = float(np.median(np.abs(valid - median))) if valid.size > 1 else 0.0
        # Conservative MAD filter; fall back to percentile if MAD is zero.
        if mad > 1e-6:
            lower = median - self.mad_multiplier * 1.4826 * mad
            upper = median + self.mad_multiplier * 1.4826 * mad
            filtered = valid[(valid >= lower) & (valid <= upper)]
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
