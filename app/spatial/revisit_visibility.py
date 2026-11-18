"""Conservative ID-specific visibility checks for persistent-object revisits."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class RevisitVisibilityDecision:
    counts_as_missing: bool
    reason: str
    pixel: tuple[float, float] | None = None
    expected_depth_m: float | None = None
    observed_depth_m: float | None = None


def evaluate_revisit_visibility(
    *,
    camera_xyz: Any,
    depth_image: Any,
    width: int,
    height: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_unit_m: float,
    localization_valid: bool,
    detector_covered: bool,
    frame_reliable: bool = True,
    edge_margin_ratio: float = 0.08,
    depth_window_px: int = 4,
    background_clearance_m: float = 0.25,
    max_detection_distance_m: float = 8.0,
) -> RevisitVisibilityDecision:
    """Decide whether one frame proves that an old position is now empty.

    A point at roughly the expected depth is *not* absence evidence: it can be
    the old object with a missed detection.  Only a clearly farther background
    counts.  A closer sample is treated as occlusion.
    """
    if not localization_valid:
        return RevisitVisibilityDecision(False, "localization_unavailable")
    if not detector_covered:
        return RevisitVisibilityDecision(False, "detector_category_not_scheduled")
    if not frame_reliable:
        return RevisitVisibilityDecision(False, "frame_unreliable")
    xyz = _finite_xyz(camera_xyz)
    if xyz is None or xyz[2] <= 0.0:
        return RevisitVisibilityDecision(False, "behind_camera_or_invalid_transform")
    if xyz[2] > float(max_detection_distance_m):
        return RevisitVisibilityDecision(False, "beyond_detection_range", expected_depth_m=xyz[2])
    if min(width, height) <= 0 or min(fx, fy) <= 0:
        return RevisitVisibilityDecision(False, "invalid_intrinsics")
    u = float(fx) * xyz[0] / xyz[2] + float(cx)
    v = float(fy) * xyz[1] / xyz[2] + float(cy)
    margin_x, margin_y = width * edge_margin_ratio, height * edge_margin_ratio
    if not (margin_x <= u < width - margin_x and margin_y <= v < height - margin_y):
        return RevisitVisibilityDecision(False, "outside_usable_fov", (u, v), xyz[2])
    depth = np.asarray(depth_image) if depth_image is not None else np.asarray([])
    if depth.ndim != 2 or depth.shape[0] != height or depth.shape[1] != width:
        return RevisitVisibilityDecision(False, "depth_unavailable", (u, v), xyz[2])
    radius = max(1, int(depth_window_px))
    x, y = int(round(u)), int(round(v))
    patch = depth[max(0, y-radius):min(height, y+radius+1),
                  max(0, x-radius):min(width, x+radius+1)]
    values = patch.astype(np.float64, copy=False).reshape(-1) * float(depth_unit_m)
    values = values[np.isfinite(values) & (values > 0.05)]
    if values.size < 3:
        return RevisitVisibilityDecision(False, "depth_unavailable", (u, v), xyz[2])
    observed = float(np.median(values))
    if observed < xyz[2] - float(background_clearance_m):
        return RevisitVisibilityDecision(False, "occluded_by_nearer_surface", (u, v), xyz[2], observed)
    if observed <= xyz[2] + float(background_clearance_m):
        return RevisitVisibilityDecision(False, "surface_still_at_expected_depth", (u, v), xyz[2], observed)
    return RevisitVisibilityDecision(True, "clear_background_behind_old_position", (u, v), xyz[2], observed)


def _finite_xyz(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        result = tuple(float(value[index]) for index in range(3))
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(part) for part in result) else None
