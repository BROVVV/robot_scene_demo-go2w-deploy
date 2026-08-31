"""Projection and robust target clustering in camera optical coordinates."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product

import cv2
import numpy as np


@dataclass(frozen=True)
class FusionParameters:
    maximum_timestamp_delta_ms: float = 50.0
    minimum_mask_points: int = 12
    mask_boundary_margin_px: int = 3
    depth_mad_multiplier: float = 3.5
    cluster_tolerance_m: float = 0.12
    minimum_cluster_points: int = 8
    maximum_cluster_extent_m: float = 1.5


@dataclass(frozen=True)
class LocalizationResult:
    localized_3d: bool
    reason: str
    position_camera_m: tuple[float, float, float] | None = None
    robust_size_m: tuple[float, float, float] | None = None
    point_count: int = 0
    timestamp_delta_ms: float = 0.0
    confidence: float = 0.0


def localize_mask_points(
    points_camera: np.ndarray,
    mask: np.ndarray,
    camera_matrix: np.ndarray,
    bbox_pixels: tuple[float, float, float, float],
    timestamp_delta_ms: float,
    parameters: FusionParameters,
    distortion_coefficients: np.ndarray | None = None,
    distortion_model: str = "plumb_bob",
) -> LocalizationResult:
    delta = abs(float(timestamp_delta_ms))
    if delta > parameters.maximum_timestamp_delta_ms:
        return LocalizationResult(False, "timestamp_delta_exceeded", timestamp_delta_ms=delta)
    points = np.asarray(points_camera, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_camera must have shape Nx3")
    mask_array = np.asarray(mask)
    if mask_array.ndim != 2 or mask_array.size == 0:
        return LocalizationResult(False, "mask_unavailable", timestamp_delta_ms=delta)
    intrinsic = np.asarray(camera_matrix, dtype=np.float64)
    if intrinsic.shape != (3, 3) or intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0:
        return LocalizationResult(False, "camera_info_unavailable", timestamp_delta_ms=delta)
    finite_front = np.isfinite(points).all(axis=1) & (points[:, 2] > 0.0)
    points = points[finite_front]
    if len(points) == 0:
        return LocalizationResult(False, "target_outside_lidar_coverage", timestamp_delta_ms=delta)
    distortion = (
        np.zeros(5, dtype=np.float64)
        if distortion_coefficients is None
        else np.asarray(distortion_coefficients, dtype=np.float64).reshape(-1)
    )
    if not np.isfinite(distortion).all():
        return LocalizationResult(False, "camera_info_unavailable", timestamp_delta_ms=delta)
    if distortion_model == "equidistant":
        if distortion.size != 4:
            return LocalizationResult(False, "camera_info_unavailable", timestamp_delta_ms=delta)
        projected, _ = cv2.fisheye.projectPoints(
            points.reshape(-1, 1, 3),
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            intrinsic,
            distortion.reshape(-1, 1),
        )
    elif distortion_model in {"plumb_bob", "rational_polynomial"}:
        projected, _ = cv2.projectPoints(
            points,
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            intrinsic,
            distortion,
        )
    else:
        return LocalizationResult(False, "camera_info_unavailable", timestamp_delta_ms=delta)
    pixels = projected.reshape(-1, 2)
    margin = max(0, int(parameters.mask_boundary_margin_px))
    binary = (mask_array > 0).astype(np.uint8)
    if margin:
        size = 2 * margin + 1
        binary = cv2.erode(binary, np.ones((size, size), np.uint8))
    rounded = np.rint(pixels).astype(np.int64)
    inside = (
        (rounded[:, 0] >= 0)
        & (rounded[:, 0] < binary.shape[1])
        & (rounded[:, 1] >= 0)
        & (rounded[:, 1] < binary.shape[0])
    )
    selected = points[
        inside
        & (binary[
            np.clip(rounded[:, 1], 0, binary.shape[0] - 1),
            np.clip(rounded[:, 0], 0, binary.shape[1] - 1),
        ] > 0)
    ]
    if len(selected) < parameters.minimum_mask_points:
        return LocalizationResult(
            False, "insufficient_mask_points", point_count=len(selected), timestamp_delta_ms=delta
        )
    depths = selected[:, 2]
    median_depth = float(np.median(depths))
    mad = float(np.median(np.abs(depths - median_depth)))
    limit = max(0.03, parameters.depth_mad_multiplier * 1.4826 * mad)
    selected = selected[np.abs(depths - median_depth) <= limit]
    clusters = _euclidean_clusters(
        selected, parameters.cluster_tolerance_m, parameters.minimum_cluster_points
    )
    if not clusters:
        return LocalizationResult(
            False, "cluster_unstable", point_count=len(selected), timestamp_delta_ms=delta
        )
    center_u = (bbox_pixels[0] + bbox_pixels[2]) / 2.0
    center_v = (bbox_pixels[1] + bbox_pixels[3]) / 2.0

    def cluster_score(cluster):
        center = np.median(cluster, axis=0)
        u = intrinsic[0, 0] * center[0] / center[2] + intrinsic[0, 2]
        v = intrinsic[1, 1] * center[1] / center[2] + intrinsic[1, 2]
        pixel_distance = np.hypot(u - center_u, v - center_v)
        return len(cluster) / (1.0 + pixel_distance)

    cluster = max(clusters, key=cluster_score)
    low, high = np.quantile(cluster, [0.1, 0.9], axis=0)
    size = high - low
    if float(np.max(size)) > parameters.maximum_cluster_extent_m:
        return LocalizationResult(
            False, "cluster_extent_implausible", point_count=len(cluster), timestamp_delta_ms=delta
        )
    position = np.median(cluster, axis=0)
    confidence = min(1.0, len(cluster) / max(parameters.minimum_mask_points * 3.0, 1.0))
    confidence *= max(0.0, 1.0 - delta / parameters.maximum_timestamp_delta_ms)
    return LocalizationResult(
        True,
        "localized",
        tuple(float(item) for item in position),
        tuple(float(item) for item in size),
        len(cluster),
        delta,
        float(confidence),
    )


def _euclidean_clusters(points: np.ndarray, tolerance: float, minimum: int):
    if tolerance <= 0.0 or minimum < 1:
        raise ValueError("cluster tolerance and minimum must be positive")
    cells: dict[tuple[int, int, int], list[int]] = {}
    indices = np.floor(points / tolerance).astype(np.int64)
    for index, cell in enumerate(indices):
        cells.setdefault(tuple(cell), []).append(index)
    visited = np.zeros(len(points), dtype=bool)
    clusters = []
    offsets = list(product((-1, 0, 1), repeat=3))
    for seed in range(len(points)):
        if visited[seed]:
            continue
        visited[seed] = True
        queue = [seed]
        members = []
        while queue:
            current = queue.pop()
            members.append(current)
            base = indices[current]
            for offset in offsets:
                neighbor_cell = tuple(base + np.asarray(offset))
                for neighbor in cells.get(neighbor_cell, []):
                    if visited[neighbor]:
                        continue
                    if np.linalg.norm(points[current] - points[neighbor]) <= tolerance:
                        visited[neighbor] = True
                        queue.append(neighbor)
        if len(members) >= minimum:
            clusters.append(points[members])
    return clusters
