"""Calibration-gated LiDAR projection, overlay, and PnP helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


class CalibrationBlocked(ValueError):
    """Required measured calibration evidence is absent or invalid."""


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    distortion_model: str
    matrix: np.ndarray
    distortion: np.ndarray
    source: str


@dataclass(frozen=True)
class LidarToCameraTransform:
    rotation: np.ndarray
    translation: np.ndarray
    source: str


@dataclass(frozen=True)
class ReprojectionMetrics:
    count: int
    mean_px: float
    median_px: float
    p95_px: float
    maximum_px: float
    errors_px: tuple[float, ...]


def select_synchronized_pairs(
    image_records: list[tuple[int, int]],
    cloud_records: list[tuple[int, int]],
    *,
    maximum_delta_ns: int,
    minimum_separation_ns: int,
    maximum_pairs: int,
) -> list[dict[str, int]]:
    """Select evenly spaced closest header-stamp pairs.

    Records are ``(header_stamp_ns, bag_record_time_ns)``.  The bag time keeps
    duplicate header stamps addressable during the extraction pass.
    """
    if maximum_delta_ns < 0 or minimum_separation_ns < 0 or maximum_pairs < 1:
        raise ValueError("pair selection limits are invalid")
    images = sorted((int(stamp), int(bag)) for stamp, bag in image_records)
    clouds = sorted((int(stamp), int(bag)) for stamp, bag in cloud_records)
    if not images or not clouds:
        return []
    image_stamps = np.asarray([item[0] for item in images], dtype=np.int64)
    candidates = []
    last_cloud_stamp = None
    for cloud_stamp, cloud_bag_time in clouds:
        if (
            last_cloud_stamp is not None
            and cloud_stamp - last_cloud_stamp < minimum_separation_ns
        ):
            continue
        insertion = int(np.searchsorted(image_stamps, cloud_stamp))
        options = [index for index in (insertion - 1, insertion) if 0 <= index < len(images)]
        if not options:
            continue
        image_index = min(options, key=lambda index: abs(images[index][0] - cloud_stamp))
        image_stamp, image_bag_time = images[image_index]
        delta = abs(image_stamp - cloud_stamp)
        if delta > maximum_delta_ns:
            continue
        candidates.append(
            {
                "image_stamp_ns": image_stamp,
                "image_bag_time_ns": image_bag_time,
                "cloud_stamp_ns": cloud_stamp,
                "cloud_bag_time_ns": cloud_bag_time,
                "timestamp_delta_ns": delta,
            }
        )
        last_cloud_stamp = cloud_stamp
    if len(candidates) <= maximum_pairs:
        return candidates
    indices = np.linspace(0, len(candidates) - 1, maximum_pairs)
    selected = sorted({int(round(value)) for value in indices})
    return [candidates[index] for index in selected]


def _matrix(payload: dict, key: str, count: int) -> np.ndarray:
    value = payload.get(key, {})
    raw = value.get("data", []) if isinstance(value, dict) else value
    result = np.asarray(raw, dtype=np.float64)
    if result.size != count or not np.isfinite(result).all():
        raise CalibrationBlocked(f"{key} must contain {count} finite values")
    return result


def load_camera_model(path: str | Path) -> CameraModel:
    source = str(Path(path).expanduser().resolve())
    payload = yaml.safe_load(Path(source).read_text(encoding="utf-8")) or {}
    if payload.get("calibration_status") != "calibrated":
        raise CalibrationBlocked("camera intrinsics are not calibrated")
    width = int(payload.get("image_width", 0))
    height = int(payload.get("image_height", 0))
    matrix = _matrix(payload, "camera_matrix", 9).reshape(3, 3)
    distortion = _matrix(
        payload,
        "distortion_coefficients",
        len((payload.get("distortion_coefficients") or {}).get("data", [])),
    ).reshape(-1)
    model = str(payload.get("distortion_model", "")).strip()
    if width <= 0 or height <= 0:
        raise CalibrationBlocked("camera calibration resolution is invalid")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise CalibrationBlocked("camera focal lengths must be positive")
    if model not in {"plumb_bob", "rational_polynomial", "equidistant"}:
        raise CalibrationBlocked(f"unsupported distortion model: {model or 'empty'}")
    if distortion.size == 0:
        raise CalibrationBlocked("camera distortion coefficients are absent")
    return CameraModel(width, height, model, matrix, distortion, source)


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def rpy_from_rotation_matrix(rotation: np.ndarray) -> tuple[float, float, float]:
    value = np.asarray(rotation, dtype=np.float64)
    if value.shape != (3, 3) or not np.isfinite(value).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    pitch = math.asin(float(np.clip(-value[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(value[2, 1], value[2, 2])
        yaw = math.atan2(value[1, 0], value[0, 0])
    else:
        roll = math.atan2(-value[1, 2], value[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def _load_transform(path: str | Path, *, diagnostic: bool) -> LidarToCameraTransform:
    source = str(Path(path).expanduser().resolve())
    payload = yaml.safe_load(Path(source).read_text(encoding="utf-8")) or {}
    if diagnostic:
        if not payload.get("diagnostic_overlay_accepted"):
            raise CalibrationBlocked(
                "camera-LiDAR diagnostic overlay candidate is not accepted"
            )
    elif (
        payload.get("calibration_status") != "calibrated"
        or not payload.get("confirmed")
        or not payload.get("navigation_geometry_validated")
    ):
        raise CalibrationBlocked(
            "camera-LiDAR extrinsics are not navigation-grade calibrated and confirmed"
        )
    if payload.get("transform_parent") != "front_camera_optical_frame":
        raise CalibrationBlocked("extrinsic parent must be front_camera_optical_frame")
    if payload.get("transform_child") != "utlidar_lidar":
        raise CalibrationBlocked("extrinsic child must be utlidar_lidar")
    translation = payload.get("translation_m") or {}
    rotation = payload.get("rotation_rpy_rad") or {}
    try:
        xyz = np.asarray([translation[key] for key in ("x", "y", "z")], dtype=np.float64)
        rpy = np.asarray(
            [rotation[key] for key in ("roll", "pitch", "yaw")], dtype=np.float64
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationBlocked("extrinsic XYZ/RPY is incomplete") from exc
    if not np.isfinite(xyz).all() or not np.isfinite(rpy).all():
        raise CalibrationBlocked("extrinsic XYZ/RPY must be finite")
    return LidarToCameraTransform(
        rotation_matrix_from_rpy(*rpy), xyz, source
    )


def load_diagnostic_transform(path: str | Path) -> LidarToCameraTransform:
    """Load a candidate exclusively for stationary diagnostic visualization."""

    return _load_transform(path, diagnostic=True)


def load_confirmed_transform(path: str | Path) -> LidarToCameraTransform:
    """Load a transform that is explicitly accepted for metric 3D output."""

    return _load_transform(path, diagnostic=False)


def transform_lidar_to_camera(
    points_lidar: np.ndarray, transform: LidarToCameraTransform
) -> np.ndarray:
    points = np.asarray(points_lidar, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_lidar must have shape Nx3")
    return points @ transform.rotation.T + transform.translation.reshape(1, 3)


def project_camera_points(
    points_camera: np.ndarray, camera: CameraModel
) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points_camera, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_camera must have shape Nx3")
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > 0.0)
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    front = points[valid]
    if len(front) == 0:
        return pixels, valid
    if camera.distortion_model == "equidistant":
        projected, _ = cv2.fisheye.projectPoints(
            front.reshape(-1, 1, 3),
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            camera.matrix,
            camera.distortion.reshape(-1, 1),
        )
    else:
        projected, _ = cv2.projectPoints(
            front,
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            camera.matrix,
            camera.distortion,
        )
    pixels[valid] = projected.reshape(-1, 2)
    return pixels, valid


def render_depth_overlay(
    image_bgr: np.ndarray,
    points_lidar: np.ndarray,
    camera: CameraModel,
    transform: LidarToCameraTransform,
    *,
    minimum_depth_m: float = 0.05,
    maximum_depth_m: float = 15.0,
    radius_px: int = 2,
) -> tuple[np.ndarray, dict]:
    image = np.asarray(image_bgr)
    if image.shape[:2] != (camera.height, camera.width) or image.ndim != 3:
        raise ValueError("image resolution does not match camera calibration")
    if not (0.0 < minimum_depth_m < maximum_depth_m) or radius_px < 1:
        raise ValueError("overlay depth/radius parameters are invalid")
    camera_points = transform_lidar_to_camera(points_lidar, transform)
    pixels, front = project_camera_points(camera_points, camera)
    rounded = np.rint(pixels).astype(np.int64, casting="unsafe", copy=False)
    inside = (
        front
        & (camera_points[:, 2] >= minimum_depth_m)
        & (camera_points[:, 2] <= maximum_depth_m)
        & (rounded[:, 0] >= 0)
        & (rounded[:, 0] < camera.width)
        & (rounded[:, 1] >= 0)
        & (rounded[:, 1] < camera.height)
    )
    indices = np.flatnonzero(inside)
    depths = camera_points[indices, 2]
    normalized = np.clip(
        (depths - minimum_depth_m) / (maximum_depth_m - minimum_depth_m), 0.0, 1.0
    )
    colors = cv2.applyColorMap(
        np.rint((1.0 - normalized) * 255.0).astype(np.uint8).reshape(-1, 1),
        cv2.COLORMAP_TURBO,
    ).reshape(-1, 3)
    output = image.copy()
    for index, color in zip(indices, colors):
        cv2.circle(
            output,
            tuple(int(item) for item in rounded[index]),
            radius_px,
            tuple(int(item) for item in color),
            -1,
            lineType=cv2.LINE_AA,
        )
    return output, {
        "input_point_count": int(len(camera_points)),
        "front_point_count": int(np.count_nonzero(front)),
        "projected_inside_count": int(len(indices)),
        "minimum_depth_m": float(minimum_depth_m),
        "maximum_depth_m": float(maximum_depth_m),
    }


def reprojection_metrics(
    object_points_lidar: np.ndarray,
    image_points_px: np.ndarray,
    camera: CameraModel,
    transform: LidarToCameraTransform,
) -> ReprojectionMetrics:
    object_points = np.asarray(object_points_lidar, dtype=np.float64)
    observed = np.asarray(image_points_px, dtype=np.float64)
    if (
        object_points.ndim != 2
        or object_points.shape[1] != 3
        or observed.shape != (len(object_points), 2)
        or len(object_points) < 1
        or not np.isfinite(object_points).all()
        or not np.isfinite(observed).all()
    ):
        raise ValueError("finite Nx3 object and Nx2 image correspondences are required")
    projected, front = project_camera_points(
        transform_lidar_to_camera(object_points, transform), camera
    )
    if not front.all():
        raise CalibrationBlocked("one or more calibration points project behind the camera")
    errors = np.linalg.norm(projected - observed, axis=1)
    return ReprojectionMetrics(
        count=len(errors),
        mean_px=float(np.mean(errors)),
        median_px=float(np.median(errors)),
        p95_px=float(np.quantile(errors, 0.95)),
        maximum_px=float(np.max(errors)),
        errors_px=tuple(float(item) for item in errors),
    )


def estimate_lidar_to_camera_pnp(
    object_points_lidar: np.ndarray,
    image_points_px: np.ndarray,
    camera: CameraModel,
) -> tuple[LidarToCameraTransform, ReprojectionMetrics]:
    objects = np.asarray(object_points_lidar, dtype=np.float64)
    images = np.asarray(image_points_px, dtype=np.float64)
    if (
        objects.ndim != 2
        or objects.shape[1] != 3
        or images.shape != (len(objects), 2)
        or len(objects) < 6
        or not np.isfinite(objects).all()
        or not np.isfinite(images).all()
    ):
        raise CalibrationBlocked("at least six finite 3D-2D correspondences are required")
    if camera.distortion_model == "equidistant":
        undistorted = cv2.fisheye.undistortPoints(
            images.reshape(-1, 1, 2),
            camera.matrix,
            camera.distortion.reshape(-1, 1),
            P=camera.matrix,
        ).reshape(-1, 2)
        solve_distortion = np.zeros(4, dtype=np.float64)
        solve_images = undistorted
    else:
        solve_distortion = camera.distortion
        solve_images = images
    solved, rotation_vector, translation = cv2.solvePnP(
        objects,
        solve_images,
        camera.matrix,
        solve_distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not solved:
        raise CalibrationBlocked("PnP did not converge")
    rotation, _ = cv2.Rodrigues(rotation_vector)
    transform = LidarToCameraTransform(
        np.asarray(rotation, dtype=np.float64),
        np.asarray(translation, dtype=np.float64).reshape(3),
        "multi_scene_3d_2d_pnp_candidate",
    )
    metrics = reprojection_metrics(objects, images, camera, transform)
    return transform, metrics
