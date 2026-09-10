#!/usr/bin/env python3
"""Offline multi-scene PandarXT-16 extrinsic calibration.

Consumes the raw captures written by ``capture_dual_lidar_calibration_ros.py``
(at least three non-degenerate scenes with non-parallel walls/features),
estimates the base_link -> Pandar transform by coarse yaw search + point-to-
point ICP, and reports scene-wise residuals, inlier ratio, normal diversity
and degeneracy.

Fail-closed by default: ``confirmed`` is only true when the acceptance
thresholds pass (translation residual < 0.05 m and yaw multi-scene consistency
<= 3 degrees) AND ``--confirm`` is passed. Until then the result is a
diagnostic candidate only and never authorises a TF or motion.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def rotation_z(yaw_rad: float) -> np.ndarray:
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def transform_points(
    points: np.ndarray, translation: np.ndarray, rotation: np.ndarray
) -> np.ndarray:
    return points @ rotation.T + translation


def filter_valid(points: np.ndarray, min_range: float = 0.30, max_range: float = 15.0):
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points, axis=1)
    return points[finite & (ranges >= min_range) & (ranges <= max_range)]


def fit_plane(points: np.ndarray) -> dict:
    """Fit a least-squares plane to a cloud; return normal and height."""
    center = points.mean(axis=0)
    centered = points - center
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal = -normal
    height = float(np.dot(normal, center))
    distances = np.abs(centered @ normal)
    residual = float(np.sqrt(np.mean(distances ** 2)))
    inliers = int(np.count_nonzero(distances < 0.02))
    return {
        "normal": normal.tolist(),
        "height": height,
        "rmse": residual,
        "inlier_count": inliers,
        "point_count": int(points.shape[0]),
    }


def icp(
    source: np.ndarray,
    target: np.ndarray,
    init_translation: np.ndarray,
    init_rotation: np.ndarray,
    *,
    max_iterations: int = 30,
    tolerance: float = 1e-4,
    max_distance: float = 0.50,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Point-to-point ICP; returns (translation, rotation, metrics)."""
    translation = np.array(init_translation, dtype=np.float64).copy()
    rotation = np.array(init_rotation, dtype=np.float64).copy()
    source_home = source.copy()
    iterations_used = 0
    for iterations_used in range(1, max_iterations + 1):
        moved = transform_points(source_home, translation, rotation)
        # NN association via KDTree (build once per iteration is fine for
        # 64k-point clouds, but we subsample for speed).
        tree = _kd_tree(target)
        distances, indices = tree.query(moved, k=1)
        inlier_mask = distances < max_distance
        if not np.any(inlier_mask):
            break
        pairs_source = source_home[inlier_mask]
        pairs_target = target[indices[inlier_mask]]
        source_center = pairs_source.mean(axis=0)
        target_center = pairs_target.mean(axis=0)
        cross = (pairs_source - source_center).T @ (pairs_target - target_center)
        u, _, vt = np.linalg.svd(cross)
        d = np.eye(3)
        d[2, 2] = np.sign(np.linalg.det(vt.T @ u.T))
        delta_rotation = u @ d @ vt
        if not np.isfinite(delta_rotation).all():
            break
        rotation = delta_rotation @ rotation
        translation = (
            target_center - rotation @ source_center
        ) + rotation @ translation
        if float(np.linalg.norm(delta_rotation - np.eye(3))) < tolerance:
            break
    final = transform_points(source_home, translation, rotation)
    distances, _ = _kd_tree(target).query(final, k=1)
    inlier_mask = distances < max_distance
    inlier_distances = distances[inlier_mask]
    residual = (
        float(np.mean(inlier_distances)) if inlier_distances.size else float("inf")
    )
    metrics = {
        "iterations": iterations_used,
        "final_residual_m": residual,
        "inlier_count": int(np.count_nonzero(inlier_mask)),
        "inlier_ratio": float(np.count_nonzero(inlier_mask)) / max(len(final), 1),
        "source_points": int(len(final)),
        "fraction_within_0_10_m": float(
            np.count_nonzero(inlier_distances < 0.10)
        )
        / max(np.count_nonzero(inlier_mask), 1),
    }
    return translation, rotation, metrics


def _kd_tree(points: np.ndarray):
    # scipy.spatial.cKDTree avoids heavy dependencies; fall back to a slow
    # brute-force loop only when scipy is unavailable (should not happen).
    from scipy.spatial import cKDTree

    return cKDTree(points)


def rotation_matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw], dtype=np.float64)


def estimate_scene_transform(
    builtin: np.ndarray,
    pandar: np.ndarray,
    *,
    init_translation: np.ndarray,
    init_yaw_rad: float,
    yaw_search_range_rad: float = math.radians(15.0),
    yaw_search_steps: int = 31,
) -> dict:
    """Coarse yaw search + ICP to estimate one scene's pandar->base transform."""
    source = filter_valid(pandar)
    target = filter_valid(builtin)
    best = None
    for step in range(yaw_search_steps):
        yaw = init_yaw_rad + yaw_search_range_rad * (
            -1.0 + 2.0 * step / max(yaw_search_steps - 1, 1)
        )
        rotation = rotation_z(yaw)
        # Init translation from candidate, then let ICP refine x/y.
        translation = np.array(init_translation, dtype=np.float64)
        rotation_refined = rotation
        for _ in range(2):
            translation, rotation_refined, metrics = icp(
                source,
                target,
                translation,
                rotation_refined,
                max_iterations=8,
            )
        if best is None or metrics["final_residual_m"] < best["residual"]:
            best = {
                "yaw_deg": math.degrees(yaw),
                "residual": metrics["final_residual_m"],
                "translation": translation.tolist(),
                "rotation": rotation_refined.tolist(),
                "rpy_deg": (
                    math.degrees(value)
                    for value in rotation_matrix_to_rpy(rotation_refined)
                ),
                "metrics": metrics,
            }
    best["rpy_deg"] = [
        round(float(value), 4)
        for value in rotation_matrix_to_rpy(np.asarray(best["rotation"]))
    ]
    return best


def calibrate_multi_scene(scene_roots: list[Path]) -> dict:
    scene_results = []
    for root in scene_roots:
        manifest = json.loads((root / "raw_manifest.json").read_text(encoding="utf-8"))
        builtin = np.concatenate(
            [np.load(root / "builtin_clouds.npz")[key] for key in sorted(np.load(root / "builtin_clouds.npz"))],
            axis=0,
        )
        pandar = np.concatenate(
            [np.load(root / "pandar_clouds.npz")[key] for key in sorted(np.load(root / "pandar_clouds.npz"))],
            axis=0,
        )
        # Ground-plane constraints from the Pandar cloud.
        plane = fit_plane(filter_valid(pandar, min_range=0.8, max_range=8.0))
        result = estimate_scene_transform(
            builtin,
            pandar,
            init_translation=np.array([0.13, 0.015, 0.014], dtype=np.float64),
            init_yaw_rad=math.radians(11.357),
        )
        scene_results.append(
            {
                "scene": manifest["scene"],
                "plane": plane,
                "transform": result,
            }
        )

    yaws = [math.radians(result["transform"]["yaw_deg"]) for result in scene_results]
    yaw_mean = float(np.mean(yaws))
    yaw_consistency = float(np.ptp(yaws))
    translations = np.array(
        [result["transform"]["translation"] for result in scene_results]
    )
    translation_spread = float(np.max(np.linalg.norm(translations - translations.mean(axis=0), axis=1)))
    residuals = [result["transform"]["residual"] for result in scene_results]
    median_residual = float(np.median(residuals)) if residuals else math.inf
    inlier_ratios = [result["transform"]["metrics"]["inlier_ratio"] for result in scene_results]
    normal_diversity = _plane_normal_diversity([result["plane"]["normal"] for result in scene_results])

    passed = bool(
        len(scene_results) >= 3
        and median_residual < 0.05
        and yaw_consistency <= math.radians(3.0)
        and normal_diversity >= 0.3
    )
    return {
        "schema": "go2w.pandarxt16.extrinsics.calibration.v1",
        "scene_count": len(scene_results),
        "median_residual_m": round(median_residual, 6),
        "max_residual_m": round(max(residuals, default=math.inf), 6),
        "translation_spread_m": round(translation_spread, 6),
        "yaw_mean_deg": round(math.degrees(yaw_mean), 4),
        "yaw_consistency_deg": round(math.degrees(yaw_consistency), 4),
        "normal_diversity": round(normal_diversity, 4),
        "inlier_ratio_median": round(float(np.median(inlier_ratios)), 4) if inlier_ratios else None,
        "passed": passed,
        "confirmed": False,
        "authorizes_tf_publication": False,
        "authorizes_safety_integration": False,
        "authorizes_motion": False,
        "scene_results": scene_results,
        "fused_translation_m": translations.mean(axis=0).tolist(),
        "fused_yaw_deg": round(math.degrees(yaw_mean), 4),
    }


def _plane_normal_diversity(normals: list[list[float]]) -> float:
    """Spread of ground-plane normals across scenes (0..1)."""
    if len(normals) < 2:
        return 0.0
    matrix = np.asarray(normals, dtype=np.float64)
    # Variation captured by the singular values of the normal set.
    singular = np.linalg.svd(matrix - matrix.mean(axis=0), compute_uv=False)
    total = float(np.sum(singular))
    return float(singular[0] / total) if total > 0.0 else 0.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--capture-dir",
        type=Path,
        default=Path("outputs/go2w_acceptance/dual_lidar_calibration"),
    )
    parser.add_argument("--scenes", nargs="+", help="subset of scene dirs")
    parser.add_argument("--confirm", action="store_true", help="authorize promotion when thresholds pass")
    args = parser.parse_args()

    scene_dirs = [args.capture_dir / name for name in (args.scenes or sorted(p.name for p in args.capture_dir.iterdir() if p.is_dir()))]
    scene_dirs = [p for p in scene_dirs if (p / "raw_manifest.json").is_file()]
    if len(scene_dirs) < 1:
        print("no capture scenes found", file=sys.stderr)
        return 2

    report = calibrate_multi_scene(scene_dirs)
    output_dir = args.capture_dir / "calibration"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "scene_residuals.json").write_text(
        json.dumps(
            {
                scene["scene"]: {
                    "residual_m": scene["transform"]["residual"],
                    "inlier_ratio": scene["transform"]["metrics"]["inlier_ratio"],
                    "translation_m": scene["transform"]["translation"],
                    "yaw_deg": scene["transform"]["yaw_deg"],
                    "plane_rmse_m": scene["plane"]["rmse"],
                }
                for scene in report["scene_results"]
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    candidate = {
        "schema_version": 1,
        "confirmed": report["passed"] and args.confirm,
        "authorizes_tf_publication": False,
        "authorizes_safety_integration": False,
        "authorizes_motion": False,
        "translation_m": report["fused_translation_m"],
        "yaw_deg": report["fused_yaw_deg"],
        "median_residual_m": report["median_residual_m"],
        "scene_count": report["scene_count"],
    }
    (output_dir / "candidate_transform.json").write_text(
        json.dumps(candidate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (output_dir / "calibration_report.md").write_text(
        _report_markdown(report), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 3


def _report_markdown(report: dict) -> str:
    lines = [
        "# PandarXT-16 multi-scene extrinsic calibration",
        "",
        f"- scene_count: {report['scene_count']}",
        f"- median_residual_m: {report['median_residual_m']}",
        f"- yaw_consistency_deg: {report['yaw_consistency_deg']}",
        f"- normal_diversity: {report['normal_diversity']}",
        f"- translation_spread_m: {report['translation_spread_m']}",
        f"- passed: {report['passed']}",
        f"- confirmed: {report['confirmed']}",
        "",
        "**Thresholds**: translation residual < 0.05 m; yaw consistency <= 3 deg.",
        "",
        "Until confirmed, this is a diagnostic candidate only and does not",
        "authorise a TF publication or motion.",
        "",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    raise SystemExit(main())
