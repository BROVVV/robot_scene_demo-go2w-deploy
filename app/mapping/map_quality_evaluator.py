"""三维地图与语义标注质量评估器（20260908 计划书 §17 / §19–§24 / §30）。

离线、可复现、机器可读。只依赖 numpy + scipy.spatial.cKDTree，不引入 Open3D。
输入统一用 ``.npy`` / ``.npz``（录制脚本落盘格式）：

* ``points``：``(N, 3) float`` 点云
* ``trajectory``：``(M, 3) float``，列为 ``x, y, yaw``
* ``objects``：JSON ``{"label": [[x, y, z], ...], ...}`` 同一实例的多帧观测

输出 ``metrics.json``，字段与计划书 §24 一致。
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree


def load_points(path: str) -> np.ndarray:
    """读取 ``.npy`` / ``.npz`` 点云，返回 (N, 3)。"""
    if path.endswith(".npz"):
        with np.load(path) as data:
            key = "points" if "points" in data else list(data.keys())[0]
            cloud = np.asarray(data[key], dtype=float)
    else:
        cloud = np.asarray(np.load(path), dtype=float)
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        raise ValueError(f"point cloud must be (N,3): {cloud.shape}")
    return np.ascontiguousarray(cloud[:, :3])


def nn_overlap_metrics(source: np.ndarray, target: np.ndarray,
                       max_dist_m: float = 0.10) -> Dict[str, float]:
    """§19.1：source 到 target 的最近邻重合度。"""
    if len(source) == 0 or len(target) == 0:
        return {"overlap_rmse_m": float("nan"), "overlap_p95_m": float("nan"),
                "overlap_inlier_ratio": 0.0, "overlap_points": 0}
    dist, _ = cKDTree(target).query(source, k=1)
    dist = np.asarray(dist, dtype=float)
    return {
        "overlap_rmse_m": float(math.sqrt(float(np.mean(dist ** 2)))),
        "overlap_p95_m": float(np.percentile(dist, 95)),
        "overlap_inlier_ratio": float(np.mean(dist <= max_dist_m)),
        "overlap_points": int(len(dist)),
    }


def surface_thickness_p95(cloud: np.ndarray, k: int = 30) -> float:
    """§19.2：局部平面拟合后的双侧厚度 P95（米）。

    对每个点取其 k 近邻做主成分分析，最小特征向量即局部法向，
    厚度 = |(p - centroid) · n|。两层墙会让该指标显著变大。
    """
    if len(cloud) < k + 1:
        return float("nan")
    tree = cKDTree(cloud)
    _, idx = tree.query(cloud, k=k)
    thickness = np.empty(len(cloud), dtype=float)
    for i in range(len(cloud)):
        neighbours = cloud[idx[i]]
        centred = neighbours - neighbours.mean(axis=0)
        _, vectors = np.linalg.eigh(np.cov(centred.T))
        normal = vectors[:, 0]
        thickness[i] = abs(float(np.dot(cloud[i] - neighbours.mean(axis=0), normal)))
    return float(np.percentile(thickness, 95))


def endpoint_error(trajectory: np.ndarray) -> Dict[str, float]:
    """§20.2：起终点位姿误差（x, y, yaw）。"""
    if len(trajectory) < 2:
        return {"endpoint_translation_error_m": float("nan"),
                "endpoint_yaw_error_deg": float("nan")}
    start, end = trajectory[0], trajectory[-1]
    dyaw = math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2]))
    return {
        "endpoint_translation_error_m": float(math.hypot(end[0] - start[0],
                                                         end[1] - start[1])),
        "endpoint_yaw_error_deg": float(math.degrees(abs(dyaw))),
    }


def object_spread(observations: Sequence[Sequence[float]]) -> Dict[str, float]:
    """§21.1：同一实例多次观测的位置稳定性。"""
    points = np.asarray(observations, dtype=float)
    if points.ndim != 2 or len(points) == 0:
        return {"observations": 0, "centroid": [], "centroid_rms_error_m": float("nan"),
                "centroid_p95_error_m": float("nan")}
    centroid = points.mean(axis=0)
    dist = np.linalg.norm(points - centroid, axis=1)
    return {
        "observations": int(len(points)),
        "centroid": [float(v) for v in centroid],
        "centroid_rms_error_m": float(math.sqrt(float(np.mean(dist ** 2)))),
        "centroid_p95_error_m": float(np.percentile(dist, 95)),
    }


def duplicate_metrics(predicted: Dict[str, int],
                      expected: Dict[str, int]) -> Dict[str, Any]:
    """§22：按 label 比较实例数，给出 duplicate_rate / false_split / false_merge。"""
    duplicate = 0.0
    false_split = 0
    false_merge = 0
    per_label: Dict[str, Dict[str, int]] = {}
    for label, want in expected.items():
        got = int(predicted.get(label, 0))
        per_label[label] = {"expected": int(want), "predicted": got}
        if want > 0:
            duplicate += max(0, got - int(want)) / float(want)
        if got > int(want):
            false_split += got - int(want)
        elif got < int(want):
            false_merge += int(want) - got
    for label, got in predicted.items():
        if label not in expected:
            per_label[label] = {"expected": 0, "predicted": int(got)}
            false_split += int(got)
    labels = max(1, len(per_label))
    return {
        "duplicate_rate": float(duplicate / labels),
        "false_split": int(false_split),
        "false_merge": int(false_merge),
        "per_label": per_label,
    }


def load_objects(path: str) -> Dict[str, List[List[float]]]:
    with open(path, "r", encoding="utf-8") as handle:
        raw = json.load(handle)
    return {str(label): [[float(v) for v in obs] for obs in items]
            for label, items in raw.items()}


def build_report(*, map_path: str | None, trajectory_path: str | None,
                 objects_path: str | None, expected_instances: Dict[str, int] | None,
                 overlap_max_dist_m: float) -> Dict[str, Any]:
    report: Dict[str, Any] = {"schema_version": "slam_quality_report_v1"}

    if map_path:
        cloud = load_points(map_path)
        report["map"] = {
            "point_count": int(len(cloud)),
            "extent_m": [float(v) for v in (cloud.max(axis=0) - cloud.min(axis=0))],
            "surface_thickness_p95_m": surface_thickness_p95(cloud),
        }
    if trajectory_path:
        trajectory = np.asarray(np.load(trajectory_path), dtype=float)
        report["trajectory"] = dict(endpoint_error(trajectory),
                                    samples=int(len(trajectory)))
    if objects_path:
        objects = load_objects(objects_path)
        per_object = {label: object_spread(obs) for label, obs in objects.items()}
        report["semantic"] = {
            "objects": per_object,
            "object_position_rms_m": float(np.nanmax(
                [v["centroid_rms_error_m"] for v in per_object.values()] or [float("nan")])),
            "object_position_p95_m": float(np.nanmax(
                [v["centroid_p95_error_m"] for v in per_object.values()] or [float("nan")])),
            "predicted_instances": {label: 1 for label in objects},
        }
        if expected_instances:
            report["dedup"] = duplicate_metrics(
                {label: 1 for label in objects}, expected_instances)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", dest="map_path", default=None,
                        help="落盘点云 .npy/.npz，键 points")
    parser.add_argument("--trajectory", dest="trajectory_path", default=None,
                        help="轨迹 .npy，列 x,y,yaw")
    parser.add_argument("--objects", dest="objects_path", default=None,
                        help="语义观测 JSON：{label: [[x,y,z], ...]}")
    parser.add_argument("--expected-instances", default="",
                        help="JSON 字符串，如 '{\"chair\": 2}'")
    parser.add_argument("--overlap-max-dist-m", type=float, default=0.10)
    parser.add_argument("--output", required=True, help="metrics.json 输出路径")
    args = parser.parse_args()

    expected = json.loads(args.expected_instances) if args.expected_instances else None
    report = build_report(
        map_path=args.map_path,
        trajectory_path=args.trajectory_path,
        objects_path=args.objects_path,
        expected_instances=expected,
        overlap_max_dist_m=args.overlap_max_dist_m,
    )
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
