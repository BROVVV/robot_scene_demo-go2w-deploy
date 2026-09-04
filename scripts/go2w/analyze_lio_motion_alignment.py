#!/usr/bin/env python3
"""计划书 §8.3：只读分析 LIO 动态位姿的时间对齐、轴向和外参可信度。

输入是 §8.2 用遥控器采集的 rosbag2 目录（可选一份动作时间表 JSON）。脚本只读
bag，把结论写到 ``outputs/lio_alignment/``，绝不下发运动、绝不改标定文件。

输出：
- ``/utlidar/imu`` 与 Pandar 点云时间差的 median / p95 / max；
- IMU ``angular_velocity.{x,y,z}`` 与 ``/go2w/odom/fused`` yaw rate 的相关系数；
- 最佳时间偏移及其 95% 置信区间（Fisher-z）；
- 左/右转时角速度符号是否与 fused odom 一致；
- 每个转向段 wheel Δxy/Δyaw 与 pslam Δxy/Δyaw；
- 每段点云 scan-to-scan ICP 的旋转和平移残差（手写 ICP，本机没有 open3d）。

判定顺序按计划书 §8.3：先轴向、再固定时间偏移、再外参、最后才是匹配参数。
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

# numpy 2 renamed trapz to trapezoid; the robot still ships numpy 1.17.
_trapz = getattr(np, "trapezoid", None) or np.trapz
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
IMU_TOPIC = "/utlidar/imu"
CORRECTED_IMU_TOPIC = "/go2w/slam/imu"
CLOUD_TOPIC = "/go2w/slam/pandar_points"
RAW_CLOUD_TOPIC = "/hesai/pandarxt16/points_raw"
FUSED_TOPIC = "/go2w/odom/fused"
PSLAM_TOPIC = "/go2w/slam/odom_base"


def stamp_sec(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def storage_identifier(bag: Path) -> str:
    payload = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8")) or {}
    info = payload.get("rosbag2_bagfile_information") or payload
    return str(info.get("storage_identifier") or "sqlite3")


def read_bag(bag: Path, wanted: set[str]) -> dict[str, list]:
    """Deserialize the wanted topics once; the bag itself is never modified.

    每条消息保留 ``(message, receive_sec)``：只有把 header stamp 和接收时间放在
    一起，才能分清"传感器时钟纪元差"和"真正的采样时刻错位"。
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_identifier(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = sorted(name for name in wanted if name not in types)
    classes = {name: get_message(types[name]) for name in wanted if name in types}
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(classes)))
    collected: dict[str, list] = {name: [] for name in classes}
    while reader.has_next():
        topic, payload, receive_ns = reader.read_next()
        collected[topic].append(
            (deserialize_message(payload, classes[topic]), float(receive_ns) * 1e-9))
    collected["__missing__"] = missing
    return collected


def stamps(pairs: list) -> tuple[np.ndarray, np.ndarray]:
    header = np.array([stamp_sec(item) for item, _ in pairs], dtype=float)
    receive = np.array([value for _, value in pairs], dtype=float)
    return header, receive


def yaw_of(orientation) -> float:
    x, y, z, w = (float(orientation.x), float(orientation.y),
                  float(orientation.z), float(orientation.w))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odom_track(pairs: list) -> dict[str, np.ndarray]:
    messages = [item for item, _ in pairs]
    return {
        "t": np.array([stamp_sec(item) for item in messages], dtype=float),
        "x": np.array([item.pose.pose.position.x for item in messages], dtype=float),
        "y": np.array([item.pose.pose.position.y for item in messages], dtype=float),
        "yaw": np.array([yaw_of(item.pose.pose.orientation) for item in messages]),
        "yaw_rate": np.array([item.twist.twist.angular.z for item in messages]),
    }


def imu_track(pairs: list) -> dict[str, np.ndarray]:
    messages = [item for item, _ in pairs]
    track = {"t": np.array([stamp_sec(item) for item in messages], dtype=float),
             "receive_t": np.array([value for _, value in pairs], dtype=float)}
    for axis in ("x", "y", "z"):
        track[axis] = np.array(
            [getattr(item.angular_velocity, axis) for item in messages], dtype=float)
    return track


def yaw_rate_series(track: dict[str, np.ndarray]) -> tuple[np.ndarray, str]:
    """Prefer the published twist; fall back to differentiating yaw."""
    published = track["yaw_rate"]
    if published.size and float(np.max(np.abs(published))) > 1e-3:
        return published, "twist.angular.z"
    times, yaw = track["t"], np.unwrap(track["yaw"])
    if times.size < 3:
        return np.zeros_like(times), "unavailable"
    return np.gradient(yaw, times), "d(yaw)/dt"


def percentiles(values: np.ndarray) -> dict[str, float]:
    absolute = np.abs(values)
    return {
        "count": int(values.size),
        "median_s": float(np.median(absolute)),
        "p95_s": float(np.percentile(absolute, 95)),
        "max_s": float(np.max(absolute)),
        "signed_median_s": float(np.median(values)),
    }


def nearest_difference(reference: np.ndarray, other: np.ndarray) -> np.ndarray:
    order = np.argsort(other)
    sorted_other = other[order]
    slot = np.clip(np.searchsorted(sorted_other, reference), 1, sorted_other.size - 1)
    left, right = sorted_other[slot - 1], sorted_other[slot]
    nearest = np.where(np.abs(reference - left) <= np.abs(reference - right),
                       left, right)
    return reference - nearest


def stream_time_difference(cloud: tuple[np.ndarray, np.ndarray],
                           imu: tuple[np.ndarray, np.ndarray]) -> dict:
    """Cloud vs IMU header-stamp difference, epoch offset separated out.

    直接做最近邻会把"传感器时钟差 726 秒"和"采样时刻错位几毫秒"混成一个数：
    两个纪元不重叠时最近邻永远落在流的端点上，median/p95/max 只反映 bag 长度。
    所以先用接收时间求纪元差，再报去掉纪元差之后的残差。
    """
    cloud_header, cloud_receive = cloud
    imu_header, imu_receive = imu
    if cloud_header.size == 0 or imu_header.size == 0:
        return {"count": 0, "reason": "one of the streams is empty"}
    cloud_latency = float(np.median(cloud_receive - cloud_header))
    imu_latency = float(np.median(imu_receive - imu_header))
    shift = imu_latency - cloud_latency
    residual = nearest_difference(cloud_header, imu_header + shift)
    return {
        **percentiles(residual),
        "alignment_shift_s": round(shift, 6),
        "cloud_header_minus_receive_s": round(-cloud_latency, 6),
        "imu_header_minus_receive_s": round(-imu_latency, 6),
        "imu_clock_drift_s": round(float(
            (imu_receive[-1] - imu_header[-1]) - (imu_receive[0] - imu_header[0])), 6),
    }


def excitation(times: np.ndarray, yaw_rate: np.ndarray,
               speed: np.ndarray) -> dict:
    """静止 bag 里没有转向信号，任何"轴向错误"或"时间偏移"结论都是编的。"""
    rotation_deg = math.degrees(float(_trapz(np.abs(yaw_rate), times)))
    travel_m = float(_trapz(np.abs(speed), times))
    peak = float(np.max(np.abs(yaw_rate))) if yaw_rate.size else 0.0
    return {
        "peak_yaw_rate_rad_s": round(peak, 4),
        "total_abs_rotation_deg": round(rotation_deg, 2),
        "total_travel_m": round(travel_m, 3),
        "sufficient": bool(rotation_deg >= 30.0 and peak >= 0.15),
    }




def pearson(first: np.ndarray, second: np.ndarray) -> float:
    if first.size < 3:
        return float("nan")
    first = first - first.mean()
    second = second - second.mean()
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator <= 1e-12:
        return float("nan")
    return float(np.dot(first, second) / denominator)


def lag_scan(imu: dict, imu_axis: str, reference_t: np.ndarray,
             reference_rate: np.ndarray, span_s: float,
             step_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Correlation of one gyro axis against the reference yaw rate per lag.

    A positive lag means the IMU stamp is late: sampling the gyro at
    ``reference_t + lag`` lines the two peaks up.
    """
    lags = np.arange(-span_s, span_s + 0.5 * step_s, step_s)
    values = np.empty(lags.size, dtype=float)
    inside = (reference_t >= imu["t"][0] - span_s) & (reference_t <= imu["t"][-1] + span_s)
    base_t, base_rate = reference_t[inside], reference_rate[inside]
    for index, lag in enumerate(lags):
        sampled = np.interp(base_t + lag, imu["t"], imu[imu_axis],
                            left=np.nan, right=np.nan)
        usable = np.isfinite(sampled)
        values[index] = (pearson(sampled[usable], base_rate[usable])
                         if usable.sum() >= 3 else float("nan"))
    return lags, values


def fisher_interval(correlation: float, samples: int) -> tuple[float, float]:
    if samples <= 4 or not math.isfinite(correlation):
        return (float("nan"), float("nan"))
    # 无噪声数据会给出 |r| = 1，atanh 在那里没有定义；夹一下让区间退化成峰值本身。
    z = math.atanh(max(-0.999999, min(0.999999, correlation)))
    spread = 1.96 / math.sqrt(samples - 3)
    return (math.tanh(z - spread), math.tanh(z + spread))


def offset_confidence(lags: np.ndarray, values: np.ndarray,
                      samples: int) -> dict:
    """95% lag interval: every lag whose |r| stays inside the peak's Fisher band."""
    magnitude = np.abs(values)
    if not np.isfinite(magnitude).any():
        return {"reason": "no finite correlation"}
    peak = int(np.nanargmax(magnitude))
    low, high = fisher_interval(float(magnitude[peak]), samples)
    if not math.isfinite(low):
        return {"best_offset_s": float(lags[peak]),
                "best_correlation": float(values[peak]),
                "reason": "sample count too small for a Fisher-z band"}
    inside = np.where(np.nan_to_num(magnitude, nan=-1.0) >= low)[0]
    return {
        "best_offset_s": float(lags[peak]),
        "best_correlation": float(values[peak]),
        "correlation_ci_95": [float(low), float(high)],
        "offset_ci_95_s": [float(lags[inside.min()]), float(lags[inside.max()])],
        "ci_method": "fisher_z_band_on_peak_correlation",
        "ci_samples": int(samples),
    }


def load_timetable(path: Path, bag_start: float) -> list[dict]:
    """§8.2 动作时间表：[{"label": "左转30度", "start_s": 61.0, "end_s": 66.0}]。"""
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload["segments"] if isinstance(payload, dict) else payload
    segments = []
    for index, row in enumerate(rows, start=1):
        start, end = float(row["start_s"]), float(row["end_s"])
        if start < bag_start:  # relative times are allowed and more convenient
            start, end = start + bag_start, end + bag_start
        segments.append({"index": index, "label": str(row.get("label") or f"seg{index}"),
                         "start_s": start, "end_s": end, "source": "timetable"})
    return segments


def detect_segments(times: np.ndarray, yaw_rate: np.ndarray, speed: np.ndarray,
                    *, turn_rate_min: float = 0.08, speed_min: float = 0.05,
                    hold_s: float = 0.5) -> list[dict]:
    """Fallback when no timetable is supplied: split on sustained motion."""
    moving = (np.abs(yaw_rate) > turn_rate_min) | (speed > speed_min)
    segments: list[dict] = []
    start = None
    for index, flag in enumerate(moving):
        if flag and start is None:
            start = index
        elif not flag and start is not None:
            segments.append((start, index - 1))
            start = None
    if start is not None:
        segments.append((start, moving.size - 1))
    kept = []
    for first, last in segments:
        if times[last] - times[first] < hold_s:
            continue
        turn = float(_trapz(yaw_rate[first:last + 1], times[first:last + 1]))
        travel = float(_trapz(speed[first:last + 1], times[first:last + 1]))
        if abs(math.degrees(turn)) >= 10.0:
            label = "left_turn" if turn > 0 else "right_turn"
        elif travel >= 0.2:
            label = "straight"
        else:
            label = "small_motion"
        kept.append({"index": len(kept) + 1, "label": label,
                     "start_s": float(times[first]), "end_s": float(times[last]),
                     "source": "auto"})
    return kept


def cloud_points(message, voxel_m: float, max_points: int) -> np.ndarray:
    from sensor_msgs_py import point_cloud2

    # read_points_numpy 要求所有字段同 datatype；plain_slam schema 是
    # x/y/z/intensity FLOAT32 + timestamp FLOAT64，只能走结构化读取。
    raw = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True)
    points = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).astype(float)
    radius = np.linalg.norm(points, axis=1)
    points = points[(radius > 0.4) & (radius < 40.0)]
    if voxel_m > 0.0 and points.size:
        keys = np.floor(points / voxel_m).astype(np.int64)
        _, unique = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(unique)]
    if points.shape[0] > max_points:
        stride = points.shape[0] // max_points + 1
        points = points[::stride]
    return points


def icp_align(source: np.ndarray, target: np.ndarray, *, iterations: int = 30,
              correspondence_m: float = 0.5) -> dict:
    """Point-to-point ICP via Kabsch; open3d is not installed on this host."""
    from scipy.spatial import cKDTree

    if source.shape[0] < 50 or target.shape[0] < 50:
        return {"converged": False, "reason": "too few points"}
    tree = cKDTree(target)
    rotation = np.eye(3)
    translation = np.zeros(3)
    moved = source
    pairs = 0
    for _ in range(iterations):
        distances, indices = tree.query(moved, distance_upper_bound=correspondence_m)
        usable = np.isfinite(distances)
        pairs = int(usable.sum())
        if pairs < 50:
            break
        source_set, target_set = moved[usable], target[indices[usable]]
        source_mean, target_mean = source_set.mean(axis=0), target_set.mean(axis=0)
        covariance = (source_set - source_mean).T @ (target_set - target_mean)
        left, _, right = np.linalg.svd(covariance)
        flip = np.diag([1.0, 1.0, float(np.sign(np.linalg.det(right.T @ left.T)))])
        step_rotation = right.T @ flip @ left.T
        step_translation = target_mean - step_rotation @ source_mean
        rotation = step_rotation @ rotation
        translation = step_rotation @ translation + step_translation
        moved = moved @ step_rotation.T + step_translation
        if (float(np.linalg.norm(step_translation)) < 1e-4
                and abs(rotation_angle(step_rotation)) < 1e-5):
            break
    distances, _ = tree.query(moved, distance_upper_bound=correspondence_m)
    usable = np.isfinite(distances)
    return {
        "converged": pairs >= 50,
        "rotation_deg": math.degrees(rotation_angle(rotation)),
        "yaw_deg": math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])),
        "translation_m": float(np.linalg.norm(translation)),
        "translation_xyz_m": [round(float(value), 4) for value in translation],
        "residual_rms_m": (float(np.sqrt(np.mean(distances[usable] ** 2)))
                           if usable.any() else float("nan")),
        "inlier_ratio": float(usable.sum()) / float(max(1, moved.shape[0])),
    }


def rotation_angle(matrix: np.ndarray) -> float:
    cosine = (float(np.trace(matrix)) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, cosine)))


def pose_delta(track: dict[str, np.ndarray], start: float, end: float) -> dict:
    times = track["t"]
    inside = np.where((times >= start) & (times <= end))[0]
    if inside.size < 2:
        return {"available": False}
    first, last = int(inside[0]), int(inside[-1])
    dx = float(track["x"][last] - track["x"][first])
    dy = float(track["y"][last] - track["y"][first])
    yaw = np.unwrap(track["yaw"][first:last + 1])
    return {
        "available": True,
        "dxy_m": round(math.hypot(dx, dy), 4),
        "dx_m": round(dx, 4),
        "dy_m": round(dy, 4),
        "dyaw_deg": round(math.degrees(float(yaw[-1] - yaw[0])), 3),
        "samples": int(inside.size),
    }


def segment_icp(clouds: list, cloud_times: np.ndarray, start: float, end: float,
                *, voxel_m: float, max_points: int, max_pairs: int) -> dict:
    inside = np.where((cloud_times >= start) & (cloud_times <= end))[0]
    if inside.size < 2:
        return {"available": False, "reason": "fewer than two scans in the segment"}
    stride = max(1, int(inside.size) // max(1, max_pairs))
    chosen = list(inside[::stride])
    if chosen[-1] != inside[-1]:
        chosen.append(int(inside[-1]))
    rotation, translation = np.eye(3), np.zeros(3)
    residuals, yaws, pairs = [], [], []
    for previous, current in zip(chosen, chosen[1:]):
        target = cloud_points(clouds[previous], voxel_m, max_points)
        source = cloud_points(clouds[current], voxel_m, max_points)
        result = icp_align(source, target)
        if not result.get("converged"):
            continue
        step = np.eye(3)
        angle = math.radians(result["yaw_deg"])
        step[:2, :2] = [[math.cos(angle), -math.sin(angle)],
                        [math.sin(angle), math.cos(angle)]]
        offset = np.array(result["translation_xyz_m"], dtype=float)
        translation = rotation @ offset + translation
        rotation = rotation @ step
        residuals.append(result["residual_rms_m"])
        yaws.append(result["yaw_deg"])
        pairs.append({"t_s": round(float(cloud_times[current] - cloud_times[0]), 3),
                      "yaw_deg": round(result["yaw_deg"], 3),
                      "translation_m": round(result["translation_m"], 4),
                      "residual_rms_m": round(result["residual_rms_m"], 4),
                      "inlier_ratio": round(result["inlier_ratio"], 3)})
    if not pairs:
        return {"available": False, "reason": "ICP did not converge on any pair"}
    return {
        "available": True,
        "pair_count": len(pairs),
        "accumulated_yaw_deg": round(math.degrees(
            math.atan2(rotation[1, 0], rotation[0, 0])), 3),
        "accumulated_yaw_unwrapped_deg": round(float(sum(yaws)), 3),
        "net_translation_m": round(float(np.linalg.norm(translation)), 4),
        "residual_rms_median_m": round(float(np.median(residuals)), 4),
        "residual_rms_max_m": round(float(np.max(residuals)), 4),
        "pairs": pairs,
    }


def classify(fused: dict, label: str) -> str:
    text = label.lower()
    if "360" in text or "一周" in label or "circle" in text:
        return "full_circle"
    if not fused.get("available"):
        return "unknown"
    turn = abs(float(fused["dyaw_deg"]))
    if turn >= 300.0:
        return "full_circle"
    if 15.0 <= turn <= 50.0:
        return "turn_30"
    if turn < 30.0 and float(fused["dxy_m"]) >= 0.5:
        return "straight"
    return "other"


def gate_rows(kind: str, fused: dict, pslam: dict) -> list[dict]:
    """§8.6 动态验收门槛。不通过就是不通过，不允许放宽阈值掩盖。"""
    if not (fused.get("available") and pslam.get("available")):
        return [{"gate": "data", "passed": False,
                 "detail": "fused or pslam odometry is missing in this segment"}]
    checks: list[dict] = []

    def check(gate: str, value: float, limit: float, passed: bool, unit: str) -> None:
        checks.append({"gate": gate, "value": round(float(value), 4),
                       "limit": round(float(limit), 4), "unit": unit,
                       "passed": bool(passed)})

    yaw_gap = abs(float(pslam["dyaw_deg"]) - float(fused["dyaw_deg"]))
    same_sign = (float(pslam["dyaw_deg"]) * float(fused["dyaw_deg"])) > 0.0
    if kind == "turn_30":
        check("fused_dxy", fused["dxy_m"], 0.10, fused["dxy_m"] <= 0.10, "m")
        check("pslam_dxy", pslam["dxy_m"], 0.20, pslam["dxy_m"] <= 0.20, "m")
        checks.append({"gate": "yaw_sign_agrees", "passed": same_sign,
                       "detail": f"fused={fused['dyaw_deg']}° pslam={pslam['dyaw_deg']}°"})
        check("yaw_difference", yaw_gap, 5.0, yaw_gap <= 5.0, "deg")
    elif kind == "full_circle":
        check("pslam_position_closure", pslam["dxy_m"], 0.30,
              pslam["dxy_m"] <= 0.30, "m")
        closure = abs((float(pslam["dyaw_deg"]) + 180.0) % 360.0 - 180.0)
        check("pslam_yaw_closure", closure, 10.0, closure <= 10.0, "deg")
    elif kind == "straight":
        cosine = ((float(fused["dx_m"]) * float(pslam["dx_m"])
                   + float(fused["dy_m"]) * float(pslam["dy_m"]))
                  / max(1e-6, float(fused["dxy_m"]) * float(pslam["dxy_m"])))
        checks.append({"gate": "direction_agrees", "passed": cosine >= 0.9,
                       "value": round(float(cosine), 4), "limit": 0.9})
        error = abs(float(pslam["dxy_m"]) - float(fused["dxy_m"]))
        limit = max(0.20, 0.20 * float(fused["dxy_m"]))
        check("length_error", error, limit, error <= limit, "m")
        check("end_yaw_drift", abs(float(pslam["dyaw_deg"])), 5.0,
              abs(float(pslam["dyaw_deg"])) <= 5.0, "deg")
    return checks


def gyro_summary(imu: dict, axis: str, start: float, end: float,
                 offset_s: float) -> dict:
    inside = np.where((imu["t"] >= start + offset_s) & (imu["t"] <= end + offset_s))[0]
    if inside.size < 2:
        return {"available": False}
    values = imu[axis][inside]
    integrated = float(_trapz(values, imu["t"][inside]))
    return {"available": True, "axis": axis,
            "mean_rad_s": round(float(values.mean()), 5),
            "peak_rad_s": round(float(values[np.argmax(np.abs(values))]), 5),
            "integrated_deg": round(math.degrees(integrated), 3)}


def axis_report(imu: dict, reference_t: np.ndarray, reference_rate: np.ndarray,
                span_s: float, step_s: float) -> dict:
    report: dict = {}
    for axis in ("x", "y", "z"):
        lags, values = lag_scan(imu, axis, reference_t, reference_rate, span_s, step_s)
        aligned = np.interp(reference_t, imu["t"], imu[axis], left=np.nan, right=np.nan)
        usable = np.isfinite(aligned)
        report[axis] = {
            "correlation_at_zero_lag": round(pearson(
                aligned[usable], reference_rate[usable]), 4),
            **offset_confidence(lags, values, int(usable.sum())),
        }
    best = max(report, key=lambda name: abs(report[name].get("best_correlation") or 0.0))
    report["dominant_axis"] = best
    report["expected_axis"] = "z"
    report["axis_ok"] = bool(best == "z"
                             and (report[best].get("best_correlation") or 0.0) > 0.0)
    return report


def verdict(report: dict) -> dict:
    """计划书 §8.3 的判定顺序：轴向 → 固定时间偏移 → 外参 → 匹配参数。"""
    axes = report["angular_velocity_vs_fused_yaw_rate"]
    axis = axes[axes["dominant_axis"]]
    offset = float(axis.get("best_offset_s") or 0.0)
    failed = [row for segment in report["segments"] for row in segment["gates"]
              if not row["passed"]]
    if not report["excitation"]["sufficient"]:
        step = (f"这段数据没有足够激励（累计转动 "
                f"{report['excitation']['total_abs_rotation_deg']}°，峰值 "
                f"{report['excitation']['peak_yaw_rate_rad_s']}rad/s）："
                "只能确认时钟纪元和消息率，轴向、时间偏移和 §8.6 门槛都无法判定，"
                "必须按 §8.2 用遥控器采一段动态 bag")
    elif not axes["axis_ok"]:
        step = ("先修 IMU 轴向/符号：主相关轴是 "
                f"{axes['dominant_axis']}，相关系数 {axis.get('best_correlation')}")
    elif abs(offset) > 0.02:
        step = f"先修固定时间偏移：最佳偏移 {offset:+.3f}s 超过 20ms"
    elif failed:
        step = f"时间和轴向可用，接下来求 Pandar↔IMU 外参；{len(failed)} 项 §8.6 门槛未过"
    else:
        step = "时间、轴向、§8.6 门槛全部通过；只剩 LIO 匹配参数微调"
    return {"next_action": step, "failed_gate_count": len(failed),
            "excitation_sufficient": report["excitation"]["sufficient"],
            "extrinsic_upgrade_allowed": bool(report["excitation"]["sufficient"]
                                              and axes["axis_ok"]
                                              and abs(offset) <= 0.02
                                              and not failed)}



def print_report(report: dict) -> None:
    print("== §8.3 LIO 动态对齐分析 ==")
    print(f"bag: {report['bag']}")
    print(f"时长: {report['duration_s']:.1f}s  消息: {report['message_counts']}")
    for title, key in (("原始 IMU", "imu_vs_cloud_stamp_difference"),
                       ("适配器输出 IMU", "corrected_imu_vs_cloud_stamp_difference")):
        block = report[key]
        print(f"\n-- {title} vs 点云时间差 ({block.get('count', 0)} 帧) --")
        if not block.get("count"):
            print(f"  {block.get('reason', 'no data')}")
            continue
        print(f"  IMU 时钟纪元      header-receive = "
              f"{block['imu_header_minus_receive_s']:+.6f}s "
              f"(全程漂移 {block['imu_clock_drift_s']:+.6f}s)")
        print(f"  点云 stamp 语义   header-receive = "
              f"{block['cloud_header_minus_receive_s']:+.6f}s")
        print(f"  对齐平移量        {block['alignment_shift_s']:+.6f}s")
        print(f"  平移后最近邻残差  median={block['median_s']:.6f}s "
              f"p95={block['p95_s']:.6f}s max={block['max_s']:.6f}s")
    excite = report["excitation"]
    print(f"\n-- 激励 --\n  累计转动={excite['total_abs_rotation_deg']}° "
          f"峰值角速度={excite['peak_yaw_rate_rad_s']}rad/s "
          f"行走={excite['total_travel_m']}m 足够={excite['sufficient']}")
    axes = report["angular_velocity_vs_fused_yaw_rate"]
    print(f"\n-- 角速度 vs fused yaw rate (参考量: {report['fused_yaw_rate_source']}, "
          f"IMU: {report['imu_topic_used_for_correlation']}) --")
    for axis in ("x", "y", "z"):
        row = axes[axis]
        interval = row.get("offset_ci_95_s") or [float("nan")] * 2
        print(f"  {axis}: r0={row['correlation_at_zero_lag']:+.4f} "
              f"peak_r={row.get('best_correlation', float('nan')):+.4f} "
              f"offset={row.get('best_offset_s', float('nan')):+.3f}s "
              f"CI95=[{interval[0]:+.3f},{interval[1]:+.3f}]s")
    print(f"  主轴={axes['dominant_axis']} 期望=z 轴向判定="
          f"{'OK' if axes['axis_ok'] else ('无法判定(无激励)' if not excite['sufficient'] else 'FAIL')}")
    print("\n-- 分段对比 (fused / pslam / scan-to-scan ICP) --")
    for segment in report["segments"]:
        fused, pslam, icp = segment["fused"], segment["pslam"], segment["icp"]
        print(f"  [{segment['index']}] {segment['label']} ({segment['kind']}) "
              f"{segment['start_rel_s']:.1f}s→{segment['end_rel_s']:.1f}s")
        if fused.get("available"):
            print(f"      fused  Δxy={fused['dxy_m']:.3f}m Δyaw={fused['dyaw_deg']:+.2f}°")
        if pslam.get("available"):
            print(f"      pslam  Δxy={pslam['dxy_m']:.3f}m Δyaw={pslam['dyaw_deg']:+.2f}°")
        gyro = segment["gyro"]
        if gyro.get("available"):
            print(f"      gyro   ∫{gyro['axis']}={gyro['integrated_deg']:+.2f}° "
                  f"peak={gyro['peak_rad_s']:+.3f}rad/s 符号一致="
                  f"{'YES' if segment['sign_agrees'] else 'NO'}")
        if icp.get("available"):
            print(f"      ICP    Δyaw={icp['accumulated_yaw_deg']:+.2f}° "
                  f"Δxy={icp['net_translation_m']:.3f}m "
                  f"残差 median={icp['residual_rms_median_m']:.3f}m "
                  f"max={icp['residual_rms_max_m']:.3f}m ({icp['pair_count']} 对)")
        else:
            print(f"      ICP    不可用: {icp.get('reason')}")
        for row in segment["gates"]:
            mark = "PASS" if row["passed"] else "FAIL"
            print(f"      §8.6 {row['gate']:<24} {mark} "
                  f"{row.get('value', '')} <= {row.get('limit', '')} "
                  f"{row.get('detail', '')}".rstrip())
    print(f"\n-- 判定 --\n  {report['verdict']['next_action']}")
    print(f"  未过门槛: {report['verdict']['failed_gate_count']}  "
          f"允许升级外参: {report['verdict']['extrinsic_upgrade_allowed']}")
    print(f"\n报告: {report['report_path']}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory recorded per §8.2")
    parser.add_argument("--timetable", type=Path, default=None,
                        help="§8.2 action timetable JSON; segments are auto-detected "
                             "when it is absent")
    parser.add_argument("--cloud-topic", default=CLOUD_TOPIC)
    parser.add_argument("--lag-span-s", type=float, default=0.5)
    parser.add_argument("--lag-step-s", type=float, default=0.005)
    parser.add_argument("--icp-voxel-m", type=float, default=0.15)
    parser.add_argument("--icp-max-points", type=int, default=6000)
    parser.add_argument("--icp-max-pairs", type=int, default=10)
    parser.add_argument("--output-dir", type=Path,
                        default=PROJECT_ROOT / "outputs/lio_alignment")
    arguments = parser.parse_args()

    topics = {IMU_TOPIC, CORRECTED_IMU_TOPIC, arguments.cloud_topic, RAW_CLOUD_TOPIC,
              FUSED_TOPIC, PSLAM_TOPIC}
    data = read_bag(arguments.bag, topics)
    missing = data.pop("__missing__")
    cloud_topic = (arguments.cloud_topic if data.get(arguments.cloud_topic)
                   else RAW_CLOUD_TOPIC)
    clouds = [item for item, _ in data.get(cloud_topic, [])]
    cloud_stamps = stamps(data.get(cloud_topic, []))
    cloud_times = cloud_stamps[0]
    raw_imu = imu_track(data.get(IMU_TOPIC, []))
    corrected_imu = imu_track(data.get(CORRECTED_IMU_TOPIC, []))
    fused = odom_track(data.get(FUSED_TOPIC, []))
    pslam = odom_track(data.get(PSLAM_TOPIC, []))
    if raw_imu["t"].size < 10 or fused["t"].size < 10:
        print(f"bag 缺少必要数据: imu={raw_imu['t'].size} fused={fused['t'].size} "
              f"missing_topics={missing}")
        return 2

    # 相关分析必须在同一时钟上做：优先用适配器输出，否则把原始流按纪元差平移。
    if corrected_imu["t"].size >= 10:
        imu, imu_source = corrected_imu, CORRECTED_IMU_TOPIC
    else:
        imu, imu_source = dict(raw_imu), f"{IMU_TOPIC} (shifted by the epoch offset)"
        imu["t"] = raw_imu["t"] + float(np.median(
            raw_imu["receive_t"] - raw_imu["t"]))

    fused_rate, rate_source = yaw_rate_series(fused)
    bag_start = float(min(fused["t"][0], imu["t"][0]))
    bag_end = float(max(fused["t"][-1], imu["t"][-1]))
    speed = np.hypot(np.gradient(fused["x"], fused["t"]),
                     np.gradient(fused["y"], fused["t"]))
    segments = (load_timetable(arguments.timetable, bag_start)
                if arguments.timetable
                else detect_segments(fused["t"], fused_rate, speed))

    report = {
        "bag": str(arguments.bag),
        "cloud_topic": cloud_topic,
        "imu_topic_used_for_correlation": imu_source,
        "missing_topics": missing,
        "duration_s": float(bag_end - bag_start),
        "message_counts": {name: len(values) for name, values in data.items()},
        "fused_yaw_rate_source": rate_source,
        "excitation": excitation(fused["t"], fused_rate, speed),
        "imu_vs_cloud_stamp_difference": stream_time_difference(
            cloud_stamps, stamps(data.get(IMU_TOPIC, []))),
        "corrected_imu_vs_cloud_stamp_difference": stream_time_difference(
            cloud_stamps, stamps(data.get(CORRECTED_IMU_TOPIC, []))),
        "angular_velocity_vs_fused_yaw_rate": axis_report(
            imu, fused["t"], fused_rate, arguments.lag_span_s, arguments.lag_step_s),
        "segment_source": segments[0]["source"] if segments else "none",
        "segments": [],
    }

    axis = report["angular_velocity_vs_fused_yaw_rate"]["dominant_axis"]
    offset_s = float(report["angular_velocity_vs_fused_yaw_rate"][axis]
                     .get("best_offset_s") or 0.0)

    for segment in segments:
        fused_delta = pose_delta(fused, segment["start_s"], segment["end_s"])
        pslam_delta = pose_delta(pslam, segment["start_s"], segment["end_s"])
        gyro = gyro_summary(imu, axis, segment["start_s"], segment["end_s"], offset_s)
        kind = classify(fused_delta, segment["label"])
        sign_agrees = bool(gyro.get("available") and fused_delta.get("available")
                           and float(gyro["integrated_deg"])
                           * float(fused_delta["dyaw_deg"]) > 0.0)
        report["segments"].append({
            **segment,
            "start_rel_s": round(segment["start_s"] - bag_start, 3),
            "end_rel_s": round(segment["end_s"] - bag_start, 3),
            "kind": kind,
            "fused": fused_delta,
            "pslam": pslam_delta,
            "gyro": gyro,
            "sign_agrees": sign_agrees,
            "icp": segment_icp(clouds, cloud_times, segment["start_s"],
                               segment["end_s"], voxel_m=arguments.icp_voxel_m,
                               max_points=arguments.icp_max_points,
                               max_pairs=arguments.icp_max_pairs),
            "gates": gate_rows(kind, fused_delta, pslam_delta),
        })

    report["verdict"] = verdict(report)
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    destination = arguments.output_dir / f"{arguments.bag.name}_alignment.json"
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                           encoding="utf-8")
    report["report_path"] = str(destination)
    print_report(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())










