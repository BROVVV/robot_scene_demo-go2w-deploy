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
import os
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


def surface_thickness_p95(cloud: np.ndarray, radius_m: float = 0.20,
                          min_neighbours: int = 6, max_points: int = 4000,
                          seed: int = 0) -> float:
    """§19.2：局部平面拟合后的双侧厚度 P95（米）。

    邻域用**固定半径**而不是固定 k 近邻。固定 k 会让稀疏地图的邻域跨过墙角/地面交界，
    把"点稀疏"误判成"墙变厚"——2026-09-08 实测：同一套系统 Dataset A（14486 点）→ 0.096 m，
    Dataset B（3123 点）→ 0.186 m，而 B 在 k=6/10/20/30/60 下厚度单调 0.052/0.077/0.134/0.186/0.310，
    证明是邻域尺度问题而不是地图变差。固定半径保证不同点密度下空间尺度一致。
    点云大于 ``max_points`` 时随机子采样以控制耗时。
    """
    if len(cloud) < min_neighbours + 1:
        return float("nan")
    if len(cloud) > max_points:
        rng = np.random.default_rng(seed)
        sample = cloud[rng.choice(len(cloud), size=max_points, replace=False)]
    else:
        sample = cloud
    tree = cKDTree(cloud)
    thickness: List[float] = []
    for index, neighbours_idx in enumerate(
            tree.query_ball_point(sample, r=radius_m)):
        if len(neighbours_idx) < min_neighbours:
            continue
        neighbours = cloud[neighbours_idx]
        centroid = neighbours.mean(axis=0)
        centred = neighbours - centroid
        covariance = centred.T @ centred / float(len(neighbours))
        _, vectors = np.linalg.eigh(covariance)
        thickness.append(abs(float(np.dot(sample[index] - centroid, vectors[:, 0]))))
    if not thickness:
        return float("nan")
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


def _pointcloud2_xyz(message: Any) -> np.ndarray:
    """PointCloud2 -> (N, 3) float，只解析 x/y/z（Pandar 为 FLOAT32）。"""
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=float)
    point_step = int(message.point_step)
    count = int(message.width) * int(message.height)
    if point_step <= 0 or count <= 0:
        return np.empty((0, 3), dtype=float)
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(count, point_step)
    columns = []
    for name in ("x", "y", "z"):
        offset = int(fields[name].offset)
        columns.append(raw[:, offset:offset + 4].copy().view(np.float32).ravel())
    return np.stack(columns, axis=1).astype(float)


def _yaw_of(quaternion: Any) -> float:
    return math.atan2(2.0 * (quaternion.z * quaternion.w),
                      1.0 - 2.0 * (quaternion.z * quaternion.z))


def load_bag(bag_path: str, topic_prefix: str = "/go2w/slam") -> Dict[str, Any]:
    """§17：从 rosbag2 的 sqlite3 存储提取评估所需的最小数据。

    直接读 ``bag_0.db3`` 而不经 ``rosbag2_py``：被 SIGINT 中断的录制可能缺
    ``metadata.yaml``，但 sqlite 内容仍是完整的（2026-09-08 实测）。
    """
    import sqlite3

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    db_path = os.path.join(bag_path, "bag_0.db3")
    connection = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True)
    topics = {row[0]: (row[1], row[2]) for row in
              connection.execute("SELECT id, name, type FROM topics")}
    map_cloud = np.empty((0, 3), dtype=float)
    trajectory: List[List[float]] = []
    trajectory_stamps: List[float] = []
    keyframes = 0
    loop_edge_points = 0
    loop_edge_markers = 0
    loop_stamps: List[float] = []
    first_loop_stamp = None
    first_scan = None
    last_scan = None
    # §20.4：回环发生前/后的最后一帧扫描（用于比较回环是否让地图变好）。
    pre_loop_scan = None
    post_loop_scan = None
    cursor = connection.execute(
        "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")
    for topic_id, stamp_ns, data in cursor:
        name, type_name = topics.get(topic_id, (None, None))
        if name is None:
            continue
        if name == f"{topic_prefix}/map_3d":
            message = deserialize_message(data, get_message(type_name))
            points = _pointcloud2_xyz(message)
            if len(points):
                map_cloud = points
        elif name == f"{topic_prefix}/odom_base":
            message = deserialize_message(data, get_message(type_name))
            pose = message.pose.pose
            trajectory.append([float(pose.position.x), float(pose.position.y),
                               _yaw_of(pose.orientation)])
            trajectory_stamps.append(float(stamp_ns) * 1e-9)
        elif name == f"{topic_prefix}/aligned_scan":
            message = deserialize_message(data, get_message(type_name))
            points = _pointcloud2_xyz(message)
            if len(points):
                frame_id = message.header.frame_id
                stamp_s = float(stamp_ns) * 1e-9
                if first_scan is None:
                    first_scan = (stamp_s, points, frame_id)
                last_scan = (stamp_s, points, frame_id)
                if first_loop_stamp is None:
                    pre_loop_scan = (stamp_s, points, frame_id)
                else:
                    post_loop_scan = (stamp_s, points, frame_id)
        elif name == f"{topic_prefix}/pose_graph_poses":
            message = deserialize_message(data, get_message(type_name))
            keyframes = max(keyframes, len(message.poses))
        elif name == f"{topic_prefix}/pose_graph_loop_edges":
            # 2026-09-09：回环边是 visualization_msgs/Marker（slam_3d_node 的
            # loop_edges_pub_），points 里每 2 个点构成一条边。据此统计回环数，
            # 让 loop_closure_count 有真实来源，而不是永远 NOT_VERIFIED。
            message = deserialize_message(data, get_message(type_name))
            count = len(getattr(message, "points", []) or [])
            if count:
                loop_edge_markers += 1
                loop_edge_points = max(loop_edge_points, count)
                stamp_s = float(stamp_ns) * 1e-9
                loop_stamps.append(stamp_s)
                if first_loop_stamp is None:
                    first_loop_stamp = stamp_s
    connection.close()
    return {
        "points": map_cloud,
        "trajectory": (np.asarray(trajectory, dtype=float) if trajectory
                       else np.empty((0, 3), dtype=float)),
        "trajectory_stamps": trajectory_stamps,
        "keyframes": int(keyframes),
        "loop_edges": int(loop_edge_points // 2),
        "loop_edge_markers": int(loop_edge_markers),
        "loop_stamps": sorted(loop_stamps),
        "first_scan": first_scan,
        "last_scan": last_scan,
        "pre_loop_scan": pre_loop_scan,
        "post_loop_scan": post_loop_scan,
    }


def _pose_at(stamps: Sequence[float], trajectory: np.ndarray,
             target: float) -> np.ndarray:
    index = int(np.argmin(np.abs(np.asarray(stamps, dtype=float) - float(target))))
    return trajectory[index]


def _apply_pose(points: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    x, y, yaw = (float(v) for v in pose)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    out = np.array(points, dtype=float, copy=True)
    out[:, 0] = x + cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
    out[:, 1] = y + sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
    return out


def _scan_to_odom(points: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    return _apply_pose(points, pose)


def _compose_pose(first: Sequence[float],
                  second: Sequence[float]) -> Tuple[float, float, float]:
    """``T_a_c = T_a_b ∘ T_b_c``（2D 平面位姿）。"""
    ax, ay, ayaw = (float(v) for v in first)
    bx, by, byaw = (float(v) for v in second)
    return (ax + math.cos(ayaw) * bx - math.sin(ayaw) * by,
            ay + math.sin(ayaw) * bx + math.cos(ayaw) * by,
            ayaw + byaw)


def _invert_pose(pose: Sequence[float]) -> Tuple[float, float, float]:
    x, y, yaw = (float(v) for v in pose)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return (-(cos_yaw * x + sin_yaw * y),
            -(-sin_yaw * x + cos_yaw * y),
            -yaw)


def scan_overlap_metrics(first_scan: Any, last_scan: Any,
                         stamps: Sequence[float], trajectory: np.ndarray,
                         max_dist_m: float = 0.10) -> Dict[str, Any] | None:
    """§19.1：起/终子图在 odom 系的重合度（A→B→A 回到同一面墙时该值应很小）。

    先把末帧按**起终点位姿差**补偿到起始位姿，再比最近邻——这样指标只反映
    LIO 漂移 / 地图形变，不会被"机器人没转回原位"污染（2026-09-08 实测：
    Dataset A′ 因净朝向差 6.03° 让未补偿的 overlap 从 0.133 涨到 0.231）。
    未补偿的值同时保留为 ``overlap_rmse_raw_m`` 供对照。
    """
    if (first_scan is None or last_scan is None or len(stamps) == 0
            or len(trajectory) == 0):
        return None
    start_pose = _pose_at(stamps, trajectory, first_scan[0])
    end_pose = _pose_at(stamps, trajectory, last_scan[0])
    # aligned_scan 的 frame_id 实测就是 pslam_odom（点云数值本身就是 odom 坐标，
    # x 从 10 m 到 36 m）。此时**绝不能再变换一次**，否则得到 1631 m 这种荒谬值
    # （2026-09-08 实测）。只有真正的传感器系点云才需要按位姿变换。
    world_frames = {"pslam_odom", "pslam_map", "odom", "map"}
    frame_id = (first_scan[2] if len(first_scan) > 2 else "") or ""
    already_world = frame_id.lstrip("/") in world_frames
    start = (first_scan[1] if already_world
             else _scan_to_odom(first_scan[1], start_pose))
    end = (last_scan[1] if already_world
           else _scan_to_odom(last_scan[1], end_pose))
    metrics = nn_overlap_metrics(start, end, max_dist_m=max_dist_m)
    metrics["overlap_rmse_raw_m"] = metrics["overlap_rmse_m"]
    if already_world:
        # 点云已经在 odom/map 系：首末帧的位置差已经包含了 LIO 的对齐结果
        # （"机器人没回原位"不会额外污染它），再做一次位姿补偿就是**重复补偿**，
        # 实测会把 0.0433 放大到 0.0680。所以这里直接返回。
        metrics["pose_correction_applied"] = False
        metrics["source"] = ("rosbag:aligned_scan first/last "
                             f"(already in {frame_id.lstrip('/')})")
        return metrics
    delta = _compose_pose(start_pose, _invert_pose(end_pose))
    compensated = _apply_pose(end, delta)
    fixed = nn_overlap_metrics(start, compensated, max_dist_m=max_dist_m)
    metrics.update({key: value for key, value in fixed.items()
                    if key != "overlap_points"})
    metrics["pose_correction_applied"] = True
    metrics["source"] = ("rosbag:aligned_scan first/last in odom "
                         "(end pose-compensated)")
    return metrics


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def post_loop_jitter(trajectory: np.ndarray, stamps: Sequence[float],
                     loop_stamps: Sequence[float], start_offset_s: float = 3.0,
                     end_offset_s: float = 5.0, static_step_m: float = 0.01,
                     min_static_samples: int = 3) -> Dict[str, Any]:
    """§20.3：回环发生后 3–5 s 窗口内的位姿抖动 P95。

    口径修正（2026-09-10 真机实测）：若回环发生在机器人**仍在运动**时，窗口内的位移
    本身就是运动（实测 0.25–0.30 m，与走速吻合），把它当"抖动"会自欺。因此：

    * 始终输出窗口内行程 ``post_loop_window_travel_m``（用于判断窗口是否混入运动）；
    * 若窗口内存在 ≥``min_static_samples`` 个准静止样本（相邻帧位移 < ``static_step_m``），
      **只用准静止样本**算抖动，``post_loop_static_samples`` 给出其数量；
    * 否则退回全窗口计算，``post_loop_static_samples=0`` 提示调用方该值混入了运动。

    窗口内点太少（数据集太短）时退化为"回环后到 +5 s"，仍少于 3 个点则如实返回 None
    （由 acceptance 判为 NOT_VERIFIED，而不是伪造 0）。
    """
    empty = {"post_loop_translation_jitter_m": None,
             "post_loop_yaw_jitter_deg": None,
             "post_loop_window": None, "post_loop_samples": 0,
             "post_loop_static_samples": 0, "post_loop_window_travel_m": None}
    if not loop_stamps or len(trajectory) < 3 or len(stamps) != len(trajectory):
        return empty
    loop_time = float(min(loop_stamps))
    stamps_arr = np.asarray(stamps, dtype=float)
    in_window = ((stamps_arr >= loop_time + start_offset_s)
                 & (stamps_arr <= loop_time + end_offset_s))
    window = "loop+3..5s"
    if int(np.count_nonzero(in_window)) < 3:
        in_window = ((stamps_arr >= loop_time)
                     & (stamps_arr <= loop_time + end_offset_s))
        window = "loop+0..5s"
    if int(np.count_nonzero(in_window)) < 3:
        return empty
    poses = trajectory[in_window]
    times = stamps_arr[in_window]
    steps = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1) if len(poses) > 1 \
        else np.empty(0, dtype=float)
    travel = float(steps.sum())
    static_mask = (np.concatenate([[True], steps < static_step_m])
                   if len(steps) else np.array([True]))
    static_count = int(np.count_nonzero(static_mask))
    use_static = static_count >= min_static_samples
    if use_static:
        sample_poses, sample_times = poses[static_mask], times[static_mask]
        mode = "static-subsample"
    else:
        # 窗口混入了运动（实测 travel 0.55–0.63 m）：用线性趋势拟合去掉匀速运动分量，
        # 残差 P95 才是"抖动"。原始口径（相对中位数）同时保留供对照。
        sample_poses, sample_times = poses, times
        mode = "detrended"
    centre = np.median(sample_poses, axis=0)
    raw = np.linalg.norm(sample_poses[:, :2] - centre[:2], axis=1)
    offsets = sample_times - sample_times[0]
    yaw_series = np.unwrap(np.array([float(v) for v in sample_poses[:, 2]]))
    residual = []
    for axis in (0, 1):
        if len(offsets) >= 3:
            coeffs = np.polyfit(offsets, sample_poses[:, axis], 1)
            residual.append(sample_poses[:, axis] - np.polyval(coeffs, offsets))
        else:
            residual.append(sample_poses[:, axis] - centre[axis])
    detrended = np.linalg.norm(np.stack(residual, axis=1), axis=1)
    # yaw 同样去趋势：转弯窗口里 yaw 单调变化同样会被误判成"抖动"。
    if not use_static and len(offsets) >= 3:
        yaw_trend = np.polyval(np.polyfit(offsets, yaw_series, 1), offsets)
        yaw_residual = yaw_series - yaw_trend
    else:
        yaw_residual = yaw_series - np.median(yaw_series)
    jitter = float(np.percentile(raw if use_static else detrended, 95))
    return {
        "post_loop_translation_jitter_m": jitter,
        "post_loop_translation_jitter_raw_m": float(np.percentile(raw, 95)),
        "post_loop_yaw_jitter_deg": float(np.degrees(
            np.percentile(np.abs(yaw_residual), 95))),
        "post_loop_yaw_jitter_raw_deg": float(np.degrees(np.percentile(
            np.abs(np.array([_wrap_angle(float(v) - float(centre[2]))
                             for v in sample_poses[:, 2]])), 95))),
        "post_loop_jitter_mode": mode,
        "post_loop_window": window,
        "post_loop_samples": int(len(poses)),
        "post_loop_static_samples": static_count if use_static else 0,
        "post_loop_window_travel_m": travel,
    }


def loop_overlap_improvement(first_scan: Any, pre_loop_scan: Any,
                             post_loop_scan: Any,
                             max_dist_m: float = 0.10) -> Dict[str, Any] | None:
    """§20.4：回环前/后末帧与起始扫描的重合度，判断回环是否让地图变好。

    三帧都是 ``/go2w/slam/aligned_scan``，frame_id 实测为 ``pslam_odom``（点云已是
    odom 坐标），因此可直接比较，不再做位姿变换。frame 不是世界系或任一帧缺失时
    返回 None（NOT_VERIFIED），不伪造改善比例。
    """
    if first_scan is None or pre_loop_scan is None or post_loop_scan is None:
        return None
    frames = {(scan[2] if len(scan) > 2 else "") or "" for scan in
              (first_scan, pre_loop_scan, post_loop_scan)}
    world_frames = {"pslam_odom", "pslam_map", "odom", "map"}
    if {frame.lstrip("/") for frame in frames} - world_frames:
        return None
    pre = nn_overlap_metrics(first_scan[1], pre_loop_scan[1], max_dist_m=max_dist_m)
    post = nn_overlap_metrics(first_scan[1], post_loop_scan[1], max_dist_m=max_dist_m)
    pre_rmse = pre["overlap_rmse_m"]
    ratio = ((pre_rmse - post["overlap_rmse_m"]) / pre_rmse) if pre_rmse > 0 else 0.0
    return {
        "pre_loop_overlap_rmse_m": pre_rmse,
        "post_loop_overlap_rmse_m": post["overlap_rmse_m"],
        "pre_loop_overlap_p95_m": pre["overlap_p95_m"],
        "post_loop_overlap_p95_m": post["overlap_p95_m"],
        "overlap_improvement_ratio": float(ratio),
        "loop_overlap_improved": bool(post["overlap_rmse_m"] < pre_rmse),
        "source": ("rosbag:aligned_scan first vs last-before-loop vs "
                   "last-after-loop (already in world frame)"),
    }


def instance_counts(objects: Dict[str, List[List[float]]]) -> Dict[str, int]:
    """§22：把"实例键"归成 label -> 预测实例数。

    ``load_object_events`` 的键是 ``label#persistent_id``（同一 label 的多个持久实例
    应计为多个），``load_objects`` 的键是纯 label（每个键一组观测）。只按 ``#`` 前的
    label 计数，绝不按 label 合并不同持久实例。
    """
    counts: Dict[str, int] = {}
    for key in objects:
        label = str(key).split("#", 1)[0]
        counts[label] = counts.get(label, 0) + 1
    return counts


def acceptance_check(report: Dict[str, Any],
                     thresholds: Dict[str, float] | None = None) -> Dict[str, Any]:
    """§23 / §29：按阈值逐项给出 PASS / FAIL / NOT_VERIFIED。"""
    limits = {
        "surface_thickness_p95_m_max": 0.15,
        "endpoint_translation_error_m_max": 0.20,
        "endpoint_yaw_error_deg_max": 5.0,
        "object_position_rms_m_max": 0.15,
        "object_position_p95_m_max": 0.25,
        "overlap_rmse_m_max": 0.10,
        "overlap_p95_m_max": 0.18,
        "overlap_inlier_ratio_min": 0.75,
        "loop_closure_count_min": 1.0,
        "post_loop_translation_jitter_m_max": 0.08,
        "post_loop_yaw_jitter_deg_max": 2.0,
        "duplicate_rate_max": 0.0,
        "false_split_max": 0.0,
        "false_merge_max": 0.0,
    }
    if thresholds:
        limits.update(thresholds)
    checks: Dict[str, Any] = {}

    def add(name: str, value: Any, limit: float, comparator: str) -> None:
        if value is None or (isinstance(value, float) and math.isnan(value)):
            checks[name] = {"value": None, "limit": limit, "status": "NOT_VERIFIED"}
        elif comparator == "le":
            checks[name] = {"value": float(value), "limit": limit,
                            "status": "PASS" if float(value) <= limit else "FAIL"}
        else:
            checks[name] = {"value": float(value), "limit": limit,
                            "status": "PASS" if float(value) >= limit else "FAIL"}

    map_section = report.get("map") or {}
    add("surface_thickness_p95_m", map_section.get("surface_thickness_p95_m"),
        limits["surface_thickness_p95_m_max"], "le")
    trajectory = report.get("trajectory") or {}
    add("endpoint_translation_error_m", trajectory.get("endpoint_translation_error_m"),
        limits["endpoint_translation_error_m_max"], "le")
    add("endpoint_yaw_error_deg", trajectory.get("endpoint_yaw_error_deg"),
        limits["endpoint_yaw_error_deg_max"], "le")
    semantic = report.get("semantic") or {}
    add("object_position_rms_m", semantic.get("object_position_rms_m"),
        limits["object_position_rms_m_max"], "le")
    add("object_position_p95_m", semantic.get("object_position_p95_m"),
        limits["object_position_p95_m_max"], "le")
    overlap = report.get("overlap") or {}
    add("overlap_rmse_m", overlap.get("overlap_rmse_m"),
        limits["overlap_rmse_m_max"], "le")
    add("overlap_p95_m", overlap.get("overlap_p95_m"),
        limits["overlap_p95_m_max"], "le")
    add("overlap_inlier_ratio", overlap.get("overlap_inlier_ratio"),
        limits["overlap_inlier_ratio_min"], "ge")
    loop = report.get("loop") or {}
    add("loop_closure_count", loop.get("loop_closure_count"),
        limits["loop_closure_count_min"], "ge")
    add("post_loop_translation_jitter_m", loop.get("post_loop_translation_jitter_m"),
        limits["post_loop_translation_jitter_m_max"], "le")
    add("post_loop_yaw_jitter_deg", loop.get("post_loop_yaw_jitter_deg"),
        limits["post_loop_yaw_jitter_deg_max"], "le")
    dedup = report.get("dedup") or {}
    add("duplicate_rate", dedup.get("duplicate_rate"),
        limits["duplicate_rate_max"], "le")
    add("false_split", dedup.get("false_split"),
        limits["false_split_max"], "le")
    add("false_merge", dedup.get("false_merge"),
        limits["false_merge_max"], "le")

    failed = [name for name, item in checks.items() if item["status"] == "FAIL"]
    verified = [name for name, item in checks.items() if item["status"] != "NOT_VERIFIED"]
    return {"checks": checks, "failed": failed, "verified": verified,
            "accepted": not failed and bool(verified)}


def load_objects(path: str) -> Dict[str, List[List[float]]]:
    with open(path, "r", encoding="utf-8") as handle:
        raw = json.load(handle)
    return {str(label): [[float(v) for v in obs] for obs in items]
            for label, items in raw.items()}


def load_object_events(path: str) -> Dict[str, List[List[float]]]:
    """从 worker JSONL 导出逐帧原始 map_xyz，并按持久实例分组。

    只接收事件中已有的 ``persistent_object_id``；没有严格 source-id 关联的
    观测不会按 label 猜归属。
    """
    grouped: Dict[str, List[List[float]]] = {}
    labels_by_id: Dict[str, Dict[str, int]] = {}
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            event = json.loads(line)
            if event.get("event") != "semantic_frame_objects_localized":
                continue
            for item in event.get("objects") or []:
                persistent_id = str(item.get("persistent_object_id") or "").strip()
                xyz = item.get("map_xyz")
                if not persistent_id or not isinstance(xyz, list) or len(xyz) < 3:
                    continue
                label = str(
                    item.get("label_zh") or item.get("label")
                    or item.get("name_zh") or item.get("name") or "object"
                ).strip()
                # Identity is the persistent id, not the mutable VLM label.
                # Keep one group across label drift (e.g. 桌子/办公桌), while
                # retaining a deterministic display label for the report.
                labels = labels_by_id.setdefault(persistent_id, {})
                labels[label] = labels.get(label, 0) + 1
                grouped.setdefault(persistent_id, []).append(
                    [float(v) for v in xyz[:3]]
                )
    result: Dict[str, List[List[float]]] = {}
    for persistent_id, observations in grouped.items():
        labels = labels_by_id[persistent_id]
        display_label = sorted(labels, key=lambda value: (-labels[value], value))[0]
        result[f"{display_label}#{persistent_id}"] = observations
    return result


def build_report(*, map_path: str | None, trajectory_path: str | None,
                 objects_path: str | None, expected_instances: Dict[str, int] | None,
                 overlap_max_dist_m: float,
                 object_events_path: str | None = None) -> Dict[str, Any]:
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
    if objects_path or object_events_path:
        objects = (
            load_object_events(object_events_path)
            if object_events_path else load_objects(objects_path)
        )
        per_object = {label: object_spread(obs) for label, obs in objects.items()}
        repeated = [
            metrics for metrics in per_object.values()
            if metrics["observations"] >= 2
        ]
        report["semantic"] = {
            "objects": per_object,
            # 单帧到自身质心的误差必为 0，不能冒充跨帧稳定性通过。
            "object_position_rms_m": (
                float(np.nanmax([v["centroid_rms_error_m"] for v in repeated]))
                if repeated else float("nan")
            ),
            "object_position_p95_m": (
                float(np.nanmax([v["centroid_p95_error_m"] for v in repeated]))
                if repeated else float("nan")
            ),
            "repeated_object_count": len(repeated),
            "predicted_instances": instance_counts(objects),
        }
        if expected_instances:
            report["dedup"] = duplicate_metrics(
                instance_counts(objects), expected_instances)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic-prefix", default="/go2w/slam",
                        help="SLAM 话题前缀（rko 实例为 /go2w/rko_slam）")
    parser.add_argument("--bag", dest="bag_path", default=None,
                        help="rosbag2 目录（读 /go2w/slam/map_3d 与 /go2w/slam/odom_base）")
    parser.add_argument("--map", dest="map_path", default=None,
                        help="落盘点云 .npy/.npz，键 points")
    parser.add_argument("--trajectory", dest="trajectory_path", default=None,
                        help="轨迹 .npy，列 x,y,yaw")
    parser.add_argument("--objects", dest="objects_path", default=None,
                        help="语义观测 JSON：{实例: [[x,y,z], ...]}")
    parser.add_argument("--object-events", default=None,
                        help="worker JSONL；读取 semantic_frame_objects_localized 原始观测")
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
        object_events_path=args.object_events,
    )
    if args.bag_path:
        bag = load_bag(args.bag_path, args.topic_prefix)
        cloud = bag["points"]
        if len(cloud):
            report["map"] = {
                "point_count": int(len(cloud)),
                "extent_m": [float(v) for v in (cloud.max(axis=0) - cloud.min(axis=0))],
                "surface_thickness_p95_m": surface_thickness_p95(cloud),
                "source": "rosbag:/go2w/slam/map_3d",
            }
        trajectory = bag["trajectory"]
        if len(trajectory):
            report["trajectory"] = dict(endpoint_error(trajectory),
                                        samples=int(len(trajectory)),
                                        source="rosbag:/go2w/slam/odom_base")
        overlap = scan_overlap_metrics(
            bag["first_scan"], bag["last_scan"], bag["trajectory_stamps"],
            bag["trajectory"], max_dist_m=args.overlap_max_dist_m)
        if overlap is not None:
            report["overlap"] = overlap
        report["loop"] = {
            "loop_closure_count": int(bag.get("loop_edges", 0)),
            "loop_edge_markers": int(bag.get("loop_edge_markers", 0)),
            "keyframes": bag["keyframes"],
            "source": ("rosbag:/go2w/slam/pose_graph_loop_edges "
                       "(visualization_msgs/Marker, points//2)"),
            "note": ("回环数来自 slam_3d_node 发布的 loop_edges Marker；"
                     "本数据集若为 0，说明该段轨迹没有形成闭环，属真实结果而非缺失。"),
        }
        report["loop"].update(post_loop_jitter(
            trajectory, bag["trajectory_stamps"], bag.get("loop_stamps") or []))
        improvement = loop_overlap_improvement(
            bag["first_scan"], bag.get("pre_loop_scan"), bag.get("post_loop_scan"),
            max_dist_m=args.overlap_max_dist_m)
        if improvement is not None:
            report["loop"].update(improvement)
    report["acceptance"] = acceptance_check(report)
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
