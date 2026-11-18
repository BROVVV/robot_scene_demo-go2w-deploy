"""Read the latest mapping-assist point-cloud snapshot for FastAPI.

计划书 §2.3：`/api/slam/map3d` 是前端唯一的三维接口，它读**两份**显式路径的文件
——地图快照和对象快照 —— 并在这里合成。合成的唯一判据是两个标识同时相等：
``map_epoch`` 和 ``frame_id``。两个进程各自写的对象与地图，只有这两个都相等才
叠画；否则返回空 ``objects``，并在 ``object_overlay_status`` 里说明是哪一种不匹配。
**不用坐标包围盒、不用文件修改时间**推断"属于同一张地图"（那条路已被证伪）。
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from app.spatial.object_display_snapshot import load_object_snapshot

#: ``object_overlay_status`` 的取值。前三个是计划书 §2.3 点名的三种"不叠画"，
#: ``OK`` 是叠画成功了 —— 前端据此显示"物体来自另一张地图"之类的提示。
OVERLAY_OK = "OK"
OVERLAY_NO_SNAPSHOT = "NO_OBJECT_SNAPSHOT"
OVERLAY_FRAME_MISMATCH = "FRAME_MISMATCH"
OVERLAY_EPOCH_MISMATCH = "MAP_EPOCH_MISMATCH"

_EMPTY_OBJECT_STATS: dict[str, Any] = {
    "input": 0, "displayed": 0, "filtered": 0, "dropped": {},
}

#: 解析过的大地图 JSON 按 (mtime_ns, size) 缓存。60k 点的快照约 1.4 MB，
#: 前端每 850 ms 拉一次，不缓存就是每轮重解析一遍。
_PARSED_CACHE: dict[str, tuple[tuple[int, int], dict[str, Any]]] = {}


def unavailable_snapshot(reason: str) -> dict[str, Any]:
    return {
        "schema_version": "go2w_slam_web_cloud_v2",
        "available": False,
        "reason": reason,
        "points": [],
        "point_count": 0,
        "objects": [],
        "object_filter_stats": dict(_EMPTY_OBJECT_STATS),
        "object_overlay_status": OVERLAY_NO_SNAPSHOT,
        "mapping_mode": "mapping_assist",
        "motion_authorized": False,
        "safety_authorized": False,
        "mapping_health": "UNAVAILABLE",
        "health_reason": "建图快照还没生成",
        "lio_pose_valid": False,
        "motion_odom_seen": False,
        "canonical_frame": "pslam_map",
        "permanent_source": "/go2w/slam/map_3d",
        "mapping_session_id": 0,
        "map_revision": 0,
        "source_map_points": 0,
        "global_cached_voxels": 0,
        "web_display_points": 0,
        "map_extent_m": [0.0, 0.0, 0.0],
        "capacity_limited": False,
        "preview": {"frame_id": "", "ros_stamp": 0.0, "point_count": 0, "points": []},
        "rejected_counts": {},
    }


def load_slam_map_snapshot(
    path: str | Path,
    objects_path: str | Path | None = None,
) -> dict[str, Any]:
    value, problem = _load_json_cached(Path(path))
    if problem == "snapshot_not_ready":
        return unavailable_snapshot("snapshot_not_ready")
    if problem == "snapshot_unreadable":
        return unavailable_snapshot("snapshot_unreadable")
    if value is None or not isinstance(value.get("points"), list):
        return unavailable_snapshot("snapshot_invalid")
    is_grid = value.get("map_kind") == "occupancy_grid_2d"
    grid = value.get("grid")
    if is_grid and not _valid_grid(grid):
        return unavailable_snapshot("snapshot_invalid_grid")
    now = time.time()
    generated_at = value.get("generated_at")
    try:
        age = max(0.0, now - float(generated_at))
    except (TypeError, ValueError):
        age = None
    result = dict(value)
    result["age_seconds"] = age
    # fresh 是"写端心跳还活着"（generated_at 每 period_s 重写一次必然刷新），
    # 不是"地图数据新"。写端 period_s 默认 5.0 s，门限必须大于它 —— 原来的
    # 3.0 s 比节拍还短，于是页面每隔几秒就绿黄跳一次，实测 12 次采样 5 次 STALE。
    result["fresh"] = age is not None and age <= 6.0
    result["available"] = bool(grid) if is_grid else (
        bool(value.get("available", True)) and bool(value["points"])
    )
    result["point_count"] = len(value["points"])
    if is_grid:
        result["web_display_points"] = int(grid["width"]) * int(grid["height"])
    else:
        result["web_display_points"] = len(value["points"])
    # §9.3：全局地图和当前 scan 是两个时间轴，网页必须分别显示，不能混成一个。
    # 用地图自己的 ros_stamp 算年龄，不用 map_updated_at（= 写端收到消息的时刻）：
    # map_3d 是 transient_local 发布，写端一重启就会立刻收到重投的那一帧，
    # map_updated_at 于是变成"刚刚"，而数据其实还是几十钟头前那张 —— 页面上会
    # 显示"地图 18.1s 前"这种假 LIVE。ros_stamp 跟着数据走，重启也不会变。
    map_stamp = value.get("ros_stamp")
    if not isinstance(map_stamp, (int, float)) or map_stamp <= 0.0:
        map_stamp = value.get("map_updated_at")
    result["map_age_seconds"] = _age(now, map_stamp)
    result["preview_age_seconds"] = _age(
        now, (value.get("preview") or {}).get("wall_time"))
    # These are facts about this isolated display path, not inferred flags.
    result["mapping_mode"] = "navigation_2d" if value.get("map_kind") == "occupancy_grid_2d" else "mapping_assist"
    result["motion_authorized"] = False
    result["safety_authorized"] = False
    _compose_objects(result, objects_path)
    return result


def _valid_grid(grid: Any) -> bool:
    if not isinstance(grid, dict):
        return False
    try:
        width, height = int(grid["width"]), int(grid["height"])
        resolution = float(grid["resolution"])
        origin = grid["origin"]
        numbers = (resolution, float(origin["x"]), float(origin["y"]), float(origin["yaw"]))
    except (KeyError, TypeError, ValueError):
        return False
    data = grid.get("data")
    return (width > 0 and height > 0 and resolution > 0
            and isinstance(data, list) and len(data) == width * height
            and all(number == number and abs(number) != float("inf") for number in numbers))


def _load_json_cached(snapshot_path: Path) -> tuple[dict[str, Any] | None, str]:
    """按 (mtime_ns, size) 缓存解析结果；返回 (内容, 问题标记)。"""
    try:
        stat = snapshot_path.stat()
    except OSError:
        return None, "snapshot_not_ready"
    key = str(snapshot_path)
    fingerprint = (stat.st_mtime_ns, stat.st_size)
    cached = _PARSED_CACHE.get(key)
    if cached is not None and cached[0] == fingerprint:
        return cached[1], ""
    try:
        value = json.loads(snapshot_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None, "snapshot_unreadable"
    if not isinstance(value, dict):
        return None, "snapshot_invalid"
    _PARSED_CACHE[key] = (fingerprint, value)
    return value, ""


def _compose_objects(result: dict[str, Any],
                     objects_path: str | Path | None) -> None:
    """§2.3：两个标识同时相等才把对象叠到地图上。

    不等就返回空 ``objects`` 并写明原因 —— "画错了地图"与"这张图还没有物体"
    在页面上必须能分辨，否则错位的鬼影看起来和真实检测一模一样。
    """
    result["objects"] = []
    result["object_filter_stats"] = dict(_EMPTY_OBJECT_STATS)
    if objects_path is None:
        result["object_overlay_status"] = OVERLAY_NO_SNAPSHOT
        return
    snapshot = load_object_snapshot(objects_path)
    if snapshot is None:
        result["object_overlay_status"] = OVERLAY_NO_SNAPSHOT
        return
    if str(snapshot.get("frame_id") or "") != str(result.get("frame_id") or ""):
        result["object_overlay_status"] = OVERLAY_FRAME_MISMATCH
        return
    map_epoch = str(result.get("map_epoch") or "")
    if not map_epoch or str(snapshot.get("map_epoch") or "") != map_epoch:
        result["object_overlay_status"] = OVERLAY_EPOCH_MISMATCH
        return
    result["objects"] = list(snapshot.get("objects") or [])
    stats = snapshot.get("object_filter_stats")
    result["object_filter_stats"] = (
        stats if isinstance(stats, dict) else dict(_EMPTY_OBJECT_STATS))
    result["object_overlay_status"] = OVERLAY_OK


def _age(now: float, stamp: Any) -> float | None:
    try:
        value = float(stamp)
    except (TypeError, ValueError):
        return None
    return max(0.0, now - value) if value > 0.0 else None
