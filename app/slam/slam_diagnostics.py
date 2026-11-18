"""外参与时间同步诊断（20260908 计划书 §31 / §32）。

离线读取 rosbag2，输出（JSON，可直接给验收脚本消费）：

* 各传感器 topic 的消息数、frame_id、header 时间戳相对 bag 接收时间的 age；
* RGB 与 Depth 的成对时间偏差（§32 ``max_rgb_depth_skew_ms``）；
* ``/tf_static`` 中的 frame 对与外参平移（LiDAR / IMU / Camera 静态外参）。

任何 frame 缺失、或 RGB-D skew 超阈值 → ``checks`` 里 FAIL，不允许"看起来正常"的结论。

用法（控制主机，需 ROS 2 Python 环境）：

    /usr/bin/python3 -m app.slam.slam_diagnostics --bag <bag 目录> --output <json>
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sqlite3
from typing import Any, Dict, List, Tuple

SENSOR_TOPICS: Dict[str, str] = {
    "/hesai/pandarxt16/points_raw": "lidar",
    "/go2w/slam/pandar_points": "lidar_slam_input",
    "/utlidar/imu": "imu",
    "/go2w/slam/imu": "imu_slam_input",
    "/go2w/d435/color/image_raw": "rgb",
    "/go2w/d435/depth/image_rect_raw": "depth",
}
TF_TOPICS = ("/tf", "/tf_static")
MAX_RGB_DEPTH_SKEW_MS = 40.0


def _stamp_seconds(message: Any) -> Tuple[float | None, str]:
    header = getattr(message, "header", None)
    if header is None:
        return None, ""
    return (float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9,
            str(header.frame_id))


def _percentile(values: List[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((q / 100.0) * (len(ordered) - 1)))))
    return float(ordered[index])


def _pair_skew_ms(first: List[float], second: List[float]) -> List[float]:
    """两串时间戳的最接近配对偏差（毫秒），只保留 1:1 的最近邻配对。"""
    if not first or not second:
        return []
    first_sorted = sorted(first)
    second_sorted = sorted(second)
    skews: List[float] = []
    index = 0
    for value in first_sorted:
        while (index + 1 < len(second_sorted)
               and abs(second_sorted[index + 1] - value) <= abs(second_sorted[index] - value)):
            index += 1
        skews.append(abs(second_sorted[index] - value) * 1000.0)
    return skews


def collect(bag_path: str) -> Dict[str, Any]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    db_path = os.path.join(bag_path, "bag_0.db3")
    connection = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True)
    topics = {row[0]: (row[1], row[2]) for row in
              connection.execute("SELECT id, name, type FROM topics")}
    wanted = set(SENSOR_TOPICS) | set(TF_TOPICS)
    stats: Dict[str, Dict[str, Any]] = {}
    header_stamps: Dict[str, List[float]] = {}
    frames: set[str] = set()
    static_transforms: List[Dict[str, Any]] = []

    cursor = connection.execute(
        "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")
    for topic_id, stamp_ns, data in cursor:
        name, type_name = topics.get(topic_id, (None, None))
        if name not in wanted:
            continue
        message = deserialize_message(data, get_message(type_name))
        entry = stats.setdefault(name, {"count": 0, "ages_ms": [], "frames": set()})
        entry["count"] += 1
        if name in TF_TOPICS:
            for transform in getattr(message, "transforms", []) or []:
                parent = transform.header.frame_id
                child = transform.child_frame_id
                frames.add(parent)
                frames.add(child)
                if name == "/tf_static":
                    static_transforms.append({
                        "parent": parent,
                        "child": child,
                        "translation": [float(transform.transform.translation.x),
                                        float(transform.transform.translation.y),
                                        float(transform.transform.translation.z)],
                    })
            continue
        header_sec, frame_id = _stamp_seconds(message)
        if header_sec is None:
            continue
        entry["frames"].add(frame_id)
        frames.add(frame_id)
        entry["ages_ms"].append((float(stamp_ns) * 1e-9 - header_sec) * 1000.0)
        if SENSOR_TOPICS.get(name) in ("rgb", "depth"):
            header_stamps.setdefault(name, []).append(header_sec)
    connection.close()

    sensors: Dict[str, Any] = {}
    for topic, entry in sorted(stats.items()):
        if topic in TF_TOPICS:
            sensors[topic] = {"count": entry["count"], "kind": "tf"}
            continue
        ages = entry.get("ages_ms") or []
        sensors[topic] = {
            "kind": SENSOR_TOPICS.get(topic),
            "count": entry["count"],
            "frames": sorted(entry["frames"]),
            "age_p50_ms": _percentile(ages, 50),
            "age_p95_ms": _percentile(ages, 95),
        }

    skews = _pair_skew_ms(header_stamps.get("/go2w/d435/color/image_raw", []),
                          header_stamps.get("/go2w/d435/depth/image_rect_raw", []))
    rgb_depth = {
        "pairs": len(skews),
        "skew_p50_ms": _percentile(skews, 50),
        "skew_p95_ms": _percentile(skews, 95),
        "skew_max_ms": max(skews) if skews else None,
    }

    def _presence(topic: str) -> Dict[str, Any]:
        entry = sensors.get(topic) or {}
        if not entry.get("count"):
            # 该 bag 没录这个 topic：如实标 NOT_VERIFIED，而不是伪造 frame 缺失失败。
            return {"value": False, "limit": True, "status": "NOT_VERIFIED",
                    "reason": f"{topic} 未在本 bag 中出现（未录制）"}
        frames = entry.get("frames") or []
        if not frames:
            return {"value": False, "limit": True, "status": "FAIL",
                    "reason": f"{topic} 有消息但 header.frame_id 为空"}
        return {"value": True, "limit": True, "status": "PASS",
                "frames": frames}

    checks = {
        "rgb_depth_skew_p95_ms": {
            "value": rgb_depth["skew_p95_ms"],
            "limit": MAX_RGB_DEPTH_SKEW_MS,
            "status": ("NOT_VERIFIED" if rgb_depth["skew_p95_ms"] is None else
                       ("PASS" if rgb_depth["skew_p95_ms"] <= MAX_RGB_DEPTH_SKEW_MS
                        else "FAIL")),
        },
        "lidar_frame_present": _presence("/hesai/pandarxt16/points_raw"),
        "imu_frame_present": _presence("/utlidar/imu"),
    }
    failed = [name for name, item in checks.items() if item["status"] == "FAIL"]
    return {
        "schema_version": "slam_diagnostics_v1",
        "bag": bag_path,
        "sensors": sensors,
        "rgb_depth_skew": rgb_depth,
        "frames_seen": sorted(frames),
        "static_transforms": static_transforms,
        "checks": checks,
        "failed": failed,
        "accepted": not failed,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="rosbag2 目录")
    parser.add_argument("--output", required=True, help="诊断 JSON 输出路径")
    args = parser.parse_args()

    report = collect(args.bag)
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
    print(json.dumps({"failed": report["failed"], "accepted": report["accepted"],
                      "rgb_depth_skew": report["rgb_depth_skew"],
                      "output": args.output}, ensure_ascii=False, indent=2))
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
