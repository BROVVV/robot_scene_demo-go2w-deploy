#!/usr/bin/env python3
"""Render a rosbag replay with the WebUI semantic map and object topology.

This is a read-only post-processor.  It reuses the audited prompt-free YOLOE
detections from ``detections.jsonl`` and never publishes ROS topics or sends
robot commands.  RGB-D detections are projected into the relative fused-odom
map, associated into persistent object nodes, and drawn using the same visual
language as ``manual_web_demo`` (occupancy grid + semantic object markers and
dark object-node cards).
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from cv_bridge import CvBridge
from PIL import Image, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.go2w.replay_remote_mapping_bag import (  # noqa: E402
    CLOUD_TOPIC,
    COLOR_TOPIC,
    DEPTH_TOPIC,
    INFO_TOPIC,
    FONT_BOLD,
    FONT_REGULAR,
    cell_of,
    first_pass,
    lidar_points,
    load_geometry,
    open_reader,
    update_grid,
)


CANONICAL_LABELS = {
    "car": ("车辆", "fixed"),
    "suv": ("车辆", "fixed"),
    "van": ("车辆", "fixed"),
    "tow truck": ("车辆", "fixed"),
    "trailer": ("车辆", "fixed"),
    "waste container": ("垃圾容器", "fixed"),
    "bin": ("垃圾容器", "fixed"),
    "barrel": ("桶", "fixed"),
    "eucalyptus tree": ("树木", "fixed"),
    "birch tree": ("树木", "fixed"),
    "birch": ("树木", "fixed"),
    "bench": ("长椅", "fixed"),
    "motorcycle": ("摩托车", "semi_static"),
    "bicycle": ("自行车", "semi_static"),
    "trolley": ("手推车", "semi_static"),
    "cart": ("手推车", "semi_static"),
    "lamp": ("路灯", "fixed"),
    "flag": ("旗帜", "fixed"),
    "traffic cone": ("锥桶", "semi_static"),
    "extinguisher": ("灭火器", "fixed"),
}

MIN_SCORE = 0.35
MAX_BOX_AREA = 0.32
ASSOCIATION_DISTANCE_M = 1.35
CONFIRM_OBSERVATIONS = 3
MAX_VISIBLE_OBJECTS = 12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--replay-dir", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--map-resolution", type=float, default=0.08)
    return parser.parse_args()


def load_records(path: Path) -> list[dict[str, Any]]:
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    records.sort(key=lambda item: int(item["frame"]))
    return records


def depth_metres(bridge: CvBridge, msg: Any) -> np.ndarray:
    depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
    if str(msg.encoding).upper() in {"16UC1", "MONO16"}:
        depth *= 0.001
    return depth


def project_detection(
    item: dict[str, Any], depth: np.ndarray, intrinsics: tuple[float, float, float, float],
    pose: tuple[float, float, float],
) -> tuple[float, float, float] | None:
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = (float(value) for value in item["bbox_2d"])
    if max(0.0, x2 - x1) * max(0.0, y2 - y1) > MAX_BOX_AREA:
        return None
    # The central 50% is more robust than the full open-vocabulary box: its
    # boundary frequently contains background pixels.
    cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
    half_w, half_h = 0.25 * (x2 - x1), 0.25 * (y2 - y1)
    xa, xb = max(0, int((cx - half_w) * width)), min(width, int(math.ceil((cx + half_w) * width)))
    ya, yb = max(0, int((cy - half_h) * height)), min(height, int(math.ceil((cy + half_h) * height)))
    if xb <= xa or yb <= ya:
        return None
    values = depth[ya:yb, xa:xb]
    values = values[np.isfinite(values) & (values > 0.25) & (values < 12.0)]
    if values.size < 8:
        return None
    distance = float(np.median(values))
    fx, fy, px, py = intrinsics
    u, v = cx * width, cy * height
    camera_x = (u - px) * distance / fx
    camera_y = (v - py) * distance / fy
    base_x, base_y, base_z = distance + 0.10, -camera_x, 0.30 - camera_y
    x, y, yaw = pose
    return (
        x + math.cos(yaw) * base_x - math.sin(yaw) * base_y,
        y + math.sin(yaw) * base_x + math.cos(yaw) * base_y,
        base_z,
    )


def collect_observations(
    bag: Path, records: list[dict[str, Any]], hazard_report: dict[str, Any]
) -> tuple[list[dict[str, Any]], tuple[float, float, float, float]]:
    reader, types = open_reader(bag)
    bridge = CvBridge()
    intrinsics: tuple[float, float, float, float] | None = None
    depth_index = 0
    observations: list[dict[str, Any]] = []
    while reader.has_next():
        topic, data, _stamp_ns = reader.read_next()
        if topic == INFO_TOPIC and intrinsics is None:
            msg = deserialize_message(data, get_message(types[topic]))
            intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))
            continue
        if topic != DEPTH_TOPIC:
            continue
        depth_index += 1
        if depth_index > len(records) or intrinsics is None:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        depth = depth_metres(bridge, msg)
        record = records[depth_index - 1]
        pose = tuple(float(value) for value in record["pose"])
        for item in record.get("objects") or []:
            label = str(item.get("label") or "").casefold()
            if label not in CANONICAL_LABELS or float(item.get("score") or 0.0) < MIN_SCORE:
                continue
            xyz = project_detection(item, depth, intrinsics, pose)
            if xyz is None:
                continue
            name, mobility = CANONICAL_LABELS[label]
            observations.append({
                "frame": int(record["frame"]), "raw_label": label, "label_zh": name,
                "mobility_class": mobility, "score": float(item["score"]), "xyz": xyz,
            })
    if intrinsics is None:
        raise RuntimeError("camera intrinsics are unavailable")
    map_xy = hazard_report.get("map_xy")
    if map_xy and hazard_report.get("confirmed_frame"):
        observations.append({
            "frame": int(hazard_report["confirmed_frame"]),
            "raw_label": "simulated pipeline liquid leak",
            "label_zh": "模拟管道液体渗漏危险源",
            "mobility_class": "fixed",
            "score": 1.0,
            "xyz": (float(map_xy[0]), float(map_xy[1]), 0.0),
            "is_hazard": True,
            "forced_confirmed": True,
            "observation_count": int(hazard_report.get("observation_count") or 1),
        })
    return observations, intrinsics


def associate_tracks(observations: list[dict[str, Any]]) -> list[dict[str, Any]]:
    tracks: list[dict[str, Any]] = []
    for observation in sorted(observations, key=lambda item: (item["frame"], -item["score"])):
        if observation.get("is_hazard"):
            tracks.append({
                "object_id": "obj_hazard_001", "label_zh": observation["label_zh"],
                "raw_label": observation["raw_label"], "mobility_class": "fixed",
                "is_hazard": True, "observations": [observation],
                "confirm_frame": observation["frame"],
                "observation_count_override": observation["observation_count"],
            })
            continue
        candidates: list[tuple[float, dict[str, Any]]] = []
        for track in tracks:
            if track.get("is_hazard") or track["label_zh"] != observation["label_zh"]:
                continue
            recent = track["observations"][-12:]
            center_x = float(np.median([item["xyz"][0] for item in recent]))
            center_y = float(np.median([item["xyz"][1] for item in recent]))
            distance = math.hypot(observation["xyz"][0] - center_x, observation["xyz"][1] - center_y)
            if distance <= ASSOCIATION_DISTANCE_M:
                candidates.append((distance, track))
        if candidates:
            track = min(candidates, key=lambda item: item[0])[1]
            # Multiple overlapping labels in one source frame are not multiple
            # observations of one persistent entity.
            if track["observations"][-1]["frame"] != observation["frame"]:
                track["observations"].append(observation)
                if len(track["observations"]) == CONFIRM_OBSERVATIONS:
                    track["confirm_frame"] = observation["frame"]
        else:
            tracks.append({
                "object_id": "", "label_zh": observation["label_zh"],
                "raw_label": observation["raw_label"], "mobility_class": observation["mobility_class"],
                "is_hazard": False, "observations": [observation], "confirm_frame": None,
            })

    confirmed = [track for track in tracks if track.get("confirm_frame") is not None]
    regular = sorted(
        (track for track in confirmed if not track.get("is_hazard")),
        key=lambda track: (-len(track["observations"]), track["confirm_frame"]),
    )[:MAX_VISIBLE_OBJECTS - 1]
    hazards = [track for track in confirmed if track.get("is_hazard")]
    selected = sorted([*regular, *hazards], key=lambda track: (track["confirm_frame"], track.get("is_hazard", False)))
    next_id = 1
    for track in selected:
        if not track["object_id"]:
            track["object_id"] = f"obj_{next_id:03d}"
            next_id += 1
    return selected


def track_state(track: dict[str, Any], frame: int) -> dict[str, Any] | None:
    if int(track["confirm_frame"]) > frame:
        return None
    visible = [item for item in track["observations"] if int(item["frame"]) <= frame]
    if not visible:
        return None
    x = float(np.median([item["xyz"][0] for item in visible]))
    y = float(np.median([item["xyz"][1] for item in visible]))
    z = float(np.median([item["xyz"][2] for item in visible]))
    count = int(track.get("observation_count_override") or len(visible))
    return {
        "object_id": track["object_id"], "label_zh": track["label_zh"],
        "raw_label": track["raw_label"], "x": x, "y": y, "z": z,
        "map_xyz": [x, y, z], "status": "CONFIRMED", "lifecycle_state": "active",
        "mobility_class": track["mobility_class"], "observation_count": count,
        "confidence": float(np.mean([item["score"] for item in visible])),
        "is_hazard": bool(track.get("is_hazard")),
    }


def semantic_edges(objects: list[dict[str, Any]]) -> list[dict[str, Any]]:
    candidates: list[tuple[float, dict[str, Any]]] = []
    for index, left in enumerate(objects):
        for right in objects[index + 1:]:
            distance = math.hypot(right["x"] - left["x"], right["y"] - left["y"])
            if distance > 4.0:
                continue
            relation = "near" if distance <= 1.5 else "spatial_neighbor"
            candidates.append((distance, {
                "edge_id": "", "source": left["object_id"], "target": right["object_id"],
                "relation": relation, "relation_zh": "邻近" if relation == "near" else "空间邻接",
                "distance_m": round(distance, 3), "confidence": 1.0,
            }))
    # A sparse graph reads like the WebUI and avoids a complete hairball.
    selected: list[dict[str, Any]] = []
    degree: defaultdict[str, int] = defaultdict(int)
    for _distance, edge in sorted(candidates, key=lambda item: item[0]):
        if degree[edge["source"]] >= 2 or degree[edge["target"]] >= 2:
            continue
        selected.append(edge)
        degree[edge["source"]] += 1
        degree[edge["target"]] += 1
    for index, edge in enumerate(selected, 1):
        edge["edge_id"] = f"rel_{index:03d}"
    return selected


_FONTS: dict[int, ImageFont.FreeTypeFont] = {}


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    key = -size if bold else size
    if key not in _FONTS:
        path = FONT_BOLD if bold and FONT_BOLD.is_file() else FONT_REGULAR
        _FONTS[key] = ImageFont.truetype(str(path), size)
    return _FONTS[key]


def pil_text(image: np.ndarray, rows: list[tuple[str, tuple[int, int], tuple[int, int, int], int, bool]]) -> np.ndarray:
    pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    for value, position, bgr, size, bold in rows:
        draw.text(position, value, fill=(bgr[2], bgr[1], bgr[0]), font=font(size, bold), stroke_width=1, stroke_fill=(0, 0, 0))
    return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


def occupancy_bitmap(free: np.ndarray, occupied: np.ndarray) -> np.ndarray:
    grid = np.empty((*free.shape, 3), dtype=np.uint8)
    grid[:] = (50, 46, 42)  # WebUI unknown: RGB 42,46,50.
    grid[free > 0] = (238, 238, 238)
    occupancy = np.clip(occupied.astype(np.float32) * 12.5, 0, 100)
    shades = np.rint(238 - occupancy * 2.18).astype(np.uint8)
    mask = occupied > 0
    grid[mask] = np.stack((shades[mask], shades[mask], shades[mask]), axis=1)
    return grid


def map_panel(
    free: np.ndarray, occupied: np.ndarray, path: list[tuple[float, float]],
    objects: list[dict[str, Any]], pose: tuple[float, float, float], bounds: dict[str, float],
    size: tuple[int, int],
) -> np.ndarray:
    width, height = size
    source = occupancy_bitmap(free, occupied)
    if len(path) > 1:
        points = np.asarray([cell_of(x, y, bounds)[::-1] for x, y in path], np.int32)
        cv2.polylines(source, [points], False, (42, 145, 238), 2, cv2.LINE_AA)

    scale = min((width - 20) / source.shape[1], (height - 62) / source.shape[0])
    target = cv2.resize(source, (int(source.shape[1] * scale), int(source.shape[0] * scale)), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((height, width, 3), (16, 11, 7), np.uint8)
    xoff, yoff = (width - target.shape[1]) // 2, 50 + (height - 50 - target.shape[0]) // 2
    canvas[yoff:yoff + target.shape[0], xoff:xoff + target.shape[1]] = target

    labels = [("二维语义地图 · WebUI 占据栅格", (14, 10), (248, 250, 252), 22, True)]
    for item in objects:
        row, column = cell_of(item["x"], item["y"], bounds)
        point = (xoff + int(column * scale), yoff + int(row * scale))
        color = (68, 68, 239) if item["is_hazard"] else (21, 204, 250)
        radius = 8 if item["status"] == "CONFIRMED" else 6
        cv2.circle(canvas, point, radius, color, -1, cv2.LINE_AA)
        cv2.circle(canvas, point, radius, (255, 255, 255), 1, cv2.LINE_AA)
        labels.append((item["label_zh"], (point[0] + 9, point[1] - 17), (248, 250, 252), 14, False))
    px, py, yaw = pose
    row, column = cell_of(px, py, bounds)
    center = (xoff + int(column * scale), yoff + int(row * scale))
    angle = -yaw
    triangle = np.asarray([
        (center[0] + 12 * math.cos(angle), center[1] + 12 * math.sin(angle)),
        (center[0] + 8 * math.cos(angle + 2.55), center[1] + 8 * math.sin(angle + 2.55)),
        (center[0] + 8 * math.cos(angle - 2.55), center[1] + 8 * math.sin(angle - 2.55)),
    ], np.int32)
    cv2.fillPoly(canvas, [triangle], (94, 197, 34), cv2.LINE_AA)
    return pil_text(canvas, labels)


def topology_panel(objects: list[dict[str, Any]], edges: list[dict[str, Any]], size: tuple[int, int]) -> np.ndarray:
    width, height = size
    canvas = np.full((height, width, 3), (23, 17, 13), np.uint8)  # #0d1117
    labels: list[tuple[str, tuple[int, int], tuple[int, int, int], int, bool]] = [
        ("物体拓扑 · 持久语义节点", (14, 9), (248, 250, 252), 22, True),
    ]
    if not objects:
        labels.append(("等待物体完成多帧确认…", (24, 78), (168, 148, 120), 18, False))
        return pil_text(canvas, labels)
    columns = 3 if len(objects) >= 3 else len(objects)
    rows_count = int(math.ceil(len(objects) / columns))
    card_w = min(300, int((width - 24 - max(0, columns - 1) * 16) / columns))
    card_h = 66 if height < 500 else 100
    gap_x = (width - 24 - columns * card_w) / max(1, columns - 1) if columns > 1 else 0
    row_step = card_h + 14
    if rows_count > 1:
        row_step = min(row_step, (height - 48 - card_h - 8) / (rows_count - 1))
    positions: dict[str, tuple[int, int]] = {}
    for index, item in enumerate(objects):
        row, column = divmod(index, columns)
        x = int(12 + column * (card_w + gap_x))
        y = int(48 + row * row_step)
        positions[item["object_id"]] = (x + card_w // 2, y + card_h // 2)
    edge_by_pair = {(edge["source"], edge["target"]): edge for edge in edges}
    for (source, target), edge in edge_by_pair.items():
        if source not in positions or target not in positions:
            continue
        p1, p2 = positions[source], positions[target]
        cv2.line(canvas, p1, p2, (238, 211, 34), 2, cv2.LINE_AA)
        midpoint = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
        labels.append((edge["relation_zh"], (midpoint[0] + 3, midpoint[1] - 18), (238, 211, 34), 12, False))
    for item in objects:
        cx, cy = positions[item["object_id"]]
        x, y = cx - card_w // 2, cy - card_h // 2
        accent = (113, 113, 248) if item["is_hazard"] else ((60, 146, 251) if item["mobility_class"] == "semi_static" else (153, 217, 52))
        cv2.rectangle(canvas, (x, y), (x + card_w, y + card_h), (36, 27, 20), -1, cv2.LINE_AA)
        cv2.rectangle(canvas, (x, y), (x + card_w, y + card_h), accent, 2, cv2.LINE_AA)
        cv2.circle(canvas, (x + 13, y + 16), 5, accent, -1, cv2.LINE_AA)
        visible_label = item["label_zh"]
        if card_w < 250 and item["is_hazard"]:
            visible_label = "管道液体渗漏危险源"
        labels.extend([
            (item["object_id"], (x + 23, y + 4), accent, 13, True),
            (visible_label, (x + 8, y + 24), (248, 250, 252), 14, True),
            (f"固定物体 · 已确认 · {item['observation_count']}次", (x + 8, y + card_h - 22), (179, 163, 148), 10, False),
        ])
    return pil_text(canvas, labels)


def status_panel(
    frame: int, total: int, elapsed: float, duration: float, objects: list[dict[str, Any]],
    hazard_confirmed: bool, cloud_index: int,
) -> np.ndarray:
    canvas = np.full((360, 1920, 3), (27, 20, 17), np.uint8)
    cv2.line(canvas, (0, 0), (1920, 0), (60, 48, 40), 2)
    danger = (68, 68, 239) if hazard_confirmed else (100, 210, 100)
    state = "已确认：模拟管道液体渗漏危险源" if hazard_confirmed else "YOLOE-PF 无提示词模式持续监测"
    rows = [
        ("遥操建图 rosbag · WebUI 语义回放", (30, 24), (248, 250, 252), 34, True),
        (f"时间 {elapsed:6.1f}/{duration:.1f} 秒   帧 {frame}/{total}   雷达 {cloud_index} 帧", (30, 78), (218, 210, 200), 23, False),
        (f"持久物体节点 {len(objects)} 个（多帧确认后写入）", (30, 120), (218, 210, 200), 23, False),
        (state, (30, 174), danger, 30, True),
        ("地图：灰=未知 白=自由 黑=占据 黄=物体 红=危险源  绿色箭头=机器狗", (30, 232), (178, 163, 148), 19, False),
        ("拓扑：只显示物体节点及空间关系；坐标来自 RGB-D + 相对融合里程计", (30, 270), (178, 163, 148), 19, False),
    ]
    if objects:
        names = "、".join(item["label_zh"] for item in objects)
        rows.append((f"当前语义实体：{names}", (760, 82), (248, 250, 252), 20, False))
    return pil_text(canvas, rows)


def write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> int:
    args = parse_args()
    bag = args.bag.resolve()
    replay_dir = args.replay_dir.resolve()
    output_video = (args.output or replay_dir / "mapping_process_webui_semantic.mp4").resolve()
    records = load_records(replay_dir / "detections.jsonl")
    hazard_report = json.loads((replay_dir / "hazard_report.json").read_text(encoding="utf-8"))
    summary = first_pass(bag)
    observations, _intrinsics = collect_observations(bag, records, hazard_report)
    tracks = associate_tracks(observations)
    print(f"localized_observations={len(observations)} confirmed_tracks={len(tracks)}", flush=True)

    xs, ys = [row[1] for row in summary["odom"]], [row[2] for row in summary["odom"]]
    final_track_states = [state for track in tracks if (state := track_state(track, len(records))) is not None]
    object_xs = [item["x"] for item in final_track_states]
    object_ys = [item["y"] for item in final_track_states]
    padding, resolution = 7.0, float(args.map_resolution)
    bounds = {
        "xmin": math.floor(min(min(xs) - padding, min(object_xs, default=min(xs)) - 2.0) / resolution) * resolution,
        "xmax": math.ceil(max(max(xs) + padding, max(object_xs, default=max(xs)) + 2.0) / resolution) * resolution,
        "ymin": math.floor(min(min(ys) - padding, min(object_ys, default=min(ys)) - 2.0) / resolution) * resolution,
        "ymax": math.ceil(max(max(ys) + padding, max(object_ys, default=max(ys)) + 2.0) / resolution) * resolution,
        "resolution": resolution,
    }
    grid_width = int(round((bounds["xmax"] - bounds["xmin"]) / resolution)) + 1
    grid_height = int(round((bounds["ymax"] - bounds["ymin"]) / resolution)) + 1
    free = np.zeros((grid_height, grid_width), dtype=np.uint8)
    occupied = np.zeros((grid_height, grid_width), dtype=np.uint16)
    lidar_config, _mapping_config = load_geometry()

    camera_video = replay_dir / "yoloe_prompt_free_video.mp4"
    capture = cv2.VideoCapture(str(camera_video))
    fps = float(capture.get(cv2.CAP_PROP_FPS) or summary["fps"])
    raw_video = output_video.with_name(output_video.stem + "_raw.mp4")
    writer = cv2.VideoWriter(str(raw_video), cv2.VideoWriter_fourcc(*"mp4v"), fps, (1920, 1080))
    if not capture.isOpened() or not writer.isOpened():
        raise RuntimeError("failed to open source/output video")

    reader, types = open_reader(bag)
    odom = summary["odom"]
    odom_stamps = [row[0] for row in odom]
    path: list[tuple[float, float]] = []
    current_pose = (0.0, 0.0, 0.0)
    frame_index = 0
    cloud_index = 0
    last_objects: list[dict[str, Any]] = []
    last_edges: list[dict[str, Any]] = []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        stamp = stamp_ns / 1e9
        index = bisect.bisect_right(odom_stamps, stamp) - 1
        if index >= 0:
            current_pose = (odom[index][1], odom[index][2], odom[index][3])
        if topic == CLOUD_TOPIC:
            msg = deserialize_message(data, get_message(types[topic]))
            update_grid(free, occupied, lidar_points(msg, lidar_config), current_pose, bounds)
            cloud_index += 1
            continue
        if topic != COLOR_TOPIC:
            continue
        frame_index += 1
        if frame_index > len(records):
            break
        ok, camera = capture.read()
        if not ok:
            raise RuntimeError(f"annotated camera video ended at frame {frame_index}")
        camera = cv2.resize(camera, (1280, 720), interpolation=cv2.INTER_AREA)
        record = records[frame_index - 1]
        current_pose = tuple(float(value) for value in record["pose"])
        if not path or math.hypot(current_pose[0] - path[-1][0], current_pose[1] - path[-1][1]) >= 0.08:
            path.append((current_pose[0], current_pose[1]))
        objects = [state for track in tracks if (state := track_state(track, frame_index)) is not None]
        edges = semantic_edges(objects)
        hazard_confirmed = any(item["is_hazard"] for item in objects)
        map_view = map_panel(free, occupied, path, objects, current_pose, bounds, (640, 360))
        topology_view = topology_panel(objects, edges, (640, 360))
        canvas = np.full((1080, 1920, 3), (23, 17, 13), np.uint8)
        canvas[:720, :1280] = camera
        canvas[:360, 1280:] = map_view
        canvas[360:720, 1280:] = topology_view
        canvas[720:, :] = status_panel(
            frame_index, len(records), float(record["elapsed_s"]), float(summary["duration_s"]),
            objects, hazard_confirmed, cloud_index,
        )
        writer.write(canvas)
        last_objects, last_edges = objects, edges
        if frame_index % 100 == 0 or frame_index == len(records):
            print(f"render frame={frame_index}/{len(records)} objects={len(objects)} clouds={cloud_index}", flush=True)
    capture.release()
    writer.release()

    subprocess.run([
        "ffmpeg", "-y", "-loglevel", "error", "-i", str(raw_video),
        "-c:v", "libx264", "-preset", "medium", "-crf", "20",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output_video),
    ], check=True)
    raw_video.unlink()

    final_map = map_panel(free, occupied, path, last_objects, current_pose, bounds, (1600, 1000))
    final_topology = topology_panel(last_objects, last_edges, (1600, 1000))
    map_png = replay_dir / "webui_semantic_map.png"
    topology_png = replay_dir / "webui_object_topology.png"
    cv2.imwrite(str(map_png), final_map)
    cv2.imwrite(str(topology_png), final_topology)

    object_snapshot = {
        "schema": "semantic_object_snapshot_v1", "frame_id": "relative_fused_odom_start",
        "map_epoch": "rosbag_replay", "objects": last_objects,
    }
    topology_snapshot = {
        "schema": "semantic_object_topology_v1", "frame_id": "relative_fused_odom_start",
        "map_epoch": "rosbag_replay", "nodes": last_objects, "edges": last_edges,
    }
    write_json(replay_dir / "webui_semantic_objects.json", object_snapshot)
    write_json(replay_dir / "webui_object_topology.json", topology_snapshot)
    write_json(replay_dir / "webui_replay_manifest.json", {
        "video": output_video.name, "map": map_png.name, "topology": topology_png.name,
        "objects": len(last_objects), "edges": len(last_edges),
        "hazard_confirmed": any(item["is_hazard"] for item in last_objects),
        "coordinate_frame": "relative_fused_odom_start",
        "projection_quality": "nominal_d435_extrinsic",
        "safety": "read-only bag post-processing; no ROS publishers or robot commands",
    })
    print(json.dumps({
        "ok": True, "video": str(output_video), "objects": len(last_objects),
        "hazard": any(item["is_hazard"] for item in last_objects),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
