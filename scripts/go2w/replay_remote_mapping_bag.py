#!/usr/bin/env python3
"""Read-only remote-mapping replay for a recorded Go2-W rosbag2 session.

The script never publishes ROS topics.  It reads camera, depth, fused odometry
and Unitree LiDAR messages directly from sqlite3, runs the production
prompt-free YOLOE worker, applies the production slime/leak enhancer, and
renders two auditable videos plus final 2-D/topological artifacts.

Run with the ROS 2 system Python after sourcing Humble.  The YOLOE process is
started from the repository's isolated ``.yoloe_env`` automatically.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import subprocess
import sys
import time
from collections import Counter, deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
from PIL import Image, ImageDraw, ImageFont

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.detectors.base import DetectedObject  # noqa: E402
from app.detectors.vocabulary import resolve_display_label  # noqa: E402
from app.perception.pipe_context_tracker import PipeContextTracker  # noqa: E402
from app.perception.slime_leak_enhancer import (  # noqa: E402
    enhance_slime_leak_detections,
    select_low_confidence_pipe_contexts,
)


COLOR_TOPIC = "/go2w/d435/color/image_raw"
DEPTH_TOPIC = "/go2w/d435/depth/image_rect_raw"
INFO_TOPIC = "/go2w/d435/color/camera_info"
ODOM_TOPIC = "/go2w/odom/fused"
CLOUD_TOPIC = "/utlidar/cloud"
FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--bag", required=True, type=Path)
    result.add_argument("--output", required=True, type=Path)
    result.add_argument("--model", type=Path, default=PROJECT_ROOT / "models/yoloe-26s-seg-pf.pt")
    result.add_argument("--yoloe-python", type=Path, default=PROJECT_ROOT / ".yoloe_env/bin/python")
    result.add_argument("--confidence", type=float, default=0.12)
    result.add_argument("--image-size", type=int, default=960)
    result.add_argument("--map-resolution", type=float, default=0.08)
    return result


def open_reader(path: Path) -> tuple[Any, dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {COLOR_TOPIC, DEPTH_TOPIC, INFO_TOPIC, ODOM_TOPIC, CLOUD_TOPIC}
    missing = sorted(required.difference(types))
    if missing:
        raise RuntimeError(f"bag missing required topics: {missing}")
    return reader, types


def yaw_of(orientation: Any) -> float:
    q = orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def first_pass(bag: Path) -> dict[str, Any]:
    reader, types = open_reader(bag)
    odom: list[tuple[float, float, float, float]] = []
    camera_stamps: list[float] = []
    counts = Counter()
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == ODOM_TOPIC:
            msg = deserialize_message(data, get_message(types[topic]))
            p = msg.pose.pose.position
            odom.append((stamp_ns / 1e9, float(p.x), float(p.y), yaw_of(msg.pose.pose.orientation)))
            counts["odom"] += 1
        elif topic == COLOR_TOPIC:
            camera_stamps.append(stamp_ns / 1e9)
            counts["color"] += 1
        elif topic == DEPTH_TOPIC:
            counts["depth"] += 1
        elif topic == CLOUD_TOPIC:
            counts["cloud"] += 1
    if not odom or len(camera_stamps) < 2:
        raise RuntimeError("bag does not contain enough odometry/camera data")
    t0, x0, y0, yaw0 = odom[0]
    cosine, sine = math.cos(-yaw0), math.sin(-yaw0)
    relative = []
    for stamp, x, y, yaw in odom:
        dx, dy = x - x0, y - y0
        relative.append((stamp, cosine * dx - sine * dy, sine * dx + cosine * dy, wrap(yaw - yaw0)))
    camera_fps = (len(camera_stamps) - 1) / max(1e-6, camera_stamps[-1] - camera_stamps[0])
    return {
        "counts": dict(counts),
        "odom": relative,
        "camera_stamps": camera_stamps,
        "fps": camera_fps,
        "duration_s": camera_stamps[-1] - camera_stamps[0],
        "start_s": camera_stamps[0],
        "end_s": camera_stamps[-1],
    }


def build_topology(odom: list[tuple[float, float, float, float]]) -> list[dict[str, Any]]:
    nodes: list[dict[str, Any]] = []
    for stamp, x, y, yaw in odom:
        if not nodes:
            nodes.append({"id": "P1", "stamp": stamp, "x": x, "y": y, "yaw": yaw, "objects": Counter()})
            continue
        previous = nodes[-1]
        distance = math.hypot(x - previous["x"], y - previous["y"])
        heading = abs(wrap(yaw - previous["yaw"]))
        if distance >= 8.0 or heading >= math.radians(35.0):
            nodes.append({"id": f"P{len(nodes) + 1}", "stamp": stamp, "x": x, "y": y, "yaw": yaw, "objects": Counter()})
    stamp, x, y, yaw = odom[-1]
    if math.hypot(x - nodes[-1]["x"], y - nodes[-1]["y"]) >= 2.0:
        nodes.append({"id": f"P{len(nodes) + 1}", "stamp": stamp, "x": x, "y": y, "yaw": yaw, "objects": Counter()})
    return nodes


class YoloeWorker:
    def __init__(self, python: Path, model: Path, confidence: float, image_size: int, log: Path) -> None:
        self.log_handle = log.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            [
                str(python), str(PROJECT_ROOT / "app/detectors/yoloe_seg_worker.py"),
                "--serve", "--model", str(model), "--device", "cuda:0",
                "--precision", "fp16", "--confidence", str(confidence),
                "--image-size", str(image_size), "--max-detections", "100",
            ],
            cwd=str(PROJECT_ROOT), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self.log_handle, text=True, bufsize=1,
        )
        self.confidence = confidence
        self.image_size = image_size

    def detect(self, image: Path, frame_id: str) -> dict[str, Any]:
        if self.process.stdin is None or self.process.stdout is None:
            raise RuntimeError("YOLOE pipes are unavailable")
        request = {
            "operation": "detect", "request_id": frame_id, "frame_id": frame_id,
            "image": str(image), "confidence": self.confidence,
            "image_size": self.image_size, "max_detections": 100,
        }
        self.process.stdin.write(json.dumps(request, ensure_ascii=False) + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError(f"YOLOE exited unexpectedly: {self.process.poll()}")
        payload = json.loads(line)
        if payload.get("error"):
            raise RuntimeError(str(payload["error"]))
        if payload.get("request_id") != frame_id:
            raise RuntimeError("YOLOE request/response id mismatch")
        return payload

    def close(self) -> None:
        try:
            if self.process.poll() is None and self.process.stdin and self.process.stdout:
                self.process.stdin.write('{"operation":"shutdown","request_id":"shutdown"}\n')
                self.process.stdin.flush()
                self.process.stdout.readline()
                self.process.wait(timeout=10)
        except Exception:
            self.process.terminate()
        finally:
            self.log_handle.close()


def detected_object(item: dict[str, Any]) -> DetectedObject:
    size = item.get("mask_size")
    return DetectedObject(
        label=str(item.get("label") or "object"),
        label_zh=display_name(str(item.get("label") or "object")),
        category="object", bbox_2d=tuple(float(v) for v in item["bbox_2d"]),
        score=float(item.get("score") or 0.0),
        mask_rle=item.get("mask_rle"),
        mask_size=tuple(int(v) for v in size) if size else None,
        mask_area_ratio=item.get("mask_area_ratio"),
        mask_source="yoloe_seg_mask" if item.get("mask_rle") else None,
        source="yoloe_seg", source_prompt_term=str(item.get("label") or "object"),
        detection_slots=("scene",),
    )


def display_name(label: str) -> str:
    overrides = {
        "leak": "泄漏", "soap bubble": "泡状物", "hot spring": "积液/水面",
        "pipe bowl": "管口", "biology laboratory": "建筑区域", "beam": "横梁",
        "fume hood": "设施区域", "brickwork": "砖墙", "bin": "容器",
    }
    if label.casefold() in overrides:
        return overrides[label.casefold()]
    translated = resolve_display_label(label)
    return label if translated == "未分类物体" else translated


def box_area(item: DetectedObject) -> float:
    x1, y1, x2, y2 = item.bbox_2d
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def load_geometry() -> tuple[dict[str, Any], dict[str, Any]]:
    lidar = yaml.safe_load((PROJECT_ROOT / "configs/go2w/lidar_preprocess.yaml").read_text(encoding="utf-8"))
    mapping = yaml.safe_load((PROJECT_ROOT / "configs/go2w/semantic_mapping.yaml").read_text(encoding="utf-8"))
    return lidar, mapping


def lidar_points(msg: Any, lidar: dict[str, Any]) -> np.ndarray:
    raw = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float32)
    pitch = -0.263392653559
    cosine, sine = math.cos(pitch), math.sin(pitch)
    rotation = np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]], dtype=np.float32)
    points = points @ rotation.T + np.array([0.28945, 0.0, -0.046825], dtype=np.float32)
    ranges = np.linalg.norm(points, axis=1)
    mask = (
        np.isfinite(points).all(axis=1)
        & (ranges >= float(lidar["range_m"]["minimum"]))
        & (ranges <= float(lidar["range_m"]["maximum"]))
        & (points[:, 2] > float(lidar["ground_separation_height_m"]))
        & (points[:, 2] <= float(lidar["collision_height_m"]["maximum"]))
    )
    # Base footprint and reviewed body/mount masks match the live preprocessor.
    half_length, half_width = 0.35, 0.215
    margin = float(lidar["self_filter_margin_m"])
    self_mask = (np.abs(points[:, 0]) <= half_length + margin) & (np.abs(points[:, 1]) <= half_width + margin)
    for region in lidar.get("self_regions") or []:
        self_mask |= (
            (points[:, 0] >= float(region["x_min"])) & (points[:, 0] <= float(region["x_max"]))
            & (points[:, 1] >= float(region["y_min"])) & (points[:, 1] <= float(region["y_max"]))
            & (points[:, 2] >= float(region["z_min"])) & (points[:, 2] <= float(region["z_max"]))
        )
    return points[mask & ~self_mask]


def cell_of(x: float, y: float, bounds: dict[str, float]) -> tuple[int, int]:
    column = int(round((x - bounds["xmin"]) / bounds["resolution"]))
    row = int(round((bounds["ymax"] - y) / bounds["resolution"]))
    return row, column


def update_grid(
    free: np.ndarray, occupied: np.ndarray, points: np.ndarray,
    pose: tuple[float, float, float], bounds: dict[str, float],
) -> None:
    if not len(points):
        return
    px, py, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    world_x = px + cosine * points[:, 0] - sine * points[:, 1]
    world_y = py + sine * points[:, 0] + cosine * points[:, 1]
    angles = np.arctan2(points[:, 1], points[:, 0])
    ranges = np.hypot(points[:, 0], points[:, 1])
    bins = np.floor((angles + math.pi) / (2.0 * math.pi) * 180.0).astype(np.int32)
    bins = np.clip(bins, 0, 179)
    order = np.argsort(ranges)
    _, first = np.unique(bins[order], return_index=True)
    chosen = order[first]
    start = cell_of(px, py, bounds)
    height, width = free.shape
    for index in chosen:
        end = cell_of(float(world_x[index]), float(world_y[index]), bounds)
        if 0 <= end[0] < height and 0 <= end[1] < width:
            cv2.line(free, (start[1], start[0]), (end[1], end[0]), 1, 1, cv2.LINE_8)
    rows = np.rint((bounds["ymax"] - world_y) / bounds["resolution"]).astype(np.int32)
    cols = np.rint((world_x - bounds["xmin"]) / bounds["resolution"]).astype(np.int32)
    valid = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    np.add.at(occupied, (rows[valid], cols[valid]), 1)


def load_fonts() -> dict[str, Any]:
    regular = FONT_REGULAR if FONT_REGULAR.is_file() else FONT_BOLD
    bold = FONT_BOLD if FONT_BOLD.is_file() else regular
    return {
        "small": ImageFont.truetype(str(regular), 22),
        "body": ImageFont.truetype(str(regular), 28),
        "bold": ImageFont.truetype(str(bold), 32),
        "title": ImageFont.truetype(str(bold), 38),
    }


def put_text(image: np.ndarray, rows: list[tuple[str, tuple[int, int], tuple[int, int, int], str]]) -> np.ndarray:
    fonts = load_fonts_cached()
    pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    for text, position, color, font_key in rows:
        draw.text(position, text, fill=(color[2], color[1], color[0]), font=fonts[font_key], stroke_width=1, stroke_fill=(0, 0, 0))
    return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


_FONTS: dict[str, Any] | None = None


def load_fonts_cached() -> dict[str, Any]:
    global _FONTS
    if _FONTS is None:
        _FONTS = load_fonts()
    return _FONTS


def annotate_camera(image: np.ndarray, objects: list[DetectedObject], hazards: list[DetectedObject], confirmed: bool) -> np.ndarray:
    output = image.copy()
    height, width = output.shape[:2]
    labels: list[tuple[str, tuple[int, int], tuple[int, int, int], str]] = []
    shown = sorted((item for item in objects if item.score >= 0.25 and box_area(item) <= 0.80), key=lambda item: item.score, reverse=True)[:12]
    for index, item in enumerate(shown):
        x1, y1, x2, y2 = item.bbox_2d
        p1 = (int(x1 * width), int(y1 * height))
        p2 = (int(x2 * width), int(y2 * height))
        color = (70, 210, 255) if item.label not in {"leak", "pipe bowl", "water pipe", "pipe"} else (40, 190, 255)
        cv2.rectangle(output, p1, p2, color, 2, cv2.LINE_AA)
        labels.append((f"{display_name(item.label)}  {item.score:.2f}", (p1[0] + 3, max(2, p1[1] - 30)), color, "small"))
    for hazard in hazards:
        x1, y1, x2, y2 = hazard.bbox_2d
        p1 = (int(x1 * width), int(y1 * height))
        p2 = (int(x2 * width), int(y2 * height))
        color = (0, 0, 255) if confirmed else (0, 140, 255)
        cv2.rectangle(output, p1, p2, color, 5, cv2.LINE_AA)
        labels.append((("危险源：模拟管道液体渗漏" if confirmed else "疑似模拟管道渗漏"), (p1[0], max(2, p1[1] - 42)), color, "bold"))
    overlay = output.copy()
    cv2.rectangle(overlay, (0, 0), (width, 56), (10, 14, 22), -1)
    output = cv2.addWeighted(overlay, 0.78, output, 0.22, 0)
    labels.append(("YOLOE-PF 无提示词模式 · 实时实例分割检测", (20, 8), (255, 255, 255), "bold"))
    return put_text(output, labels)


def map_image(
    free: np.ndarray, occupied: np.ndarray, path: list[tuple[float, float]],
    nodes: list[dict[str, Any]], current: tuple[float, float, float],
    bounds: dict[str, float], size: tuple[int, int], hazard_xy: tuple[float, float] | None,
) -> np.ndarray:
    grid = np.full((*free.shape, 3), (55, 58, 64), dtype=np.uint8)
    grid[free > 0] = (235, 235, 235)
    grid[occupied >= 2] = (18, 18, 18)
    if len(path) > 1:
        points = [cell_of(x, y, bounds)[::-1] for x, y in path]
        cv2.polylines(grid, [np.asarray(points, np.int32)], False, (255, 150, 45), 2, cv2.LINE_AA)
    for first, second in zip(nodes, nodes[1:]):
        p1 = cell_of(first["x"], first["y"], bounds)[::-1]
        p2 = cell_of(second["x"], second["y"], bounds)[::-1]
        cv2.line(grid, p1, p2, (190, 80, 210), 2, cv2.LINE_AA)
    for node in nodes:
        column, row = cell_of(node["x"], node["y"], bounds)[::-1]
        cv2.circle(grid, (column, row), 6, (190, 80, 210), -1, cv2.LINE_AA)
        cv2.putText(grid, node["id"], (column + 7, row - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 30, 145), 1, cv2.LINE_AA)
    if hazard_xy is not None:
        column, row = cell_of(hazard_xy[0], hazard_xy[1], bounds)[::-1]
        cv2.drawMarker(grid, (column, row), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 3, cv2.LINE_AA)
    px, py, yaw = current
    column, row = cell_of(px, py, bounds)[::-1]
    tip = (int(column + 13 * math.cos(yaw)), int(row - 13 * math.sin(yaw)))
    cv2.circle(grid, (column, row), 7, (30, 190, 60), -1, cv2.LINE_AA)
    cv2.line(grid, (column, row), tip, (30, 90, 20), 3, cv2.LINE_AA)
    target_w, target_h = size
    scale = min(target_w / grid.shape[1], target_h / grid.shape[0])
    resized = cv2.resize(grid, (max(1, int(grid.shape[1] * scale)), max(1, int(grid.shape[0] * scale))), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((target_h, target_w, 3), (22, 25, 31), dtype=np.uint8)
    xoff, yoff = (target_w - resized.shape[1]) // 2, (target_h - resized.shape[0]) // 2
    canvas[yoff:yoff + resized.shape[0], xoff:xoff + resized.shape[1]] = resized
    return canvas


def topology_graph_image(nodes: list[dict[str, Any]], hazard_confirmed: bool) -> np.ndarray:
    """Render a readable standalone relation graph instead of metric overlap."""
    width, height = 1800, 1000
    canvas = np.full((height, width, 3), (24, 27, 34), dtype=np.uint8)
    positions: list[tuple[int, int]] = []
    columns = min(5, max(1, len(nodes)))
    for index in range(len(nodes)):
        row, column = divmod(index, columns)
        if row % 2:
            column = columns - 1 - column
        positions.append((170 + column * 355, 270 + row * 410))
    for left, right in zip(positions, positions[1:]):
        cv2.arrowedLine(canvas, left, right, (100, 170, 245), 5, cv2.LINE_AA, tipLength=0.035)
    text_rows: list[tuple[str, tuple[int, int], tuple[int, int, int], str]] = [
        ("遥操建图最终拓扑图", (42, 28), (255, 255, 255), "title"),
        ("节点按真实行驶顺序连接；每个节点列出高频视觉观测", (42, 82), (190, 200, 215), "body"),
    ]
    for index, (node, position) in enumerate(zip(nodes, positions)):
        hazard = bool(hazard_confirmed and index == len(nodes) - 1)
        color = (0, 50, 245) if hazard else (205, 80, 205)
        cv2.circle(canvas, position, 48, color, -1, cv2.LINE_AA)
        cv2.circle(canvas, position, 52, (245, 245, 245), 3, cv2.LINE_AA)
        text_rows.append((node["id"], (position[0] - 28, position[1] - 24), (255, 255, 255), "bold"))
        observations = node["objects"].most_common(3)
        for offset, (name, count) in enumerate(observations):
            text_rows.append((f"{name} ×{count}", (position[0] - 120, position[1] + 72 + offset * 32), (205, 212, 224), "small"))
        if hazard:
            text_rows.append(("危险源：模拟管道液体渗漏", (max(30, position[0] - 140), position[1] - 105), (0, 60, 255), "bold"))
    return put_text(canvas, text_rows)


def approximate_hazard_xy(
    hazard: DetectedObject, depth: np.ndarray | None, intrinsics: tuple[float, float, float, float] | None,
    pose: tuple[float, float, float],
) -> tuple[float, float] | None:
    if depth is None or intrinsics is None:
        return None
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = hazard.bbox_2d
    xa, xb = max(0, int(x1 * width)), min(width, int(math.ceil(x2 * width)))
    ya, yb = max(0, int(y1 * height)), min(height, int(math.ceil(y2 * height)))
    values = depth[ya:yb, xa:xb]
    values = values[np.isfinite(values) & (values > 0.15) & (values < 8.0)]
    if not len(values):
        return None
    z = float(np.median(values))
    fx, fy, cx, cy = intrinsics
    u, v = 0.5 * (xa + xb), 0.5 * (ya + yb)
    camera_x = (u - cx) * z / fx
    # Nominal forward-camera transform used by the mapping stack.  This is
    # explicitly approximate because the repository has no validated D435 TF.
    base_x, base_y = z + 0.10, -camera_x
    px, py, yaw = pose
    return (
        px + math.cos(yaw) * base_x - math.sin(yaw) * base_y,
        py + math.sin(yaw) * base_x + math.cos(yaw) * base_y,
    )


def ffmpeg_encode(raw: Path, final: Path) -> None:
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-i", str(raw), "-c:v", "libx264",
         "-preset", "medium", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(final)],
        check=True,
    )
    raw.unlink()


def main() -> int:
    args = parser().parse_args()
    started = time.time()
    bag = args.bag.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not args.model.is_file() or not args.yoloe_python.is_file():
        raise SystemExit("YOLOE model/python missing")
    summary = first_pass(bag)
    odom = summary["odom"]
    nodes = build_topology(odom)
    xs, ys = [row[1] for row in odom], [row[2] for row in odom]
    padding = 7.0
    resolution = float(args.map_resolution)
    bounds = {
        "xmin": math.floor((min(xs) - padding) / resolution) * resolution,
        "xmax": math.ceil((max(xs) + padding) / resolution) * resolution,
        "ymin": math.floor((min(ys) - padding) / resolution) * resolution,
        "ymax": math.ceil((max(ys) + padding) / resolution) * resolution,
        "resolution": resolution,
    }
    width = int(round((bounds["xmax"] - bounds["xmin"]) / resolution)) + 1
    height = int(round((bounds["ymax"] - bounds["ymin"]) / resolution)) + 1
    free = np.zeros((height, width), dtype=np.uint8)
    occupied = np.zeros((height, width), dtype=np.uint16)
    lidar_config, mapping_config = load_geometry()
    visual_config = mapping_config["hazard"]["visual_enhancement"]
    pipe_terms = {str(value).casefold() for value in visual_config["pipe_labels"]}
    tracker = PipeContextTracker(
        max_age_s=float(visual_config.get("pipe_context_max_age_s", 8.0)),
        min_quality=float(visual_config.get("pipe_tracking_min_quality", 0.65)),
    )

    fps = float(summary["fps"])
    annotated_raw = output / "yoloe_prompt_free_raw.mp4"
    ui_raw = output / "mapping_process_ui_raw.mp4"
    annotated_writer = cv2.VideoWriter(str(annotated_raw), cv2.VideoWriter_fourcc(*"mp4v"), fps, (1280, 720))
    ui_writer = cv2.VideoWriter(str(ui_raw), cv2.VideoWriter_fourcc(*"mp4v"), fps, (1920, 1080))
    if not annotated_writer.isOpened() or not ui_writer.isOpened():
        raise RuntimeError("failed to open video writers")

    worker = YoloeWorker(args.yoloe_python.resolve(), args.model.resolve(), args.confidence, args.image_size, output / "yoloe_worker.stderr.log")
    reader, types = open_reader(bag)
    bridge = CvBridge()
    current_pose = (0.0, 0.0, 0.0)
    path: list[tuple[float, float]] = []
    latest_depth: np.ndarray | None = None
    intrinsics: tuple[float, float, float, float] | None = None
    image_path = output / ".current_frame.jpg"
    detections_path = (output / "detections.jsonl").open("w", encoding="utf-8")
    hazard_support: deque[tuple[int, float, tuple[float, float, float, float]]] = deque(maxlen=3)
    hazard_confirmed = False
    hazard_first_frame: int | None = None
    hazard_confirm_frame: int | None = None
    hazard_xy: tuple[float, float] | None = None
    hazard_observations: list[dict[str, Any]] = []
    frame_index = 0
    cloud_index = 0
    node_stamps = [node["stamp"] for node in nodes]
    last_progress = time.time()
    try:
        while reader.has_next():
            topic, data, stamp_ns = reader.read_next()
            stamp = stamp_ns / 1e9
            if topic == ODOM_TOPIC:
                msg = deserialize_message(data, get_message(types[topic]))
                # Interpolate through the already normalized odometry timeline
                # by storage timestamp; this avoids duplicating frame math.
                index = bisect.bisect_right([row[0] for row in odom], stamp) - 1
                if index >= 0:
                    current_pose = (odom[index][1], odom[index][2], odom[index][3])
                    if not path or math.hypot(current_pose[0] - path[-1][0], current_pose[1] - path[-1][1]) >= 0.08:
                        path.append((current_pose[0], current_pose[1]))
            elif topic == CLOUD_TOPIC:
                msg = deserialize_message(data, get_message(types[topic]))
                update_grid(free, occupied, lidar_points(msg, lidar_config), current_pose, bounds)
                cloud_index += 1
            elif topic == DEPTH_TOPIC:
                msg = deserialize_message(data, get_message(types[topic]))
                latest_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
                if str(msg.encoding).upper() in {"16UC1", "MONO16"}:
                    latest_depth *= 0.001
            elif topic == INFO_TOPIC:
                msg = deserialize_message(data, get_message(types[topic]))
                intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))
            elif topic != COLOR_TOPIC:
                continue
            else:
                frame_index += 1
                msg = deserialize_message(data, get_message(types[topic]))
                image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                cv2.imwrite(str(image_path), image, [cv2.IMWRITE_JPEG_QUALITY, 94])
                payload = worker.detect(image_path, f"frame_{frame_index:04d}")
                objects = [detected_object(item) for item in payload.get("objects") or []]
                persistent = [item for item in objects if item.score >= 0.35 and box_area(item) <= 0.65]
                context_only = select_low_confidence_pipe_contexts(objects, visual_config, persistent_min_confidence=0.35)
                tracked = tracker.contexts(
                    str(image_path), [*persistent, *context_only], timestamp=stamp,
                    pipe_terms=pipe_terms,
                )
                enhanced = enhance_slime_leak_detections(
                    str(image_path), persistent, visual_config,
                    context_detections=context_only, tracked_pipe_contexts=tracked,
                    display_language="zh",
                )
                hazards = [item for item in enhanced if item.label == "simulated liquid leak" and item.hazard_state == "confirmed_demo_candidate"]
                if hazards:
                    if hazard_first_frame is None:
                        hazard_first_frame = frame_index
                    best = max(hazards, key=lambda item: item.score)
                    hazard_support.append((frame_index, stamp, best.bbox_2d))
                    hazard_observations.append({
                        "frame": frame_index, "elapsed_s": stamp - summary["start_s"],
                        "score": best.score, "bbox_2d": list(best.bbox_2d),
                        "context_quality": best.context_quality,
                    })
                    if len(hazard_support) >= 2 and hazard_support[-1][0] - hazard_support[-2][0] <= 2 and hazard_support[-1][1] - hazard_support[-2][1] <= 12.0:
                        if not hazard_confirmed:
                            hazard_confirmed = True
                            hazard_confirm_frame = frame_index
                            located = approximate_hazard_xy(best, latest_depth, intrinsics, current_pose)
                            if located is not None:
                                hazard_xy = located
                        confirmed_view = annotate_camera(image, objects, hazards, True)
                        if not (output / "hazard_confirmed.jpg").exists():
                            cv2.imwrite(str(output / "hazard_confirmed.jpg"), confirmed_view)
                else:
                    # Evidence window is frame-bounded; non-hit frames remain
                    # explicit negatives and prevent distant hits confirming.
                    while hazard_support and frame_index - hazard_support[0][0] > 2:
                        hazard_support.popleft()

                node_index = max(0, bisect.bisect_right(node_stamps, stamp) - 1)
                for item in persistent:
                    nodes[node_index]["objects"][display_name(item.label)] += 1
                compact = {
                    "frame": frame_index, "stamp": stamp,
                    "elapsed_s": round(stamp - summary["start_s"], 3),
                    "pose": [round(value, 4) for value in current_pose],
                    "objects": [
                        {"label": item.label, "name": display_name(item.label), "score": round(item.score, 4), "bbox_2d": [round(v, 6) for v in item.bbox_2d]}
                        for item in objects
                    ],
                    "hazard": bool(hazards), "hazard_confirmed": hazard_confirmed,
                    "inference_latency_ms": payload.get("diagnostics", {}).get("inference_latency_ms"),
                }
                detections_path.write(json.dumps(compact, ensure_ascii=False) + "\n")

                annotated = annotate_camera(image, objects, hazards, hazard_confirmed)
                annotated_writer.write(annotated)
                canvas = np.full((1080, 1920, 3), (17, 20, 27), dtype=np.uint8)
                canvas[:720, :1280] = annotated
                active_nodes = [node for node in nodes if node["stamp"] <= stamp]
                map_panel = map_image(free, occupied, path, active_nodes, current_pose, bounds, (640, 720), hazard_xy)
                canvas[:720, 1280:] = map_panel
                elapsed = stamp - summary["start_s"]
                status_color = (0, 0, 255) if hazard_confirmed else (0, 180, 255) if hazards else (80, 210, 100)
                status = "已确认：模拟管道液体渗漏危险源" if hazard_confirmed else "发现疑似泄漏，等待连续帧确认" if hazards else "持续监测中"
                rows = [
                    ("遥操建图 rosbag 离线回放", (24, 748), (255, 255, 255), "title"),
                    (f"时间  {elapsed:6.1f} / {summary['duration_s']:.1f} 秒    帧 {frame_index}/{summary['counts']['color']}", (24, 802), (200, 210, 225), "body"),
                    (f"YOLOE-PF：无提示词 · {len(objects)} 个候选 · {payload.get('diagnostics', {}).get('inference_latency_ms', 0):.1f} ms", (24, 848), (200, 210, 225), "body"),
                    (f"二维雷达：{cloud_index}/{summary['counts']['cloud']} 帧    拓扑节点：{len(active_nodes)}/{len(nodes)}", (24, 894), (200, 210, 225), "body"),
                    (status, (24, 950), status_color, "bold"),
                    ("右侧：二维占据点图 + 轨迹 + 拓扑（绿=机器人，紫=拓扑，红=危险源）", (760, 1018), (185, 195, 210), "small"),
                ]
                canvas = put_text(canvas, rows)
                ui_writer.write(canvas)
                if time.time() - last_progress >= 10.0:
                    print(f"progress frame={frame_index}/{summary['counts']['color']} clouds={cloud_index} hazard={hazard_confirmed}", flush=True)
                    last_progress = time.time()
    finally:
        worker.close()
        detections_path.close()
        annotated_writer.release()
        ui_writer.release()
        if image_path.exists():
            image_path.unlink()

    final_map = map_image(free, occupied, path, nodes, current_pose, bounds, (1600, 1000), hazard_xy)
    final_map = put_text(final_map, [("最终二维占据地图、轨迹与拓扑", (30, 24), (255, 255, 255), "title")])
    cv2.imwrite(str(output / "occupancy_map_topology.png"), final_map)
    cv2.imwrite(str(output / "topology_graph.png"), topology_graph_image(nodes, hazard_confirmed))
    topology_payload = {
        "session": bag.parent.name,
        "coordinate_frame": "relative_fused_odom_start",
        "nodes": [
            {
                "id": node["id"], "stamp": node["stamp"],
                "pose": {"x": node["x"], "y": node["y"], "yaw": node["yaw"]},
                "objects": [{"name": name, "observations": count} for name, count in node["objects"].most_common(12)],
                "hazard": bool(hazard_confirmed and index == len(nodes) - 1),
            }
            for index, node in enumerate(nodes)
        ],
        "edges": [
            {
                "id": f"E{index}", "from": left["id"], "to": right["id"],
                "distance_m": math.hypot(right["x"] - left["x"], right["y"] - left["y"]),
            }
            for index, (left, right) in enumerate(zip(nodes, nodes[1:]), start=1)
        ],
    }
    (output / "topology.json").write_text(json.dumps(topology_payload, ensure_ascii=False, indent=2), encoding="utf-8")
    hazard_report = {
        "result": "CONFIRMED_DANGER_SOURCE" if hazard_confirmed else "NOT_CONFIRMED",
        "hazard_type": "simulated_pipeline_liquid_leak" if hazard_confirmed else None,
        "display_name": "模拟管道液体渗漏危险源" if hazard_confirmed else None,
        "confirmation_rule": "two supporting source frames within three frames and 12 seconds",
        "first_support_frame": hazard_first_frame,
        "confirmed_frame": hazard_confirm_frame,
        "observation_count": len(hazard_observations),
        "observations": hazard_observations,
        "map_xy": list(hazard_xy) if hazard_xy is not None else None,
        "map_position_quality": "nominal_camera_extrinsic" if hazard_xy is not None else "unavailable",
        "detector": {
            "backend": "yoloe_seg", "prompt_mode": "prompt_free",
            "model": args.model.name, "confidence": args.confidence, "image_size": args.image_size,
        },
        "enhancement": "HSV fluorescent cue constrained by mask-backed YOLOE pipe context",
        "scope": "controlled fluorescent-slime simulation; not a general industrial leak classifier",
    }
    (output / "hazard_report.json").write_text(json.dumps(hazard_report, ensure_ascii=False, indent=2), encoding="utf-8")
    ffmpeg_encode(annotated_raw, output / "yoloe_prompt_free_video.mp4")
    ffmpeg_encode(ui_raw, output / "mapping_process_ui.mp4")
    manifest = {
        "bag": str(bag), "output": str(output), "generated_at_unix": time.time(),
        "wall_time_s": time.time() - started, "source": summary,
        "grid": {**bounds, "width": width, "height": height},
        "hazard_result": hazard_report["result"],
        "artifacts": [
            "mapping_process_ui.mp4", "yoloe_prompt_free_video.mp4",
            "occupancy_map_topology.png", "topology_graph.png", "topology.json", "hazard_report.json",
            "hazard_confirmed.jpg", "detections.jsonl", "yoloe_worker.stderr.log",
        ],
        "safety": "direct rosbag read; no ROS publishers and no robot commands",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"ok": True, "output": str(output), "hazard": hazard_report["result"]}, ensure_ascii=False))
    return 0 if hazard_confirmed else 3


if __name__ == "__main__":
    raise SystemExit(main())
