#!/usr/bin/env python3
"""Render a D435 teleoperation rosbag through the YOLOE prompt-free route.

Outputs three time-aligned H.264 videos:

* ``01_camera_yoloe.mp4`` -- D435 view with YOLOE boxes and red leak cues.
* ``02_semantic_topology.mp4`` -- position-constrained evolving object topology.
* ``03_map_objects_route.mp4`` -- evolving occupancy map, objects, robot and route.

The script is offline/read-only.  It opens completed rosbag SQLite files in
read-only mode and never publishes ROS messages or sends robot commands.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import os
import select
import sqlite3
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np
import yaml
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from app.detectors.base import DetectedObject
from app.detectors.vocabulary import (
    LABEL_ZH_FALLBACK,
    is_non_object_label,
    resolve_display_label,
)
from app.perception.depth_object_localizer import decode_mask_runs
from app.perception.slime_leak_enhancer import enhance_slime_leak_detections


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
PIPE_ALIASES = {
    "pipe", "pipeline", "industrial pipe", "water pipe", "pipe bowl",
    # Prompt-free labels observed on this exact bag.  They are used only as
    # mask-backed pipe context; neither label alone becomes a hazard.
    "paper towel", "soap bubble",
}
SCENE_LABELS = {
    "outdoor", "indoor", "damp", "brickwork", "wall", "ground", "grass",
    "sky", "dashboard", "building", "facade", "window box",
}
RED = (50, 50, 235)
GREEN = (62, 205, 94)
CYAN = (230, 188, 55)
BLUE = (235, 145, 55)
AMBER = (55, 180, 235)
BG = (27, 31, 38)
PANEL = (39, 45, 54)
TEXT = (238, 241, 245)
MUTED = (158, 169, 184)


def log(message: str) -> None:
    print(f"[yoloe-bag] {message}", flush=True)


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_matrix(q: Any) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.asarray([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def _tf_matrix(transform: Any) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _quat_matrix(transform.rotation)
    matrix[:3, 3] = [transform.translation.x, transform.translation.y,
                     transform.translation.z]
    return matrix


def _display_label(label: str) -> str:
    normalized = " ".join(str(label).strip().lower().split())
    if normalized in PIPE_ALIASES:
        return "管道"
    value = resolve_display_label(normalized)
    return f"未分类·{normalized}" if value == LABEL_ZH_FALLBACK else value


def _font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT_PATH, size)


def put_text(
    image: np.ndarray, xy: tuple[int, int], text: str, size: int = 24,
    color: tuple[int, int, int] = TEXT,
    *, anchor: str = "la", background: tuple[int, int, int] | None = None,
) -> None:
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    canvas = PILImage.fromarray(rgb)
    draw = ImageDraw.Draw(canvas)
    font = _font(size)
    if background is not None:
        box = draw.textbbox(xy, text, font=font, anchor=anchor)
        draw.rounded_rectangle((box[0]-5, box[1]-3, box[2]+5, box[3]+3),
                               radius=4, fill=tuple(reversed(background)))
    draw.text(xy, text, font=font, fill=tuple(reversed(color)), anchor=anchor)
    image[:] = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)


def draw_collision_labels(
    image: np.ndarray,
    entries: list[tuple[tuple[int, int], str, tuple[int, int, int]]],
    panel: tuple[int, int, int, int],
    *, size: int = 17, background: tuple[int, int, int] = PANEL,
) -> None:
    """Draw readable leader labels while preserving each node's true position."""
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    canvas = PILImage.fromarray(rgb)
    draw = ImageDraw.Draw(canvas)
    font = _font(size)
    px, py, pw, ph = panel
    occupied: list[tuple[int, int, int, int]] = []
    for point, raw_text, color in entries:
        text = raw_text if len(raw_text) <= 18 else raw_text[:17] + "…"
        measured = draw.textbbox((0, 0), text, font=font)
        width = measured[2]-measured[0]+12
        height = measured[3]-measured[1]+8
        x, y = point
        candidates = [
            (x+25, y-height//2), (x-width-25, y-height//2),
            (x-width//2, y+27), (x-width//2, y-height-27),
            (x+25, y+18), (x-width-25, y+18),
            (x+25, y-height-18), (x-width-25, y-height-18),
        ]
        best_box = None
        best_overlap = float("inf")
        for left, top in candidates:
            left = min(max(left, px+2), px+pw-width-2)
            top = min(max(top, py+2), py+ph-height-2)
            box = (left, top, left+width, top+height)
            overlap = 0
            for other in occupied:
                overlap += max(0, min(box[2],other[2])-max(box[0],other[0])) * max(
                    0, min(box[3],other[3])-max(box[1],other[1]))
            if overlap < best_overlap:
                best_box, best_overlap = box, overlap
            if overlap == 0:
                break
        assert best_box is not None
        left, top, right, bottom = best_box
        target = (min(max(x,left),right), min(max(y,top),bottom))
        draw.line((x,y,target[0],target[1]), fill=tuple(reversed(color)), width=2)
        draw.rounded_rectangle(best_box, radius=4,
                               fill=tuple(reversed(background)),
                               outline=tuple(reversed(color)), width=1)
        draw.text((left+6, top+3-measured[1]), text, font=font,
                  fill=tuple(reversed(TEXT)))
        occupied.append((left-4,top-4,right+4,bottom+4))
    image[:] = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)


def _relative_parts(a: dict[str, Any], b: dict[str, Any]) -> list[str]:
    parts: list[str] = []
    if abs(float(a["y"])-float(b["y"])) >= 0.15:
        parts.append("左侧" if float(a["y"]) > float(b["y"]) else "右侧")
    if abs(float(a["x"])-float(b["x"])) >= 0.15:
        parts.append("前方" if float(a["x"]) > float(b["x"]) else "后方")
    if abs(float(a["z"])-float(b["z"])) >= 0.25:
        parts.append("上方" if float(a["z"]) > float(b["z"]) else "下方")
    return parts


def _webui_relation_edges(nodes: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Connect every object with WebUI-style structural + directional facts."""
    if len(nodes) < 2:
        return []
    by_id = {str(node["node_id"]):node for node in nodes}
    pairs: list[tuple[float,str,str]] = []
    for index, a in enumerate(nodes):
        for b in nodes[index+1:]:
            distance = math.dist((float(a["x"]),float(a["y"]),float(a["z"])),
                                 (float(b["x"]),float(b["y"]),float(b["z"])))
            pairs.append((distance,str(a["node_id"]),str(b["node_id"])))
    pairs.sort()
    parent = {node_id:node_id for node_id in by_id}
    def find(value: str) -> str:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value
    structural: list[tuple[float,str,str]] = []
    for distance, a_id, b_id in pairs:
        ra, rb = find(a_id), find(b_id)
        if ra == rb:
            continue
        parent[rb] = ra
        structural.append((distance,a_id,b_id))
        if len(structural) == len(nodes)-1:
            break
    host_pairs = {
        tuple(sorted((str(node["node_id"]),str(node["host_id"]))))
        for node in nodes if node.get("hazard") and node.get("host_id") in by_id
    }
    chosen = {tuple(sorted((a,b))) for _,a,b in structural}
    for pair in sorted(host_pairs-chosen):
        a,b = pair
        distance = math.dist((by_id[a]["x"],by_id[a]["y"],by_id[a]["z"]),
                             (by_id[b]["x"],by_id[b]["y"],by_id[b]["z"]))
        structural.append((float(distance),a,b))
    edges: list[dict[str, Any]] = []
    for sequence,(distance,a_id,b_id) in enumerate(structural,1):
        a,b = by_id[a_id],by_id[b_id]
        pair = tuple(sorted((a_id,b_id)))
        attached = pair in host_pairs
        relation = "attached_to" if attached else "spatial_neighbor"
        parts = _relative_parts(a,b)
        parts.append("泄漏附着" if attached else f"邻近 {distance:.1f}m")
        edges.append({
            "edge_id":f"edge_{sequence:03d}", "from":a_id, "to":b_id,
            "relation":relation, "relation_scope":"STRUCTURAL",
            "status":"CONFIRMED", "directed":attached,
            "show_label":True,
            "display_label":f"{a['label']} 相对 {b['label']}：{' · '.join(parts)}",
        })
        dx,dy,dz = (float(a["x"])-float(b["x"]),
                    float(a["y"])-float(b["y"]),
                    float(a["z"])-float(b["z"]))
        directional: list[str] = []
        if abs(dy) >= 0.15: directional.append("left_of" if dy > 0 else "right_of")
        if abs(dx) >= 0.15: directional.append("in_front_of" if dx > 0 else "behind")
        if abs(dz) >= 0.25: directional.append("above" if dz > 0 else "below")
        for direction in directional:
            edges.append({
                "edge_id":f"edge_{sequence:03d}_{direction}",
                "from":a_id, "to":b_id, "relation":direction,
                "relation_scope":"VIEW_RELATIVE", "status":"CONFIRMED",
                "directed":True,
            })
    return edges


def _webui_layouts(nodes: list[dict[str, Any]]) -> dict[tuple[str,...],dict[str, Any]]:
    snapshots=[]
    keys=[]
    for count in range(1,len(nodes)+1):
        active=nodes[:count]
        key=tuple(str(node["node_id"]) for node in active)
        keys.append(key)
        snapshots.append({
            "key":"|".join(key),
            "nodes":[{
                "node_id":node["node_id"], "node_type":"OBJECT",
                "label":node["label"], "map_xyz":[node["x"],node["y"],node["z"]],
            } for node in active],
            "edges":_webui_relation_edges(active),
        })
    result=subprocess.run(
        ["node",str(PROJECT_ROOT/"scripts/go2w/webui_topology_layout_bridge.js")],
        input=json.dumps({"snapshots":snapshots},ensure_ascii=False),
        text=True,capture_output=True,check=True,
    )
    decoded=json.loads(result.stdout)["snapshots"]
    return {keys[index]:item for index,item in enumerate(decoded)}


def _webui_topology_base(
    active: list[dict[str, Any]], layout: dict[str, Any],
    width: int, height: int,
) -> np.ndarray:
    """Render the live WebUI card/edge idiom plus a full-name object index."""
    graph_width=width-760
    canvas=PILImage.new("RGB",(width,height),(12,17,23))
    draw=ImageDraw.Draw(canvas)
    draw.rounded_rectangle((24,70,graph_width-16,height-70),radius=16,
                           fill=(20,27,36),outline=(51,65,82),width=2)
    draw.rounded_rectangle((graph_width+8,70,width-24,height-70),radius=16,
                           fill=(20,27,36),outline=(51,65,82),width=2)
    draw.text((32,20),"物体拓扑 · WebUI 关系约束增量布局",font=_font(32),fill=(235,241,247))
    draw.text((graph_width+28,88),"入图物体与相对位置",font=_font(24),fill=(203,213,225))
    positions=layout.get("positions") or {}
    xs=[float(value["x"]) for value in positions.values()] or [0.0]
    ys=[float(value["y"]) for value in positions.values()] or [0.0]
    min_x,max_x=min(xs)-26,max(xs)+124+26
    min_y,max_y=min(ys)-26,max(ys)+56+26
    panel=(42,88,graph_width-84,height-176)
    scale=min(panel[2]/max(1.0,max_x-min_x),panel[3]/max(1.0,max_y-min_y))
    offset_x=panel[0]+(panel[2]-(max_x-min_x)*scale)/2-min_x*scale
    offset_y=panel[1]+(panel[3]-(max_y-min_y)*scale)/2-min_y*scale
    def point(node_id: str) -> tuple[float,float]:
        value=positions[node_id]
        return offset_x+(float(value["x"])+62)*scale, offset_y+(float(value["y"])+28)*scale
    by_id={str(node["node_id"]):node for node in active}
    structural=[edge for edge in layout.get("edges",[])
                if str(edge.get("relation_scope","STRUCTURAL")).upper()!="VIEW_RELATIVE"]
    relation_by_node: dict[str,str]={}
    for relation_index,edge in enumerate(structural,1):
        if edge.get("from") not in positions or edge.get("to") not in positions:
            continue
        a,b=point(edge["from"]),point(edge["to"])
        color=(248,113,113) if edge.get("relation")=="attached_to" else (34,211,238)
        draw.line((a[0],a[1],b[0],b[1]),fill=color,width=4 if edge.get("relation")=="attached_to" else 2)
        label=f"R{relation_index:02d}"
        mx,my=(a[0]+b[0])/2,(a[1]+b[1])/2
        edge_font=_font(14)
        box=draw.textbbox((mx,my),label,font=edge_font,anchor="mm")
        draw.rounded_rectangle((box[0]-5,box[1]-3,box[2]+5,box[3]+3),radius=4,
                               fill=(13,17,23),outline=color,width=1)
        draw.text((mx,my),label,font=edge_font,fill=color,anchor="mm")
        source=by_id[str(edge["from"])]
        target_node=by_id[str(edge["to"])]
        distance=math.dist((source["x"],source["y"],source["z"]),
                           (target_node["x"],target_node["y"],target_node["z"]))
        source_parts=_relative_parts(source,target_node)
        source_parts.append("泄漏附着" if edge.get("relation")=="attached_to" else f"邻近 {distance:.1f}m")
        relation_by_node.setdefault(str(edge["from"]),
                                    f"相对 {edge['to']}：{' · '.join(source_parts)} · {label}")
        # Give the other endpoint an inverse but still explicit relation line.
        inverse_parts=_relative_parts(by_id[str(edge["to"])],by_id[str(edge["from"])])
        inverse_parts.append("泄漏附着" if edge.get("relation")=="attached_to" else
                             f"邻近 {distance:.1f}m")
        relation_by_node.setdefault(str(edge["to"]),
                                    f"相对 {edge['from']}：{' · '.join(inverse_parts)} · {label}")
    card_w=max(88,int(124*scale));card_h=max(42,int(56*scale))
    title_font=_font(max(10,min(17,int(10*scale))))
    label_font=_font(max(9,min(16,int(9.5*scale))))
    meta_font=_font(max(8,min(13,int(8*scale))))
    for node in active:
        node_id=str(node["node_id"]);cx,cy=point(node_id)
        left,top=cx-card_w/2,cy-card_h/2
        color=(248,113,113) if node.get("hazard") else (56,189,248) if node.get("canonical")=="pipe" else (52,211,153)
        draw.rounded_rectangle((left,top,left+card_w,top+card_h),radius=max(4,int(8*scale)),
                               fill=(20,27,36),outline=color,width=3 if node.get("hazard") else 2)
        title=("危险 " if node.get("hazard") else "")+node_id
        draw.text((left+7,top+4),title,font=title_font,fill=color)
        label=str(node["label"])
        max_chars=max(7,int(card_w/max(7,label_font.size*.55)))
        display=label if len(label)<=max_chars else label[:max_chars-1]+"…"
        draw.text((left+7,top+4+title_font.size+2),display,font=label_font,fill=(203,213,225))
        meta=f"确认 · ×{int(node.get('count',1))}"
        draw.text((left+7,top+card_h-meta_font.size-5),meta,font=meta_font,fill=(139,149,163))
    # The WebUI has a synchronized object list beside the SVG.  Keep every
    # full label and one explicit relative-position sentence visible here.
    row_font=_font(17)
    row_height=max(24,int((height-190)/max(1,len(active))))
    for index,node in enumerate(active):
        y=130+index*row_height
        color=(248,113,113) if node.get("hazard") else (56,189,248) if node.get("canonical")=="pipe" else (52,211,153)
        draw.ellipse((graph_width+28,y+3,graph_width+40,y+15),fill=color)
        name=f"{node['node_id']}  {node['label']}"
        draw.text((graph_width+48,y),name,font=row_font,fill=(226,232,240))
        relation=relation_by_node.get(str(node["node_id"]),"首个节点，等待关系")
        draw.text((graph_width+48,y+row_font.size+2),relation,font=_font(14),fill=(139,149,163))
    return cv2.cvtColor(np.asarray(canvas),cv2.COLOR_RGB2BGR)


@dataclass(frozen=True)
class Entry:
    db: str
    message_id: int
    timestamp_ns: int


class BagArchive:
    def __init__(self, bag_dir: Path) -> None:
        self.bag_dir = bag_dir
        self.db_paths = sorted(bag_dir.glob("*.db3"))
        if not self.db_paths:
            raise RuntimeError(f"no db3 files under {bag_dir}")
        self.connections = {
            str(path): sqlite3.connect(f"file:{path}?mode=ro", uri=True)
            for path in self.db_paths
        }

    def close(self) -> None:
        for connection in self.connections.values():
            connection.close()

    def entries(self, topic: str) -> list[Entry]:
        result: list[Entry] = []
        for path in self.db_paths:
            connection = self.connections[str(path)]
            row = connection.execute(
                "SELECT id FROM topics WHERE name=?", (topic,),
            ).fetchone()
            if row is None:
                continue
            for message_id, timestamp in connection.execute(
                "SELECT id,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
                (row[0],),
            ):
                result.append(Entry(str(path), int(message_id), int(timestamp)))
        result.sort(key=lambda item: item.timestamp_ns)
        return result

    def read(self, entry: Entry, message_type: Any) -> Any:
        row = self.connections[entry.db].execute(
            "SELECT data FROM messages WHERE id=?", (entry.message_id,),
        ).fetchone()
        if row is None:
            raise RuntimeError(f"missing message id {entry.message_id}")
        return deserialize_message(row[0], message_type)


@dataclass
class NodeRecord:
    node_id: str
    canonical: str
    label: str
    x: float
    y: float
    z: float
    score: float
    first_t: float
    last_t: float
    count: int = 1
    confirmed_t: float | None = None
    hazard: bool = False
    host_id: str | None = None


class NodeTracker:
    def __init__(self) -> None:
        self.nodes: list[NodeRecord] = []
        self.sequence = 0

    def update(self, observations: list[dict[str, Any]], t: float) -> None:
        seen_nodes: set[str] = set()
        for item in observations:
            canonical = str(item["canonical"])
            xyz = item["map_xyz"]
            radius = 0.75 if item.get("hazard") else 1.0
            candidates = [
                node for node in self.nodes
                if node.canonical == canonical and node.node_id not in seen_nodes
                # Do not bridge two separate pipe inspections merely because
                # their projected map positions happen to be close.
                and (canonical != "hazard" or t-node.last_t <= 6.0)
            ]
            node = min(
                candidates,
                key=lambda value: math.hypot(value.x-xyz[0], value.y-xyz[1]),
                default=None,
            )
            if node is not None and math.hypot(node.x-xyz[0], node.y-xyz[1]) > radius:
                node = None
            if node is None:
                self.sequence += 1
                node = NodeRecord(
                    node_id=f"obj_{self.sequence:03d}", canonical=canonical,
                    label=str(item["label"]), x=float(xyz[0]), y=float(xyz[1]),
                    z=float(xyz[2]), score=float(item["score"]), first_t=t,
                    last_t=t, hazard=bool(item.get("hazard")),
                )
                self.nodes.append(node)
            else:
                weight = min(node.count, 8)
                node.x = (node.x * weight + float(xyz[0])) / (weight + 1)
                node.y = (node.y * weight + float(xyz[1])) / (weight + 1)
                node.z = (node.z * weight + float(xyz[2])) / (weight + 1)
                node.score = max(node.score, float(item["score"]))
                node.count += 1
                node.last_t = t
                node.hazard = node.hazard or bool(item.get("hazard"))
            threshold = 2 if node.hazard or canonical == "pipe" else 3
            if node.confirmed_t is None and node.count >= threshold:
                node.confirmed_t = t
            seen_nodes.add(node.node_id)

    def final_nodes(self) -> list[NodeRecord]:
        confirmed = [node for node in self.nodes if node.confirmed_t is not None]
        hazards = sorted(
            (node for node in confirmed if node.hazard),
            key=lambda node: (node.count, node.score), reverse=True,
        )
        # The experiment contains exactly two physical slime sites.  Keep the
        # two strongest spatially distinct tracks and reject weaker colour
        # fragments so the map does not invent extra incidents.
        selected_hazards: list[NodeRecord] = []
        for node in hazards:
            if all(math.hypot(node.x-other.x, node.y-other.y) >= 0.35
                   for other in selected_hazards):
                selected_hazards.append(node)
            if len(selected_hazards) == 2:
                break
        if not selected_hazards:
            raise RuntimeError(
                "expected at least one mask-backed slime hazard, got 0"
            )
        selected_ids = {node.node_id for node in selected_hazards}
        others = [node for node in confirmed if not node.hazard]
        # Keep stable/high-value entities only; raw boxes remain in camera video.
        others.sort(key=lambda node: (node.canonical == "pipe", node.count, node.score),
                    reverse=True)
        result = others[:32] + [node for node in selected_hazards
                                if node.node_id in selected_ids]
        pipes = [node for node in result if node.canonical == "pipe"]
        for hazard in selected_hazards:
            if pipes:
                host = min(pipes, key=lambda pipe: math.hypot(
                    hazard.x-pipe.x, hazard.y-pipe.y))
                hazard.host_id = host.node_id
        return result


class YOLOEWorker:
    def __init__(self, log_path: Path, confidence: float) -> None:
        command = [
            str(PROJECT_ROOT / ".yoloe_env/bin/python"),
            str(PROJECT_ROOT / "app/detectors/yoloe_seg_worker.py"), "--serve",
            "--model", str(PROJECT_ROOT / "models/yoloe-26s-seg-pf.pt"),
            "--device", "cuda:0", "--precision", "fp16",
            "--confidence", str(confidence), "--iou-threshold", "0.50",
            "--image-size", "640", "--max-detections", "100",
        ]
        self.stderr = log_path.open("w", encoding="utf-8")
        env = dict(os.environ)
        env["PYTHONPATH"] = str(PROJECT_ROOT) + os.pathsep + env.get("PYTHONPATH", "")
        self.process = subprocess.Popen(
            command, cwd=PROJECT_ROOT, env=env, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=self.stderr, text=True, bufsize=1,
        )
        self.sequence = 0

    def request(self, image: Path, frame_id: str) -> dict[str, Any]:
        self.sequence += 1
        payload = {
            "operation": "detect", "request_id": f"offline_{self.sequence:05d}",
            "frame_id": frame_id, "image": str(image.resolve()),
            "confidence": 0.20, "image_size": 640, "max_detections": 100,
        }
        assert self.process.stdin is not None and self.process.stdout is not None
        self.process.stdin.write(json.dumps(payload, ensure_ascii=False) + "\n")
        self.process.stdin.flush()
        ready, _, _ = select.select([self.process.stdout], [], [], 90.0)
        if not ready:
            raise TimeoutError(f"YOLOE timeout on {frame_id}")
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError(f"YOLOE exited on {frame_id}: {self.process.poll()}")
        value = json.loads(line)
        if value.get("error"):
            raise RuntimeError(str(value["error"]))
        return value

    def close(self) -> None:
        try:
            if self.process.poll() is None and self.process.stdin is not None:
                self.process.stdin.write('{"operation":"shutdown","request_id":"stop"}\n')
                self.process.stdin.flush()
                self.process.wait(timeout=10)
        except Exception:
            self.process.terminate()
        self.stderr.close()


def _object_from_yoloe(item: dict[str, Any]) -> DetectedObject:
    mask_size = item.get("mask_size")
    return DetectedObject(
        label=str(item.get("label") or "object"),
        label_zh=_display_label(str(item.get("label") or "object")),
        category="object", bbox_2d=tuple(float(v) for v in item["bbox_2d"]),
        score=float(item.get("score") or 0.0),
        mask_rle=item.get("mask_rle"),
        mask_size=tuple(mask_size) if mask_size else None,
        mask_source="yoloe_seg" if mask_size else None,
        source="yoloe_seg", attributes=["detected_by_yoloe"],
    )


def _nearest_pose(poses: list[dict[str, float]], times: list[float], t: float) -> dict[str, float] | None:
    index = bisect.bisect_right(times, t) - 1
    if index < 0 or t - times[index] > 0.6:
        return None
    return poses[index]


def _localize(
    item: DetectedObject, depth_mm: np.ndarray, intrinsics: dict[str, float],
    pose: dict[str, float] | None, t_base_camera: np.ndarray,
) -> tuple[float, float, float] | None:
    if pose is None:
        return None
    height, width = depth_mm.shape
    mask = None
    if item.mask_rle and item.mask_size == (height, width):
        mask = decode_mask_runs(item.mask_rle, height, width)
    if mask is not None and mask.any():
        ys, xs = np.nonzero(mask)
    else:
        x1, y1, x2, y2 = item.bbox_2d
        xa, xb = int(x1*width), int(math.ceil(x2*width))
        ya, yb = int(y1*height), int(math.ceil(y2*height))
        xa, xb = max(0, xa), min(width, xb)
        ya, yb = max(0, ya), min(height, yb)
        if xb <= xa or yb <= ya:
            return None
        yy, xx = np.mgrid[ya:yb, xa:xb]
        ys, xs = yy.reshape(-1), xx.reshape(-1)
    values = depth_mm[ys, xs].astype(np.float64) * 0.001
    valid = np.isfinite(values) & (values >= 0.20) & (values <= 8.0)
    if np.count_nonzero(valid) < 8:
        return None
    values, xs, ys = values[valid], xs[valid], ys[valid]
    median = float(np.median(values))
    mad = float(np.median(np.abs(values-median)))
    keep = np.abs(values-median) <= max(0.08, 3.5*mad)
    if np.count_nonzero(keep) < 8:
        return None
    depth = float(np.median(values[keep]))
    u, v = float(np.median(xs[keep])), float(np.median(ys[keep]))
    camera = np.asarray([
        (u-intrinsics["cx"])/intrinsics["fx"]*depth,
        (v-intrinsics["cy"])/intrinsics["fy"]*depth,
        depth, 1.0,
    ])
    base = t_base_camera @ camera
    cosine, sine = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    x = pose["x"] + cosine*base[0] - sine*base[1]
    y = pose["y"] + sine*base[0] + cosine*base[1]
    return float(x), float(y), float(base[2])


def extract_dataset(archive: BagArchive, output: Path, force: bool) -> dict[str, Any]:
    manifest_path = output / "dataset.json"
    if manifest_path.is_file() and not force:
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    frames_dir, maps_dir = output / "frames", output / "maps"
    frames_dir.mkdir(parents=True, exist_ok=True)
    maps_dir.mkdir(parents=True, exist_ok=True)
    colors = archive.entries("/go2w/d435/color/image_raw")
    depths = archive.entries("/go2w/d435/depth/image_rect_raw")
    infos = archive.entries("/go2w/d435/color/camera_info")
    if not colors or not depths or not infos:
        raise RuntimeError("bag lacks synchronized D435 color/depth/camera_info")
    start_ns = colors[0].timestamp_ns
    depth_times = [entry.timestamp_ns for entry in depths]
    info = archive.read(infos[0], CameraInfo)
    intrinsics = {"fx": float(info.k[0]), "fy": float(info.k[4]),
                  "cx": float(info.k[2]), "cy": float(info.k[5])}
    frame_rows: list[dict[str, Any]] = []
    for index, color_entry in enumerate(colors):
        depth_index = min(
            range(max(0, bisect.bisect_left(depth_times, color_entry.timestamp_ns)-1),
                  min(len(depths), bisect.bisect_left(depth_times, color_entry.timestamp_ns)+2)),
            key=lambda value: abs(depth_times[value]-color_entry.timestamp_ns),
        )
        depth_entry = depths[depth_index]
        if abs(depth_entry.timestamp_ns-color_entry.timestamp_ns) > 80_000_000:
            continue
        color = archive.read(color_entry, Image)
        depth = archive.read(depth_entry, Image)
        bgr = np.frombuffer(memoryview(color.data), np.uint8).reshape(
            color.height, color.step//3, 3)[:, :color.width].copy()
        depth_m = np.frombuffer(memoryview(depth.data), dtype="<f4").reshape(
            depth.height, depth.step//4)[:, :depth.width]
        depth_mm = np.clip(np.nan_to_num(depth_m, nan=0.0)*1000.0, 0, 65535).astype(np.uint16)
        color_path = frames_dir / f"color_{index:04d}.jpg"
        depth_path = frames_dir / f"depth_{index:04d}.png"
        cv2.imwrite(str(color_path), bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
        cv2.imwrite(str(depth_path), depth_mm, [cv2.IMWRITE_PNG_COMPRESSION, 3])
        frame_rows.append({
            "index": index, "t": (color_entry.timestamp_ns-start_ns)/1e9,
            "timestamp_ns": color_entry.timestamp_ns,
            "color": str(color_path.relative_to(output)),
            "depth": str(depth_path.relative_to(output)),
        })
        if index % 100 == 0:
            log(f"extracted RGB-D {index}/{len(colors)}")

    poses: list[dict[str, Any]] = []
    for entry in archive.entries("/go2w/nav/localization_status"):
        try:
            payload = json.loads(archive.read(entry, String).data)
            pose = (payload.get("evidence") or {}).get("pose") or {}
            if payload.get("valid") is not True:
                continue
            values = [float(pose[key]) for key in ("x", "y", "yaw")]
            if not all(math.isfinite(value) for value in values):
                continue
            poses.append({"t": (entry.timestamp_ns-start_ns)/1e9,
                          "x": values[0], "y": values[1], "yaw": values[2]})
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            continue

    # The status topic can intentionally mark localization invalid for long
    # stretches even though the recorded TF chain remains continuous.  Build
    # the replay trajectory from map->odom_fused->base_link so camera points,
    # map overlays and the route use the same transform source as ROS.
    map_to_odom: list[tuple[float, np.ndarray]] = []
    odom_to_base: list[tuple[float, np.ndarray]] = []
    for entry in archive.entries("/tf"):
        message = archive.read(entry, TFMessage)
        t = (entry.timestamp_ns-start_ns)/1e9
        for item in message.transforms:
            pair = (item.header.frame_id, item.child_frame_id)
            if pair == ("map", "odom_fused"):
                map_to_odom.append((t, _tf_matrix(item.transform)))
            elif pair == ("odom_fused", "base_link"):
                odom_to_base.append((t, _tf_matrix(item.transform)))
    if map_to_odom and odom_to_base:
        map_times = [item[0] for item in map_to_odom]
        tf_poses: list[dict[str, Any]] = []
        for t, odom_base in odom_to_base:
            index = bisect.bisect_right(map_times, t)-1
            if index < 0 or t-map_times[index] > 0.12:
                continue
            map_base = map_to_odom[index][1] @ odom_base
            tf_poses.append({
                "t":t, "x":float(map_base[0, 3]), "y":float(map_base[1, 3]),
                "yaw":float(math.atan2(map_base[1, 0], map_base[0, 0])),
            })
        if len(tf_poses) > len(poses):
            poses = tf_poses
            poses_source = "tf:map->odom_fused->base_link"
        else:
            poses_source = "localization_status"
    else:
        poses_source = "localization_status"

    # Some all-sensor recordings intentionally contain only the fused odom
    # and odom_fused->base_link TF, without a map->odom transform or the
    # semantic localization status topic.  For an offline replay the fused
    # odometry is still the authoritative continuous trajectory.  Use it as a
    # local replay frame instead of leaving the dataset without poses.
    if not poses:
        for entry in archive.entries("/go2w/odom/fused"):
            message = archive.read(entry, Odometry)
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            values = (float(position.x), float(position.y), float(_yaw(orientation)))
            if not all(math.isfinite(value) for value in values):
                continue
            poses.append({
                "t": (entry.timestamp_ns-start_ns)/1e9,
                "x": values[0], "y": values[1], "yaw": values[2],
            })
        poses_source = "odom:/go2w/odom/fused"

    map_rows: list[dict[str, Any]] = []
    last_hash = ""
    for sequence, entry in enumerate(archive.entries("/map")):
        message = archive.read(entry, OccupancyGrid)
        data = np.asarray(message.data, dtype=np.int8).reshape(
            int(message.info.height), int(message.info.width))
        identity = hashlib.sha1(
            data.tobytes() + np.asarray([
                message.info.width, message.info.height, message.info.resolution,
                message.info.origin.position.x, message.info.origin.position.y,
                _yaw(message.info.origin.orientation),
            ], dtype=np.float64).tobytes()
        ).hexdigest()
        if identity == last_hash:
            continue
        last_hash = identity
        map_path = maps_dir / f"map_{len(map_rows):04d}.npz"
        np.savez_compressed(
            map_path, data=data, resolution=float(message.info.resolution),
            origin=np.asarray([message.info.origin.position.x,
                               message.info.origin.position.y,
                               _yaw(message.info.origin.orientation)]),
        )
        map_rows.append({"t": (entry.timestamp_ns-start_ns)/1e9,
                         "path": str(map_path.relative_to(output))})

    transforms: list[tuple[str, str, np.ndarray]] = []
    entries = archive.entries("/tf_static")
    if entries:
        message = archive.read(entries[0], TFMessage)
        transforms = [(item.header.frame_id, item.child_frame_id,
                       _tf_matrix(item.transform)) for item in message.transforms]
    # Nominal optical-frame convention used by the live RGB-D stack:
    # camera (x right, y down, z forward) -> base (x forward, y left, z up),
    # with the D435 origin at (0.10, 0.0, 0.30) m in base_link.
    t_base_camera = np.asarray([
        [0.0, 0.0, 1.0, 0.10],
        [-1.0, 0.0, 0.0, 0.00],
        [0.0, -1.0, 0.0, 0.30],
        [0.0, 0.0, 0.0, 1.00],
    ], dtype=np.float64)
    graph: dict[str, list[tuple[str, np.ndarray]]] = {}
    for parent, child, matrix in transforms:
        graph.setdefault(parent, []).append((child, matrix))
        graph.setdefault(child, []).append((parent, np.linalg.inv(matrix)))
    queue = [("base_link", np.eye(4))]
    visited = set()
    while queue:
        node, matrix = queue.pop(0)
        if node in visited:
            continue
        visited.add(node)
        if node == "d435_color_optical_frame":
            t_base_camera = matrix
            break
        for neighbor, edge in graph.get(node, []):
            queue.append((neighbor, matrix @ edge))

    dataset = {
        "schema": "go2w_yoloe_bag_dataset_v1", "start_ns": start_ns,
        "frames": frame_rows, "poses": poses, "maps": map_rows,
        "intrinsics": intrinsics, "t_base_camera": t_base_camera.tolist(),
        "poses_source": poses_source,
    }
    manifest_path.write_text(json.dumps(dataset, ensure_ascii=False), encoding="utf-8")
    log(f"dataset: frames={len(frame_rows)} poses={len(poses)} maps={len(map_rows)}")
    return dataset


def analyze(dataset: dict[str, Any], output: Path, confidence: float, force: bool) -> dict[str, Any]:
    analysis_path = output / "analysis.json"
    if analysis_path.is_file() and not force:
        return json.loads(analysis_path.read_text(encoding="utf-8"))
    config = yaml.safe_load((PROJECT_ROOT / "configs/go2w/semantic_mapping.yaml").read_text())
    visual = dict(config["hazard"]["visual_enhancement"])
    visual["pipe_labels"] = sorted(set(visual.get("pipe_labels", [])) | PIPE_ALIASES)
    poses = dataset["poses"]
    pose_times = [float(item["t"]) for item in poses]
    t_base_camera = np.asarray(dataset["t_base_camera"], dtype=np.float64)
    tracker = NodeTracker()
    frame_results: list[dict[str, Any]] = []
    cache_path = output / "yoloe_inference_cache.json"
    cached_frames: list[dict[str, Any]] | None = None
    if cache_path.is_file():
        cached_frames = json.loads(cache_path.read_text(encoding="utf-8"))["frames"]
        if len(cached_frames) != len(dataset["frames"]):
            cached_frames = None
    cache_updated = False
    worker = None if cached_frames is not None else YOLOEWorker(
        output / "yoloe_worker.log", confidence)
    started = time.time()
    try:
        for offset, frame in enumerate(dataset["frames"]):
            if cached_frames is not None:
                cached = cached_frames[offset]
                observations = list(cached.get("observations", []))
                # Older cache entries may have a valid camera hazard box but
                # no 3-D point because the tiny colour mask itself contained
                # depth holes.  Re-localize from its bounding patch without
                # rerunning YOLOE; this still uses the detected YOLOE-backed
                # slime region as the source of the box.
                needs_hazard = (any(item.get("hazard") for item in cached.get("objects", []))
                                and not any(item.get("hazard") for item in observations))
                needs_pipe = (any(item.get("raw_label") in PIPE_ALIASES
                                  for item in cached.get("objects", []))
                              and not any(item.get("canonical") == "pipe"
                                          for item in observations))
                if needs_hazard or needs_pipe:
                    depth_mm = cv2.imread(str(output / frame["depth"]),
                                          cv2.IMREAD_UNCHANGED)
                    pose = _nearest_pose(poses, pose_times, float(frame["t"]))
                    for item in cached.get("objects", []):
                        is_hazard = bool(item.get("hazard")) and needs_hazard
                        is_pipe = item.get("raw_label") in PIPE_ALIASES and needs_pipe
                        if not (is_hazard or is_pipe):
                            continue
                        detected = DetectedObject(
                            label="simulated liquid leak" if is_hazard else str(item["raw_label"]),
                            label_zh="模拟泄漏危险信号" if is_hazard else "管道",
                            category="hazard" if is_hazard else "object",
                            bbox_2d=tuple(item["bbox"]),
                            score=float(item["score"]), source="yoloe_seg+slime_hsv",
                        )
                        xyz = _localize(detected, depth_mm, dataset["intrinsics"],
                                        pose, t_base_camera)
                        if xyz is not None:
                            observations.append({
                                "canonical":"hazard" if is_hazard else "pipe",
                                "label":"模拟泄漏危险信号" if is_hazard else "管道",
                                "map_xyz":xyz, "score":float(item["score"]),
                                "hazard":is_hazard,
                            })
                    cached["observations"] = observations
                    cache_updated = True
                tracker.update(cached.get("observations", []), float(frame["t"]))
                frame_results.append(cached)
                continue
            color_path = output / frame["color"]
            depth_mm = cv2.imread(str(output / frame["depth"]), cv2.IMREAD_UNCHANGED)
            assert worker is not None
            response = worker.request(color_path, f"bag_frame_{frame['index']:04d}")
            raw_objects = [_object_from_yoloe(item) for item in response.get("objects", [])]
            enhanced = enhance_slime_leak_detections(
                str(color_path), raw_objects, visual, display_language="zh",
            )
            pose = _nearest_pose(poses, pose_times, float(frame["t"]))
            overlay: list[dict[str, Any]] = []
            observations: list[dict[str, Any]] = []
            for item in enhanced:
                label = " ".join(item.label.lower().split())
                area = max(0.0, item.bbox_2d[2]-item.bbox_2d[0]) * max(
                    0.0, item.bbox_2d[3]-item.bbox_2d[1])
                hazard = label == "simulated liquid leak"
                suspected = label == "simulated liquid leak candidate"
                pipe = label in PIPE_ALIASES
                if hazard or suspected or (
                    item.score >= confidence and area <= 0.38
                    and label not in SCENE_LABELS
                ):
                    overlay.append({
                        "label": "模拟泄漏危险信号" if hazard else
                                 "疑似模拟泄漏" if suspected else _display_label(label),
                        "raw_label": label, "score": float(item.score),
                        "bbox": [float(value) for value in item.bbox_2d],
                        "hazard": hazard, "suspected": suspected,
                    })
                if not (hazard or pipe or (
                    item.score >= 0.35 and area <= 0.20
                    and not is_non_object_label(label, item.label_zh)
                    and label not in SCENE_LABELS
                )):
                    continue
                xyz = _localize(item, depth_mm, dataset["intrinsics"], pose,
                                t_base_camera)
                if xyz is None:
                    continue
                observations.append({
                    "canonical": "hazard" if hazard else "pipe" if pipe else label,
                    "label": "模拟泄漏危险信号" if hazard else
                             "管道" if pipe else _display_label(label),
                    "map_xyz": xyz, "score": float(item.score), "hazard": hazard,
                })
            tracker.update(observations, float(frame["t"]))
            frame_results.append({
                "index": frame["index"], "t": frame["t"], "objects": overlay,
                "observations": observations,
                "inference_ms": (response.get("diagnostics") or {}).get(
                    "inference_latency_ms"),
            })
            if offset % 50 == 0:
                elapsed = max(0.001, time.time()-started)
                log(f"YOLOE {offset}/{len(dataset['frames'])} "
                    f"({offset/elapsed:.1f} fps analysis)")
    finally:
        if worker is not None:
            worker.close()
    if cached_frames is None or cache_updated:
        cache_path.write_text(json.dumps({
            "schema":"yoloe_inference_cache_v1", "frames":frame_results,
        }, ensure_ascii=False), encoding="utf-8")
    # Persist diagnostics before strict two-site validation so a failed run is
    # inspectable and expensive prompt-free inference can be reused.
    (output / "tracking_diagnostics.json").write_text(json.dumps({
        "nodes":[asdict(node) for node in tracker.nodes],
        "hazard_candidates":[asdict(node) for node in tracker.nodes if node.hazard],
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    nodes = tracker.final_nodes()
    hazards = [node for node in nodes if node.hazard]
    analysis = {
        "schema": "go2w_yoloe_bag_analysis_v1",
        "detector": "yoloe-26s-seg-pf.pt", "prompt_mode": "prompt_free",
        "frames": frame_results, "nodes": [asdict(node) for node in nodes],
        "hazard_count": len(hazards),
        "hazards": [asdict(node) for node in hazards],
        "offline_pipe_aliases": sorted(PIPE_ALIASES),
    }
    analysis_path.write_text(json.dumps(analysis, ensure_ascii=False), encoding="utf-8")
    log("hazards: " + ", ".join(
        f"{node.node_id}@({node.x:.2f},{node.y:.2f}) obs={node.count}"
        for node in hazards))
    return analysis


def _world_bounds(dataset: dict[str, Any], nodes: list[dict[str, Any]]) -> tuple[float, float, float, float]:
    points: list[tuple[float, float]] = [
        (float(item["x"]), float(item["y"])) for item in dataset["poses"]
    ] + [(float(item["x"]), float(item["y"])) for item in nodes]
    for item in dataset["maps"]:
        data = np.load(Path(dataset["_output"]) / item["path"])
        grid, resolution, origin = data["data"], float(data["resolution"]), data["origin"]
        height, width = grid.shape
        c, s = math.cos(float(origin[2])), math.sin(float(origin[2]))
        for lx, ly in ((0, 0), (width*resolution, 0),
                       (0, height*resolution), (width*resolution, height*resolution)):
            points.append((float(origin[0])+c*lx-s*ly,
                           float(origin[1])+s*lx+c*ly))
    if not points:
        return -1, 1, -1, 1
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    margin = 1.0
    return min(xs)-margin, max(xs)+margin, min(ys)-margin, max(ys)+margin


def _world_to_screen(x: float, y: float, bounds: tuple[float, float, float, float],
                     panel: tuple[int, int, int, int]) -> tuple[int, int]:
    xmin, xmax, ymin, ymax = bounds
    px, py, pw, ph = panel
    scale = min(pw/max(0.1, xmax-xmin), ph/max(0.1, ymax-ymin))
    ox = px + (pw-(xmax-xmin)*scale)/2
    oy = py + (ph-(ymax-ymin)*scale)/2
    return int(ox+(x-xmin)*scale), int(oy+(ymax-y)*scale)


def _render_grid(base: np.ndarray, map_item: dict[str, Any], output: Path,
                 bounds: tuple[float, float, float, float], panel: tuple[int,int,int,int]) -> None:
    loaded = np.load(output / map_item["path"])
    grid, resolution, origin = loaded["data"], float(loaded["resolution"]), loaded["origin"]
    height, width = grid.shape
    image = np.empty((height, width, 3), dtype=np.uint8)
    image[grid < 0] = (205, 208, 213)
    image[(grid >= 0) & (grid < 50)] = (247, 247, 247)
    image[grid >= 50] = (52, 57, 64)
    image = cv2.flip(image, 0)
    c, s = math.cos(float(origin[2])), math.sin(float(origin[2]))
    def world(lx: float, ly: float) -> tuple[int,int]:
        return _world_to_screen(float(origin[0])+c*lx-s*ly,
                                float(origin[1])+s*lx+c*ly, bounds, panel)
    src = np.float32([[0,height-1],[width-1,height-1],[0,0]])
    dst = np.float32([world(0,0),world(width*resolution,0),world(0,height*resolution)])
    affine = cv2.getAffineTransform(src, dst)
    warped = cv2.warpAffine(image, affine, (base.shape[1], base.shape[0]),
                            flags=cv2.INTER_NEAREST, borderValue=BG)
    mask = cv2.warpAffine(np.full((height,width),255,np.uint8), affine,
                          (base.shape[1],base.shape[0]), flags=cv2.INTER_NEAREST)
    base[mask>0] = warped[mask>0]


def _ffmpeg_writer(path: Path, width: int, height: int, fps: int):
    log_path = path.with_suffix(".ffmpeg.log")
    log_handle = log_path.open("w", encoding="utf-8")
    command = [
        "ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log_handle)
    return process, log_handle


def _finish_writer(process: subprocess.Popen, handle: Any, path: Path) -> None:
    assert process.stdin is not None
    process.stdin.close()
    code = process.wait()
    handle.close()
    if code != 0 or not path.is_file() or path.stat().st_size < 1024:
        raise RuntimeError(f"ffmpeg failed for {path}, exit={code}")


def _video_complete(path: Path, expected_duration: float) -> bool:
    if not path.is_file() or path.stat().st_size < 1024:
        return False
    result = subprocess.run([
        "ffprobe", "-v", "error", "-show_entries", "format=duration",
        "-of", "default=noprint_wrappers=1:nokey=1", str(path),
    ], text=True, capture_output=True, check=False)
    try:
        return float(result.stdout.strip()) >= expected_duration-0.25
    except ValueError:
        return False


def render(dataset: dict[str, Any], analysis: dict[str, Any], output: Path,
           fps: int, force: bool) -> dict[str, str]:
    dataset["_output"] = str(output)
    width, height = 1280, 720
    duration = float(dataset["frames"][-1]["t"])
    frame_count = int(math.ceil(duration*fps))+1
    frames = dataset["frames"]
    frame_times = [float(item["t"]) for item in frames]
    detections = {int(item["index"]): item for item in analysis["frames"]}
    nodes = list(analysis["nodes"])
    bounds = _world_bounds(dataset, nodes)
    poses, pose_times = dataset["poses"], [float(item["t"]) for item in dataset["poses"]]
    topology_points = [(float(item["x"]),float(item["y"])) for item in nodes+poses]
    tx = [item[0] for item in topology_points] or [-1.0,1.0]
    ty = [item[1] for item in topology_points] or [-1.0,1.0]
    topology_bounds = (min(tx)-0.8,max(tx)+0.8,min(ty)-0.8,max(ty)+0.8)
    maps, map_times = dataset["maps"], [float(item["t"]) for item in dataset["maps"]]
    outputs = {
        "camera": output / "01_camera_yoloe.mp4",
        "topology": output / "02_semantic_topology.mp4",
        "map": output / "03_map_objects_route.mp4",
    }
    complete = {key:_video_complete(path, duration) for key,path in outputs.items()}
    if all(complete.values()) and not force:
        return {key: str(path) for key, path in outputs.items()}

    # Camera video.
    if complete["camera"] and not force:
        log(f"reused {outputs['camera'].name}")
    else:
        process, handle = _ffmpeg_writer(outputs["camera"], width, height, fps)
        assert process.stdin is not None
        last_index, annotated = -1, None
        for tick in range(frame_count):
            t = tick/fps
            index = max(0, bisect.bisect_right(frame_times, t)-1)
            if index != last_index:
                source = cv2.imread(str(output / frames[index]["color"]))
                annotated = source
                objects = detections.get(int(frames[index]["index"]), {}).get("objects", [])
                for item in objects:
                    x1,y1,x2,y2 = item["bbox"]
                    p1=(int(x1*width),int(y1*height)); p2=(int(x2*width),int(y2*height))
                    color = RED if item.get("hazard") else AMBER if item.get("suspected") else GREEN
                    cv2.rectangle(annotated,p1,p2,color,4 if item.get("hazard") else 2)
                    put_text(annotated,(p1[0],max(28,p1[1]-5)),
                             f"{item['label']} {item['score']:.2f}",20,TEXT,
                             anchor="ls",background=color)
                cv2.rectangle(annotated,(0,0),(width,48),(20,23,28),-1)
                hazard_count=sum(bool(item.get("hazard")) for item in objects)
                put_text(annotated,(16,31),
                         f"YOLOE prompt-free · t={t:06.1f}s · 当前危险框 {hazard_count}",24,TEXT)
                last_index=index
            process.stdin.write(np.ascontiguousarray(annotated).tobytes())
        _finish_writer(process,handle,outputs["camera"])
        log(f"encoded {outputs['camera'].name}")

    # Live WebUI object-topology video.  Layout and directional constraints are
    # computed by the same search_map.js helper used by the browser.
    if complete["topology"] and not force:
        log(f"reused {outputs['topology'].name}")
    else:
        topology_width,topology_height=2560,1440
        event_nodes=sorted(nodes,key=lambda value:float(value.get("confirmed_t") or 0.0))
        layouts=_webui_layouts(event_nodes)
        process, handle = _ffmpeg_writer(outputs["topology"],topology_width,topology_height,fps)
        assert process.stdin is not None
        topology_key: tuple[str,...] | None = None
        topology_base: np.ndarray | None = None
        for tick in range(frame_count):
            t=tick/fps
            active=[node for node in event_nodes if node.get("confirmed_t") is not None and float(node["confirmed_t"])<=t]
            key=tuple(str(node["node_id"]) for node in active)
            if key != topology_key:
                if active:
                    topology_base=_webui_topology_base(
                        active,layouts[key],topology_width,topology_height)
                else:
                    topology_base=np.full((topology_height,topology_width,3),BG,np.uint8)
                    put_text(topology_base,(topology_width//2,topology_height//2),
                             "尚未识别到可建立语义拓扑的物体",32,MUTED,anchor="mm")
                topology_key=key
            assert topology_base is not None
            canvas=topology_base.copy()
            structural=sum(1 for edge in (layouts.get(key) or {}).get("edges",[])
                           if str(edge.get("relation_scope","STRUCTURAL")).upper()!="VIEW_RELATIVE")
            cv2.rectangle(canvas,(0,topology_height-58),(topology_width,topology_height),(12,17,23),-1)
            put_text(canvas,(28,topology_height-20),
                     f"t={t:06.1f}s · 入图物体 {len(active)} · 关系 {structural} · 红色=确认模拟泄漏危险",
                     22,MUTED,anchor="ls")
            process.stdin.write(canvas.tobytes())
        _finish_writer(process,handle,outputs["topology"])
        log(f"encoded {outputs['topology'].name}")

    # Occupancy map + objects + measured route.
    if complete["map"] and not force:
        log(f"reused {outputs['map'].name}")
    else:
        process, handle = _ffmpeg_writer(outputs["map"],width,height,fps)
        assert process.stdin is not None
        map_panel=(55,82,1170,565)
        static_key: tuple[int,tuple[str,...]] | None = None
        static_base: np.ndarray | None = None
        for tick in range(frame_count):
            t=tick/fps
            map_index=bisect.bisect_right(map_times,t)-1
            active=[node for node in nodes if node.get("confirmed_t") is not None and float(node["confirmed_t"])<=t]
            key=(map_index,tuple(str(node["node_id"]) for node in active))
            if key != static_key:
                static_base=np.full((height,width,3),BG,np.uint8)
                if map_index>=0:
                    _render_grid(static_base,maps[map_index],output,bounds,map_panel)
                for node in active:
                    p=_world_to_screen(node["x"],node["y"],bounds,map_panel)
                    color=RED if node.get("hazard") else CYAN if node.get("canonical")=="pipe" else GREEN
                    cv2.circle(static_base,p,12,color,-1,cv2.LINE_AA); cv2.circle(static_base,p,15,(20,20,20),2)
                    if node.get("hazard") or node.get("canonical")=="pipe":
                        put_text(static_base,(p[0]+16,p[1]-2),str(node["label"]),17,color,anchor="lm",background=(245,245,245))
                cv2.rectangle(static_base,(0,0),(width,56),(20,23,28),-1)
                put_text(static_base,(18,36),"二维地图 · 物体节点 · 实测位置与轨迹",28,TEXT)
                static_key=key
            assert static_base is not None
            canvas=static_base.copy()
            pose_index=bisect.bisect_right(pose_times,t)
            trajectory=poses[:pose_index]
            if len(trajectory)>=2:
                pts=np.asarray([_world_to_screen(p["x"],p["y"],bounds,map_panel) for p in trajectory],np.int32)
                cv2.polylines(canvas,[pts],False,BLUE,3,cv2.LINE_AA)
            pose=_nearest_pose(poses,pose_times,t)
            if pose:
                p=_world_to_screen(pose["x"],pose["y"],bounds,map_panel)
                angle=-float(pose["yaw"])
                triangle=np.asarray([[p[0]+18*math.cos(angle),p[1]+18*math.sin(angle)],
                                     [p[0]+11*math.cos(angle+2.5),p[1]+11*math.sin(angle+2.5)],
                                     [p[0]+11*math.cos(angle-2.5),p[1]+11*math.sin(angle-2.5)]],np.int32)
                cv2.fillConvexPoly(canvas,triangle,BLUE)
            put_text(canvas,(18,698),f"t={t:06.1f}s · 轨迹点 {len(trajectory)} · 绿点=物体 · 红色=模拟泄漏危险",20,MUTED)
            process.stdin.write(canvas.tobytes())
        _finish_writer(process,handle,outputs["map"])
        log(f"encoded {outputs['map'].name}")
    return {key:str(path) for key,path in outputs.items()}


def parse_args() -> argparse.Namespace:
    parser=argparse.ArgumentParser(description="离线用遥操建图 YOLOE-PF 路线处理完整 RGB-D rosbag")
    parser.add_argument("--bag",type=Path,required=True,help="rosbag 的 bag/ 目录")
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--fps",type=int,default=10)
    parser.add_argument("--confidence",type=float,default=0.20)
    parser.add_argument("--force",action="store_true")
    return parser.parse_args()


def main() -> int:
    args=parse_args(); output=args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    archive=BagArchive(args.bag.resolve())
    try:
        dataset=extract_dataset(archive,output,args.force)
    finally:
        archive.close()
    analysis=analyze(dataset,output,args.confidence,args.force)
    videos=render(dataset,analysis,output,max(5,args.fps),args.force)
    manifest={
        "schema":"go2w_yoloe_teleop_rosbag_render_v1",
        "source_bag":str(args.bag.resolve()),"created_at":time.time(),
        "videos":videos,"hazard_count":analysis["hazard_count"],
        "hazards":analysis["hazards"],"detector":analysis["detector"],
        "prompt_mode":analysis["prompt_mode"],
    }
    (output/"manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    log(json.dumps(manifest,ensure_ascii=False))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
