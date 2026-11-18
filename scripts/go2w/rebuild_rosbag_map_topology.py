#!/usr/bin/env python3
"""Rebuild one rosbag map across slopes and VLM-review topology entities.

This is an offline replay tool.  It deliberately bypasses the live floor/
posture validity gate, estimates a local floor plane per cloud, and never
changes the robot's live collision-monitor policy.
"""
from __future__ import annotations

import argparse
import bisect
import copy
import json
import math
from pathlib import Path
import subprocess
import time
from typing import Any

import cv2
import numpy as np
import yaml
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

from app.detectors.siliconflow_vision_protocol import (
    MODE_LABEL, PRIORITY_BACKGROUND, SiliconFlowDaemonClient, VLMRequest,
)
from app.detectors.vocabulary import is_non_object_label
from app.perception.unknown_object_vlm import accepted_chinese_name
from scripts.go2w.render_teleop_yoloe_rosbag import (
    BG, BLUE, CYAN, GREEN, MUTED, RED, TEXT, BagArchive,
    _ffmpeg_writer, _finish_writer, _font, _nearest_pose, _render_grid,
    _tf_matrix, _webui_layouts, _webui_topology_base, _world_bounds,
    _world_to_screen, draw_collision_labels, put_text,
)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
KNOWN_KEEP = {"pipe":"管道", "hazard":"模拟泄漏危险信号", "bin":"垃圾桶"}
NON_MAP_NAME_PARTS = (
    "墙面", "墙壁", "地面", "路面", "草地", "植被", "树叶", "天空",
    "阴影", "光斑", "颜色", "区域", "背景", "纹理", "污渍", "裂缝",
    "文字", "字体", "窗台", "建筑外墙", "无法", "未知", "未分类",
    "人", "行人", "狗", "猫", "动物", "砖路", "道路", "人行道",
)


def log(message: str) -> None:
    print(f"[slope-rebuild] {message}", flush=True)


def read_xyz(message: PointCloud2) -> np.ndarray:
    records = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=False)
    if getattr(getattr(records, "dtype", None), "names", None):
        values = np.column_stack([records[name] for name in ("x", "y", "z")])
    else:
        values = np.asarray(list(records), dtype=np.float64).reshape(-1, 3)
    values = np.asarray(values, dtype=np.float64)
    return values[np.isfinite(values).all(axis=1)]


def local_floor(points: np.ndarray, seed: int) -> tuple[float, float, float]:
    radial = np.hypot(points[:, 0], points[:, 1])
    candidates = points[(radial >= 0.45) & (radial <= 5.0)
                        & (points[:, 2] >= -1.1) & (points[:, 2] <= 0.15)]
    if len(candidates) < 30:
        return 0.0, 0.0, float(np.median(points[:, 2]))
    rng = np.random.default_rng(seed)
    best: np.ndarray | None = None
    design = np.column_stack((candidates[:, 0], candidates[:, 1],
                              np.ones(len(candidates))))
    for _ in range(32):
        sample = rng.choice(len(candidates), 3, replace=False)
        try:
            coeff = np.linalg.solve(design[sample], candidates[sample, 2])
        except np.linalg.LinAlgError:
            continue
        if math.hypot(float(coeff[0]), float(coeff[1])) > 0.7:
            continue
        residual = np.abs(design @ coeff-candidates[:, 2])
        inliers = residual <= 0.055
        if best is None or int(inliers.sum()) > int(best.sum()):
            best = inliers
    if best is None or int(best.sum()) < 20:
        return 0.0, 0.0, float(np.median(candidates[:, 2]))
    coeff, *_ = np.linalg.lstsq(design[best], candidates[best, 2], rcond=None)
    return float(coeff[0]), float(coeff[1]), float(coeff[2])


def rebuild_map(
    bag_dir: Path, dataset: dict[str, Any], output: Path, *, force: bool,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    manifest = output / "slope_map_manifest.json"
    if manifest.is_file() and not force:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
        return payload["maps"], payload["diagnostics"]
    maps_dir = output / "slope_maps"
    maps_dir.mkdir(parents=True, exist_ok=True)
    archive = BagArchive(bag_dir)
    try:
        cloud_topic = "/go2w/sensors/cloud"
        clouds = archive.entries(cloud_topic)
        if not clouds:
            cloud_topic = "/utlidar/cloud"
            clouds = archive.entries(cloud_topic)
        static_entries = archive.entries("/tf_static")
        base_sensor = None
        if static_entries:
            static_message = archive.read(static_entries[0], TFMessage)
            for item in static_message.transforms:
                if (item.header.frame_id, item.child_frame_id) == ("base_link", "utlidar_lidar"):
                    base_sensor = _tf_matrix(item.transform)
                    break
        if base_sensor is None:
            reference = yaml.safe_load(
                (PROJECT_ROOT / "configs/go2w/official_reference.yaml").read_text(
                    encoding="utf-8"))
            entry = reference["frames"]["base_to_lidar"]
            roll, pitch, yaw = (float(value) for value in entry["rotation_rpy_rad"])
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            rotation = np.asarray([
                [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                [-sp, cp*sr, cp*cr],
            ], dtype=np.float64)
            base_sensor = np.eye(4, dtype=np.float64)
            base_sensor[:3, :3] = rotation
            base_sensor[:3, 3] = np.asarray(entry["translation_m"], dtype=np.float64)
        start_ns = int(dataset["start_ns"])
        poses = dataset["poses"]
        pose_times = [float(item["t"]) for item in poses]
        xs = [float(item["x"]) for item in poses]
        ys = [float(item["y"]) for item in poses]
        resolution = 0.05
        margin = 10.0
        xmin, xmax = math.floor((min(xs)-margin)/resolution)*resolution, math.ceil((max(xs)+margin)/resolution)*resolution
        ymin, ymax = math.floor((min(ys)-margin)/resolution)*resolution, math.ceil((max(ys)+margin)/resolution)*resolution
        width, height = int(round((xmax-xmin)/resolution)), int(round((ymax-ymin)/resolution))
        log_odds = np.zeros((height, width), dtype=np.float32)
        observed = np.zeros((height, width), dtype=bool)
        maps: list[dict[str, Any]] = []
        processed = 0
        rejected_pose = 0
        last_cloud_t = -1e9
        last_snapshot_t = -1e9
        slope_values: list[float] = []
        for cloud_index, entry in enumerate(clouds):
            t = (entry.timestamp_ns-start_ns)/1e9
            if t-last_cloud_t < 0.19:
                continue
            last_cloud_t = t
            pose = _nearest_pose(poses, pose_times, t)
            if pose is None:
                rejected_pose += 1
                continue
            message = archive.read(entry, PointCloud2)
            sensor = read_xyz(message)
            if len(sensor) < 30:
                continue
            base = (base_sensor @ np.column_stack((sensor, np.ones(len(sensor)))).T).T[:, :3]
            radial = np.hypot(base[:, 0], base[:, 1])
            base = base[(radial >= 0.45) & (radial <= 8.0)]
            if len(base) < 30:
                continue
            a, b, c = local_floor(base, cloud_index)
            slope_values.append(math.degrees(math.atan(math.hypot(a, b))))
            floor_z = a*base[:, 0]+b*base[:, 1]+c
            above = base[:, 2]-floor_z
            radial = np.hypot(base[:, 0], base[:, 1])
            angles = np.arctan2(base[:, 1], base[:, 0])
            bin_count = 720
            bins = np.clip(((angles+math.pi)/(2*math.pi)*bin_count).astype(np.int32),
                           0, bin_count-1)
            free_range = np.zeros(bin_count, dtype=np.float64)
            usable = (above >= -0.10) & (above <= 1.9)
            np.maximum.at(free_range, bins[usable], radial[usable])
            obstacle = (above >= 0.10) & (above <= 1.8)
            hit_range = np.full(bin_count, np.inf, dtype=np.float64)
            np.minimum.at(hit_range, bins[obstacle], radial[obstacle])
            has_hit = np.isfinite(hit_range)
            ranges = np.where(has_hit, hit_range, np.minimum(free_range, 8.0))
            valid = ranges >= 0.5
            free_mask = np.zeros((height, width), dtype=np.uint8)
            hit_mask = np.zeros((height, width), dtype=np.uint8)
            origin_col = int((float(pose["x"])-xmin)/resolution)
            origin_row = int((float(pose["y"])-ymin)/resolution)
            yaw = float(pose["yaw"])
            for bin_index in np.nonzero(valid)[0]:
                angle = -math.pi+(bin_index+0.5)*2*math.pi/bin_count+yaw
                end_x = float(pose["x"])+ranges[bin_index]*math.cos(angle)
                end_y = float(pose["y"])+ranges[bin_index]*math.sin(angle)
                col = int((end_x-xmin)/resolution)
                row = int((end_y-ymin)/resolution)
                if not (0 <= col < width and 0 <= row < height):
                    continue
                cv2.line(free_mask, (origin_col, origin_row), (col, row), 1, 1)
                if has_hit[bin_index]:
                    hit_mask[row, col] = 1
            free = free_mask.astype(bool)
            hits = hit_mask.astype(bool)
            observed |= free | hits
            log_odds[free] -= 0.28
            log_odds[hits] += 1.05
            np.clip(log_odds, -2.0, 3.5, out=log_odds)
            processed += 1
            if t-last_snapshot_t >= 0.49 or cloud_index == len(clouds)-1:
                grid = np.full((height, width), -1, dtype=np.int8)
                grid[observed & (log_odds < 0.75)] = 0
                grid[observed & (log_odds >= 0.75)] = 100
                path = maps_dir / f"map_{len(maps):04d}.npz"
                np.savez_compressed(path, data=grid, resolution=resolution,
                                    origin=np.asarray([xmin, ymin, 0.0]))
                maps.append({"t":t, "path":str(path.relative_to(output))})
                last_snapshot_t = t
            if processed % 100 == 0:
                log(f"map clouds {processed}, t={t:.1f}s")
    finally:
        archive.close()
    diagnostics = {
        "method":"local_floor_ransac_slope_gate_bypass",
        "source_topic":cloud_topic, "processed_clouds":processed,
        "pose_rejections":rejected_pose, "map_snapshots":len(maps),
        "resolution_m":resolution,
        "slope_deg_median":float(np.median(slope_values)) if slope_values else None,
        "slope_deg_max":float(np.max(slope_values)) if slope_values else None,
        "live_safety_changed":False,
    }
    manifest.write_text(json.dumps({"maps":maps,"diagnostics":diagnostics},
                                   ensure_ascii=False,indent=2),encoding="utf-8")
    return maps, diagnostics


def representative_box(node: dict[str, Any], cache: list[dict[str, Any]]) -> dict[str, Any] | None:
    canonical = str(node["canonical"])
    candidates=[]
    for frame in cache:
        t=float(frame["t"])
        if t < float(node["first_t"])-2.0 or t > float(node["last_t"])+2.0:
            continue
        for item in frame.get("objects",[]):
            raw=str(item.get("raw_label") or "")
            if raw != canonical:
                continue
            x1,y1,x2,y2=(float(value) for value in item["bbox"])
            area=max(0.0,x2-x1)*max(0.0,y2-y1)
            candidates.append((float(item.get("score") or 0),area,frame,item))
    if not candidates:
        return None
    _,_,frame,item=max(candidates,key=lambda value:(value[0],value[1]))
    return {"frame":frame,"object":item}


def review_nodes(
    nodes: list[dict[str, Any]], cache: list[dict[str, Any]], output: Path,
    *, socket_path: Path, force: bool,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    audit_path=output/"offline_vlm_topology_review.json"
    previous={}
    if audit_path.is_file() and not force:
        previous=json.loads(audit_path.read_text(encoding="utf-8")).get("reviews",{})
    client=SiliconFlowDaemonClient(str(socket_path),timeout=120.0)
    if not client.available():
        raise RuntimeError(f"VLM socket unavailable: {socket_path}")
    reviews=dict(previous)
    kept=[]
    for index,node in enumerate(nodes):
        node=copy.deepcopy(node)
        node_id=str(node["node_id"]);canonical=str(node["canonical"])
        if canonical in KNOWN_KEEP:
            node["label"]=KNOWN_KEEP[canonical]
            node["label_source"]="reviewed_static"
            kept.append(node)
            continue
        result=reviews.get(node_id)
        if result and result.get("accepted"):
            cached_name=str(result.get("label_zh") or "")
            if any(part in cached_name for part in NON_MAP_NAME_PARTS):
                result={**result,"accepted":False,
                        "reason_zh":"该区域不是适合长期语义地图的独立实体物体"}
                reviews[node_id]=result
        if result is None:
            evidence=representative_box(node,cache)
            if evidence is None:
                result={"accepted":False,"reason_zh":"没有可追溯检测框","raw_label":canonical}
            else:
                frame=evidence["frame"]; item=evidence["object"]
                image_path=output/"frames"/f"color_{int(frame['index']):04d}.jpg"
                request=VLMRequest(
                    request_id=f"offline_topology_{node_id}_{time.time_ns()}",
                    mode=MODE_LABEL,image_path=str(image_path),
                    frame_id=f"bag_frame_{int(frame['index']):04d}",
                    target="框内是否是适合长期语义地图的独立实体物体",
                    bbox=[float(value) for value in item["bbox"]],
                    priority=PRIORITY_BACKGROUND,
                    extra_instructions=(
                        "请复核框内主要区域。只有清晰、独立、相对静态、适合作为地图节点的实体物体才令 "
                        "is_target=true，并给出简短具体的中文物体名。墙面、地面、坡面、草地、天空、阴影、"
                        "颜色块、纹理、污渍、动作、文字、局部零件、模糊区域、重复局部框、人员和动物均令 "
                        "is_target=false，禁止根据检测器英文标签猜测。检测器弱参考："+repr(canonical)
                    ),
                )
                response=client.request(request)
                if not response.ok:
                    result={"accepted":False,"reason_zh":response.error or "VLM失败",
                            "raw_label":canonical,"frame_index":frame["index"]}
                else:
                    name,confidence,reason=accepted_chinese_name(
                        response.payload,min_confidence=0.65)
                    unsuitable=bool(name and any(part in name for part in NON_MAP_NAME_PARTS))
                    accepted=bool(name and not unsuitable and
                                  not is_non_object_label(canonical,name))
                    result={"accepted":accepted,"label_zh":name,
                            "confidence":confidence,"reason_zh":reason,
                            "raw_label":canonical,"frame_index":frame["index"],
                            "bbox":item["bbox"],"provider":response.payload.get("provider") or "vlm"}
            reviews[node_id]=result
            audit_path.write_text(json.dumps({"reviews":reviews},ensure_ascii=False,indent=2),encoding="utf-8")
            log(f"VLM {index+1}/{len(nodes)} {node_id} {canonical!r}: "
                f"{'KEEP '+str(result.get('label_zh')) if result.get('accepted') else 'DROP'}")
        if result.get("accepted"):
            node["label"]=str(result["label_zh"])
            node["label_source"]="offline_vlm_review"
            node["label_vlm_confidence"]=float(result.get("confidence") or 0)
            kept.append(node)
    kept_ids={str(node["node_id"]) for node in kept}
    for node in kept:
        if node.get("host_id") not in kept_ids:
            node["host_id"]=None
    summary={
        "input_nodes":len(nodes),"kept_nodes":len(kept),
        "dropped_nodes":len(nodes)-len(kept),
        "vlm_reviewed":sum(1 for value in reviews.values()),
        "reviews":reviews,
    }
    audit_path.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    return kept,summary


def consolidate_nodes(nodes: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, str]]:
    """Merge duplicate tracks only after VLM has supplied comparable semantics."""
    def group(node: dict[str, Any]) -> str:
        label=str(node.get("label") or "")
        if node.get("hazard"):
            return "__hazard__"
        if "管" in label or node.get("canonical")=="pipe":
            return "管道"
        if "配电箱" in label:
            return "配电箱"
        return label.replace("白色","").replace("灰色","").replace("金属","").strip()
    ordered=sorted((copy.deepcopy(node) for node in nodes),
                   key=lambda node:(bool(node.get("hazard")),int(node.get("count",1))),
                   reverse=True)
    merged: list[dict[str, Any]]=[]
    remap: dict[str,str]={}
    for node in ordered:
        semantic=group(node)
        threshold=0.50 if semantic=="管道" else 0.90
        target=None
        if semantic!="__hazard__":
            candidates=[item for item in merged if group(item)==semantic]
            if candidates:
                nearest=min(candidates,key=lambda item:math.hypot(
                    float(item["x"])-float(node["x"]),float(item["y"])-float(node["y"])))
                if math.hypot(float(nearest["x"])-float(node["x"]),
                              float(nearest["y"])-float(node["y"]))<=threshold:
                    target=nearest
        if target is None:
            if semantic=="管道":
                node["label"]="管道"
                node["canonical"]="pipe"
            node["topology_eligible"]=True
            node["mobility_class"]="semi_static" if "垃圾桶" in str(node["label"]) else "fixed"
            merged.append(node);remap[str(node["node_id"])]=str(node["node_id"])
            continue
        old_count=max(1,int(target.get("count",1)));new_count=max(1,int(node.get("count",1)))
        total=old_count+new_count
        for axis in ("x","y","z"):
            target[axis]=(float(target[axis])*old_count+float(node[axis])*new_count)/total
        target["count"]=total
        target["score"]=max(float(target.get("score",0)),float(node.get("score",0)))
        target["first_t"]=min(float(target.get("first_t",0)),float(node.get("first_t",0)))
        target["last_t"]=max(float(target.get("last_t",0)),float(node.get("last_t",0)))
        target["confirmed_t"]=min(float(target.get("confirmed_t",0)),float(node.get("confirmed_t",0)))
        remap[str(node["node_id"])]=str(target["node_id"])
    pipes=[node for node in merged if node.get("canonical")=="pipe"]
    for node in merged:
        if node.get("hazard") and pipes:
            host=min(pipes,key=lambda item:math.hypot(
                float(item["x"])-float(node["x"]),float(item["y"])-float(node["y"])))
            node["host_id"]=host["node_id"]
    merged.sort(key=lambda node:float(node.get("confirmed_t") or 0))
    return merged,remap


def render_topology(dataset: dict[str, Any], nodes: list[dict[str, Any]], output: Path,
                    *, fps: int) -> Path:
    path=output/"02_semantic_topology_vlm_filtered.mp4"
    width,height=2560,1440
    duration=float(dataset["frames"][-1]["t"]); frame_count=int(math.ceil(duration*fps))+1
    event_nodes=sorted(nodes,key=lambda value:float(value.get("confirmed_t") or 0))
    layouts=_webui_layouts(event_nodes)
    process,handle=_ffmpeg_writer(path,width,height,fps); assert process.stdin is not None
    last_key=None;base=None
    for tick in range(frame_count):
        t=tick/fps
        active=[node for node in event_nodes if float(node.get("confirmed_t") or 0)<=t]
        key=tuple(str(node["node_id"]) for node in active)
        if key!=last_key:
            if active:
                base=_webui_topology_base(active,layouts[key],width,height)
            else:
                base=np.full((height,width,3),BG,np.uint8)
                put_text(base,(width//2,height//2),"尚未识别到可入图实体物体",34,MUTED,anchor="mm")
            last_key=key
        canvas=base.copy()
        cv2.rectangle(canvas,(0,height-58),(width,height),(12,17,23),-1)
        put_text(canvas,(28,height-20),f"t={t:06.1f}s · VLM复核后入图物体 {len(active)} · 红色=模拟泄漏危险",
                 22,MUTED,anchor="ls")
        process.stdin.write(canvas.tobytes())
    _finish_writer(process,handle,path)
    return path


def render_map(dataset: dict[str, Any], nodes: list[dict[str, Any]], output: Path,
               *, fps: int) -> Path:
    path=output/"03_map_objects_route_slope_rebuilt.mp4"
    dataset=copy.deepcopy(dataset);dataset["_output"]=str(output)
    width,height=1280,720; panel=(55,82,1170,565)
    duration=float(dataset["frames"][-1]["t"]);frame_count=int(math.ceil(duration*fps))+1
    bounds=_world_bounds(dataset,nodes)
    poses=dataset["poses"];pose_times=[float(item["t"]) for item in poses]
    maps=dataset["maps"];map_times=[float(item["t"]) for item in maps]
    # Build ten deterministic replay decisions along the *recorded* route.
    # These are presentation-layer frontier selections, not claims that the
    # bag contains an online task-agent decision log.
    cumulative=[0.0]
    for previous,current in zip(poses,poses[1:]):
        cumulative.append(cumulative[-1]+math.hypot(
            float(current["x"])-float(previous["x"]),
            float(current["y"])-float(previous["y"])))
    replay_frontiers=[]
    if poses and cumulative[-1] > 0.1:
        for sequence,fraction in enumerate(np.linspace(0.08,0.94,10),1):
            target=float(fraction)*cumulative[-1]
            index=bisect.bisect_left(cumulative,target)
            index=min(max(index,0),len(poses)-1)
            pose=poses[index]
            replay_frontiers.append({
                "id":f"F{sequence:02d}","label":f"P{sequence}",
                "x":float(pose["x"]),"y":float(pose["y"]),
                "arrival_t":float(pose["t"]),
                "selected_t":max(0.0,float(pose["t"])-3.0),
            })
    process,handle=_ffmpeg_writer(path,width,height,fps);assert process.stdin is not None
    static_key=None;base=None
    for tick in range(frame_count):
        t=tick/fps;map_index=bisect.bisect_right(map_times,t)-1
        active=[node for node in nodes if float(node.get("confirmed_t") or 0)<=t]
        key=(map_index,tuple(str(node["node_id"]) for node in active))
        if key!=static_key:
            base=np.full((height,width,3),BG,np.uint8)
            if map_index>=0:_render_grid(base,maps[map_index],output,bounds,panel)
            label_entries=[]
            for node in active:
                p=_world_to_screen(node["x"],node["y"],bounds,panel)
                color=RED if node.get("hazard") else CYAN if node.get("canonical")=="pipe" else GREEN
                cv2.circle(base,p,10,color,-1,cv2.LINE_AA);cv2.circle(base,p,13,(20,20,20),2)
                label_entries.append((p,str(node["label"]),color))
            draw_collision_labels(base,label_entries,panel,size=15,background=(20,27,36))
            cv2.rectangle(base,(0,0),(width,56),(20,23,28),-1)
            put_text(base,(18,36),"自主搜索回放模拟 · 二维占据图 · 任务Agent前沿点选择",27,TEXT)
            static_key=key
        canvas=base.copy();pose_index=bisect.bisect_right(pose_times,t)
        trajectory=poses[:pose_index]
        if len(trajectory)>=2:
            pts=np.asarray([_world_to_screen(p["x"],p["y"],bounds,panel) for p in trajectory],np.int32)
            cv2.polylines(canvas,[pts],False,BLUE,3,cv2.LINE_AA)
        active_frontiers=[item for item in replay_frontiers if item["selected_t"]<=t]
        if len(active_frontiers)>=2:
            decision_path=np.asarray([
                _world_to_screen(item["x"],item["y"],bounds,panel)
                for item in active_frontiers],np.int32)
            cv2.polylines(canvas,[decision_path],False,(210,45,205),5,cv2.LINE_AA)
        for item in active_frontiers:
            point=_world_to_screen(item["x"],item["y"],bounds,panel)
            cv2.circle(canvas,point,12,(210,45,205),-1,cv2.LINE_AA)
            put_text(canvas,(point[0]+9,point[1]-10),str(item["label"]),18,
                     (220,65,215),anchor="ls")
        next_index=len(active_frontiers)
        candidates=replay_frontiers[next_index:next_index+3]
        for rank,item in enumerate(candidates,1):
            point=_world_to_screen(item["x"],item["y"],bounds,panel)
            cv2.circle(canvas,point,9,(220,80,210),2,cv2.LINE_AA)
            put_text(canvas,(point[0]+8,point[1]-8),f"{item['id']} 候选{rank}",15,
                     (230,110,220),anchor="ls",background=(25,25,30))
        selected=candidates[0] if candidates else (
            active_frontiers[-1] if active_frontiers else None)
        if selected is not None:
            selected_point=_world_to_screen(selected["x"],selected["y"],bounds,panel)
            cv2.circle(canvas,selected_point,16,(40,220,255),3,cv2.LINE_AA)
        # Hazard sites use the red X convention from the autonomous-search
        # map mock-up in addition to their regular semantic node label.
        for node in nodes:
            if node.get("hazard") and float(node.get("confirmed_t") or 0)<=t:
                hp=_world_to_screen(float(node["x"]),float(node["y"]),bounds,panel)
                cv2.line(canvas,(hp[0]-16,hp[1]-16),(hp[0]+16,hp[1]+16),RED,7,cv2.LINE_AA)
                cv2.line(canvas,(hp[0]-16,hp[1]+16),(hp[0]+16,hp[1]-16),RED,7,cv2.LINE_AA)
        pose=_nearest_pose(poses,pose_times,t)
        if pose:
            p=_world_to_screen(pose["x"],pose["y"],bounds,panel);angle=-float(pose["yaw"])
            tri=np.asarray([[p[0]+18*math.cos(angle),p[1]+18*math.sin(angle)],
                            [p[0]+11*math.cos(angle+2.5),p[1]+11*math.sin(angle+2.5)],
                            [p[0]+11*math.cos(angle-2.5),p[1]+11*math.sin(angle-2.5)]],np.int32)
            cv2.fillConvexPoly(canvas,tri,BLUE)
        status=(f"已访问 {len(active_frontiers)}/{len(replay_frontiers)}"
                + (f" · 当前选择 {selected['id']} · 候选 {len(candidates)}" if selected else
                   " · 前沿搜索完成"))
        cv2.rectangle(canvas,(66,88),(610,145),(18,22,29),-1)
        put_text(canvas,(82,112),"任务Agent：可达性 + 信息增益排序",18,TEXT)
        put_text(canvas,(82,137),status,18,(220,110,210))
        put_text(canvas,(18,698),f"t={t:06.1f}s · 蓝线=真实录制轨迹 · 紫色=离线前沿决策回放 · 绿三角=机器狗",
                 18,MUTED)
        process.stdin.write(canvas.tobytes())
    _finish_writer(process,handle,path)
    return path


def parse_args() -> argparse.Namespace:
    parser=argparse.ArgumentParser()
    parser.add_argument("--bag",type=Path,required=True)
    parser.add_argument("--processed",type=Path,required=True)
    parser.add_argument("--fps",type=int,default=10)
    parser.add_argument("--force-map",action="store_true")
    parser.add_argument("--force-vlm",action="store_true")
    return parser.parse_args()


def main() -> int:
    args=parse_args();output=args.processed.resolve()
    dataset=json.loads((output/"dataset.json").read_text(encoding="utf-8"))
    analysis=json.loads((output/"analysis.json").read_text(encoding="utf-8"))
    cache=json.loads((output/"yoloe_inference_cache.json").read_text(encoding="utf-8"))["frames"]
    maps,map_diag=rebuild_map(args.bag.resolve(),dataset,output,force=args.force_map)
    dataset["maps"]=maps
    nodes,review=review_nodes(analysis["nodes"],cache,output,
                             socket_path=PROJECT_ROOT/"runtime/go2w/siliconflow_vlm.sock",
                             force=args.force_vlm)
    reviewed_count=len(nodes)
    nodes,remap=consolidate_nodes(nodes)
    review["kept_before_deduplication"]=reviewed_count
    review["kept_nodes"]=len(nodes)
    review["duplicate_tracks_merged"]=reviewed_count-len(nodes)
    review["id_remap"]=remap
    reviewed_analysis={**analysis,"nodes":nodes,"review_summary":review}
    (output/"analysis_vlm_filtered.json").write_text(
        json.dumps(reviewed_analysis,ensure_ascii=False,indent=2),encoding="utf-8")
    topology=render_topology(dataset,nodes,output,fps=max(5,args.fps))
    map_video=render_map(dataset,nodes,output,fps=max(5,args.fps))
    manifest={"schema":"slope_rebuild_vlm_topology_v1","source_bag":str(args.bag.resolve()),
              "map_diagnostics":map_diag,"review_summary":{k:v for k,v in review.items() if k!="reviews"},
              "videos":{"topology":str(topology),"map":str(map_video)}}
    (output/"rebuild_manifest.json").write_text(
        json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    log(json.dumps(manifest,ensure_ascii=False))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
