#!/usr/bin/env python3
"""Render the rosbag camera view with the offline VLM topology review applied.

The prompt-free detector can emit open-vocabulary labels that are not useful
object names (for example ``villa`` for a wall).  The topology rebuild already
reviews spatial tracks and either assigns a Chinese entity name or rejects the
track.  This renderer applies that same decision to every camera box:

* known Chinese detector labels remain visible;
* an unclassified box is shown only when its localized observation belongs to
  an accepted reviewed track, and then uses the reviewed Chinese name;
* confirmed slime hazards remain red;
* rejected or untraceable unclassified boxes are omitted.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from render_teleop_yoloe_rosbag import (
    AMBER,
    GREEN,
    PIPE_ALIASES,
    RED,
    TEXT,
    _ffmpeg_writer,
    _finish_writer,
    put_text,
)


def _canonical(item: dict[str, Any]) -> str:
    if bool(item.get("hazard")):
        return "hazard"
    raw = " ".join(str(item.get("raw_label") or "").strip().lower().split())
    return "pipe" if raw in PIPE_ALIASES else raw


def _match_observations(
    objects: list[dict[str, Any]], observations: list[dict[str, Any]],
) -> dict[int, dict[str, Any]]:
    """Associate overlays with localized observations from the same YOLOE pass."""
    result: dict[int, dict[str, Any]] = {}
    unused = set(range(len(observations)))
    for object_index, item in enumerate(objects):
        canonical = _canonical(item)
        score = float(item.get("score") or 0.0)
        candidates = [
            index for index in unused
            if str(observations[index].get("canonical")) == canonical
        ]
        if not candidates:
            continue
        chosen = min(
            candidates,
            key=lambda index: abs(float(observations[index].get("score") or 0.0)-score),
        )
        # Scores are copied from the same detection.  A small tolerance also
        # supports older caches that rounded their JSON values.
        if abs(float(observations[chosen].get("score") or 0.0)-score) <= 0.025:
            result[object_index] = observations[chosen]
            unused.remove(chosen)
    return result


def _accepted_track_labels(
    original_nodes: list[dict[str, Any]], reviewed: dict[str, Any],
) -> tuple[dict[str, str], dict[str, dict[str, Any]]]:
    final_nodes = {str(node["node_id"]): node for node in reviewed["nodes"]}
    remap = {
        str(source): str(target)
        for source, target in (reviewed.get("review_summary", {}).get("id_remap", {}) or {}).items()
    }
    labels: dict[str, str] = {}
    originals = {str(node["node_id"]): node for node in original_nodes}
    for source_id, target_id in remap.items():
        if source_id in originals and target_id in final_nodes:
            labels[source_id] = str(final_nodes[target_id]["label"])
    return labels, originals


def _reviewed_label_for_observation(
    observation: dict[str, Any], t: float,
    accepted_labels: dict[str, str], original_nodes: dict[str, dict[str, Any]],
) -> str | None:
    xyz = observation.get("map_xyz")
    if not isinstance(xyz, list) or len(xyz) < 3:
        return None
    canonical = str(observation.get("canonical") or "")
    candidates: list[tuple[float, str]] = []
    for node_id, node in original_nodes.items():
        if str(node.get("canonical")) != canonical:
            continue
        if t < float(node.get("first_t") or 0.0)-0.8:
            continue
        if t > float(node.get("last_t") or 0.0)+0.8:
            continue
        distance = math.dist(
            (float(xyz[0]), float(xyz[1]), float(xyz[2])),
            (float(node["x"]), float(node["y"]), float(node["z"])),
        )
        candidates.append((distance, node_id))
    if not candidates:
        return None
    distance, node_id = min(candidates)
    threshold = 0.95 if canonical == "hazard" else 1.35
    if distance > threshold:
        return None
    return accepted_labels.get(node_id)


def _filter_frame(
    frame: dict[str, Any], accepted_labels: dict[str, str],
    original_nodes: dict[str, dict[str, Any]], stats: dict[str, int],
) -> list[dict[str, Any]]:
    objects = list(frame.get("objects") or [])
    observations = list(frame.get("observations") or [])
    matched = _match_observations(objects, observations)
    result: list[dict[str, Any]] = []
    t = float(frame.get("t") or 0.0)
    for index, source in enumerate(objects):
        stats["input_boxes"] += 1
        item = dict(source)
        label = str(item.get("label") or "")
        if item.get("hazard"):
            # The source analysis already contains only the two confirmed
            # mask-backed slime sites; keep their warning overlays red.
            item["label"] = "模拟泄漏危险信号"
            stats["hazard_boxes"] += 1
            result.append(item)
            continue
        if not label.startswith("未分类·"):
            stats["known_boxes"] += 1
            result.append(item)
            continue
        reviewed_label = None
        if index in matched:
            reviewed_label = _reviewed_label_for_observation(
                matched[index], t, accepted_labels, original_nodes)
        if reviewed_label:
            item["label"] = reviewed_label
            item["vlm_reviewed"] = True
            stats["vlm_relabelled_boxes"] += 1
            result.append(item)
        else:
            stats["dropped_unclassified_boxes"] += 1
    return result


def render(processed: Path, output: Path, fps: int) -> dict[str, int]:
    dataset = json.loads((processed / "dataset.json").read_text(encoding="utf-8"))
    analysis = json.loads((processed / "analysis.json").read_text(encoding="utf-8"))
    reviewed = json.loads(
        (processed / "analysis_vlm_filtered.json").read_text(encoding="utf-8")
    )
    accepted_labels, original_nodes = _accepted_track_labels(
        list(analysis["nodes"]), reviewed
    )
    detections = {int(frame["index"]): frame for frame in analysis["frames"]}
    frames = list(dataset["frames"])
    frame_times = [float(frame["t"]) for frame in frames]
    duration = float(frames[-1]["t"])
    frame_count = int(math.ceil(duration*fps))+1
    width, height = 1280, 720
    stats = {
        "input_boxes": 0,
        "known_boxes": 0,
        "hazard_boxes": 0,
        "vlm_relabelled_boxes": 0,
        "dropped_unclassified_boxes": 0,
        "output_boxes": 0,
    }
    process, handle = _ffmpeg_writer(output, width, height, fps)
    assert process.stdin is not None
    last_index = -1
    annotated: np.ndarray | None = None
    for tick in range(frame_count):
        t = tick/fps
        index = max(0, bisect.bisect_right(frame_times, t)-1)
        if index != last_index:
            source = cv2.imread(str(processed / frames[index]["color"]))
            if source is None:
                raise RuntimeError(f"cannot read {frames[index]['color']}")
            annotated = source.copy()
            frame = detections.get(int(frames[index]["index"]), {})
            objects = _filter_frame(frame, accepted_labels, original_nodes, stats)
            stats["output_boxes"] += len(objects)
            for item in objects:
                x1, y1, x2, y2 = [float(value) for value in item["bbox"]]
                p1 = (int(x1*width), int(y1*height))
                p2 = (int(x2*width), int(y2*height))
                color = RED if item.get("hazard") else AMBER if item.get("suspected") else GREEN
                cv2.rectangle(annotated, p1, p2, color, 4 if item.get("hazard") else 2)
                put_text(
                    annotated, (p1[0], max(56, p1[1]-5)),
                    f"{item['label']} {float(item['score']):.2f}", 20, TEXT,
                    anchor="ls", background=color,
                )
            cv2.rectangle(annotated, (0, 0), (width, 48), (20, 23, 28), -1)
            hazard_count = sum(bool(item.get("hazard")) for item in objects)
            put_text(
                annotated, (16, 31),
                f"YOLOE prompt-free + VLM复核 · t={t:06.1f}s · 当前危险框 {hazard_count}",
                24, TEXT,
            )
            last_index = index
        assert annotated is not None
        process.stdin.write(np.ascontiguousarray(annotated).tobytes())
    _finish_writer(process, handle, output)
    return stats


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("processed", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--fps", type=int, default=10)
    args = parser.parse_args()
    processed = args.processed.resolve()
    output = (args.output or processed / "01_camera_yoloe_vlm_filtered.mp4").resolve()
    stats = render(processed, output, max(1, args.fps))
    stats_path = output.with_suffix(".stats.json")
    stats_path.write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(output), **stats}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
