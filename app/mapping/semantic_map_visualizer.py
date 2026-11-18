"""SLAM 地图与语义物体可视化（20260908 计划书 §24、§37、§38）。

离线、只读、只依赖 numpy + matplotlib。两种用法：

* 语义观测（§38）：``--object-events <worker.jsonl>`` 或 ``--objects <objects.json>``
  生成 ``objects.json``（每个持久实例的融合位置）与 ``semantic_observation_scatter.png``
  （每个实例的观测散点 + 融合质心；同实例若分成两团，说明定位链异常，而不是去重问题）。
* 地图重影（§37）：``--bag <bag 目录>`` 生成 ``map_topdown.png`` 与
  ``start_end_overlay.png``（起/终帧在 odom 系的重叠）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List

import numpy as np

if __package__ in (None, ""):  # 允许 python app/mapping/semantic_map_visualizer.py
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))

from app.mapping.map_quality_evaluator import (  # noqa: E402
    load_bag,
    load_object_events,
    load_objects,
    object_spread,
)


def build_objects(objects: Dict[str, List[List[float]]],
                  min_observations: int = 1) -> List[Dict[str, Any]]:
    """把"实例键 -> 观测列表"整理成可读报告项（§24 objects.json）。"""
    items: List[Dict[str, Any]] = []
    for key, observations in sorted(objects.items()):
        if len(observations) < min_observations:
            continue
        label, _, persistent_id = str(key).partition("#")
        spread = object_spread(observations)
        items.append({
            "instance_key": key,
            "label": label,
            "persistent_object_id": persistent_id or None,
            "position_map": spread["centroid"],
            "observation_count": spread["observations"],
            "centroid_rms_error_m": spread["centroid_rms_error_m"],
            "centroid_p95_error_m": spread["centroid_p95_error_m"],
        })
    return items


def _scatter(objects: Dict[str, List[List[float]]], out_png: str,
             frame_label: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(figsize=(9, 7))
    for key, observations in sorted(objects.items()):
        points = np.asarray(observations, dtype=float)
        axes.scatter(points[:, 0], points[:, 1], s=26, alpha=0.8,
                     label=f"{key} (n={len(points)})")
        centroid = points.mean(axis=0)
        axes.plot(centroid[0], centroid[1], marker="+", markersize=14,
                  markeredgewidth=2, color="black")
    axes.set_xlabel(f"{frame_label} x [m]")
    axes.set_ylabel(f"{frame_label} y [m]")
    axes.set_title("semantic observations (black + = fused centroid)")
    axes.set_aspect("equal", adjustable="datalim")
    axes.grid(alpha=0.3)
    if objects:
        axes.legend(fontsize=7, loc="best")
    figure.tight_layout()
    figure.savefig(out_png, dpi=130)
    plt.close(figure)


def _map_figures(bag: Dict[str, Any], out_dir: str) -> List[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    written: List[str] = []
    cloud = np.asarray(bag["points"], dtype=float)
    if len(cloud):
        figure, axes = plt.subplots(figsize=(8, 8))
        axes.scatter(cloud[:, 0], cloud[:, 1], s=0.6, c=cloud[:, 2],
                     cmap="viridis", linewidths=0)
        axes.set_title(f"authoritative map top-down ({len(cloud)} points)")
        axes.set_xlabel("map x [m]")
        axes.set_ylabel("map y [m]")
        axes.set_aspect("equal")
        figure.tight_layout()
        path = os.path.join(out_dir, "map_topdown.png")
        figure.savefig(path, dpi=150)
        plt.close(figure)
        written.append(path)

    first, last = bag.get("first_scan"), bag.get("last_scan")
    if first is not None and last is not None:
        figure, axes = plt.subplots(figsize=(8, 8))
        axes.scatter(first[1][:, 0], first[1][:, 1], s=1.0, c="tab:blue",
                     label=f"start scan ({len(first[1])} pts)")
        axes.scatter(last[1][:, 0], last[1][:, 1], s=1.0, c="tab:red",
                     alpha=0.6, label=f"end scan ({len(last[1])} pts)")
        axes.set_title("start/end submap overlay (odom frame)")
        axes.set_xlabel("x [m]")
        axes.set_ylabel("y [m]")
        axes.set_aspect("equal")
        axes.legend(fontsize=8)
        figure.tight_layout()
        path = os.path.join(out_dir, "start_end_overlay.png")
        figure.savefig(path, dpi=150)
        plt.close(figure)
        written.append(path)
    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object-events", default=None,
                        help="worker JSONL（semantic_frame_objects_localized）")
    parser.add_argument("--objects", default=None,
                        help="实例 JSON：{实例: [[x,y,z], ...]}")
    parser.add_argument("--bag", default=None, help="rosbag2 目录（地图重影可视化）")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--min-observations", type=int, default=1)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    written: List[str] = []
    summary: Dict[str, Any] = {"schema_version": "semantic_map_preview_v1"}

    if args.object_events or args.objects:
        objects = (load_object_events(args.object_events) if args.object_events
                   else load_objects(args.objects))
        items = build_objects(objects, min_observations=args.min_observations)
        summary["objects"] = items
        summary["instance_count"] = len(items)
        objects_path = os.path.join(args.output_dir, "objects.json")
        with open(objects_path, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)
        written.append(objects_path)
        if objects:
            png = os.path.join(args.output_dir, "semantic_observation_scatter.png")
            _scatter(objects, png, "pslam_map")
            written.append(png)

    if args.bag:
        bag = load_bag(args.bag)
        summary["map"] = {
            "point_count": int(len(bag["points"])),
            "loop_closure_count": int(bag.get("loop_edges", 0)),
            "keyframes": int(bag.get("keyframes", 0)),
        }
        written.extend(_map_figures(bag, args.output_dir))

    print(json.dumps({"written": written, "summary": summary},
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
