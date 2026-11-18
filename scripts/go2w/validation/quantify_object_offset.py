#!/usr/bin/env python3
"""量化语义物体相对地图的整体偏移：物体点到最近地图点的距离 + 物体周边局部渲染。

判断口径：
  * 若物体真落在地图结构上（桌面上、柜子前），到最近地图点的距离应为几厘米；
  * 若整堆物体系统性偏移，该距离会整体偏大（十几厘米到米级），且方向一致。
"""
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

map_json, events_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
payload = json.load(open(map_json))
points = np.asarray(payload.get("points") or [], dtype=float)

objects = {}
for line in open(events_path, encoding="utf-8"):
    try:
        event = json.loads(line)
    except json.JSONDecodeError:
        continue
    if event.get("event") != "semantic_object_localized":
        continue
    obj = event.get("object") or {}
    xyz = obj.get("map_xyz")
    if not xyz:
        continue
    objects[obj.get("object_id") or obj.get("label")] = (
        obj.get("label"), np.asarray(xyz, dtype=float), obj.get("observation_count"))

print(f"map points={len(points)}  objects={len(objects)}")
tree = cKDTree(points[:, :2])
offsets = []
for key, (label, xyz, count) in objects.items():
    distance, index = tree.query(xyz[:2], k=1)
    nearest = points[index]
    delta = xyz[:2] - nearest[:2]
    offsets.append(delta)
    print(f"{label:<8} obs={count}  map=({xyz[0]:6.2f},{xyz[1]:6.2f})  "
          f"nearest_map_point=({nearest[0]:6.2f},{nearest[1]:6.2f})  "
          f"dist={distance:5.2f} m  offset=({delta[0]:+.2f},{delta[1]:+.2f})")

if offsets:
    array = np.asarray(offsets)
    print(f"\nmean offset = ({array[:, 0].mean():+.3f}, {array[:, 1].mean():+.3f}) m")
    print(f"offset spread (std) = ({array[:, 0].std():.3f}, {array[:, 1].std():.3f}) m")

figure, axes = plt.subplots(1, 2, figsize=(19, 8))
axes[0].scatter(points[:, 0], points[:, 1], s=0.4, c="lightsteelblue", linewidths=0)
for key, (label, xyz, _) in objects.items():
    axes[0].scatter(xyz[0], xyz[1], s=140, marker="*", c="crimson",
                    edgecolors="black", linewidths=0.6, zorder=5)
    axes[0].annotate(label, (xyz[0], xyz[1]), textcoords="offset points",
                     xytext=(8, 6), fontsize=10, color="darkred")
    distance, index = tree.query(xyz[:2], k=1)
    axes[0].plot([xyz[0], points[index][0]], [xyz[1], points[index][1]],
                 color="orange", linewidth=1.0, zorder=4)
axes[0].set_title("objects vs map (orange = to nearest map point)")
axes[0].set_aspect("equal", adjustable="datalim")
axes[0].grid(alpha=0.3)
axes[0].set_xlabel("x [m]")
axes[0].set_ylabel("y [m]")

# 局部放大：物体所在区域 8 m 见方
centre = np.mean([xyz[:2] for _, xyz, _ in objects.values()], axis=0)
mask = ((np.abs(points[:, 0] - centre[0]) < 4) & (np.abs(points[:, 1] - centre[1]) < 4))
axes[1].scatter(points[mask, 0], points[mask, 1], s=1.2, c="steelblue", linewidths=0)
for key, (label, xyz, _) in objects.items():
    axes[1].scatter(xyz[0], xyz[1], s=120, marker="*", c="crimson",
                    edgecolors="black", linewidths=0.6, zorder=5)
    axes[1].annotate(label, (xyz[0], xyz[1]), textcoords="offset points",
                     xytext=(7, 5), fontsize=10, color="darkred")
axes[1].set_title("zoom: 8 m around objects")
axes[1].set_aspect("equal", adjustable="datalim")
axes[1].grid(alpha=0.3)
axes[1].set_xlabel("x [m]")
axes[1].set_ylabel("y [m]")
figure.tight_layout()
figure.savefig(out, dpi=130)
print("saved", out)
