#!/usr/bin/env python3
"""把 Dataset D 的语义物体叠加到当前 SLAM 地图上（俯视 + 斜视），供操作员核对位置。"""
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

map_json, events_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
payload = json.load(open(map_json))
points = np.asarray(payload.get("points") or [], dtype=float)

objects = []
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
    objects.append((obj.get("label") or "?", np.asarray(xyz, dtype=float),
                    obj.get("confidence")))

print(f"map points={len(points)} objects={len(objects)}")
for label, xyz, confidence in objects:
    print(f"  {label:<8} ({xyz[0]:7.2f},{xyz[1]:7.2f},{xyz[2]:5.2f}) conf={confidence}")

figure, axes = plt.subplots(1, 2, figsize=(19, 8))
axes[0].scatter(points[:, 0], points[:, 1], s=0.4, c="lightsteelblue", linewidths=0)
for label, xyz, _ in objects:
    axes[0].scatter(xyz[0], xyz[1], s=140, marker="*", c="crimson",
                    edgecolors="black", linewidths=0.6, zorder=5)
    axes[0].annotate(label, (xyz[0], xyz[1]), textcoords="offset points",
                     xytext=(8, 6), fontsize=11, color="darkred")
axes[0].set_title(f"semantic objects on map (top-down, {len(objects)} objects)")
axes[0].plot(0, 0, marker="o", ms=9, color="black")
axes[0].annotate("origin", (0, 0), textcoords="offset points", xytext=(8, -12), fontsize=9)
axes[0].set_xlabel("x [m]")
axes[0].set_ylabel("y [m]")
axes[0].grid(alpha=0.3)
axes[0].set_aspect("equal", adjustable="datalim")

axes[1].scatter(points[:, 0], points[:, 2], s=0.4, c="lightsteelblue", linewidths=0)
for label, xyz, _ in objects:
    axes[1].scatter(xyz[0], xyz[2], s=140, marker="*", c="crimson",
                    edgecolors="black", linewidths=0.6, zorder=5)
    axes[1].annotate(label, (xyz[0], xyz[2]), textcoords="offset points",
                     xytext=(8, 6), fontsize=10, color="darkred")
axes[1].set_title("side view (x-z)")
axes[1].set_xlabel("x [m]")
axes[1].set_ylabel("z [m]")
axes[1].grid(alpha=0.3)
figure.tight_layout()
figure.savefig(out, dpi=120)
print("saved", out)
