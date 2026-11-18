#!/usr/bin/env python3
"""导出 Dataset D 的语义标签清单：按标签分组、空间聚类估实例数，供操作员对照现场核对。"""
import json
import sys
from collections import defaultdict

import numpy as np

path = sys.argv[1]
cluster_radius = float(sys.argv[2]) if len(sys.argv) > 2 else 0.8

observations = []
for line in open(path, encoding="utf-8"):
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
    observations.append({
        "object_id": obj.get("object_id"),
        "label": obj.get("label") or "?",
        "xyz": np.asarray(xyz, dtype=float),
        "quality": obj.get("spatial_quality"),
        "confidence": obj.get("confidence"),
        "count": obj.get("observation_count"),
        "first_seen": obj.get("first_seen"),
        "last_seen": obj.get("last_seen"),
    })

print(f"total localized observations: {len(observations)}")
by_label = defaultdict(list)
for item in observations:
    by_label[item["label"]].append(item)

print(f"labels: {len(by_label)}")
print(f"cluster radius = {cluster_radius} m\n")
print(f"{'label':<12}{'obs':>5}{'cluster':>10}   clusters (map xyz)")
for label, items in sorted(by_label.items(), key=lambda kv: -len(kv[1])):
    clusters = []
    for item in items:
        placed = False
        for cluster in clusters:
            if np.linalg.norm(cluster["centre"] - item["xyz"]) <= cluster_radius:
                cluster["members"].append(item)
                cluster["centre"] = np.mean([m["xyz"] for m in cluster["members"]], axis=0)
                placed = True
                break
        if not placed:
            clusters.append({"centre": item["xyz"].copy(), "members": [item]})
    detail = " | ".join(
        f"({c['centre'][0]:.2f},{c['centre'][1]:.2f})x{len(c['members'])}" for c in clusters)
    print(f"{label:<12}{len(items):>5}{len(clusters):>10}   {detail}")

print("\n--- 供操作员核对：每个标签的观测明细 ---")
for label, items in sorted(by_label.items(), key=lambda kv: -len(kv[1])):
    print(f"\n[{label}]")
    for item in sorted(items, key=lambda i: i["first_seen"] or 0):
        print(f"  {item['object_id']:<8} map=({item['xyz'][0]:7.2f},{item['xyz'][1]:7.2f},"
              f"{item['xyz'][2]:5.2f})  conf={item['confidence']}  "
              f"quality={item['quality']}  obs={item['count']}")
