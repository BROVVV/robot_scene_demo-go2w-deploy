#!/usr/bin/env bash
# Save the plain_slam mapping-assist artifacts:
#   outputs/maps/<timestamp>/
#     map_3d.pcd      (from /go2w/slam/map_3d, latest snapshot)
#     map_2d.pgm      (P5; 0=occupied 254=free 205=unknown)
#     map_2d.yaml     (map_server metadata)
#     provenance.json (extrinsic status + mapping-assist boundary)
#
# Only reads topics.  If the 3D map has no data yet, the 2D map and
# provenance are still saved and a warning is printed.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
workspace="${project_root}/ros2_ws"
runtime_dir="${project_root}/runtime/go2w/plain_slam"
out_dir="${project_root}/outputs/maps/$(date +%Y%m%d_%H%M%S)"
mkdir -p "${out_dir}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${workspace}/install/setup.bash" 2>/dev/null || true
set -u

timeout 10 ros2 topic echo /go2w/slam/map_2d --once >/dev/null 2>&1 || {
  printf 'WARNING: /go2w/slam/map_2d echo timed out (large grid); python probe used instead.\n' >&2
}

python3 - "${out_dir}" "${project_root}" <<'PY'
import json
import struct
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

out_dir = Path(sys.argv[1])
project_root = Path(sys.argv[2])

rclpy.init()
node = rclpy.create_node("save_plain_slam_map")

def wait_topic(msg_type, topic, timeout_s=20.0):
    received = []
    sub = node.create_subscription(msg_type, topic, received.append, 1)
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while node.get_clock().now().nanoseconds < deadline and not received:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return received[0] if received else None

# --- 3D map -> PCD (x y z intensity timestamp, binary little-endian) -------
cloud = wait_topic(PointCloud2, "/go2w/slam/map_3d")
pcd_saved = False
if cloud is not None and cloud.width > 0:
    step = cloud.point_step
    data = bytes(cloud.data)
    fields = {f.name: f.offset for f in cloud.fields}
    xs, ys, zs, ints, tss = [], [], [], [], []
    for i in range(cloud.width * cloud.height):
        base = i * step
        def f32(off):
            return struct.unpack_from("<f", data, base + off)[0] if off >= 0 else 0.0
        def f64(off):
            return struct.unpack_from("<d", data, base + off)[0] if off >= 0 else 0.0
        xs.append(f32(fields.get("x", -1)))
        ys.append(f32(fields.get("y", -1)))
        zs.append(f32(fields.get("z", -1)))
        ints.append(f32(fields.get("intensity", -1)))
        tss.append(f64(fields.get("timestamp", -1)))
    n = len(xs)
    pcd = ["# .PCD v0.7 - Point Cloud Data file format",
           "VERSION 0.7",
           "FIELDS x y z intensity timestamp",
           "SIZE 4 4 4 4 8",
           "TYPE F F F F F",
           "COUNT 1 1 1 1 1",
           f"WIDTH {n}",
           "HEIGHT 1",
           "VIEWPOINT 0 0 0 1 0 0 0",
           f"POINTS {n}",
           "DATA binary"]
    payload = bytearray()
    for i in range(n):
        payload += struct.pack("<fff", xs[i], ys[i], zs[i])
        payload += struct.pack("<f", ints[i])
        payload += struct.pack("<d", tss[i])
    (out_dir / "map_3d.pcd").write_bytes("\n".join(pcd).encode() + b"\n" + bytes(payload))
    pcd_saved = True
    print(f"[OK] map_3d.pcd: {n} points")
else:
    print("[WARN] /go2w/slam/map_3d has no data; skipping map_3d.pcd")

# --- 2D map -> PGM + YAML ------------------------------------------------
grid = wait_topic(OccupancyGrid, "/go2w/slam/map_2d")
if grid is not None:
    width, height = grid.info.width, grid.info.height
    px = bytearray(width * height)
    for i, value in enumerate(grid.data):
        if value < 0:
            px[i] = 205  # unknown
        elif value == 0:
            px[i] = 254  # free
        else:
            px[i] = 0    # occupied
    pgm = bytearray(f"P5\n{width} {height}\n255\n".encode()) + bytes(px)
    (out_dir / "map_2d.pgm").write_bytes(bytes(pgm))
    meta = {
        "image": "map_2d.pgm",
        "resolution": grid.info.resolution,
        "origin": [grid.info.origin.position.x, grid.info.origin.position.y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.35,
    }
    import yaml
    (out_dir / "map_2d.yaml").write_text(
        yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")
    print(f"[OK] map_2d.pgm/yaml: {width}x{height} @ {grid.info.resolution} m")
else:
    print("[ERROR] /go2w/slam/map_2d timed out", file=sys.stderr)
    sys.exit(2)

# --- provenance -----------------------------------------------------------
provenance = {}
prov_file = project_root / "runtime/go2w/plain_slam/config_provenance.json"
if prov_file.exists():
    provenance = json.loads(prov_file.read_text("utf-8"))
provenance.update({
    "pandar_model": "Hesai PandarXT-16",
    "imu_source": "/utlidar/imu",
    "saved_at": __import__("datetime").datetime.now().isoformat(),
    "mapping_assist_only": True,
    "map_3d_saved": pcd_saved,
})
(out_dir / "provenance.json").write_text(
    json.dumps(provenance, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

node.destroy_node()
rclpy.shutdown()
print(f"[OK] saved to {out_dir}")
PY

printf 'Saved: %s\n' "${out_dir}"