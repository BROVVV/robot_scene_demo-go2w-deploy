#!/usr/bin/env python3
"""Read-only dual-LiDAR calibration capture for the Go2-W + PandarXT-16.

For one named stationary scene, capture independently stamped built-in L2
clouds (transformed into base_link) and Pandar raw clouds plus their header stamps, host receive
stamps, frame ids and the current candidate transform/config hash. The raw
data is preserved so an offline multi-scene calibration
(``calibrate_pandarxt16_extrinsics.py``) can be reproduced later.

Capture is read-only: no TF is published and no motion is commanded.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.live_robot.current_hardware import (  # noqa: E402
    load_current_hardware_geometry,
)


class DualLidarCapture(Node):
    def __init__(
        self,
        *,
        frames: int,
        scene: str,
        builtin_topic: str,
        pandar_topic: str,
    ) -> None:
        super().__init__("dual_lidar_calibration_capture")
        pandar_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        builtin_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.frames = frames
        self.scene = scene
        self.builtin: list[dict] = []
        self.pandar: list[dict] = []
        self.builtin_transform: dict | None = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2, builtin_topic, self._on_builtin, builtin_qos
        )
        self.create_subscription(
            PointCloud2, pandar_topic, self._on_pandar, pandar_qos
        )

    def _on_builtin(self, message: PointCloud2) -> None:
        if len(self.builtin) >= self.frames:
            return
        xyz = _read_fields(message, ("x", "y", "z"))
        if message.header.frame_id != "base_link":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exc:
                self.get_logger().warning(f"built-in cloud TF unavailable: {exc}")
                return
            xyz = _transform_xyz(xyz, transform.transform)
            if self.builtin_transform is None:
                t, q = transform.transform.translation, transform.transform.rotation
                self.builtin_transform = {
                    "source_frame": message.header.frame_id,
                    "target_frame": "base_link",
                    "translation_xyz_m": [t.x, t.y, t.z],
                    "rotation_xyzw": [q.x, q.y, q.z, q.w],
                }
        elif self.builtin_transform is None:
            self.builtin_transform = {
                "source_frame": "base_link", "target_frame": "base_link",
                "translation_xyz_m": [0.0, 0.0, 0.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        self.builtin.append(
            {
                "stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                "arrival_s": self.get_clock().now().nanoseconds / 1e9,
                "frame_id": "base_link",
                "points": int(message.width * message.height),
                "xyz": xyz,
            }
        )

    def _on_pandar(self, message: PointCloud2) -> None:
        if len(self.pandar) >= self.frames:
            return
        values = _read_fields(message, ("x", "y", "z", "ring", "timestamp"))
        self.pandar.append(
            {
                "stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                "arrival_s": self.get_clock().now().nanoseconds / 1e9,
                "frame_id": message.header.frame_id,
                "points": int(message.width * message.height),
                "xyz": values[:, :3],
                "ring": values[:, 3].astype(np.int64),
                "point_time": values[:, 4],
            }
        )


def _read_fields(message: PointCloud2, fields: tuple[str, ...]) -> np.ndarray:
    records = point_cloud2.read_points(message, field_names=fields, skip_nans=False)
    if getattr(getattr(records, "dtype", None), "names", None):
        return np.column_stack(
            tuple(np.asarray(records[name], dtype=np.float64) for name in fields)
        )
    return np.asarray(list(records), dtype=np.float64).reshape(-1, len(fields))


def _transform_xyz(xyz: np.ndarray, transform) -> np.ndarray:
    t, q = transform.translation, transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return xyz @ rotation.T + np.array([t.x, t.y, t.z])


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_npz(path: Path, frames: list[np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **{f"frame_{index}": frame for index, frame in enumerate(frames)})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", required=True, help="e.g. corner_01, doorframe_02")
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--timeout-seconds", type=float, default=15.0)
    parser.add_argument(
        "--builtin-topic", default="/go2w/sensors/cloud"
    )
    parser.add_argument("--pandar-topic", default="/hesai/pandarxt16/points_raw")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/go2w_acceptance/dual_lidar_calibration"),
    )
    parser.add_argument(
        "--candidate-transform",
        default="configs/go2w/hesai_pandarxt16_extrinsics.yaml",
        help="candidate transform config captured alongside raw data",
    )
    args = parser.parse_args()
    if args.frames < 2:
        parser.error("--frames must be at least 2")
    scene_dir = args.output / args.scene
    if scene_dir.exists() and any(scene_dir.iterdir()):
        parser.error(f"refusing to overwrite existing capture: {scene_dir}")

    rclpy.init()
    node = DualLidarCapture(
        frames=args.frames,
        scene=args.scene,
        builtin_topic=args.builtin_topic,
        pandar_topic=args.pandar_topic,
    )
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and (len(node.builtin) < args.frames or len(node.pandar) < args.frames)
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(node.builtin) < args.frames or len(node.pandar) < args.frames:
        print(
            f"capture incomplete: builtin={len(node.builtin)}/{args.frames} "
            f"pandar={len(node.pandar)}/{args.frames}",
            file=sys.stderr,
        )
        return 2

    scene_dir.mkdir(parents=True, exist_ok=True)
    builtin_path = scene_dir / "builtin_clouds.npz"
    pandar_path = scene_dir / "pandar_clouds.npz"
    _write_npz(builtin_path, [frame["xyz"] for frame in node.builtin])
    _write_npz(pandar_path, [frame["xyz"] for frame in node.pandar])

    geometry = None
    try:
        geometry = load_current_hardware_geometry(
            PROJECT_ROOT / "configs/go2w/current_hardware_geometry.yaml"
        ).to_dict()
    except Exception as exc:  # pragma: no cover - capture should still save raw data
        print(f"warning: geometry not loaded: {exc}", file=sys.stderr)

    candidate_text = Path(args.candidate_transform).read_text(encoding="utf-8") if Path(args.candidate_transform).is_file() else ""
    manifest = {
        "schema": "go2w.dual_lidar.calibration_capture.v1",
        "scene": args.scene,
        "captured_at": node.get_clock().now().to_msg().sec,
        "frames": args.frames,
        "builtin": {
            "topic": args.builtin_topic,
            "frame_id": node.builtin[0]["frame_id"] if node.builtin else None,
            "transform_to_base_link": node.builtin_transform,
            "points_per_frame": node.builtin[0]["points"] if node.builtin else None,
            "clouds_file": "builtin_clouds.npz",
            "clouds_sha256": _sha256_file(builtin_path),
            "header_stamps_s": [f["stamp"] for f in node.builtin],
            "arrival_stamps_s": [f["arrival_s"] for f in node.builtin],
        },
        "pandar": {
            "topic": args.pandar_topic,
            "frame_id": node.pandar[0]["frame_id"] if node.pandar else None,
            "points_per_frame": node.pandar[0]["points"] if node.pandar else None,
            "clouds_file": "pandar_clouds.npz",
            "clouds_sha256": _sha256_file(pandar_path),
            "header_stamps_s": [f["stamp"] for f in node.pandar],
            "arrival_stamps_s": [f["arrival_s"] for f in node.pandar],
            "point_time_span_s": [
                float(np.ptp(frame["point_time"])) if frame["point_time"].size else None
                for frame in node.pandar
            ],
            "rings": sorted(
                {
                    int(value)
                    for frame in node.pandar
                    for value in np.unique(frame["ring"])
                }
            ),
        },
        "candidate_transform_config": str(args.candidate_transform),
        "candidate_transform_sha256": hashlib.sha256(candidate_text.encode("utf-8")).hexdigest(),
        "current_hardware_geometry": geometry,
        "read_only": True,
        "authorizes_motion": False,
        "authorizes_tf_publication": False,
    }
    manifest_path = scene_dir / "raw_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
