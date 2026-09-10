#!/usr/bin/env python3
"""Extract synchronized camera/cloud scenes from a read-only ROS 2 bag."""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = PROJECT_ROOT / "ros2_ws/src/go2w_rgb_lidar_fusion"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from go2w_rgb_lidar_fusion.overlay_core import (  # noqa: E402
    CalibrationBlocked,
    select_synchronized_pairs,
)


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def atomic_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_text(path: Path, payload: str) -> None:
    atomic_bytes(path, payload.encode("utf-8"))


def atomic_npy(path: Path, values: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    os.close(descriptor)
    try:
        with Path(temporary).open("wb") as stream:
            np.save(stream, values, allow_pickle=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def storage_identifier(bag: Path) -> str:
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise CalibrationBlocked(f"ROS bag metadata is absent: {metadata_path}")
    payload = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = payload.get("rosbag2_bagfile_information") or payload
    value = str(info.get("storage_identifier") or "").strip()
    if not value:
        raise CalibrationBlocked("ROS bag storage identifier is absent")
    return value


def open_reader(bag: Path):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=storage_identifier(bag)
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def topic_classes(reader) -> dict:
    from rosidl_runtime_py.utilities import get_message

    return {
        item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
    }


def closest_info_record(
    info_records: list[tuple[int, int]], image_stamp_ns: int, maximum_delta_ns: int
) -> tuple[int, int]:
    if not info_records:
        raise CalibrationBlocked("bag has no CameraInfo records")
    value = min(info_records, key=lambda item: abs(item[0] - image_stamp_ns))
    if abs(value[0] - image_stamp_ns) > maximum_delta_ns:
        raise CalibrationBlocked("CameraInfo is not synchronized with selected image")
    return value


def cloud_data(message) -> tuple[np.ndarray, np.ndarray | None, list[str]]:
    from sensor_msgs_py import point_cloud2

    available = {field.name for field in message.fields}
    attribute_names = [
        name for name in ("intensity", "ring", "time") if name in available
    ]
    field_names = ("x", "y", "z", *attribute_names)
    raw = point_cloud2.read_points(
        message, field_names=field_names, skip_nans=True
    )
    if getattr(raw.dtype, "names", None):
        points = np.column_stack([raw[name] for name in ("x", "y", "z")])
        attributes = (
            np.column_stack([raw[name] for name in attribute_names])
            if attribute_names
            else None
        )
    else:
        decoded = np.asarray(raw).reshape(-1, len(field_names))
        points = decoded[:, :3]
        attributes = decoded[:, 3:] if attribute_names else None
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise CalibrationBlocked("selected cloud did not decode to finite Nx3 points")
    if attributes is not None:
        attributes = np.asarray(attributes, dtype=np.float32)
        if (
            attributes.shape != (len(points), len(attribute_names))
            or not np.isfinite(attributes).all()
        ):
            raise CalibrationBlocked("selected cloud attributes are invalid")
    return points, attributes, attribute_names


def camera_info_payload(message) -> dict:
    calibrated = (
        len(message.k) == 9
        and message.k[0] > 0.0
        and message.k[4] > 0.0
        and len(message.p) == 12
        and message.p[0] > 0.0
        and message.p[5] > 0.0
    )
    return {
        "header_stamp_ns": stamp_ns(message),
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "d": [float(item) for item in message.d],
        "k": [float(item) for item in message.k],
        "r": [float(item) for item in message.r],
        "p": [float(item) for item in message.p],
        "calibrated": bool(calibrated),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--maximum-pairs", type=int, default=10)
    parser.add_argument("--maximum-delta-ms", type=float, default=50.0)
    parser.add_argument("--minimum-separation-seconds", type=float, default=1.0)
    parser.add_argument("--image-topic", default="/camera/front/image_raw/compressed")
    parser.add_argument("--camera-info-topic", default="/camera/front/camera_info")
    parser.add_argument("--cloud-topic", default="/go2w/sensors/cloud")
    parser.add_argument(
        "--distance-band",
        choices=("near", "medium", "far", "moved_recheck", "unassigned"),
        default="unassigned",
    )
    parser.add_argument("--scene-prefix", default="scene")
    args = parser.parse_args()
    if args.maximum_pairs < 1 or args.maximum_delta_ms <= 0.0:
        raise SystemExit("pair count and delta must be positive")
    if args.minimum_separation_seconds < 0.0:
        raise SystemExit("minimum separation must be non-negative")
    scene_prefix = args.scene_prefix.strip().lower()
    if not scene_prefix or not all(
        character.isalnum() or character in "_-" for character in scene_prefix
    ):
        raise SystemExit("scene prefix must contain only letters, digits, _ or -")
    bag = args.bag.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    try:
        from rclpy.serialization import deserialize_message

        reader = open_reader(bag)
        classes = topic_classes(reader)
        required = (args.image_topic, args.camera_info_topic, args.cloud_topic)
        missing = [topic for topic in required if topic not in classes]
        if missing:
            raise CalibrationBlocked("bag topics are absent: " + ", ".join(missing))
        image_records = []
        info_records = []
        cloud_records = []
        while reader.has_next():
            topic, serialized, bag_time = reader.read_next()
            if topic not in required:
                continue
            message = deserialize_message(serialized, classes[topic])
            record = (stamp_ns(message), int(bag_time))
            if topic == args.image_topic:
                image_records.append(record)
            elif topic == args.camera_info_topic:
                info_records.append(record)
            else:
                cloud_records.append(record)
        maximum_delta_ns = int(args.maximum_delta_ms * 1e6)
        pairs = select_synchronized_pairs(
            image_records,
            cloud_records,
            maximum_delta_ns=maximum_delta_ns,
            minimum_separation_ns=int(args.minimum_separation_seconds * 1e9),
            maximum_pairs=args.maximum_pairs,
        )
        if not pairs:
            raise CalibrationBlocked("bag has no image/cloud pair inside the sync limit")
        for pair in pairs:
            info_stamp, info_bag_time = closest_info_record(
                info_records, pair["image_stamp_ns"], maximum_delta_ns
            )
            pair["camera_info_stamp_ns"] = info_stamp
            pair["camera_info_bag_time_ns"] = info_bag_time

        wanted = {}
        for index, pair in enumerate(pairs, start=1):
            wanted[(args.image_topic, pair["image_bag_time_ns"])] = (index, "image")
            wanted[(args.camera_info_topic, pair["camera_info_bag_time_ns"])] = (
                index,
                "camera_info",
            )
            wanted[(args.cloud_topic, pair["cloud_bag_time_ns"])] = (index, "cloud")
        extracted: dict[int, dict] = {index: {} for index in range(1, len(pairs) + 1)}
        reader = open_reader(bag)
        classes = topic_classes(reader)
        while reader.has_next() and wanted:
            topic, serialized, bag_time = reader.read_next()
            key = (topic, int(bag_time))
            selection = wanted.pop(key, None)
            if selection is None:
                continue
            index, kind = selection
            message = deserialize_message(serialized, classes[topic])
            extracted[index][kind] = message
        incomplete = [index for index, values in extracted.items() if len(values) != 3]
        if incomplete:
            raise CalibrationBlocked(f"selected records could not be extracted: {incomplete}")

        scene_summaries = []
        for index, pair in enumerate(pairs, start=1):
            scene = output / f"scene_{index:03d}"
            if scene.exists():
                raise CalibrationBlocked(f"refusing to overwrite scene directory: {scene}")
            values = extracted[index]
            image_payload = bytes(values["image"].data)
            if not image_payload.startswith(b"\xff\xd8"):
                raise CalibrationBlocked("selected compressed image is not JPEG")
            points, point_attributes, point_attribute_names = cloud_data(values["cloud"])
            info = camera_info_payload(values["camera_info"])
            scene_label = f"{scene_prefix}_{index:03d}"
            metadata = {
                "schema_version": "1.0",
                "scene_label": scene_label,
                "distance_band": args.distance_band,
                "image_stamp_ns": pair["image_stamp_ns"],
                "cloud_stamp_ns": pair["cloud_stamp_ns"],
                "timestamp_delta_ms": pair["timestamp_delta_ns"] / 1e6,
                "image_frame": values["image"].header.frame_id,
                "cloud_frame": values["cloud"].header.frame_id,
                "point_count": int(len(points)),
                "point_attribute_names": point_attribute_names,
                "camera_info_calibrated": info["calibrated"],
                "motion_commands_sent": False,
            }
            correspondence_template = {
                "scene_label": scene_label,
                "distance_band": args.distance_band,
                "instructions": (
                    "select structural LiDAR edge point indices and their matching image pixels"
                ),
                "correspondences": [],
            }
            atomic_bytes(scene / "image.jpg", image_payload)
            atomic_npy(scene / "points.npy", points)
            if point_attributes is not None:
                atomic_npy(scene / "point_attributes.npy", point_attributes)
            atomic_text(
                scene / "camera_info.yaml",
                yaml.safe_dump(info, sort_keys=False, allow_unicode=True),
            )
            atomic_text(
                scene / "scene_metadata.yaml",
                yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
            )
            atomic_text(
                scene / "correspondences.yaml",
                yaml.safe_dump(correspondence_template, sort_keys=False, allow_unicode=True),
            )
            scene_summaries.append(metadata)
        result = {
            "schema_version": "1.0",
            "passed": True,
            "bag": str(bag),
            "scene_count": len(scene_summaries),
            "maximum_delta_ms": args.maximum_delta_ms,
            "scenes": scene_summaries,
            "authorizes_fusion": False,
            "authorizes_motion": False,
            "next_step": "annotate 3D-2D structural-edge correspondences",
        }
        atomic_text(
            output / "dataset_report.json",
            json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except CalibrationBlocked as exc:
        result = {
            "schema_version": "1.0",
            "passed": False,
            "blocker": str(exc),
            "authorizes_fusion": False,
            "authorizes_motion": False,
        }
        atomic_text(
            output / "dataset_report.json",
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        )
        print("BLOCKED: " + str(exc), file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
