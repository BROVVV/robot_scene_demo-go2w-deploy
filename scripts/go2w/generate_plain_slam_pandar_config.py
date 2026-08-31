#!/usr/bin/env python3
# Copyright 2026 robot_scene_demo maintainers

"""Generate the Go2-W plain_slam_ros2 runtime configuration artifacts.

Reads the human-maintained sources and automatically derives everything an
operator must not be asked to measure:

* ``configs/go2w/official_reference.yaml``        -> base_link -> utlidar_lidar
                                                      -> utlidar_imu geometry
* ``configs/go2w/hesai_pandarxt16_extrinsics.yaml`` -> Pandar candidate
                                                      (candidate_unconfirmed)
* ``configs/go2w/plain_slam{,_lio_params,_slam_params,_bridge}.yaml``
                                                      -> tuning + topics

Derives ``T_imu_pandar`` (plain_slam wants IMU -> LiDAR, not base -> Pandar):

    T_base_imu   = T_base_lidar * T_lidar_imu
    T_imu_pandar = inverse(T_base_imu) * T_base_pandar

and writes:

    runtime/go2w/plain_slam/generated_lio_3d_config.yaml
    runtime/go2w/plain_slam/generated_lio_3d_params.yaml
    runtime/go2w/plain_slam/generated_slam_3d_config.yaml
    runtime/go2w/plain_slam/generated_slam_3d_params.yaml
    runtime/go2w/plain_slam/config_provenance.json

Safety contract (never violated):
    - never modifies any source YAML;
    - never flips confirmed / authorizes_* to true;
    - provenance always records mapping_assist_only + candidate_unconfirmed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_DIR = PROJECT_ROOT / "runtime" / "go2w" / "plain_slam"

CONFIG_DIR = PROJECT_ROOT / "configs" / "go2w"

SOURCES = {
    "official": CONFIG_DIR / "official_reference.yaml",
    "extrinsics": CONFIG_DIR / "hesai_pandarxt16_extrinsics.yaml",
    "master": CONFIG_DIR / "plain_slam.yaml",
    "lio_params": CONFIG_DIR / "plain_slam_lio_params.yaml",
    "slam_params": CONFIG_DIR / "plain_slam_slam_params.yaml",
}

PANDAR_EXTRINSIC_STATUS = "candidate_unconfirmed"
MODE = "mapping_assist_only"

# Sanity reference from the implementation plan §2.2 (unit-test only; the
# generator itself never trusts these constants).
REFERENCE_IMU_TO_PANDAR_TRANSLATION = (-0.13042, 0.02966, 0.09357)
REFERENCE_IMU_TO_PANDAR_ROTATION = (
    0.942376, -0.188276, 0.276548,
    0.196897, 0.980418, -0.003478,
    -0.270478, 0.057729, 0.960994,
)


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # local import: python3-yaml is required on host
    except ImportError as exc:  # pragma: no cover - environment precondition
        raise SystemExit(
            "ERROR: PyYAML is not installed (python3-yaml)."
        ) from exc
    with path.open("r", encoding="utf-8") as handle:
        value = yaml.safe_load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a mapping at top level")
    return value


def pose_to_matrix(
    translation: list[float] | tuple[float, float, float],
    rpy_rad: list[float] | tuple[float, float, float],
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Rigid transform as (R row-major 3x3, t) from fixed-axis RPY.

    Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll) applied to column
    vectors (ROS REP-103 style fixed-axis roll-pitch-yaw).
    """
    rx, ry, rz = (float(v) for v in rpy_rad)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # Rz * Ry * Rx
    r00 = cz * cy
    r01 = cz * sy * sx - sz * cx
    r02 = cz * sy * cx + sz * sx
    r10 = sz * cy
    r11 = sz * sy * sx + cz * cx
    r12 = sz * sy * cx - cz * sx
    r20 = -sy
    r21 = cy * sx
    r22 = cy * cx
    t = tuple(float(v) for v in translation)
    return (r00, r01, r02, r10, r11, r12, r20, r21, r22), t


def compose(
    a: tuple[tuple[float, ...], tuple[float, ...]],
    b: tuple[tuple[float, ...], tuple[float, ...]],
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Compose rigid transforms: result = a @ b."""
    ra, ta = a
    rb, tb = b
    r = tuple(
        sum(ra[i * 3 + k] * rb[k * 3 + j] for k in range(3))
        for i in range(3)
        for j in range(3)
    )
    t = tuple(
        sum(ra[i * 3 + k] * tb[k] for k in range(3)) + ta[i]
        for i in range(3)
    )
    return r, t


def inverse_rigid(
    a: tuple[tuple[float, ...], tuple[float, ...]],
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Inverse of a rigid transform: R' = R^T, t' = -R^T t."""
    r, t = a
    rt = tuple(r[i * 3 + j] for j in range(3) for i in range(3))
    tinv = tuple(-sum(rt[i * 3 + k] * t[k] for k in range(3)) for i in range(3))
    return rt, tinv


def extrinsic_status(extrinsics: dict[str, Any]) -> dict[str, Any]:
    """Extract the officially recorded Pandar safety/status facts."""
    candidate = extrinsics.get("transform_candidate") or {}
    translation = candidate.get("translation_m") or {}
    rotation_deg = candidate.get("rotation_rpy_deg") or {}
    return {
        "calibration_status": str(
            extrinsics.get("calibration_status", PANDAR_EXTRINSIC_STATUS)
        ),
        "confirmed": bool(extrinsics.get("confirmed", False)),
        "authorizes_tf_publication": bool(
            extrinsics.get("authorizes_tf_publication", False)
        ),
        "authorizes_safety_integration": bool(
            extrinsics.get("authorizes_safety_integration", False)
        ),
        "authorizes_motion": bool(extrinsics.get("authorizes_motion", False)),
        "translation_m": (
            float(translation.get("x", 0.0)),
            float(translation.get("y", 0.0)),
            float(translation.get("z", 0.0)),
        ),
        "rotation_rpy_deg": (
            float(rotation_deg.get("roll", 0.0)),
            float(rotation_deg.get("pitch", 0.0)),
            float(rotation_deg.get("yaw", 0.0)),
        ),
    }


def derive_imu_to_pandar() -> dict[str, Any]:
    """Derive T_imu_pandar together with full provenance facts."""
    official = load_yaml(SOURCES["official"])
    extrinsics = load_yaml(SOURCES["extrinsics"])

    base_to_lidar = (official.get("frames") or {}).get("base_to_lidar") or {}
    lidar_to_imu = (official.get("frames") or {}).get("lidar_to_lidar_imu") or {}
    t_bl = tuple(float(v) for v in (base_to_lidar.get("translation_m") or (0.0, 0.0, 0.0)))
    r_bl = tuple(float(v) for v in (base_to_lidar.get("rotation_rpy_rad") or (0.0, 0.0, 0.0)))
    t_li = tuple(float(v) for v in (lidar_to_imu.get("translation_m") or (0.0, 0.0, 0.0)))
    r_li = tuple(float(v) for v in (lidar_to_imu.get("rotation_rpy_rad") or (0.0, 0.0, 0.0)))

    t_base_imu = compose(pose_to_matrix(t_bl, r_bl), pose_to_matrix(t_li, r_li))

    facts = extrinsic_status(extrinsics)
    cand_t = facts["translation_m"]
    cand_r_deg = facts["rotation_rpy_deg"]
    cand_r_rad = tuple(math.radians(v) for v in cand_r_deg)
    t_base_pandar = pose_to_matrix(cand_t, cand_r_rad)

    t_imu_pandar = compose(inverse_rigid(t_base_imu), t_base_pandar)
    rotation, translation = t_imu_pandar

    return {
        "status": {
            "calibration_status": facts["calibration_status"],
            "confirmed": facts["confirmed"],
            "authorizes_tf_publication": facts["authorizes_tf_publication"],
            "authorizes_safety_integration": facts["authorizes_safety_integration"],
            "authorizes_motion": facts["authorizes_motion"],
        },
        "source_transforms": {
            "base_to_utlidar_lidar": {
                "translation_m": list(t_bl),
                "rotation_rpy_rad": list(r_bl),
                "source": "configs/go2w/official_reference.yaml",
            },
            "utlidar_lidar_to_utlidar_imu": {
                "translation_m": list(t_li),
                "rotation_rpy_rad": list(r_li),
                "source": "configs/go2w/official_reference.yaml",
            },
            "base_to_pandarxt16_candidate": {
                "translation_m": list(cand_t),
                "rotation_rpy_deg": list(cand_r_deg),
                "source": "configs/go2w/hesai_pandarxt16_extrinsics.yaml",
            },
        },
        "derived_imu_to_pandar": {
            "translation": [round(v, 8) for v in translation],
            "rotation_matrix": [round(v, 8) for v in rotation],
        },
    }


def build_lio_runtime(result: dict[str, Any]) -> dict[str, Any]:
    master = load_yaml(SOURCES["master"])
    lio_template = load_yaml(SOURCES["lio_params"])
    lio_params = (lio_template.get("lio_3d_node") or {}).get("ros__parameters") or {}
    topics = master.get("topics") or {}
    frames = master.get("frames") or {}
    runtime_rel = "runtime/go2w/plain_slam"

    params = {}
    params.update(lio_params)
    params["extrinsics"] = {
        "imu_to_lidar": dict(result["derived_imu_to_pandar"])
    }

    config = {
        "lio_3d_node": {
            "ros__parameters": {
                "lidar_type": "hesai_pandarxt16",
                "use_as_localizer": False,
                "map_cloud_dir": "/tmp/go2w_plain_slam/",
                "param_files_dir": str(PROJECT_ROOT / runtime_rel),
                "pointcloud_topic": topics.get("pandar_adapted"),
                "imu_topic": topics.get("imu"),
                "imu_pose_topic": topics.get("imu_pose_raw"),
                "imu_odom_topic": topics.get("imu_odom_raw"),
                "lio_map_cloud_topic": topics.get("lio_map_cloud"),
                "aligned_scan_cloud_topic": topics.get("aligned_scan"),
                "deskewed_scan_cloud_topic": topics.get("deskewed_scan"),
                "odom_frame": frames.get("odom"),
                "imu_frame": frames.get("imu"),
            }
        }
    }
    return {
        "config": config,
        # NOTE: upstream lio_3d_interface.cpp loads "<param_files_dir>/
        # lio_3d_params.yaml" WITHOUT the node-name wrapper; the runtime
        # params files are therefore written unwrapped (see write_runtime_files).
        "params": dict(params),
    }


# Fixed upstream params file names (plain_slam_ros2 reads exactly these).
LIO_PARAMS_FILENAME = "lio_3d_params.yaml"
SLAM_PARAMS_FILENAME = "slam_3d_params.yaml"


def build_slam_runtime(result: dict[str, Any]) -> dict[str, Any]:
    master = load_yaml(SOURCES["master"])
    slam_template = load_yaml(SOURCES["slam_params"])
    slam_params = (slam_template.get("slam_3d_node") or {}).get("ros__parameters") or {}
    topics = master.get("topics") or {}
    frames = master.get("frames") or {}
    # Runtime dir as a stable relative path (project layout is fixed; this
    # also keeps the generator testable with a redirected RUNTIME_DIR).
    runtime_rel = "runtime/go2w/plain_slam"

    config = {
        "slam_3d_node": {
            "ros__parameters": {
                "map_cloud_dir": "/tmp/go2w_plain_slam/",
                "param_files_dir": str(PROJECT_ROOT / runtime_rel),
                "imu_pose_topic": topics.get("imu_pose_raw"),
                "deskewed_scan_cloud_topic": topics.get("deskewed_scan"),
                "filtered_map_cloud_topic": topics.get("map_3d"),
                "graph_nodes_topic": topics.get("graph_nodes"),
                "odom_edges_topic": topics.get("graph_odom_edges"),
                "loop_edges_topic": topics.get("graph_loop_edges"),
                "graph_poses_topic": topics.get("graph_poses"),
                "map_frame": frames.get("map"),
            }
        }
    }
    return {
        "config": config,
        "params": dict(slam_params),
    }


def build_provenance(result: dict[str, Any], master: dict[str, Any]) -> dict[str, Any]:
    return {
        "pandar_extrinsic_source": "configs/go2w/hesai_pandarxt16_extrinsics.yaml",
        "pandar_extrinsic_status": result["status"]["calibration_status"],
        "confirmed": result["status"]["confirmed"],
        "used_for": MODE,
        "authorizes_motion": result["status"]["authorizes_motion"],
        "authorizes_safety": result["status"]["authorizes_safety_integration"],
        "authorizes_tf_publication": result["status"]["authorizes_tf_publication"],
        "imu_source": master.get("imu_source_default"),
        "mode": MODE,
        "ready_semantics": master.get("ready_semantics"),
        "motion_odom_topic": master.get("motion_odom_topic"),
        "source_transforms": result["source_transforms"],
        "derived_imu_to_pandar": result["derived_imu_to_pandar"],
        "generator": "scripts/go2w/generate_plain_slam_pandar_config.py",
    }


def write_runtime_files() -> dict[str, Any]:
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    result = derive_imu_to_pandar()
    lio = build_lio_runtime(result)
    slam = build_slam_runtime(result)
    master = load_yaml(SOURCES["master"])

    import yaml  # local import

    def dump(value: Any) -> str:
        return yaml.safe_dump(value, sort_keys=False, allow_unicode=True)

    (RUNTIME_DIR / "generated_lio_3d_config.yaml").write_text(
        dump(lio["config"]), encoding="utf-8"
    )
    # Upstream reads exactly "lio_3d_params.yaml"; the generated_* name is a
    # plan-mandated alias with identical unwrapped content.
    lio_params_text = dump(lio["params"])
    (RUNTIME_DIR / "generated_lio_3d_params.yaml").write_text(
        lio_params_text, encoding="utf-8"
    )
    (RUNTIME_DIR / LIO_PARAMS_FILENAME).write_text(
        lio_params_text, encoding="utf-8"
    )
    (RUNTIME_DIR / "generated_slam_3d_config.yaml").write_text(
        dump(slam["config"]), encoding="utf-8"
    )
    slam_params_text = dump(slam["params"])
    (RUNTIME_DIR / "generated_slam_3d_params.yaml").write_text(
        slam_params_text, encoding="utf-8"
    )
    (RUNTIME_DIR / SLAM_PARAMS_FILENAME).write_text(
        slam_params_text, encoding="utf-8"
    )
    provenance = build_provenance(result, master)
    (RUNTIME_DIR / "config_provenance.json").write_text(
        json.dumps(provenance, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return provenance


def check_mode() -> int:
    result = derive_imu_to_pandar()
    status = result["status"]
    derived = result["derived_imu_to_pandar"]
    t = derived["translation"]
    ref = REFERENCE_IMU_TO_PANDAR_TRANSLATION
    tolerance = 2e-3
    ok = all(abs(float(t[i]) - ref[i]) < tolerance for i in range(3))

    print(f"Pandar extrinsic: {status['calibration_status']}")
    print(f"Mode: {MODE}")
    print(f"Derived imu_to_pandar: {'OK' if ok else 'MISMATCH'}")
    print(f"Motion authorization changed: {'YES' if status['authorizes_motion'] else 'NO'}")
    print(f"Safety authorization changed: {'YES' if status['authorizes_safety_integration'] else 'NO'}")
    if ok:
        print("Derived translation:", [round(float(v), 5) for v in t])
    else:
        print("Derived translation:", [round(float(v), 5) for v in t])
        print("Reference translation:", list(ref))
        print(
            "ERROR: derived imu_to_pandar translation deviates from the "
            "implementation-plan sanity reference; refusing to proceed."
        )
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate Go2-W plain_slam runtime config + provenance"
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate derivation against the plan sanity reference without writing files",
    )
    args = parser.parse_args()

    if args.check:
        return check_mode()

    provenance = write_runtime_files()
    status = provenance["pandar_extrinsic_status"]
    derived = provenance["derived_imu_to_pandar"]
    print(f"Wrote runtime configs to {RUNTIME_DIR}")
    print(f"Pandar extrinsic: {status}")
    print(f"Mode: {provenance['used_for']}")
    print(
        "Derived imu_to_pandar translation:",
        [round(float(v), 6) for v in derived["translation"]],
    )
    print(f"Motion authorization changed: {'YES' if provenance['authorizes_motion'] else 'NO'}")
    print(f"Safety authorization changed: {'YES' if provenance['authorizes_safety'] else 'NO'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
