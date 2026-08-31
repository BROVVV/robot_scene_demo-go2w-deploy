# Copyright 2026 robot_scene_demo maintainers

"""Unit tests for runtime config generation (plan §4.2 / §4.3 / §18).

The generator must produce the four runtime YAML files plus a provenance JSON
that never claims motion/safety authorization, and the runtime topics/frames
must stay inside the isolated ``/go2w/slam/*`` + ``pslam_*`` namespaces.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_GO2W = PROJECT_ROOT / "scripts" / "go2w"
if str(SCRIPTS_GO2W) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_GO2W))

import generate_plain_slam_pandar_config as gen  # noqa: E402

EXPECTED_RUNTIME_FILES = (
    "generated_lio_3d_config.yaml",
    "generated_lio_3d_params.yaml",
    "lio_3d_params.yaml",
    "generated_slam_3d_config.yaml",
    "generated_slam_3d_params.yaml",
    "slam_3d_params.yaml",
    "config_provenance.json",
)


def test_runtime_files_are_written(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(gen, "RUNTIME_DIR", tmp_path)
    provenance = gen.write_runtime_files()
    assert provenance is not None
    for name in EXPECTED_RUNTIME_FILES:
        assert (tmp_path / name).is_file(), f"missing runtime artifact {name}"


def test_provenance_contract(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(gen, "RUNTIME_DIR", tmp_path)
    gen.write_runtime_files()
    provenance = json.loads((tmp_path / "config_provenance.json").read_text("utf-8"))
    assert provenance["pandar_extrinsic_source"].endswith(
        "hesai_pandarxt16_extrinsics.yaml"
    )
    assert provenance["pandar_extrinsic_status"] == "candidate_unconfirmed"
    assert provenance["confirmed"] is False
    assert provenance["used_for"] == "mapping_assist_only"
    assert provenance["authorizes_motion"] is False
    assert provenance["authorizes_safety"] is False
    assert provenance["authorizes_tf_publication"] is False
    assert provenance["mode"] == "mapping_assist_only"
    assert provenance["ready_semantics"]["motion_authorized"] is False
    assert provenance["ready_semantics"]["safety_authorized"] is False
    derived = provenance["derived_imu_to_pandar"]["translation"]
    for value, reference in zip(
        derived, gen.REFERENCE_IMU_TO_PANDAR_TRANSLATION
    ):
        assert abs(float(value) - reference) < 1e-3


def test_lio_runtime_config_namespace(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(gen, "RUNTIME_DIR", tmp_path)
    gen.write_runtime_files()
    config = yaml.safe_load(
        (tmp_path / "generated_lio_3d_config.yaml").read_text("utf-8")
    )
    params = config["lio_3d_node"]["ros__parameters"]
    assert params["lidar_type"] == "hesai_pandarxt16"
    assert params["pointcloud_topic"] == "/go2w/slam/pandar_points"
    assert params["imu_topic"] == "/go2w/slam/imu"
    assert params["odom_frame"] == "pslam_odom"
    assert params["imu_frame"] == "pslam_imu"
    for topic in (
        params["imu_pose_topic"],
        params["imu_odom_topic"],
        params["aligned_scan_cloud_topic"],
        params["deskewed_scan_cloud_topic"],
        params["lio_map_cloud_topic"],
    ):
        assert topic.startswith("/go2w/slam/"), f"leaked topic {topic}"


def test_lio_runtime_params_extrinsics_and_tuning(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(gen, "RUNTIME_DIR", tmp_path)
    gen.write_runtime_files()
    # Upstream format: unwrapped top-level keys (no node-name wrapper), and
    # the fixed filename lio_3d_params.yaml must carry the same content.
    params = yaml.safe_load(
        (tmp_path / "lio_3d_params.yaml").read_text("utf-8")
    )
    generated = yaml.safe_load(
        (tmp_path / "generated_lio_3d_params.yaml").read_text("utf-8")
    )
    assert params == generated
    assert "lio_3d_node" not in params
    extrinsic = params["extrinsics"]["imu_to_lidar"]
    for value, reference in zip(
        extrinsic["translation"], gen.REFERENCE_IMU_TO_PANDAR_TRANSLATION
    ):
        assert abs(float(value) - reference) < 1e-3
    assert len(extrinsic["rotation_matrix"]) == 9
    assert params["imu_params"]["acc_scale"] == 1.0
    assert params["estimator"]["use_loose_coupling"] is True
    assert params["scan_cloud_preprocess"]["clip_range"] == 0.35


def test_slam_runtime_config_namespace(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(gen, "RUNTIME_DIR", tmp_path)
    gen.write_runtime_files()
    config = yaml.safe_load(
        (tmp_path / "generated_slam_3d_config.yaml").read_text("utf-8")
    )
    params = config["slam_3d_node"]["ros__parameters"]
    assert params["map_frame"] == "pslam_map"
    assert params["filtered_map_cloud_topic"] == "/go2w/slam/map_3d"
    for topic in (
        params["imu_pose_topic"],
        params["deskewed_scan_cloud_topic"],
        params["graph_nodes_topic"],
        params["graph_poses_topic"],
    ):
        assert topic.startswith("/go2w/slam/"), f"leaked topic {topic}"
