from pathlib import Path

import pytest
import yaml

from go2w_rgb_lidar_fusion.config_gate import (
    load_diagnostic_extrinsics_gate,
    load_diagnostic_fusion_gate,
    load_extrinsics_gate,
    load_fusion_gate,
)


def test_project_configs_open_only_diagnostic_fusion():
    root = Path(__file__).parents[4]
    fusion, _ = load_diagnostic_fusion_gate(
        root / "configs/go2w/rgb_lidar_fusion.yaml",
        root / "configs/go2w/camera_intrinsics.yaml",
        root / "configs/go2w/sensor_extrinsics.yaml",
    )
    assert fusion.get("enabled") is True
    assert fusion.get("validation_status") == "experimental"
    assert fusion.get("authorizes_3d_localization") is False
    assert "acceptance_override" in fusion
    with pytest.raises(ValueError, match="disabled or unvalidated"):
        load_fusion_gate(
            root / "configs/go2w/rgb_lidar_fusion.yaml",
            root / "configs/go2w/camera_intrinsics.yaml",
            root / "configs/go2w/sensor_extrinsics.yaml",
        )


def test_project_configs_open_only_diagnostic_extrinsics():
    root = Path(__file__).parents[4]
    _, extrinsics = load_diagnostic_extrinsics_gate(
        root / "configs/go2w/camera_intrinsics.yaml",
        root / "configs/go2w/sensor_extrinsics.yaml",
    )
    assert extrinsics.get("diagnostic_overlay_accepted") is True
    assert extrinsics.get("confirmed") is False
    assert "acceptance_override" in extrinsics
    with pytest.raises(ValueError, match="not navigation-grade"):
        load_extrinsics_gate(
            root / "configs/go2w/camera_intrinsics.yaml",
            root / "configs/go2w/sensor_extrinsics.yaml",
        )


def test_diagnostic_gate_still_fail_closed_when_disabled(tmp_path):
    root = Path(__file__).parents[4]
    source = (root / "configs/go2w/rgb_lidar_fusion.yaml").read_text(encoding="utf-8")
    candidate = tmp_path / "fusion_disabled.yaml"
    candidate.write_text(source.replace("enabled: true", "enabled: false"), encoding="utf-8")
    with pytest.raises(ValueError, match="diagnostic overlay is disabled"):
        load_diagnostic_fusion_gate(
            candidate,
            root / "configs/go2w/camera_intrinsics.yaml",
            root / "configs/go2w/sensor_extrinsics.yaml",
        )


def test_navigation_gate_requires_explicit_grade_and_moved_recheck(tmp_path):
    root = Path(__file__).parents[4]
    fusion = yaml.safe_load(
        (root / "configs/go2w/rgb_lidar_fusion.yaml").read_text(encoding="utf-8")
    )
    extrinsics = yaml.safe_load(
        (root / "configs/go2w/sensor_extrinsics.yaml").read_text(encoding="utf-8")
    )
    fusion.update(
        validation_status="validated",
        navigation_geometry_validated=True,
        authorizes_3d_localization=True,
    )
    extrinsics.update(
        calibration_status="calibrated",
        confirmed=True,
        navigation_geometry_validated=True,
    )
    fusion_path = tmp_path / "fusion.yaml"
    extrinsics_path = tmp_path / "extrinsics.yaml"
    fusion_path.write_text(yaml.safe_dump(fusion), encoding="utf-8")
    extrinsics_path.write_text(yaml.safe_dump(extrinsics), encoding="utf-8")
    with pytest.raises(ValueError, match="moved-position"):
        load_fusion_gate(
            fusion_path,
            root / "configs/go2w/camera_intrinsics.yaml",
            extrinsics_path,
        )
    extrinsics["validation"]["moved_position_recheck_passed"] = True
    extrinsics_path.write_text(yaml.safe_dump(extrinsics), encoding="utf-8")
    opened, _ = load_fusion_gate(
        fusion_path,
        root / "configs/go2w/camera_intrinsics.yaml",
        extrinsics_path,
    )
    assert opened["authorizes_3d_localization"] is True
