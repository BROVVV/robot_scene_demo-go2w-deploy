from pathlib import Path
import json

import pytest
import yaml

from go2w_lidar_preprocessor.config import load_safety_ready_config


def test_unmeasured_project_configs_close_safety_gate():
    root = Path(__file__).parents[4]
    with pytest.raises(ValueError, match="not measured and confirmed"):
        load_safety_ready_config(
            root / "configs/go2w/lidar_preprocess.yaml",
            root / "configs/go2w/physical_measurements.yaml",
        )


def test_validated_stationary_config_accepts_pinned_official_geometry():
    root = Path(__file__).parents[4]
    config, parameters = load_safety_ready_config(
        root / "configs/go2w/lidar_preprocess.yaml",
        root / "configs/go2w/official_reference.yaml",
    )
    # 2026-08-14（79b093a）把 YAML 的 revalidation_required 由 false 改成 true，
    # 并配了 revalidation_note 说明原因：自掩膜按新装的 PandarXT-16 保护框扩到
    # |y|=0.42，且侧向旋转净空尚未独立验证（同文件 rotation_clearance_validation
    # .valid 也是 false）。那次提交只改了 YAML，没同步这条断言，于是本用例自
    # 79b093a 起一直红着。这里对齐到 YAML 的保守取值——**不是放宽**：断言 true
    # 意味着"仍需复验、旋转未获授权"，比 false 更严。
    # 注意 load_safety_ready_config 返回的就是原始 YAML dict，标志不经代码改写。
    assert config["revalidation_required"] is True
    assert parameters.minimum_height == -0.588
    assert parameters.maximum_height == 0.972
    assert parameters.ground_height == -0.448
    assert parameters.collision_maximum_height == 0.052
    assert parameters.self_half_length == 0.35
    assert parameters.self_half_width == 0.215
    assert parameters.front_corridor_half_width == 0.315
    assert parameters.rotation_envelope_radius == 0.511
    assert (-0.36, -0.08, 0.255, 0.56, -0.19, 0.06) in parameters.self_regions
    assert (0.12, 0.36, 0.255, 0.52, -0.19, 0.06) in parameters.self_regions


def test_recorded_profile_keeps_reviewed_fixed_self_regions():
    root = Path(__file__).parents[4]
    _, parameters = load_safety_ready_config(
        root / "configs/go2w/lidar_preprocess.yaml",
        root / "configs/go2w/official_reference.yaml",
        root / "configs/go2w/navigation_sensor_profile.yaml",
    )
    assert (-0.36, -0.08, -0.56, -0.255, -0.19, 0.06) in parameters.self_regions
    assert (0.12, 0.36, -0.52, -0.255, -0.19, 0.06) in parameters.self_regions


def test_rotation_clearance_cannot_be_enabled_by_boolean_alone(tmp_path):
    root = Path(__file__).parents[4]
    payload = yaml.safe_load(
        (root / "configs/go2w/lidar_preprocess.yaml").read_text(encoding="utf-8")
    )
    payload["rotation_clearance_validation"]["valid"] = True
    candidate = tmp_path / "lidar.yaml"
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="method is missing or unsupported"):
        load_safety_ready_config(candidate, root / "configs/go2w/official_reference.yaml")


def test_rotation_clearance_accepts_complete_physical_evidence_contract(tmp_path):
    root = Path(__file__).parents[4]
    payload = yaml.safe_load(
        (root / "configs/go2w/lidar_preprocess.yaml").read_text(encoding="utf-8")
    )
    claim = payload["rotation_clearance_validation"]
    evidence_path = tmp_path / "rotation_evidence.json"
    evidence_path.write_text(json.dumps({
        "validation_type": "go2w_rotation_clearance_physical_crosscheck",
        "robot_model": "Unitree Go2-W",
        "passed": True,
        "robot_motion_commanded": False,
        "operator": "TEST_OPERATOR",
        "posture": "stationary_standing",
        "validated_rotation_envelope_radius_m": 0.511,
        "authorization_scope": {"type": "persistent_sensor_coverage"},
        "near_field_mitigation": {
            "sensor_only_observability_complete": True,
            "method": "additional_coverage_sensor",
        },
        "checks": {key: True for key in claim["checks"]},
    }), encoding="utf-8")
    claim.update({
        "valid": True,
        "validation_method": "physical_360_clearance_with_lidar_crosscheck",
        "validated_at": "2026-08-13T18:00:00+08:00",
        "operator": "TEST_OPERATOR",
        "posture": "stationary_standing",
        "validated_rotation_envelope_radius_m": 0.511,
        "evidence_paths": [str(evidence_path)],
    })
    claim["checks"] = {key: True for key in claim["checks"]}
    candidate = tmp_path / "lidar.yaml"
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")
    config, _ = load_safety_ready_config(
        candidate, root / "configs/go2w/official_reference.yaml"
    )
    assert config["rotation_clearance_validation"]["valid"] is True


def test_rotation_clearance_rejects_short_lived_pose_bound_evidence(tmp_path):
    root = Path(__file__).parents[4]
    payload = yaml.safe_load(
        (root / "configs/go2w/lidar_preprocess.yaml").read_text(encoding="utf-8")
    )
    claim = payload["rotation_clearance_validation"]
    evidence_path = tmp_path / "rotation_evidence.json"
    evidence_path.write_text(json.dumps({
        "validation_type": "go2w_rotation_clearance_physical_crosscheck",
        "robot_model": "Unitree Go2-W",
        "passed": True,
        "robot_motion_commanded": False,
        "operator": "TEST_OPERATOR",
        "posture": "stationary_standing",
        "validated_rotation_envelope_radius_m": 0.511,
        "authorization_scope": {
            "type": "initial_pose_in_place_rotation_only",
        },
        "checks": {key: True for key in claim["checks"]},
    }), encoding="utf-8")
    claim.update({
        "valid": True,
        "validation_method": "physical_360_clearance_with_lidar_crosscheck",
        "validated_at": "2026-08-13T18:00:00+08:00",
        "operator": "TEST_OPERATOR",
        "posture": "stationary_standing",
        "validated_rotation_envelope_radius_m": 0.511,
        "evidence_paths": [str(evidence_path)],
    })
    claim["checks"] = {key: True for key in claim["checks"]}
    candidate = tmp_path / "lidar.yaml"
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="pose-bound rotation evidence"):
        load_safety_ready_config(
            candidate, root / "configs/go2w/official_reference.yaml"
        )


def test_rotation_clearance_rejects_too_small_validated_envelope(tmp_path):
    root = Path(__file__).parents[4]
    payload = yaml.safe_load(
        (root / "configs/go2w/lidar_preprocess.yaml").read_text(encoding="utf-8")
    )
    claim = payload["rotation_clearance_validation"]
    claim.update({
        "valid": True,
        "validation_method": "physical_360_clearance_with_lidar_crosscheck",
        "validated_at": "2026-08-13T18:00:00+08:00",
        "operator": "TEST_OPERATOR",
        "posture": "stationary_standing",
        "validated_rotation_envelope_radius_m": 0.50,
        "evidence_paths": ["outputs/test/rotation_evidence.json"],
    })
    claim["checks"] = {key: True for key in claim["checks"]}
    candidate = tmp_path / "lidar.yaml"
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="smaller than configured envelope"):
        load_safety_ready_config(candidate, root / "configs/go2w/official_reference.yaml")


def test_rotation_clearance_rejects_missing_evidence_file(tmp_path):
    root = Path(__file__).parents[4]
    payload = yaml.safe_load(
        (root / "configs/go2w/lidar_preprocess.yaml").read_text(encoding="utf-8")
    )
    claim = payload["rotation_clearance_validation"]
    claim.update({
        "valid": True,
        "validation_method": "physical_360_clearance_with_lidar_crosscheck",
        "validated_at": "2026-08-13T18:00:00+08:00",
        "operator": "TEST_OPERATOR",
        "posture": "stationary_standing",
        "validated_rotation_envelope_radius_m": 0.511,
        "evidence_paths": [str(tmp_path / "missing.json")],
    })
    claim["checks"] = {key: True for key in claim["checks"]}
    candidate = tmp_path / "lidar.yaml"
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="evidence is unreadable"):
        load_safety_ready_config(
            candidate, root / "configs/go2w/official_reference.yaml"
        )
