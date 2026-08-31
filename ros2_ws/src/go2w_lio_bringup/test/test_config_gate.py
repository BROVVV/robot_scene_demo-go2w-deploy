from pathlib import Path

import pytest
import yaml

from go2w_lio_bringup.config_gate import load_lio_gate, load_point_lio_gate


ROOT = Path(__file__).parents[4]


def load_project_gate():
    return load_point_lio_gate(
        ROOT / "configs/go2w/point_lio.yaml",
        ROOT / "configs/go2w/official_reference.yaml",
        ROOT / "configs/go2w/time_sync.yaml",
    )


def test_project_point_lio_config_resolves_pinned_official_extrinsics():
    lio = load_project_gate()
    assert lio["validation_scope"] == "stationary_read_only"
    assert lio["safety"]["authorizes_motion"] is False
    assert lio["resolved_extrinsics"]["lidar2base_quat_xyzw_xyz"] == pytest.approx(
        (0.0, -0.13131596829419406, 0.0, 0.9913405653310865,
         0.28945, 0.0, -0.046825)
    )
    assert lio["resolved_extrinsics"]["imu2base_quat_xyzw_xyz"] == pytest.approx(
        (0.0, -0.13131596829419406, 0.0, 0.9913405653310865,
         0.2802809010218946, -0.014655, -0.04238926692307722)
    )


def test_gate_rejects_any_motion_authorization(tmp_path):
    payload = yaml.safe_load(
        (ROOT / "configs/go2w/point_lio.yaml").read_text(encoding="utf-8")
    )
    payload["safety"]["authorizes_motion"] = True
    config = tmp_path / "lio.yaml"
    config.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="forbid motion"):
        load_point_lio_gate(
            config,
            ROOT / "configs/go2w/official_reference.yaml",
            ROOT / "configs/go2w/time_sync.yaml",
        )


def test_gate_rejects_nonstationary_validation_scope(tmp_path):
    payload = yaml.safe_load(
        (ROOT / "configs/go2w/point_lio.yaml").read_text(encoding="utf-8")
    )
    payload["validation_scope"] = "navigation"
    config = tmp_path / "lio.yaml"
    config.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="stationary and read-only"):
        load_point_lio_gate(
            config,
            ROOT / "configs/go2w/official_reference.yaml",
            ROOT / "configs/go2w/time_sync.yaml",
        )


def test_rejected_rko_configuration_cannot_be_launched():
    with pytest.raises(ValueError, match="disabled"):
        load_lio_gate(
            ROOT / "configs/go2w/lio.yaml",
            ROOT / "configs/go2w/official_reference.yaml",
            ROOT / "configs/go2w/time_sync.yaml",
        )


def test_point_lio_gate_rejects_generic_topic_forwarding(tmp_path):
    payload = yaml.safe_load(
        (ROOT / "configs/go2w/point_lio.yaml").read_text(encoding="utf-8")
    )
    payload["bridge"]["generic_topic_forwarding"] = True
    config = tmp_path / "point_lio.yaml"
    config.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="allow list"):
        load_point_lio_gate(
            config,
            ROOT / "configs/go2w/official_reference.yaml",
            ROOT / "configs/go2w/time_sync.yaml",
        )
