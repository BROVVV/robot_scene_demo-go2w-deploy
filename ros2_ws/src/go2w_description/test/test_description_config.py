from pathlib import Path

import pytest

from go2w_description.description_config import (
    load_confirmed_measurements,
    load_official_reference,
    official_sensor_to_base_extrinsics,
    render_official_sensor_urdf,
)


def test_placeholder_geometry_is_rejected():
    path = Path(__file__).parents[4] / "configs/go2w/physical_measurements.yaml"
    with pytest.raises(ValueError, match="unmeasured"):
        load_confirmed_measurements(path)


def test_pinned_unitree_sensor_frames_are_loaded_exactly():
    path = Path(__file__).parents[4] / "configs/go2w/official_reference.yaml"
    reference = load_official_reference(path)
    envelope = reference["dimensions"]["standing_envelope_m"]
    assert (envelope["length"], envelope["width"], envelope["height"]) == (
        0.70,
        0.43,
        0.50,
    )
    assert reference["frames"]["base_to_lidar"]["translation_m"] == (
        0.28945,
        0.0,
        -0.046825,
    )
    assert reference["frames"]["base_to_lidar"]["rotation_rpy_rad"] == (
        0.0,
        pytest.approx(-0.263392653559),
        0.0,
    )
    urdf = render_official_sensor_urdf(reference)
    assert '<parent link="base_link"/><child link="utlidar_lidar"/>' in urdf
    assert '<origin xyz="0.28945 0 -0.046825" rpy="0 -0.263392654 0"/>' in urdf
    assert '<origin xyz="-0.007698 -0.014655 0.00667" rpy="0 0 0"/>' in urdf


def test_official_reference_resolves_rko_sensor_to_base_extrinsics():
    root = Path(__file__).parents[4]
    reference = load_official_reference(root / "configs/go2w/official_reference.yaml")
    extrinsics = official_sensor_to_base_extrinsics(reference)
    assert extrinsics["lidar2base_quat_xyzw_xyz"] == pytest.approx(
        (0.0, -0.13131596829419406, 0.0, 0.9913405653310865,
         0.28945, 0.0, -0.046825)
    )
    assert extrinsics["imu2base_quat_xyzw_xyz"] == pytest.approx(
        (0.0, -0.13131596829419406, 0.0, 0.9913405653310865,
         0.2802809010218946, -0.014655, -0.04238926692307722)
    )


def test_unaccepted_frame_mapping_is_rejected(tmp_path):
    source = Path(__file__).parents[4] / "configs/go2w/official_reference.yaml"
    payload = source.read_text(encoding="utf-8").replace(
        "accepted: true", "accepted: false", 1
    )
    path = tmp_path / "rejected_reference.yaml"
    path.write_text(payload, encoding="utf-8")
    with pytest.raises(ValueError, match="base frame mapping"):
        load_official_reference(path)
