import numpy as np

from go2w_rgb_lidar_fusion.fusion_core import FusionParameters, localize_mask_points


def fixture():
    rng = np.random.default_rng(4)
    cluster = np.column_stack(
        (
            rng.normal(0.0, 0.015, 40),
            rng.normal(0.0, 0.015, 40),
            rng.normal(2.0, 0.015, 40),
        )
    )
    noise = np.array([[1.0, 1.0, 2.0], [-1.0, -1.0, 2.0], [0.0, 0.0, -1.0]])
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[42:59, 42:59] = 255
    intrinsic = np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]])
    return np.vstack((cluster, noise)), mask, intrinsic


def test_localizes_robust_cluster_inside_eroded_mask():
    points, mask, intrinsic = fixture()
    result = localize_mask_points(
        points, mask, intrinsic, (42, 42, 58, 58), 10.0, FusionParameters()
    )
    assert result.localized_3d
    assert result.point_count >= 30
    assert abs(result.position_camera_m[2] - 2.0) < 0.03
    assert result.confidence > 0.5


def test_timestamp_and_mask_point_gates_never_fabricate_pose():
    points, mask, intrinsic = fixture()
    late = localize_mask_points(
        points, mask, intrinsic, (42, 42, 58, 58), 51.0, FusionParameters()
    )
    assert not late.localized_3d
    assert late.position_camera_m is None
    assert late.reason == "timestamp_delta_exceeded"
    empty = localize_mask_points(
        points, np.zeros_like(mask), intrinsic, (42, 42, 58, 58), 1.0, FusionParameters()
    )
    assert not empty.localized_3d
    assert empty.position_camera_m is None
    assert empty.reason == "insufficient_mask_points"
