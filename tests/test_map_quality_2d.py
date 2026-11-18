import numpy as np
import pytest

from app.mapping.map_quality_2d import (
    Grid2D,
    compare_world_aligned,
    health_dimensions,
    occupancy_counts,
    sector_coverage,
)


def test_world_alignment_handles_shifted_origins_without_false_flips():
    before = Grid2D(np.array([[0, 100], [-1, 0]], dtype=np.int8), 1.0, 0.0, 0.0)
    after = Grid2D(np.array([[100, -1], [0, -1]], dtype=np.int8), 1.0, 1.0, 0.0)
    result = compare_world_aligned(before, after)
    assert result["comparable_cells"] == 2
    assert result["known_flip_cells"] == 0


def test_world_alignment_vectorizes_a_large_grid_without_cell_callback(monkeypatch):
    import app.mapping.map_quality_2d as quality

    data = np.zeros((311, 331), dtype=np.int8)
    before = Grid2D(data, 0.05, -12.8, -9.1)
    after = Grid2D(data.copy(), 0.05, -12.8, -9.1)
    monkeypatch.setattr(
        quality,
        "_class_at",
        lambda *_args: (_ for _ in ()).throw(AssertionError("per-cell Python path used")),
    )

    result = compare_world_aligned(before, after)

    assert result["comparable_cells"] == 311 * 331
    assert result["known_flip_cells"] == 0


def test_epoch_mismatch_is_rejected_even_when_frame_name_matches():
    data = np.zeros((2, 2), dtype=np.int8)
    with pytest.raises(ValueError, match="epochs"):
        compare_world_aligned(
            Grid2D(data, 0.1, 0, 0, map_epoch="e1"),
            Grid2D(data, 0.1, 0, 0, map_epoch="e2"),
        )


def test_counts_keep_unknown_separate_from_free_space():
    result = occupancy_counts(Grid2D(np.array([[-1, 0, 49, 50, 100]], dtype=np.int8),
                                     0.2, 0, 0))
    assert result["unknown_cells"] == 1
    assert result["free_cells"] == 2
    assert result["occupied_cells"] == 2
    assert result["free_area_m2"] == pytest.approx(0.08)


def test_sector_coverage_reports_circular_missing_arc():
    result = sector_coverage([float("inf"), 1.0, 1.0, float("nan")],
                             range_min=0.1, range_max=5.0, sector_count=4)
    assert result["valid_bins"] == 2
    assert result["longest_missing_bins"] == 2
    assert result["longest_missing_deg"] == 180.0


def test_health_dimensions_do_not_conflate_quality_and_motion_authority():
    value = health_dimensions(
        input_health="HEALTHY", geometry_health="UNVERIFIED",
        localization_health="HEALTHY", mapping_quality="DEGRADED",
        motion_authorized=False,
    )
    assert value["localization_health"] == "HEALTHY"
    assert value["mapping_quality"] == "DEGRADED"
    assert value["motion_authorized"] is False
