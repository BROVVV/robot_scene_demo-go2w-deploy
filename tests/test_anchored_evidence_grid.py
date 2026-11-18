import json

import numpy as np

from app.mapping.anchored_evidence_grid import (
    AnchoredEvidenceGrid,
    AnchoredRayObservation,
    GridSpec,
)
from app.slam.keyframe_graph import KeyframeGraphStore


def _graph(revision, poses, epoch="e1"):
    store = KeyframeGraphStore()
    store.update_json(json.dumps({
        "schema_version": "go2w_keyframe_graph_v1",
        "map_id": "m1", "map_epoch": epoch, "frame_id": "map",
        "sensor_profile_id": "p1", "graph_revision": revision,
        "nodes": [{"anchor_id": key, "source_stamp": stamp, "corrected_pose": pose}
                  for key, stamp, pose in poses],
    }))
    return store


def _observation(observation_id, group, endpoint=(2.0, 0.0), hit=True,
                 anchor="k1", epoch="e1"):
    return AnchoredRayObservation(
        observation_id=observation_id, anchor_id=anchor, map_id="m1",
        map_epoch=epoch, source_stamp=1.0, origin_in_anchor=(0.0, 0.0),
        endpoints_in_anchor=(endpoint,), endpoint_is_hit=(hit,),
        correlation_group=group,
    )


def test_duplicate_and_correlated_scans_do_not_inflate_independent_evidence():
    fusion = AnchoredEvidenceGrid(GridSpec(1.0, -1.0, -1.0, 6, 3))
    assert fusion.add(_observation("o1", "capture-1"))
    assert not fusion.add(_observation("o1", "capture-1"))
    assert fusion.add(_observation("o2", "capture-1"))
    result = fusion.render(_graph(1, [("k1", 1.0, [0, 0, 0])]))
    assert result.hit_counts[1, 3] == 1
    assert result.grid[1, 3] == -1  # one hit is tentative, not a permanent wall


def test_independent_hits_preserve_small_obstacle_and_ray_frees_visible_cells():
    fusion = AnchoredEvidenceGrid(GridSpec(1.0, -1.0, -1.0, 6, 3))
    fusion.add(_observation("o1", "capture-1"))
    fusion.add(_observation("o2", "capture-2"))
    result = fusion.render(_graph(1, [("k1", 1.0, [0, 0, 0])]))
    assert result.grid[1, 1] == 0
    assert result.grid[1, 2] == 0
    assert result.grid[1, 3] == 100


def test_missing_return_must_be_omitted_not_encoded_as_max_range_free_ray():
    fusion = AnchoredEvidenceGrid(GridSpec(1.0, -1.0, -1.0, 6, 3))
    # No endpoint means no ray and therefore no fabricated free cells.
    fusion.add(AnchoredRayObservation(
        observation_id="missing", anchor_id="k1", map_id="m1", map_epoch="e1",
        source_stamp=1.0, origin_in_anchor=(0.0, 0.0), endpoints_in_anchor=(),
        endpoint_is_hit=(), correlation_group="capture-1",
    ))
    result = fusion.render(_graph(1, [("k1", 1.0, [0, 0, 0])]))
    assert np.all(result.grid == -1)


def test_loop_closure_rebuild_moves_each_anchor_contribution_once():
    fusion = AnchoredEvidenceGrid(GridSpec(1.0, -1.0, -1.0, 10, 8))
    fusion.add(_observation("o1", "g1", endpoint=(1, 0), anchor="k1"))
    fusion.add(_observation("o2", "g2", endpoint=(1, 0), anchor="k2"))
    fusion.add(_observation("o3", "g3", endpoint=(1, 0), anchor="k1"))
    fusion.add(_observation("o4", "g4", endpoint=(1, 0), anchor="k2"))
    before = fusion.render(_graph(1, [("k1", 1, [0, 0, 0]), ("k2", 2, [4, 0, 0])]))
    assert before.grid[1, 2] == 100
    assert before.grid[1, 6] == 100
    after = fusion.render(_graph(2, [("k1", 1, [0, 1, 0]), ("k2", 2, [4, 3, 0])]))
    assert after.grid[2, 2] == 100
    assert after.grid[4, 6] == 100
    assert after.grid[1, 2] == -1
    assert after.grid[1, 6] == -1


def test_epoch_mismatch_is_skipped_not_barely_mixed():
    fusion = AnchoredEvidenceGrid(GridSpec(1.0, -1, -1, 5, 3))
    fusion.add(_observation("o1", "g1", epoch="old"))
    result = fusion.render(_graph(1, [("k1", 1, [0, 0, 0])], epoch="new"))
    assert result.used_observations == 0
    assert result.skipped_observations == 1
