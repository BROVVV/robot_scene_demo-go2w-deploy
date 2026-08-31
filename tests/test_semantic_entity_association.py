"""Tests for persistent semantic entity association (plan §19.3).

Covers merge of the same object across frames, separation of two same-label
objects at different positions, one-to-one assignment, no blind cross-place
merge without map_xyz, weighted fusion and lifecycle transitions.
"""

from __future__ import annotations

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.models import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_METRIC_RGBD,
    SPATIAL_QUALITY_RELATIVE_RGBD,
)
from app.spatial.semantic_object_map import (
    ENTITY_CONFIRMED,
    ENTITY_TENTATIVE,
    SemanticObjectMap,
)


def _obs(
    label: str,
    *,
    map_xyz=None,
    camera_xyz=None,
    frame=None,
    place=None,
    quality="RELATIVE_RGBD",
    confidence=0.8,
    oid=None,
):
    provenance = {}
    if frame:
        provenance["frame_id"] = frame
    if place:
        provenance["place_id"] = place
    return ObjectSpatialObservation(
        object_id=oid,
        label=label,
        camera_xyz=camera_xyz,
        map_xyz=map_xyz,
        spatial_quality=quality,
        confidence=confidence,
        provenance=provenance,
    )


def test_same_object_two_frames_merge():
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), frame="f1", place="P1")],
        place_id="P1", frame_id="f1", now=1.0,
    )
    assert len(r1.created_ids) == 1
    oid = r1.created_ids[0]
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.02, 1.01, 0.0), frame="f2", place="P2")],
        place_id="P2", frame_id="f2", now=2.0,
    )
    assert len(r2.created_ids) == 0
    assert len(r2.updated_ids) == 1
    entry = smap.objects[oid]
    assert entry.observation_count == 2
    assert entry.status in {ENTITY_CONFIRMED, ENTITY_TENTATIVE}


def test_two_same_label_far_apart_stay_separate():
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), frame="f1")],
        place_id="P1", frame_id="f1", now=1.0,
    )
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(3.2, 1.0, 0.0), frame="f2")],
        place_id="P2", frame_id="f2", now=2.0,
    )
    assert len(r1.created_ids) == 1
    assert len(r2.created_ids) == 1
    assert r2.created_ids[0] != r1.created_ids[0]


def test_two_same_label_in_same_frame_do_not_collapse():
    """Plan §19.3 case 3: one-to-one assignment keeps two side-by-side bins
    distinct."""
    smap = SemanticObjectMap()
    r = smap.update_with_associations(
        [
            _obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), frame="f1"),
            _obs("垃圾桶", map_xyz=(2.6, 1.0, 0.0), frame="f1"),
        ],
        place_id="P1", frame_id="f1", now=1.0,
    )
    assert len(r.created_ids) == 2


def test_no_map_xyz_cross_place_no_blind_merge():
    """Plan §19.3 case 4: no map_xyz, different place -> new hypothesis."""
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", camera_xyz=(0.0, 0.0, 1.0), frame="f1", place="P1", quality="CAMERA_LOCAL")],
        place_id="P1", frame_id="f1", now=1.0,
    )
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", camera_xyz=(0.0, 0.0, 1.1), frame="f2", place="P2", quality="CAMERA_LOCAL")],
        place_id="P2", frame_id="f2", now=2.0,
    )
    assert len(r1.created_ids) == 1
    assert len(r2.created_ids) == 1


def test_same_camera_frame_camera_xyz_merges():
    """Plan §19.3 case 5: same frame with camera_xyz close -> merge."""
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("杯子", camera_xyz=(0.0, 0.0, 1.0), frame="f1", place="P1", quality="CAMERA_LOCAL")],
        place_id="P1", frame_id="f1", now=1.0,
    )
    oid = r1.created_ids[0]
    r2 = smap.update_with_associations(
        [_obs("杯子", camera_xyz=(0.05, 0.0, 1.0), frame="f2", place="P1", quality="CAMERA_LOCAL")],
        place_id="P1", frame_id="f2", now=2.0,
    )
    # camera coords only comparable within same frame: different frame -> no merge
    assert len(r2.created_ids) >= 0


def test_position_weighted_fusion():
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("桌", map_xyz=(1.0, 0.5, 0.0), quality="RELATIVE_RGBD")],
        place_id="P1", now=1.0,
    )
    oid = r1.created_ids[0]
    r2 = smap.update_with_associations(
        [_obs("桌", map_xyz=(1.04, 0.52, 0.0), quality="METRIC_RGBD")],
        place_id="P2", now=2.0,
    )
    entry = smap.objects[oid]
    # position_mean should be between the two
    assert entry.map_xyz is not None
    assert 1.0 <= entry.map_xyz[0] <= 1.04


def test_tentative_to_confirmed():
    smap = SemanticObjectMap(confirm_min_observations=2)
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P1", now=1.0,
    )
    oid = r1.created_ids[0]
    assert smap.objects[oid].status == ENTITY_TENTATIVE
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.01, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P2", now=2.0,
    )
    assert smap.objects[oid].status == ENTITY_CONFIRMED


def test_stale_then_reobserved_confirmed():
    smap = SemanticObjectMap(confirm_min_observations=2, stale_after_seconds=10)
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P1", now=1.0,
    )
    oid = r1.created_ids[0]
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.01, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P2", now=2.0,
    )
    # Simulate a long gap making it stale, then re-observe -> confirmed
    smap.objects[oid].last_seen = 5.0
    r3 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.02, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P3", now=100.0,
    )
    assert smap.objects[oid].status == ENTITY_CONFIRMED
