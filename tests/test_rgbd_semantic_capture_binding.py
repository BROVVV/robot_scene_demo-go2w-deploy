"""A semantic result must retain its original RGB-D acquisition after restart."""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from app.live_robot.async_semantic_observer import AsyncSemanticObservationManager
from app.live_robot.semantic_observer import semantic_observation_to_live
from app.perception.depth_object_localizer import DepthObjectLocalizer, resolve_depth_frame
from app.perception.rgbd_source import RGBDFrame
from app.spatial.semantic_object_map import SemanticObjectMap
from scripts.go2w.remote_control_semantic_mapping_worker import RemoteControlSemanticMapper


def capture(tmp_path, name, *, depth_mm=1200, provenance=None, host_timestamp=None):
    color = tmp_path / (name + '.jpg')
    depth = tmp_path / (name + '.png')
    assert cv2.imwrite(str(color), np.full((16, 16, 3), 100, dtype=np.uint8))
    assert cv2.imwrite(str(depth), np.full((16, 16), depth_mm, dtype=np.uint16))
    return RGBDFrame(frame_id='7', timestamp=time.time(), color_ref=str(color),
        depth_ref=str(depth), width=16, height=16, fx=10., fy=10., cx=8., cy=8.,
        host_timestamp=host_timestamp, provenance=provenance or {'capture_id': name})


def test_numeric_semantic_id_cannot_select_tokenized_capture_depth(tmp_path):
    current = capture(tmp_path, 'after-restart', depth_mm=4000)
    # The old result has only a counter; its source acquisition is unproven.
    assert resolve_depth_frame('7', current) is None


def test_cache_dictionary_key_does_not_override_the_actual_capture_identity(tmp_path):
    wrong = capture(tmp_path, 'after-restart', depth_mm=4000)
    assert resolve_depth_frame('old-evidence', None, {'old-evidence': wrong}) is None


def mapper_for(frame):
    return SimpleNamespace(_rgbd_source=SimpleNamespace(
        get_latest=lambda **kwargs: frame, materialize=lambda value: value, health=lambda: {}),
        args=SimpleNamespace(max_clock_skew_seconds=5.), _status={},
        _provider=SimpleNamespace(get_pose=lambda: None), _session_id=1,
        _bump=lambda *args: None, _publish_status=lambda **kwargs: None,
        _log=lambda message: None)


def test_actual_mapping_capture_uses_distinct_evidence_ids_after_counter_reset(tmp_path):
    first = capture(tmp_path, 'first')
    second = capture(tmp_path, 'second')
    old = RemoteControlSemanticMapper._capture_keyframe(mapper_for(first), (0., 0., 0.))
    new = RemoteControlSemanticMapper._capture_keyframe(mapper_for(second), (0., 0., 0.))
    assert old.frame_id != new.frame_id
    assert old.frame_id == first.evidence_id and new.frame_id == second.evidence_id
    assert first.frame_id == second.frame_id == '7'  # Raw sensor API is retained.


@pytest.mark.parametrize('provenance,host', [({'capture_id': 'epoch-a-7'}, None),
    ({'cache_nonce': 'unknown-clock-1'}, None), ({'source': 'realsense_http_atomic'}, 1790000000.)])
def test_evidence_identity_survives_materialization_clock_alignment_and_roundtrip(tmp_path, provenance, host):
    frame = capture(tmp_path, 'a', provenance=provenance, host_timestamp=host)
    changed = replace(frame, timestamp=frame.timestamp + 100., color_ref='cached.jpg',
        depth_ref='cached.png', health={'age_s': 2.}, provenance={**frame.provenance,
        'materialized': True, 'capture_clock': {'request_round_trip_seconds': .4}})
    assert frame.evidence_id == changed.evidence_id
    assert RGBDFrame.from_dict(changed.to_dict()).evidence_id == frame.evidence_id
    assert frame.evidence_id != frame.frame_id


def test_capture_tokens_take_precedence_over_equal_wallclock_and_frame_counter(tmp_path):
    first = capture(tmp_path, 'first', host_timestamp=1790000000.)
    second = replace(first, provenance={'capture_id': 'second'})
    assert first.evidence_id != second.evidence_id
    assert resolve_depth_frame(first.evidence_id, second, {first.evidence_id: first}) is first
    assert resolve_depth_frame(first.evidence_id, second, {}) is None


@pytest.mark.parametrize('host', [None, 1790000000.])
def test_legacy_non_http_frame_ids_remain_usable(tmp_path, host):
    frame = capture(tmp_path, 'old', provenance={'source': 'legacy_replay'}, host_timestamp=host)
    assert frame.evidence_id == '7'
    assert resolve_depth_frame('7', frame) is frame


def test_delayed_semantics_use_original_depth_after_restart_or_none_after_eviction(tmp_path):
    old = capture(tmp_path, 'epoch-a-7', depth_mm=1200)
    current = capture(tmp_path, 'epoch-b-7', depth_mm=4000)
    started, release = threading.Event(), threading.Event()
    def analyze(image_path, target_profile, *, frame_id, **kwargs):
        started.set()
        assert release.wait(timeout=2)
        return {'frame_id': frame_id, 'semantic_source_frame_id': frame_id,
            'image_path': image_path, 'scene_objects': [{'frame_object_id': 'obj-1',
                'label': 'cabinet', 'bbox_2d': [.25, .25, .75, .75]}]}
    manager = AsyncSemanticObservationManager(analyze, enabled=True, now=time.time)
    try:
        assert manager.submit_if_needed(image_path=old.color_ref, frame_id=old.evidence_id,
            capture_timestamp=old.timestamp, robot_pose=None, target_profile=object())
        assert started.wait(timeout=2)
        cache = {old.evidence_id: old, current.evidence_id: current}
        release.set()
        deadline = time.monotonic() + 2
        while manager.has_inflight() and time.monotonic() < deadline:
            time.sleep(.005)
        semantic = manager.get_latest_completed()
        assert semantic is not None
        selected = resolve_depth_frame(semantic.semantic_source_frame_id, current, cache)
        assert selected is old
        localized = DepthObjectLocalizer().localize(semantic.objects, selected)
        assert localized[0].depth_m == pytest.approx(1.2)
        assert localized[0].provenance['frame_id'] == old.evidence_id
        assert localized[0].provenance['sensor_frame_id'] == '7'
        live = semantic_observation_to_live(semantic, bundle_id=old.evidence_id,
            detections=semantic.objects, target_present=True, pose=None,
            image_ref=old.color_ref, depth_ref=old.depth_ref,
            rgbd_frame_id=selected.evidence_id, spatial_quality='CAMERA_LOCAL')
        assert live.semantic_source_frame_id == live.rgbd_frame_id == old.evidence_id
        assert live.image_ref == old.color_ref and live.depth_ref == old.depth_ref
        cache.pop(old.evidence_id)
        assert resolve_depth_frame(semantic.semantic_source_frame_id, current, cache) is None
    finally:
        release.set()
        manager.close()


def test_map_counts_new_acquisition_after_counter_reset_but_not_replayed_capture(tmp_path):
    frames = [capture(tmp_path, name) for name in ('epoch-a-7', 'epoch-b-7')]
    object_map = SemanticObjectMap(confirm_min_observations=2)
    localized = []
    for index, frame in enumerate(frames):
        observation = DepthObjectLocalizer().localize([{'frame_object_id': 'cabinet-1',
            'label': 'cabinet', 'confidence': .95,
            'bbox_2d': [.25, .25, .75, .75]}], frame)[0]
        # Synthetic fixed world transform; this checks identity and map fusion,
        # not the physical D435 extrinsics or the actual robot map coordinates.
        observation.map_xyz = observation.camera_xyz
        observation.spatial_quality = 'METRIC_RGBD'
        observation.provenance['map_frame'] = 'map'
        object_map.update_with_associations([observation], place_id='P1',
            frame_id=frame.evidence_id, now=100. + index)
        localized.append(observation)
    assert len(object_map.objects) == 1
    entity = next(iter(object_map.objects.values()))
    assert entity.observation_count == 2
    revision = object_map.revision
    object_map.update_with_associations([localized[-1]], place_id='P1',
        frame_id=frames[-1].evidence_id, now=110.)
    assert entity.observation_count == 2
    assert object_map.revision == revision
