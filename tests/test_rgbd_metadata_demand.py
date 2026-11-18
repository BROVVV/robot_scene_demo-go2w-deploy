"""Metadata mode must retain the same capture/geometry checks without image IO."""
import time
from unittest.mock import Mock

import pytest
from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource, RGBDFrameUnavailable


def source(tmp_path, monkeypatch):
    value = RealSenseHTTPRGBDSource('http://127.0.0.1:8080', cache_dir=tmp_path, clock_mode='remote')
    meta = {'frame_id':'7', 'host_timestamp':time.time(), 'width':1280, 'height':720,
            'color_url':'/color/7.jpg', 'depth_url':'/depth/7.png', 'health':{'age_s':.01},
            'intrinsics':{'fx':700., 'fy':701., 'cx':640., 'cy':360.}, 'depth_unit_m':.001}
    monkeypatch.setattr(value, '_get_json', lambda _: meta)
    return value, meta


def test_metadata_preserves_full_intrinsics_clock_and_remote_refs_without_download(tmp_path, monkeypatch):
    value, meta = source(tmp_path, monkeypatch)
    monkeypatch.setattr(value, '_download', lambda *a: pytest.fail('metadata mode downloaded an image'))
    frame = value.get_latest_metadata()
    assert (frame.width, frame.height, frame.fx, frame.fy, frame.cx, frame.cy) == (1280,720,700.,701.,640.,360.)
    assert frame.color_ref.endswith('/color/7.jpg') and frame.depth_ref.endswith('/depth/7.png')
    assert frame.depth_unit_m == .001 and frame.host_timestamp == meta['host_timestamp']
    assert not list(tmp_path.iterdir())


def test_default_still_materializes_full_rgbd(tmp_path, monkeypatch):
    value, _ = source(tmp_path, monkeypatch); materialize = Mock(side_effect=lambda f:f)
    monkeypatch.setattr(value, 'materialize', materialize)
    value.get_latest(); materialize.assert_called_once()


def test_metadata_does_not_bypass_staleness_validation(tmp_path, monkeypatch):
    value, meta = source(tmp_path, monkeypatch); meta['health']['age_s'] = 30.
    with pytest.raises(RGBDFrameUnavailable, match='stale'):value.get_latest_metadata()


def test_first_resumed_pair_overwrites_cache_from_an_earlier_producer(tmp_path, monkeypatch):
    value, _ = source(tmp_path, monkeypatch)
    (tmp_path/'color_7.jpg').write_bytes(b'OLD COLOR')
    (tmp_path/'depth_7.png').write_bytes(b'OLD DEPTH')
    frame = value.get_latest_metadata()
    calls = []
    def download(url, path):
        calls.append(url);path.write_bytes(url.encode())
    monkeypatch.setattr(value, '_download', download)
    fresh = value.materialize(frame)
    assert len(calls) == 2
    assert 'color/7.jpg' in fresh.color_ref or __import__('pathlib').Path(fresh.color_ref).read_bytes().endswith(b'/color/7.jpg')
    assert __import__('pathlib').Path(fresh.depth_ref).read_bytes().endswith(b'/depth/7.png')
    value.materialize(frame)
    assert len(calls) == 2  # Normal same-capture cache reuse is preserved.
