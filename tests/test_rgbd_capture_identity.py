"""Producer frame counters may repeat; a numeric ID alone is not a capture."""
from __future__ import annotations

import importlib.util
import os
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import urllib.error
import urllib.request

import pytest

from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource
from app.perception.rgbd_source import RGBDFrameUnavailable

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def camera(monkeypatch):
    monkeypatch.setitem(sys.modules, 'pyrealsense2', SimpleNamespace())
    spec = importlib.util.spec_from_file_location('capture_identity_test',
        ROOT / 'scripts/go2w/realsense_stream.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    server = module.ThreadingHTTPServer(('127.0.0.1', 0), module.Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield module, 'http://127.0.0.1:' + str(server.server_port)
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def frame(*, token=None, stamp=None, color=b'OLD COLOR', depth=b'OLD DEPTH'):
    result = {'frame_id': '7', 'host_timestamp': time.time() if stamp is None else stamp,
        'device_timestamp_ms': 100., 'color_url': '/rgbd/frame/7/color.jpg',
        'depth_url': '/rgbd/frame/7/depth.png', 'color_jpg': color,
        'depth_raw_png': depth, 'width': 4, 'height': 3,
        'intrinsics': {'fx': 2., 'fy': 3., 'cx': 1., 'cy': 1.}, 'info': {}}
    if token:
        result['capture_id'] = token
        result['color_url'] += '?capture_id=' + token
        result['depth_url'] += '?capture_id=' + token
    return result


def contents(value):
    return Path(value.color_ref).read_bytes(), Path(value.depth_ref).read_bytes()


def test_legacy_repeated_id_new_capture_cannot_reuse_or_overwrite_old_pair(camera, tmp_path):
    module, url = camera
    first = frame()
    module.FRAME_CACHE.append(first)
    source = RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, clock_mode='remote')
    old = source.get_latest()
    module.FRAME_CACHE.clear()  # A process restart loses the old server cache.
    module.FRAME_CACHE.append(frame(stamp=first['host_timestamp'] + .01,
        color=b'NEW COLOR', depth=b'NEW DEPTH'))
    new = source.get_latest()
    assert contents(new) == (b'NEW COLOR', b'NEW DEPTH')
    assert contents(old) == (b'OLD COLOR', b'OLD DEPTH')
    assert new.color_ref != old.color_ref


def test_client_restart_cannot_trust_unverified_numeric_disk_cache(camera, tmp_path):
    module, url = camera
    module.FRAME_CACHE.append(frame(color=b'CURRENT COLOR', depth=b'CURRENT DEPTH'))
    (tmp_path / 'color_7.jpg').write_bytes(b'OLD COLOR')
    (tmp_path / 'depth_7.png').write_bytes(b'OLD DEPTH')
    new = RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, clock_mode='remote').get_latest()
    assert contents(new) == (b'CURRENT COLOR', b'CURRENT DEPTH')


def test_server_token_selects_exact_pair_and_never_falls_back(camera, tmp_path):
    module, url = camera
    old, new = frame(token='epoch-a-1'), frame(token='epoch-b-1',
        color=b'NEW COLOR', depth=b'NEW DEPTH')
    module.FRAME_CACHE.extend([old, new])
    source = RealSenseHTTPRGBDSource(url, clock_mode='remote', cache_dir=tmp_path)
    try:
        meta = source._get_json('/rgbd/latest.json')
        assert meta['capture_id'] == new['capture_id']
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        for expected in (old, new):
            with opener.open(url + expected['color_url']) as response:
                assert response.read() == expected['color_jpg']
            with opener.open(url + expected['depth_url']) as response:
                assert response.read() == expected['depth_raw_png']
        for suffix, code in [('/color.jpg?capture_id=missing', 404),
                             ('/color.jpg', 409),
                             ('/color.jpg?capture_id=', 400),
                             ('/color.jpg?capture_id=a&capture_id=b', 400)]:
            with pytest.raises(urllib.error.HTTPError) as caught:
                opener.open(url + '/rgbd/frame/7' + suffix)
            assert caught.value.code == code
    finally:
        assert not list(tmp_path.iterdir())


def test_failed_depth_download_never_publishes_partial_pair_and_retry_fetches_both(camera, tmp_path, monkeypatch):
    module, url = camera
    module.FRAME_CACHE.append(frame(token='epoch-a-1'))
    source = RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, clock_mode='remote')
    meta = source.get_latest_metadata()
    download = source._download
    calls = []
    def interrupted(remote, path):
        calls.append(remote)
        if remote == meta.depth_ref:
            raise RGBDFrameUnavailable('injected disconnect')
        download(remote, path)
    monkeypatch.setattr(source, '_download', interrupted)
    with pytest.raises(RGBDFrameUnavailable, match='injected'):
        source.materialize(meta)
    assert not list(tmp_path.rglob('*.jpg'))
    assert not list(tmp_path.rglob('*.png'))
    monkeypatch.setattr(source, '_download', lambda remote, path:
        (calls.append(remote), download(remote, path))[-1])
    result = source.materialize(meta)
    assert contents(result) == (b'OLD COLOR', b'OLD DEPTH')
    assert calls == [meta.color_ref, meta.depth_ref, meta.color_ref, meta.depth_ref]
    source.materialize(meta)
    assert source.materialize(result) is result
    assert len(calls) == 4  # Repeated identical acquisition is still cached.


def test_restart_between_pair_requests_is_rejected_then_latest_capture_succeeds(camera, tmp_path, monkeypatch):
    module, url = camera
    module.FRAME_CACHE.append(frame(token='epoch-a-1'))
    source = RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, clock_mode='remote')
    old = source.get_latest_metadata()
    download = source._download
    def restart_after_color(remote, path):
        download(remote, path)
        if remote == old.color_ref:
            module.FRAME_CACHE.clear()
            module.FRAME_CACHE.append(frame(token='epoch-b-1',
                color=b'NEW COLOR', depth=b'NEW DEPTH'))
    monkeypatch.setattr(source, '_download', restart_after_color)
    with pytest.raises(RGBDFrameUnavailable):
        source.materialize(old)
    assert not list(tmp_path.rglob('*.jpg'))
    monkeypatch.setattr(source, '_download', download)
    new = source.get_latest()
    assert contents(new) == (b'NEW COLOR', b'NEW DEPTH')


def test_capture_bound_urls_cannot_be_missing_or_disagree(tmp_path, monkeypatch):
    source = RealSenseHTTPRGBDSource(cache_dir=tmp_path, clock_mode='remote')
    for color_url in ('/rgbd/frame/7/color.jpg',
                      '/rgbd/frame/7/color.jpg?capture_id=other',
                      '/rgbd/frame/7/color.jpg?capture_id=epoch-a-1&capture_id=other'):
        meta = frame(token='epoch-a-1')
        meta['color_url'] = color_url
        monkeypatch.setattr(source, '_get_json', lambda _: meta)
        with pytest.raises(RGBDFrameUnavailable, match='bind capture_id'):
            source.get_latest_metadata()
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('stamp', [None, 1., 'invalid', float('nan'), float('inf')])
def test_legacy_without_reliable_capture_stamp_does_not_revive_cached_pixels(tmp_path, monkeypatch, stamp):
    source = RealSenseHTTPRGBDSource(cache_dir=tmp_path, clock_mode='remote')
    meta = frame()
    meta['host_timestamp'] = stamp
    meta['health'] = {'age_s': 0.}
    monkeypatch.setattr(source, '_get_json', lambda _: meta)
    counter = []
    def download(url, path):
        counter.append(url)
        path.write_bytes(str(len(counter)).encode())
    monkeypatch.setattr(source, '_download', download)
    old_meta = source.get_latest_metadata()
    old = source.materialize(old_meta)
    assert source.materialize(old_meta).color_ref == old.color_ref
    new = source.get_latest()
    assert contents(old) == (b'1', b'2')
    assert contents(new) == (b'3', b'4')
    assert new.timestamp_quality == old.timestamp_quality == 'receive_time'


def test_pair_cache_is_bounded_without_orphan_depth_files(camera, tmp_path, monkeypatch):
    module, url = camera
    module.FRAME_CACHE.append(frame(token='epoch-a-1'))
    source = RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, max_cached_frames=8, clock_mode='remote')
    meta = source.get_latest_metadata()
    monkeypatch.setattr(source, '_download', lambda remote, path: path.write_bytes(b'data'))
    results = []
    for index in range(10):
        current = replace(meta, provenance={**meta.provenance, 'capture_id': str(index)})
        results.append(source.materialize(current))
    assert len(list(tmp_path.glob('capture_*'))) == 8
    assert len(list(tmp_path.rglob('*.jpg'))) == len(list(tmp_path.rglob('*.png'))) == 8
    assert sum(Path(value.color_ref).exists() for value in results) == 8
    assert all(Path(value.color_ref).exists() == Path(value.depth_ref).exists()
               for value in results)
    assert contents(results[-1]) == (b'data', b'data')


def test_current_download_cannot_be_evicted_by_filesystem_clock_rollback(camera, tmp_path, monkeypatch):
    module, url = camera
    module.FRAME_CACHE.append(frame(token='epoch-a-1'))
    source = RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, max_cached_frames=8, clock_mode='remote')
    meta = source.get_latest_metadata()
    generation = [0]
    def download(remote, path):
        path.write_bytes(b'data')
        stamp = 1. if generation[0] == 8 else 100.
        os.utime(path, (stamp, stamp))
    monkeypatch.setattr(source, '_download', download)
    for index in range(9):
        generation[0] = index
        current = replace(meta, provenance={**meta.provenance, 'capture_id': str(index)})
        assert contents(source.materialize(current)) == (b'data', b'data')
    assert len(list(tmp_path.glob('capture_*'))) == 8


def test_two_source_instances_can_commit_same_identity_without_deleting_pair(camera, tmp_path, monkeypatch):
    module, url = camera
    module.FRAME_CACHE.append(frame(token='epoch-a-1'))
    sources = [RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, clock_mode='remote') for _ in range(2)]
    meta = sources[0].get_latest_metadata()
    barrier = threading.Barrier(2)
    for source in sources:
        download = source._download
        def synchronized(remote, path, transfer=download):
            if remote == meta.color_ref:
                barrier.wait(timeout=2)
            transfer(remote, path)
        monkeypatch.setattr(source, '_download', synchronized)
    with ThreadPoolExecutor(max_workers=2) as pool:
        values = list(pool.map(lambda source: source.materialize(meta), sources))
    assert values[0].color_ref == values[1].color_ref
    assert all(contents(value) == (b'OLD COLOR', b'OLD DEPTH') for value in values)
    assert len(list(tmp_path.glob('capture_*'))) == 1
    assert not list(tmp_path.glob('.rgbd-*'))


def test_real_truncated_http_body_discards_pair_and_can_retry(camera, tmp_path, monkeypatch):
    module, url = camera
    module.FRAME_CACHE.append(frame(token='epoch-a-1'))
    source = RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, clock_mode='remote')
    original = module.Handler._send_bytes
    def truncated(handler, data, mime, *args, **kwargs):
        if mime == 'image/png':
            handler.send_response(200)
            handler.send_header('Content-Type', mime)
            handler.send_header('Content-Length', str(len(data) + 20))
            handler.send_header('Connection', 'close')
            handler.end_headers()
            handler.wfile.write(data[:2])
            handler.close_connection = True
        else:
            original(handler, data, mime, *args, **kwargs)
    monkeypatch.setattr(module.Handler, '_send_bytes', truncated)
    with pytest.raises(RGBDFrameUnavailable, match='download'):
        source.get_latest()
    assert not list(tmp_path.iterdir())
    monkeypatch.setattr(module.Handler, '_send_bytes', original)
    assert contents(source.get_latest()) == (b'OLD COLOR', b'OLD DEPTH')

