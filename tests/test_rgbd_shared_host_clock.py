import pytest

from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource
from app.perception.rgbd_source import RGBDFrameUnavailable


@pytest.mark.parametrize('url', ['http://192.168.123.18:8080', 'http://localhost:8080',
                               'http://example.invalid:8080'])
def test_shared_host_clock_cannot_be_selected_for_a_remote_or_named_host(tmp_path, url):
    with pytest.raises(ValueError, match='literal loopback'):
        RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, clock_mode='shared_host')


@pytest.mark.parametrize('url', ['http://127.0.0.1:8080', 'http://[::1]:8080'])
def test_source_uses_unshifted_capture_time_and_keeps_provenance(tmp_path, monkeypatch, url):
    from app.perception import realsense_http_rgbd_source as module
    epoch = 1792000000.
    walls, monotonic = iter([epoch+.05, epoch+.45]), iter([1., 1.4])
    monkeypatch.setattr(module.time, 'time', lambda: next(walls))
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(monotonic))
    source = RealSenseHTTPRGBDSource(url, cache_dir=tmp_path, clock_mode='shared_host')
    monkeypatch.setattr(source, '_get_json', lambda _: {
        'frame_id': '42', 'host_timestamp': epoch, 'health': {'age_s': .15}})
    monkeypatch.setattr(source, 'materialize', lambda frame: frame)
    frame = source._get_latest_once()
    assert frame.timestamp == epoch
    assert frame.host_timestamp == epoch
    assert frame.timestamp_quality == 'shared_host_capture'
    assert frame.provenance['capture_clock']['server_minus_local_seconds'] == 0.


@pytest.mark.parametrize('host,health', [(None, {'age_s': 0}), (1., {'age_s': 0}),
                                       (1792000000., {})])
def test_shared_host_never_falls_back_to_a_fresh_receive_timestamp(tmp_path, monkeypatch, host, health):
    source = RealSenseHTTPRGBDSource('http://127.0.0.1:8080', cache_dir=tmp_path,
                                   clock_mode='shared_host')
    monkeypatch.setattr(source, '_get_json', lambda _: {
        'frame_id': '42', 'host_timestamp': host, 'health': health})
    with pytest.raises(RGBDFrameUnavailable, match='metadata missing'):
        source._get_latest_once()
