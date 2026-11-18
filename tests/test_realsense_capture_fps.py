"""Measure public camera health from simulated RGB-D acquisition, not queries."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import urllib.request

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


class EndCapture(BaseException):
    pass


@pytest.fixture
def camera(monkeypatch):
    monkeypatch.setitem(sys.modules, 'pyrealsense2', SimpleNamespace())
    spec = importlib.util.spec_from_file_location('camera_fps_test',
        ROOT / 'scripts/go2w/realsense_stream.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = SimpleNamespace(mono=1000., wall_offset=0.)
    def unexpected_sleep(seconds):
        raise EndCapture('capture fixture stopped after an error')
    monkeypatch.setattr(module, 'time', SimpleNamespace(
        monotonic=lambda: clock.mono,
        time=lambda: 1790000000. + clock.mono + clock.wall_offset,
        sleep=unexpected_sleep))
    monkeypatch.setattr(module.cv2, 'imencode',
        lambda *args, **kwargs: pytest.fail('metadata/cadence encoded an image'))
    return module, clock


def capture(module, clock, ids=range(31), *, hz=30.):
    ids = iter(ids)
    color = np.full((3, 4, 3), 127, dtype=np.uint8)
    depth = np.full((3, 4), 1500, dtype=np.uint16)
    def image(array):
        return SimpleNamespace(get_data=lambda: array, get_width=lambda: 4,
            get_height=lambda: 3, get_timestamp=lambda: clock.mono * 1000.)
    device = SimpleNamespace(first_depth_sensor=lambda: None,
        query_sensors=lambda: [None, None])
    profile = SimpleNamespace(get_device=lambda: device, get_streams=lambda: [])
    class Pipeline:
        def start(self, config):
            return profile
        def wait_for_frames(self):
            try:
                number = next(ids)
            except StopIteration:
                raise EndCapture()
            clock.mono += 1. / hz
            return SimpleNamespace(frame_number=number,
                get_color_frame=lambda: image(color), get_depth_frame=lambda: image(depth))
    module.rs = SimpleNamespace(context=lambda: SimpleNamespace(devices=[device]),
        pipeline=Pipeline, config=lambda: SimpleNamespace(enable_stream=lambda *args: None),
        align=lambda stream: SimpleNamespace(process=lambda frames: frames),
        stream=SimpleNamespace(color=1, depth=2), format=SimpleNamespace(bgr8=1, z16=2),
        option=SimpleNamespace())
    module.ARGS = SimpleNamespace(width=4, height=3, fps=30, max_depth=6.,
        encoding_mode='on-demand', verbose=False)
    with pytest.raises(EndCapture):
        module.rs_capture_loop()


def get(module, path='/health'):
    payload = []
    handler = object.__new__(module.Handler)
    handler.path = path
    handler._send_json = lambda value, code=200: payload.append((code, value))
    handler.do_GET()
    assert payload and payload[-1][0] == 200
    return payload[-1][1]


def test_health_fps_measures_capture_intervals_not_last_frame_age(camera):
    module, clock = camera
    capture(module, clock)
    clock.mono += .014
    result = get(module)
    assert result['ok'] and result['streaming']
    assert result['age_s'] == pytest.approx(.014, abs=.001)
    assert result['fps'] == pytest.approx(30., abs=.01)


def test_request_phase_does_not_change_measured_fps(camera):
    module, clock = camera
    capture(module, clock)
    values = []
    for delay in (.001, .014, .025):
        module.BUFFER['stamp'] = module.time.time() - delay
        values.append(get(module)['fps'])
    assert values == pytest.approx([30., 30., 30.], abs=.01)


def test_stalled_capture_reports_zero_without_changing_health_age_policy(camera):
    module, clock = camera
    capture(module, clock)
    clock.mono += 1.5
    result = get(module)
    assert result['ok'] is True  # Existing five-second health policy remains.
    assert result['age_s'] == pytest.approx(1.5, abs=.001)
    assert result['fps'] == 0.


def test_inactive_stream_never_reports_a_positive_capture_rate(camera):
    module, clock = camera
    capture(module, clock)
    clock.mono += .014
    module.STREAM_ACTIVE.clear()
    result = get(module)
    assert result['streaming'] is False and result['ok'] is False
    assert result['fps'] == 0.


def test_one_frame_does_not_invent_a_rate(camera):
    module, clock = camera
    capture(module, clock, ids=[1])
    clock.mono += .014
    assert get(module)['fps'] == 0.


def test_repeated_frame_id_does_not_inflate_capture_rate(camera):
    module, clock = camera
    capture(module, clock, ids=[7] * 31)
    clock.mono += .014
    assert get(module)['fps'] == 0.


def test_reconnect_does_not_reuse_previous_pipeline_rate(camera):
    module, clock = camera
    capture(module, clock)
    clock.mono += 5.
    capture(module, clock, ids=[1])
    clock.mono += .014
    assert get(module)['fps'] == 0.


def test_actual_capture_loop_assigns_distinct_tokens_despite_counter_reuse(camera):
    module, clock = camera
    capture(module, clock, ids=[7, 7])
    before = list(module.FRAME_CACHE)
    capture(module, clock, ids=[7])  # A new pipeline session resets its counter.
    frames = list(module.FRAME_CACHE)
    assert len(frames) == 3 and all(f['frame_id'] == '7' for f in frames)
    assert len({f['capture_id'] for f in frames}) == 3
    for frame in frames:
        assert frame['color_url'].endswith('?capture_id=' + frame['capture_id'])
        assert frame['depth_url'].endswith('?capture_id=' + frame['capture_id'])
        assert frame['info']['capture_id'] == frame['capture_id']
    assert frames[:2] == before


def test_slow_processed_capture_is_not_replaced_by_configured_fps(camera):
    module, clock = camera
    capture(module, clock, hz=15.)
    clock.mono += .014
    assert get(module)['fps'] == pytest.approx(15., abs=.01)


def test_health_and_metadata_polling_neither_encode_nor_count_frames(camera):
    module, clock = camera
    capture(module, clock)
    clock.mono += .014
    frame = module.FRAME_CACHE[-1]
    assert frame['_encoder']._bytes == {}
    for _ in range(100):
        assert get(module)['fps'] == pytest.approx(30., abs=.01)
        assert get(module, '/rgbd/latest.json')['frame_id'] == frame['frame_id']
    assert frame['_encoder']._bytes == {}


def test_wall_clock_change_does_not_change_capture_cadence_or_bypass_age_policy(camera):
    module, clock = camera
    capture(module, clock)
    clock.mono += .014
    clock.wall_offset += 30.
    result = get(module)
    assert result['fps'] == pytest.approx(30., abs=.01)
    assert result['age_s'] == pytest.approx(30.014, abs=.001)
    assert result['ok'] is False


def test_real_local_http_health_and_metadata_do_not_encode_or_inflate_rate(camera):
    module, clock = camera
    capture(module, clock)
    clock.mono += .014
    server = module.ThreadingHTTPServer(('127.0.0.1', 0), module.Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    url = 'http://127.0.0.1:' + str(server.server_port)
    try:
        for _ in range(10):
            with opener.open(url + '/health', timeout=2) as response:
                health = json.loads(response.read())
                assert set(health) == {'ok', 'streaming', 'age_s', 'fps'}
                assert health['fps'] == pytest.approx(30., abs=.01)
            with opener.open(url + '/rgbd/latest.json', timeout=2) as response:
                meta = json.loads(response.read())
                assert meta['frame_id'] == module.FRAME_CACHE[-1]['frame_id']
        assert module.FRAME_CACHE[-1]['_encoder']._bytes == {}
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
