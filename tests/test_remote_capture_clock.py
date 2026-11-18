import pytest

from app.perception.remote_capture_clock import align_capture_time, shared_host_capture_time
from app.perception.rgbd_source import RGBDFrameUnavailable


def test_remote_clock_offset_and_capture_age_are_both_removed():
    stamp, evidence=align_capture_time(1792000003.10, .04,
        started=1792000000.13, received=1792000000.15, elapsed=.02)
    assert stamp == pytest.approx(1792000000.10, abs=1e-6)
    assert evidence['server_minus_local_seconds'] == pytest.approx(3., abs=1e-6)
    assert evidence['uncertainty_seconds'] == pytest.approx(.011)


@pytest.mark.parametrize('age,elapsed,wall_delta', [(-.1,.01,.01),(.1,.1,.1),
    (.1,.01,1.),(float('nan'),.01,.01)])
def test_slow_invalid_or_clock_jump_observation_is_rejected(age,elapsed,wall_delta):
    with pytest.raises(RGBDFrameUnavailable):
        align_capture_time(1792000003.,age,started=1792000000.,
            received=1792000000.+wall_delta,elapsed=elapsed)


def test_reboot_offset_is_never_carried_to_the_next_response():
    def sample(host):
        return align_capture_time(host,.02,started=1792000000.,
            received=1792000000.02,elapsed=.02)
    first,a=sample(1792000003.);second,b=sample(1791999998.)
    assert first == second
    assert a['server_minus_local_seconds']-b['server_minus_local_seconds'] == pytest.approx(5.)


def test_shared_clock_preserves_real_capture_even_with_slow_http():
    host = 1792000000.
    stamp, evidence = shared_host_capture_time(host, .15,
        started=host+.05, received=host+.45, elapsed=.4, max_age_seconds=.5)
    assert stamp == host
    assert evidence['method'] == 'shared_host_capture_clock'
    assert evidence['capture_age_seconds'] == pytest.approx(.45, abs=1e-6)
    # Cross-host default retains the 30 ms uncertainty rejection.
    with pytest.raises(RGBDFrameUnavailable):
        align_capture_time(host, .15, started=host+.05, received=host+.45, elapsed=.4)


@pytest.mark.parametrize('host,age,started,received,elapsed,max_age', [
    (-1792000000., .1, 0., .1, .1, .5),  # invalid Unix epoch
    (0., -.1, -.1, .1, .2, .5),  # negative server age
    (-3., .1, 0., .1, .1, .5),  # remote clock offset
    (1., .1, 0., .1, .1, .5),  # future capture
    (0., .1, 0., .6, .6, .5),  # frame expired during a slow response
    (0., .1, 0., .1, .01, .5),  # local wall clock step
    (0., float('nan'), 0., .1, .1, .5),
])
def test_shared_clock_rejects_invalid_offset_stale_and_clock_steps(
        host, age, started, received, elapsed, max_age):
    epoch = 1792000000.
    with pytest.raises(RGBDFrameUnavailable):
        shared_host_capture_time(epoch+host, age, started=epoch+started,
            received=epoch+received, elapsed=elapsed, max_age_seconds=max_age)
