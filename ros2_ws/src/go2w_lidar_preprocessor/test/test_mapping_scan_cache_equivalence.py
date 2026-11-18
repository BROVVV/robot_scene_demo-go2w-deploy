"""Compare every output against the frozen uncached algorithm, including resets."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from go2w_lidar_preprocessor.mapping_scan_accumulator import MappingScanAccumulator

spec = importlib.util.spec_from_file_location(
    'mapping_scan_uncached_reference',
    Path(__file__).parent/'fixtures/mapping_scan_reference.py')
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


def compare(old, new, ranges, **parameters):
    before = np.asarray(ranges).tobytes()
    expected = old.add(ranges, **parameters)
    actual = new.add(ranges, **parameters)
    assert np.asarray(ranges).tobytes() == before
    assert old.status == new.status
    assert (expected is None) == (actual is None)
    if expected is not None:
        assert actual.dtype == expected.dtype and actual.shape == expected.shape
        assert actual.tobytes() == expected.tobytes()
    assert set(new._world_endpoints) <= {row[0] for row in new.frames}
    assert len(new._world_endpoints) <= new.max_frames
    return expected, actual


@pytest.mark.parametrize('bins,span', [(8,2*math.pi), (720,2*math.pi), (97,2.2)])
@pytest.mark.parametrize('seed', [9,47,313])
def test_history_cache_is_bit_exact_through_motion_clock_and_geometry_changes(bins, span, seed):
    rng = np.random.default_rng(seed)
    old = reference.MappingScanAccumulator()
    new = MappingScanAccumulator()
    stamp = 100.
    for index in range(240):
        if index in (61,181):
            stamp -= 1.  # Regression invalidates all old world endpoints.
        elif index == 121:
            stamp += 2.  # Dropout must not carry old observations forward.
        else:
            stamp += 1/15
        count = bins if index < 150 else bins+5
        active_span = span if index < 190 else span*.7
        ranges = rng.uniform(.2,7.,count).astype(np.float32)
        ranges[rng.random(count)<.35] = math.inf
        ranges[index%count] = [math.nan,-math.inf,math.inf,.37,6.2][index%5]
        block = index//40
        pose = (block*.2+(index%40)*.00005, -.001*block,
                (math.pi-.0001 if block%2 else 0.)+(index%40)*.000002)
        values = dict(stamp=stamp,pose=pose,angle_min=-span/2,
                      angle_increment=active_span/count,range_min=.37,range_max=6.2)
        expected, actual = compare(old,new,ranges,**values)
        if actual is not None and index%37 == 0:
            # Returned scans are owned outputs; consumers cannot mutate history.
            expected[:] = 5.;actual[:] = 5.


def test_current_obstacle_and_unknown_bins_survive_cached_history():
    old = reference.MappingScanAccumulator();new = MappingScanAccumulator()
    args = dict(pose=(0.,0.,0.),angle_min=-math.pi,
                angle_increment=2*math.pi/720,range_min=.37,range_max=6.2)
    for i in range(40):
        r=np.full(720,math.inf,dtype=np.float32);r[350]=4.
        if i>=14:r[350]=.5
        expected,actual=compare(old,new,r,stamp=100+i/15,**args)
        if actual is not None:
            assert actual[350]==(.5 if i>=14 else 4.)
            assert np.isinf(actual[:300]).all()


def test_cache_eviction_reset_and_input_ownership():
    old = reference.MappingScanAccumulator(window_s=2.,minimum_span_s=.2,max_frames=4)
    new = MappingScanAccumulator(window_s=2.,minimum_span_s=.2,max_frames=4)
    args=dict(pose=(0.,0.,0.),angle_min=-math.pi,angle_increment=math.pi/4,
              range_min=.37,range_max=6.2)
    for i in range(100):
        r=np.ones(8);compare(old,new,r,stamp=100+i*.1,**args);r[:]=6.
        assert all(np.all(frame[2]==1.) for frame in new.frames)
    assert new._world_endpoints and len(new._world_endpoints)==4
    with pytest.raises(ValueError):new.frames[0][2][0]=2.
    old.reset();new.reset();assert not new._world_endpoints and not new.frames
    compare(old,new,np.ones(8),stamp=120.,**args)
    for obj in (old,new):
        with pytest.raises(ValueError):obj.add(np.ones(8),stamp=121.,**dict(args,pose=(math.nan,0.,0.)))
    assert not new._world_endpoints and not new.frames
