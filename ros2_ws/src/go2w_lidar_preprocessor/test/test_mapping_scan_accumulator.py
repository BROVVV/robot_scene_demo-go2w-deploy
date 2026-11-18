import math
import numpy as np
import pytest
from go2w_lidar_preprocessor.mapping_scan_accumulator import MappingScanAccumulator, MappingKeyframeSelector


def test_keyframes_require_translation_or_rotation_and_do_not_repeat_at_rest():
    s = MappingKeyframeSelector()
    assert not s.select((0, 0, 0), stable=False)
    assert s.select((0, 0, 0), stable=True)
    for _ in range(100):
        assert not s.select((0, 0, 0), stable=True)
    assert not s.select((.04, 0, .04), stable=True)
    assert s.select((0, 0, .11), stable=True)  # Pure rotation must map.
    assert s.select((.11, 0, .11), stable=True)


def test_moving_keyframes_use_raw_scans_without_waiting_for_a_stop():
    s = MappingKeyframeSelector()
    assert not s.select((.2, 0, 0), stable=False)  # Cold map requires dense scan.
    assert s.select((.2, 0, 0), stable=True)
    assert s.select((.31, 0, 0), stable=False)
    assert s.select((.31, 0, .11), stable=False)
    assert not s.select((.31, 0, .11), stable=False)


def test_keyframes_compare_to_last_emitted_pose_and_wrap_yaw():
    s = MappingKeyframeSelector()
    assert s.select((0, 0, math.pi-.02), stable=True)
    assert not s.select((.03, 0, -math.pi+.02), stable=True)
    assert s.select((.06, 0, -math.pi+.02), stable=True)


def test_new_mapper_requests_one_stable_initial_keyframe():
    s = MappingKeyframeSelector()
    assert s.select((0, 0, 0), stable=True)
    s.request_initial_scan()
    assert not s.select((0, 0, 0), stable=False)
    assert not s.select((math.nan, 0, 0), stable=True)
    assert s.select((0, 0, 0), stable=True)
    assert not s.select((0, 0, 0), stable=True)


@pytest.mark.parametrize('threshold', [0, -1, math.nan, math.inf])
def test_keyframe_invalid_threshold_is_rejected(threshold):
    with pytest.raises(ValueError):
        MappingKeyframeSelector(minimum_translation_m=threshold)


def add(a,t,values,pose=(0,0,0)):
    return a.add(values,stamp=t,pose=pose,angle_min=-math.pi,
                 angle_increment=math.pi/4,range_min=.37,range_max=6.2)


def test_sparse_frames_fill_only_observed_bins_after_initial_prefill():
    a=MappingScanAccumulator();missing=[math.inf]*8
    r=missing.copy();r[0]=2
    assert add(a,100,r) is None
    r=missing.copy();r[2]=3
    out=add(a,100.65,r)
    assert out[0]==2 and out[2]==3
    assert np.isinf(out[[1,3,4,5,6,7]]).all()
    assert a.status['mode']=='accumulated_stationary'


def test_new_closer_obstacle_wins_over_old_return():
    a=MappingScanAccumulator();add(a,100,[2]*8)
    out=add(a,100.65,[1]*8)
    assert np.all(out==1)


@pytest.mark.parametrize('pose',[(.1,0,0),(0,0,.1)])
def test_motion_flushes_history_and_returns_fresh_raw_only(pose):
    a=MappingScanAccumulator();add(a,100,[1]*8);add(a,100.65,[1]*8)
    raw=[math.inf]*8;raw[4]=2
    out=add(a,100.7,raw,pose)
    assert out[4]==2 and np.isfinite(out).sum()==1
    assert len(a.frames)==1


def test_old_scans_expire_even_when_pose_never_changes():
    a=MappingScanAccumulator();add(a,100,[1]*8);add(a,100.65,[2]*8)
    out=add(a,101.1,[3]*8)
    assert np.all(out==3)  # Not enough span after expiry: latest raw only.
    out=add(a,101.3,[4]*8)
    assert np.all(out==2)
    assert all(t>=100.55 for t,_,_ in a.frames)


def test_clock_regression_and_geometry_change_require_new_prefill():
    a=MappingScanAccumulator();add(a,100,[1]*8);add(a,100.65,[1]*8)
    assert add(a,99,[2]*8) is None
    assert add(a,99.65,[2]*4) is None


def test_cache_is_bounded_and_caller_arrays_are_not_retained_by_reference():
    a=MappingScanAccumulator(max_frames=4);r=np.ones(8)
    add(a,100,r);r[:]=2
    assert a.frames[0][2][0]==1
    for i in range(20):add(a,100.01*(i+1),[1]*8)
    assert len(a.frames)<=4


def test_invalid_returns_do_not_clear_unknown_space():
    a=MappingScanAccumulator();raw=[math.inf,math.nan,0,.1,7,2,math.inf,math.inf]
    add(a,100,raw);out=add(a,100.65,raw)
    assert np.isfinite(out).sum()==1 and out[5]==2


def test_alignment_compensates_old_endpoint_into_current_pose():
    a=MappingScanAccumulator();r=[math.inf]*8;r[4]=2
    add(a,100,r);out=add(a,100.65,[math.inf]*8,pose=(.005,0,0))
    assert out[4]==pytest.approx(1.995)


def test_invalid_pose_never_produces_a_scan():
    a=MappingScanAccumulator()
    with pytest.raises(ValueError):add(a,100,[1]*8,pose=(math.nan,0,0))
    assert not a.frames
