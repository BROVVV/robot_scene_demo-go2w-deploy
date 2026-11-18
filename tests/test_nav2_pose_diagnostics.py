from types import SimpleNamespace as NS
import sys
import time

import pytest

from app.spatial.nav2_spatial_provider import Nav2SpatialProvider


def provider_for_transform(monkeypatch, *, age=0., x=1., error=None):
    p = Nav2SpatialProvider(enable_ros=False)
    p._valid = True
    p._valid_at = time.monotonic()
    p._valid_basis = 'navigation_health'
    now = 100.
    stamp = now - age
    transform = NS(header=NS(stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
                   transform=NS(translation=NS(x=x, y=2., z=0.),
                                rotation=NS(x=0., y=0., z=0., w=1.)))
    def lookup(*_):
        if error:
            raise LookupError(error)
        return transform
    p._tf_buffer = NS(lookup_transform=lookup)
    p._node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(now*1e9))))
    monkeypatch.setitem(sys.modules, 'rclpy', NS(time=NS(Time=lambda: object())))
    return p


@pytest.mark.parametrize('age', [.301, -.301])
def test_tf_timestamp_rejection_reports_age_without_promoting_pose(monkeypatch, age):
    p = provider_for_transform(monkeypatch, age=age)
    assert p.get_pose() is None
    d = p.pose_diagnostics()
    assert d['reason'] == 'tf_pose_timestamp_out_of_range'
    assert d['tf_age_s'] == pytest.approx(age)
    assert d['localization_valid'] is True


def test_tf_exception_is_distinct_from_missing_health(monkeypatch):
    p = provider_for_transform(monkeypatch, error='no map to base chain')
    assert p.get_pose() is None
    assert p.pose_diagnostics()['reason'] == 'tf_pose_lookup_failed'
    assert 'no map to base chain' in p.pose_diagnostics()['detail']
    p._valid = False
    assert p.get_pose() is None
    assert p.pose_diagnostics()['reason'] == 'localization_health_unavailable'
    assert 'detail' not in p.pose_diagnostics()


def test_stale_health_still_blocks_fresh_tf(monkeypatch):
    p = provider_for_transform(monkeypatch)
    p._valid_at -= 1.
    assert p.get_pose() is None
    assert p.pose_diagnostics()['reason'] == 'localization_health_unavailable'
    assert p.pose_diagnostics()['localization_receive_age_s'] >= 1.


def test_nonfinite_pose_is_not_a_tf_timestamp_error(monkeypatch):
    p = provider_for_transform(monkeypatch, x=float('nan'))
    assert p.get_pose() is None
    assert p.pose_diagnostics()['reason'] == 'tf_pose_nonfinite'


def test_recovery_replaces_failure_and_diagnostic_is_a_copy(monkeypatch):
    p = provider_for_transform(monkeypatch)
    p._valid = False
    assert p.get_pose() is None
    p._valid = True
    pose = p.get_pose()
    assert pose is not None and pose.frame_id == 'map'
    assert p.pose_diagnostics()['reason'] == 'available'
    d = p.pose_diagnostics()
    d['reason'] = 'modified by caller'
    assert p.pose_diagnostics()['reason'] == 'available'
