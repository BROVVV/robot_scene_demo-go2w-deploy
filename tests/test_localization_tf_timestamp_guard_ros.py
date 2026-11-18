"""Invalid future TF must not remain as a healthy latest-pose result."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy', reason='requires sourced ROS Python')
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

path = Path(__file__).resolve().parents[1] / 'ros2_ws/src/robot_scene_nav_bringup/scripts/localization_health_monitor.py'
spec = importlib.util.spec_from_file_location('localization_timestamp_guard_test', path)
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def transform(parent, child, stamp, x):
    result = TransformStamped()
    result.header.frame_id = parent
    result.child_frame_id = child
    ns = round(stamp * 1e9)
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(ns, 10**9)
    result.transform.translation.x = float(x)
    result.transform.rotation.w = 1.
    return result


def test_bad_future_pose_does_not_survive_recovery_in_real_tf2_buffer():
    clock = [100.]
    buffer = monitor.TimestampGuardedBuffer(SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock[0] * 1e9))))
    for parent, child, x in [('map', 'odom_fused', 1.), ('odom_fused', 'base_link', 2.)]:
        buffer.set_transform(transform(parent, child, 100., x), 'fixture')
    assert buffer.lookup_transform('map', 'base_link', Time()).transform.translation.x == 3.
    buffer.set_transform(transform('map', 'odom_fused', 102., 50.), 'fixture')
    buffer.set_transform(transform('odom_fused', 'base_link', 102., 60.), 'fixture')
    assert buffer.rejected_stamps == 2
    clock[0] = 100.1
    buffer.set_transform(transform('map', 'odom_fused', 100.1, 4.), 'fixture')
    buffer.set_transform(transform('odom_fused', 'base_link', 100.1, 5.), 'fixture')
    # Raw tf2 would retain the future 110m pose here instead of the fresh 9m.
    pose = buffer.lookup_transform('map', 'base_link', Time())
    assert pose.transform.translation.x == 9.
    assert pose.header.stamp.sec == 100 and pose.header.stamp.nanosec == 100_000_000
    assert buffer.future_recovery_pending == ['map->odom_fused', 'odom_fused->base_link']


def test_recovery_waits_for_expiry_and_newer_samples_on_both_edges():
    clock = [100.]
    buffer = monitor.TimestampGuardedBuffer(SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock[0] * 1e9))))
    for parent, child in [('map', 'odom_fused'), ('odom_fused', 'base_link')]:
        buffer.set_transform(transform(parent, child, 102., 99.), 'fixture')
    assert len(buffer.future_recovery_pending) == 2
    assert buffer.future_recovery_after_ns['map->odom_fused'] == 102_300_000_000
    clock[0] = 102.1
    for parent, child in [('map', 'odom_fused'), ('odom_fused', 'base_link')]:
        buffer.set_transform(transform(parent, child, 102.1, 0.), 'fixture')
    assert len(buffer.future_recovery_pending) == 2  # Ordinary cache still accepts the old timestamp.
    clock[0] = 102.4
    # Repeating the exact poisoned timestamp cannot replace it in other caches.
    buffer.set_transform(transform('map', 'odom_fused', 102., 0.), 'fixture')
    assert len(buffer.future_recovery_pending) == 2
    buffer.set_transform(transform('map', 'odom_fused', 102.4, 0.), 'fixture')
    assert buffer.future_recovery_pending == ['odom_fused->base_link']
    buffer.set_transform(transform('odom_fused', 'base_link', 102.4, 0.), 'fixture')
    assert buffer.future_recovery_pending == []


def test_invalid_quaternion_cannot_release_the_recovery_gate():
    clock = [100.]
    buffer = monitor.TimestampGuardedBuffer(SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock[0] * 1e9))))
    buffer.set_transform(transform('map', 'odom_fused', 102., 99.), 'fixture')
    clock[0] = 102.4
    bad = transform('map', 'odom_fused', 102.4, 0.)
    bad.transform.rotation.w = 0.
    buffer.set_transform(bad, 'fixture')
    assert buffer.future_recovery_pending == ['map->odom_fused']
    clock[0] = 102.5
    buffer.set_transform(transform('map', 'odom_fused', 102.5, 0.), 'fixture')
    assert buffer.future_recovery_pending == []


def test_newer_rejected_future_packet_is_not_lost_during_recovery_lookup():
    clock = [100.]
    buffer = monitor.TimestampGuardedBuffer(SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock[0] * 1e9))))
    buffer.set_transform(transform('map', 'odom_fused', 102., 99.), 'fixture')
    clock[0] = 102.4
    original_lookup = buffer.lookup_transform
    def concurrent_rejection(*args, **kwargs):
        value = original_lookup(*args, **kwargs)
        buffer.set_transform(transform('map', 'odom_fused', 104., 99.), 'fixture')
        return value
    buffer.lookup_transform = concurrent_rejection
    buffer.set_transform(transform('map', 'odom_fused', 102.4, 0.), 'fixture')
    assert buffer.future_recovery_pending == ['map->odom_fused']
    assert buffer.future_recovery_after_ns['map->odom_fused'] == 104_300_000_000


@pytest.mark.parametrize('stamp,accepted', [(99.71, True), (100., True),
    (100.29, True), (99.69, False), (100.31, False), (98., False), (102., False)])
def test_buffer_uses_existing_health_time_bounds(stamp, accepted):
    buffer = monitor.TimestampGuardedBuffer(SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    buffer.set_transform(transform('map', 'base_link', stamp, 1.), 'fixture')
    assert bool(buffer.can_transform('map', 'base_link', Time())) is accepted
    assert buffer.rejected_stamps == int(not accepted)


def test_static_zero_stamp_remains_valid_and_does_not_count_as_rejected():
    buffer = monitor.TimestampGuardedBuffer(SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    buffer.set_transform_static(transform('base_link', 'camera', 0., .2), 'fixture')
    assert buffer.lookup_transform('base_link', 'camera', Time()).transform.translation.x == .2
    assert buffer.rejected_stamps == 0
