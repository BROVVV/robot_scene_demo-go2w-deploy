"""Demand publishing preserves acquisition time, depth units and raw consumers."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
cv2 = pytest.importorskip('cv2')
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('d435_demand_test',
    ROOT/'scripts/go2w/realsense_rgbd_bridge.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Publisher:
    def __init__(self, subscribers=0):
        self.subscribers = subscribers
        self.messages = []

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(message)


def observer(tmp_path, *, demand=True, subscribers=(), aligned=True):
    color = tmp_path/'color.jpg'
    depth = tmp_path/'depth.png'
    assert cv2.imwrite(str(color), np.full((3, 4, 3), 127, dtype=np.uint8))
    assert cv2.imwrite(str(depth), np.full((3, 4), 1500, dtype=np.uint16))
    frame = SimpleNamespace(frame_id='1', color_ref=str(color), depth_ref=str(depth),
        width=4, height=3, fx=2., fy=3., cx=1., cy=1., depth_unit_m=.001,
        depth_aligned_to_color=aligned, timestamp_quality='host_timestamp',
        host_timestamp=1790000000.125, health={'age_s': .01})
    warnings = []
    calls = []
    node = SimpleNamespace(source=SimpleNamespace(
        get_latest=lambda **kw: (calls.append('images'), frame)[1],
        get_latest_metadata=lambda **kw: (calls.append('metadata'), frame)[1]),
        source_calls=calls,
        bridge=CvBridge(), on_demand=demand, _last_frame_id=None,
        get_logger=lambda:SimpleNamespace(info=lambda message:None, warn=warnings.append),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=Time)))
    for name in ('color_pub', 'front_color_pub', 'depth_pub',
                 'front_compressed_pub', 'info_pub', 'front_info_pub', 'health_pub'):
        setattr(node, name, Publisher(int(name in subscribers)))
    return node, frame


@pytest.mark.parametrize('subscriber', ['', 'color_pub', 'front_color_pub',
                                        'depth_pub', 'front_compressed_pub'])
def test_only_requested_images_are_decoded_and_published(tmp_path, monkeypatch, subscriber):
    node, frame = observer(tmp_path, subscribers=(subscriber,))
    decoded = []
    original = module.cv2.imread

    def read(path, flags):
        decoded.append(path)
        return original(path, flags)

    monkeypatch.setattr(module.cv2, 'imread', read)
    monkeypatch.setattr(module.cv2, 'imencode',
        lambda *args, **kw: pytest.fail('demand mode must forward the existing JPEG'))
    module.D435RGBDBridge.spin_once(node)
    for name in ('color_pub', 'front_color_pub', 'depth_pub', 'front_compressed_pub'):
        assert len(getattr(node, name).messages) == int(name == subscriber)
    expected = ([frame.color_ref] if subscriber in ('color_pub', 'front_color_pub')
                else [frame.depth_ref] if subscriber == 'depth_pub' else [])
    assert decoded == expected
    assert node.source_calls == [('images' if subscriber else 'metadata')]
    if subscriber == 'front_compressed_pub':
        message = node.front_compressed_pub.messages[0]
        assert bytes(message.data) == Path(frame.color_ref).read_bytes()
        assert message.header.stamp.sec == 1790000000
        assert message.header.stamp.nanosec == 125000000
    assert len(node.info_pub.messages) == len(node.front_info_pub.messages) == 1
    assert len(node.health_pub.messages) == 1


@pytest.mark.parametrize('aligned', [True, False])
def test_new_depth_subscriber_gets_metric_depth_with_original_stamp(tmp_path, aligned):
    node, frame = observer(tmp_path, aligned=aligned)
    module.D435RGBDBridge.spin_once(node)
    assert not node.depth_pub.messages
    node.depth_pub.subscribers = 1
    frame.frame_id = '2'
    module.D435RGBDBridge.spin_once(node)
    message = node.depth_pub.messages[0]
    assert message.encoding == '32FC1'
    assert message.header.stamp.sec == 1790000000
    assert message.header.stamp.nanosec == 125000000
    assert message.header.frame_id == ('d435_color_optical_frame' if aligned
                                       else 'd435_depth_optical_frame')
    np.testing.assert_allclose(node.bridge.imgmsg_to_cv2(message), 1.5)


def test_desktop_default_still_publishes_every_image_without_subscribers(tmp_path):
    node, frame = observer(tmp_path, demand=False)
    module.D435RGBDBridge.spin_once(node)
    for name in ('color_pub', 'front_color_pub', 'depth_pub', 'front_compressed_pub'):
        assert len(getattr(node, name).messages) == 1
    # Re-polling the identical acquisition must not duplicate its messages.
    module.D435RGBDBridge.spin_once(node)
    assert len(node.depth_pub.messages) == 1


@pytest.mark.parametrize('identity_field', ['host_timestamp', 'capture_id', 'cache_nonce'])
def test_same_counter_after_reconnect_is_a_new_ros_capture(tmp_path, identity_field):
    node, frame = observer(tmp_path, subscribers=('depth_pub',))
    frame.provenance = {} if identity_field == 'host_timestamp' else {identity_field: 'epoch-a-1'}
    module.D435RGBDBridge.spin_once(node)
    if identity_field == 'host_timestamp':
        frame.host_timestamp += .1
    else:
        frame.provenance[identity_field] = 'epoch-b-1'
    module.D435RGBDBridge.spin_once(node)
    assert len(node.depth_pub.messages) == 2
    module.D435RGBDBridge.spin_once(node)
    assert len(node.depth_pub.messages) == 2
