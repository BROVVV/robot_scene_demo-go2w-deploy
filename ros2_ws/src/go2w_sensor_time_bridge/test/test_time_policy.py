"""Navigation stamps can follow host receive time without touching raw LIO bytes."""
from types import SimpleNamespace
from go2w_sensor_time_bridge.time_bridge_node import TimeBridge


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self, message):self.messages.append(message)


def bridge():
    node=TimeBridge.__new__(TimeBridge)
    node._policy='host_receive';node._previous={};node._config=None
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_790_000_000_123_000_000))
    node._diagnostic=lambda *a,**k:None
    node._raw_cloud_pub=Publisher();node._cloud_pub=Publisher()
    return node


def cloud(stamp=20):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=0),frame_id='lidar'),
                           data=bytearray(b'xyz_and_relative_point_times'),fields=['x','y','z','time'])


def test_host_receive_needs_no_historical_fit_and_preserves_raw_payload():
    node=bridge();message=cloud();node._cloud(message)
    assert node._raw_cloud_pub.messages[0] is message
    aligned=node._cloud_pub.messages[0]
    assert aligned is not message and aligned.header is not message.header
    assert aligned.header.stamp.sec==1_790_000_000
    assert message.header.stamp.sec==20
    assert aligned.data==message.data and aligned.fields==message.fields


def test_regressing_sensor_clock_keeps_raw_lio_but_suppresses_navigation_copy():
    node=bridge();node._cloud(cloud(20));node._cloud(cloud(19))
    assert len(node._raw_cloud_pub.messages)==2
    assert len(node._cloud_pub.messages)==1
