"""Sensor relay's stream parsing and physical command separation."""
import ast
from pathlib import Path
import socket,struct
import sys,types,xml.etree.ElementTree as ET

import pytest

from compat.foxy_nav2.probes import live_inputs as relay


class Fragments:
    def __init__(self,parts):self.parts=list(parts)
    def recv(self,size):
        if not self.parts:return b''
        part=self.parts.pop(0)
        if part is None:raise socket.timeout()
        taken=part[:size]
        if len(part)>size:self.parts.insert(0,part[size:])
        return taken


def test_packet_preserves_source_time_and_fragmented_payload_across_timeout():
    data=relay.packet(4,b'unchanged CDR sensor data',123456789)
    stream=Fragments([data[:3],None,data[3:relay.HEADER.size+4],None,data[relay.HEADER.size+4:]])
    assert relay.read_packet(stream)==(4,b'unchanged CDR sensor data',123456789)


@pytest.mark.parametrize('index,size',[(65535,1),(0,0),(0,relay.MAX_PAYLOAD+1)])
def test_unlisted_topic_or_unbounded_payload_is_rejected_before_read(index,size):
    with pytest.raises(ValueError):relay.read_packet(Fragments([relay.HEADER.pack(index,size,1)]))


def test_truncated_sensor_stream_is_not_a_valid_message():
    with pytest.raises(EOFError):relay.read_packet(Fragments([relay.packet(0,b'x',1)[:-1]]))


def test_relay_source_never_creates_a_publisher_or_command_client():
    path=Path(relay.__file__);tree=ast.parse(path.read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='source')
    calls={n.func.attr for n in ast.walk(fn) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
    assert 'create_subscription' in calls
    assert not calls.intersection({'create_publisher','create_client','create_service','send_goal_async','call_async'})
    assert not any(token in topic for topic,*_ in relay.TOPICS for token in ['cmd_vel','motion','software_arm','sport_lease','emergency_stop'])
    assert len({topic for topic,*_ in relay.TOPICS})==len(relay.TOPICS)


def test_sink_checks_isolation_before_publishing_and_does_not_change_headers():
    source=Path(relay.__file__).read_text();tree=ast.parse(source)
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='sink')
    segment=ast.get_source_segment(source,fn)
    assert segment.index('isolated_environment()')<segment.index('create_publisher')
    assert 'deserialize_message(raw,classes[index])' in segment
    assert 'header.stamp' not in segment


def test_sink_offers_reliable_wheel_odometry_and_sensor_qos_for_scans(monkeypatch):
    fake=types.ModuleType('rclpy.qos');sensor=object()
    fake.QoSProfile=lambda **kwargs:types.SimpleNamespace(**kwargs)
    fake.DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL='transient',VOLATILE='volatile')
    fake.ReliabilityPolicy=types.SimpleNamespace(RELIABLE='reliable')
    fake.qos_profile_sensor_data=sensor
    monkeypatch.setitem(sys.modules,'rclpy',types.ModuleType('rclpy'))
    monkeypatch.setitem(sys.modules,'rclpy.qos',fake)
    kind=next(kind for topic,_,_,kind in relay.TOPICS if topic=='/go2w/odom/fused')
    assert relay.qos(kind).reliability=='reliable'
    assert relay.qos('sensor') is sensor


def test_loopback_discovery_has_capacity_for_full_chain_and_cleanup():
    root=ET.fromstring(relay.loopback_configuration())
    assert [n.text for n in root.iter('MaxAutoParticipantIndex')]==['32']
    assert [n.text for n in root.iter('NetworkInterfaceAddress')]==['127.0.0.1']
    assert [n.text for n in root.iter('AllowMulticast')]==['false']
    assert [n.get('Address') for n in root.iter('Peer')]==['127.0.0.1']
