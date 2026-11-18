from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

PROBE = Path(__file__).resolve().parents[1]/'compat/foxy_nav2/probes/velocity_chain.py'
spec = importlib.util.spec_from_file_location('foxy_velocity_probe', PROBE)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def environment(monkeypatch, tmp_path, *, address='127.0.0.1', multicast='false', peer='127.0.0.1'):
    path = tmp_path/'dds.xml'
    path.write_text(f'<CycloneDDS><Domain><General><NetworkInterfaceAddress>{address}</NetworkInterfaceAddress>'
                    f'<AllowMulticast>{multicast}</AllowMulticast></General>'
                    f'<Discovery><Peers><Peer Address="{peer}"/></Peers></Discovery></Domain></CycloneDDS>')
    for key, value in {'ROS_DOMAIN_ID':'99', 'ROS_LOCALHOST_ONLY':'1',
                       'CYCLONEDDS_URI':'file://'+str(path),
                       'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp'}.items():
        monkeypatch.setenv(key, value)


def test_explicit_loopback_domain_is_accepted_without_importing_ros(monkeypatch,tmp_path):
    environment(monkeypatch,tmp_path)
    probe.isolated_environment()


@pytest.mark.parametrize('key,value', [
    ('ROS_DOMAIN_ID','0'), ('ROS_LOCALHOST_ONLY','0'),
    ('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'), ('CYCLONEDDS_URI','')])
def test_production_or_unchecked_transport_is_refused(monkeypatch,tmp_path,key,value):
    environment(monkeypatch,tmp_path)
    monkeypatch.setenv(key,value)
    with pytest.raises(RuntimeError): probe.isolated_environment()


@pytest.mark.parametrize('changes', [
    {'address':'eth1'}, {'address':'192.168.123.18'},
    {'multicast':'true'}, {'peer':'192.168.123.161'}])
def test_non_loopback_dds_is_refused(monkeypatch,tmp_path,changes):
    environment(monkeypatch,tmp_path,**changes)
    with pytest.raises(RuntimeError): probe.isolated_environment()
