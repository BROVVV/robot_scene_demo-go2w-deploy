from pathlib import Path
from types import SimpleNamespace
import platform
import subprocess

import pytest

from scripts.go2w.prepare_nav2_trial import live_robot_boot_id

LOCAL = '55d17895-29b9-4063-b683-880778e79ea4'
REMOTE = 'beb4ed75-5cee-4492-aaf3-bbd0cb3d8dcc'


def fixture(monkeypatch, *, machine='aarch64', addresses='', boot=LOCAL):
    monkeypatch.setattr(platform, 'machine', lambda: machine)
    original_is_file, original_read_text = Path.is_file, Path.read_text
    monkeypatch.setattr(Path, 'is_file', lambda path:
        True if str(path) == '/etc/nv_tegra_release' else original_is_file(path))
    monkeypatch.setattr(Path, 'read_text', lambda path, *a, **kw:
        boot if str(path) == '/proc/sys/kernel/random/boot_id'
        else original_read_text(path, *a, **kw))
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        assert kwargs['check'] and kwargs['capture_output']
        if command[0] == 'ip':
            assert command == ['ip', '-4', '-o', 'addr', 'show']
            return SimpleNamespace(stdout=addresses)
        assert command[0] == 'ssh'
        assert 'StrictHostKeyChecking=yes' in command
        assert 'BatchMode=yes' in command
        return SimpleNamespace(stdout=REMOTE)

    monkeypatch.setattr(subprocess, 'run', run)
    return calls


@pytest.mark.parametrize('name', ['eth0', 'eth1', 'enP8p1s0'])
def test_robot_uses_own_boot_id_after_nic_renumbering(monkeypatch, name):
    calls = fixture(monkeypatch, addresses=f'6: {name} inet 192.168.123.18/24 scope global {name}')
    assert live_robot_boot_id() == LOCAL
    assert [command[0] for command in calls] == ['ip']


@pytest.mark.parametrize('addresses', ['', '4: wlan0 inet 192.168.3.62/24 scope global wlan0',
                                       '1: lo inet 192.168.123.18/24 scope host lo'])
def test_a_jetson_without_local_robot_ip_still_queries_the_remote_robot(monkeypatch, addresses):
    calls = fixture(monkeypatch, addresses=addresses)
    assert live_robot_boot_id() == REMOTE
    assert calls[-1][0] == 'ssh'


def test_workstation_never_substitutes_its_own_kernel_boot_id(monkeypatch):
    calls = fixture(monkeypatch, machine='x86_64',
                    addresses='6: eth0 inet 192.168.123.18/24 scope global eth0')
    assert live_robot_boot_id() == REMOTE
    assert [command[0] for command in calls] == ['ssh']


def test_duplicate_local_robot_addresses_cannot_be_certified(monkeypatch):
    calls = fixture(monkeypatch, addresses='5: eth0 inet 192.168.123.18/24 scope global eth0\n'
                    '6: eth1 inet 192.168.123.18/24 scope global eth1')
    with pytest.raises(ValueError, match='multiple interfaces'):
        live_robot_boot_id()
    assert [command[0] for command in calls] == ['ip']


def test_invalid_local_boot_id_is_rejected(monkeypatch):
    fixture(monkeypatch, addresses='6: eth1 inet 192.168.123.18/24 scope global eth1', boot='broken')
    with pytest.raises(ValueError):
        live_robot_boot_id()
