"""The robot profile must tolerate NIC renumbering and reject ambiguity."""
import os
from pathlib import Path
import subprocess

import pytest

PROFILE = Path(__file__).resolve().parents[1] / 'configs/go2w/jetson_foxy.env'


@pytest.mark.parametrize('name', ['eth0', 'eth1', 'enP8p1s0'])
def test_profile_selects_owner_of_internal_ip(tmp_path, name):
    ip = tmp_path / 'ip'
    ip.write_text('#!/bin/sh\ncat <<\'DATA\'\n'
                  '4: wlan0 inet 192.168.3.62/24 scope global wlan0\n'
                  f'6: {name} inet 192.168.123.18/24 scope global {name}\nDATA\n')
    ip.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp_path)+':'+os.environ['PATH'])
    result = subprocess.run(['bash', '-ec', 'source "$1"; printf "%s" "$GO2W_INTERFACE"',
                             '_', str(PROFILE)], env=env, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout == name


@pytest.mark.parametrize('addresses', [
    '4: wlan0 inet 192.168.3.62/24 scope global wlan0',
    '1: lo inet 192.168.123.18/24 scope host lo',
    '5: eth0 inet 192.168.123.18/24 scope global eth0\n'
    '6: eth1 inet 192.168.123.18/24 scope global eth1',
])
def test_profile_refuses_absent_or_duplicate_internal_ip(tmp_path, addresses):
    ip = tmp_path / 'ip'
    ip.write_text("#!/bin/sh\ncat <<'DATA'\n" + addresses + '\nDATA\n')
    ip.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp_path)+':'+os.environ['PATH'])
    result = subprocess.run(['bash', '-ec', 'source "$1"; echo SHOULD_NOT_START',
                             '_', str(PROFILE)], env=env, text=True, capture_output=True)
    assert result.returncode != 0
    assert 'SHOULD_NOT_START' not in result.stdout
    assert 'exactly one interface' in result.stderr
