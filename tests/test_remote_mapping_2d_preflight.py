"""Sensor preflight must not leave an unowned map writer behind."""
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "scripts/go2w/start_remote_control_mapping.sh").read_text()
PREFLIGHT = SOURCE.split("# A2. 地图模式对应", 1)[1].split("# A3. IMU", 1)[0]
PREFLIGHT = PREFLIGHT[PREFLIGHT.index("if [["):]


@pytest.mark.parametrize("smoke", ["0", "1"])
def test_existing_2d_sensors_do_not_start_any_map_writer(tmp_path, smoke):
    script = """
set -euo pipefail
map_mode=2d
topic_alive() { return 0; }
topic_publisher_count() { echo 1; }
topic_receiving() { return 0; }
ok() { :; }
die() { echo "$*" >&2; exit 2; }
setsid() { echo unexpected-process >>"$SPAWNS"; }
""" + PREFLIGHT
    env = dict(os.environ, smoke=smoke, GO2W_WEB_POINTCLOUD_BRIDGE_ENABLED="0",
               SPAWNS=str(tmp_path / "spawns"))
    result = subprocess.run(["bash", "-c", script], env=env, capture_output=True,
                            text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert not (tmp_path / "spawns").exists()


def test_missing_2d_preprocessor_cannot_reenable_cloud_outputs(tmp_path):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    script = """
set -euo pipefail
map_mode=2d; smoke=0; project_root=/project; script_dir=/scripts
runtime_dir="$RUNTIME"; log_root="$RUNTIME"
topic_alive() { return 1; }
topic_publisher_count() { echo 1; }
topic_receiving() { return 0; }
wait_topic_alive() { return 0; }
ok() { :; }
die() { echo "$*" >&2; exit 2; }
setsid() { printf '%s\n' "$@" >"$SPAWNS"; }
""" + PREFLIGHT + "\nwait\n"
    env = dict(os.environ, RUNTIME=str(runtime), SPAWNS=str(tmp_path / "spawns"),
               GO2W_PERCEPTION_MODE="2d", GO2W_WEB_POINTCLOUD_BRIDGE_ENABLED="0")
    result = subprocess.run(["bash", "-c", script], env=env, capture_output=True,
                            text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    arguments = (tmp_path / "spawns").read_text().splitlines()
    assert "publish_pointclouds:=false" in arguments
    assert "publish_floor_visibility_2d:=true" in arguments
    assert not any("map_epoch_meta_bridge" in arg for arg in arguments)


@pytest.mark.parametrize("host,probe", [("0.0.0.0", "127.0.0.1"),
                                        ("::", "[::1]"),
                                        ("192.168.3.62", "192.168.3.62")])
def test_wildcard_bind_uses_concrete_readiness_address(host, probe):
    block = SOURCE.split('case "${web_host}" in', 1)[1].split("esac", 1)[0]
    script = 'case "${web_host}" in' + block + 'esac\nprintf "%s" "$web_probe_host"'
    result = subprocess.run(["bash", "-c", script], env=dict(os.environ, web_host=host),
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout == probe
