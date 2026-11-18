import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts/go2w/one_click_remote_mapping.sh"
DISCARD = ROOT / "scripts/go2w/discard_remote_control_mapping_session.sh"
INSTALLER = ROOT / "scripts/go2w/install_remote_mapping_desktop_launcher.sh"


def test_launcher_owns_lifetime_and_starts_fresh_2d_yoloe_session():
    source = LAUNCHER.read_text(encoding="utf-8")

    assert "flock -n 9" in source
    assert "trap cleanup EXIT" in source
    assert "trap 'exit 0' HUP INT TERM" in source
    assert 'discard_remote_control_mapping_session.sh' in source
    assert 'yoloe_seg' in source
    assert 'bootstrap_webui.sh' in source
    assert '--mode mapping' in source
    assert 'GO2W_BOOTSTRAP_OWNER_MARKER' in source
    assert 'semantic_mapper.pid' in source
    assert 'web.pid' in source
    assert 'one_click_launcher.log' in source
    assert 'bootstrap_rc' in source
    assert 'cleanup' in source


def test_bootstrap_waits_for_replug_and_recovers_rgbd_service():
    source = (ROOT / "scripts/go2w/bootstrap_webui.sh").read_text(encoding="utf-8")
    reconnect = (ROOT / "scripts/go2w/wait_robot_reconnect.sh").read_text(
        encoding="utf-8"
    )
    realsense = (ROOT / "scripts/go2w/start_realsense_stream.sh").read_text(
        encoding="utf-8"
    )

    assert 'REMOTE_MAPPING_RECONNECT_TIMEOUT_SECONDS' in source
    assert 'wait_robot_reconnect.sh' in source
    assert 'endpoint_reachable' in reconnect
    assert '等待网线/网卡/机器狗恢复' in reconnect
    assert 'REMOTE_MAPPING_RGBD_RECOVERY_TIMEOUT_SECONDS' in source
    assert 'rgbd_action=restart' in source
    assert 'restart)' in realsense
    assert source.index(': >"${GO2W_BOOTSTRAP_OWNER_MARKER}"') < source.index(
        'setsid bash "${script_dir}/start_live_perception.sh"'
    )


def test_reconnect_waiter_survives_initial_probe_failure(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    marker = tmp_path / "ping_seen"
    commands = {
        "ip": "#!/bin/sh\necho '192.168.123.18 dev fake0 src 192.168.123.99'\n",
        "ping": (
            "#!/bin/sh\n"
            f"if [ -f '{marker}' ]; then exit 0; fi\n"
            f": >'{marker}'\n"
            "exit 1\n"
        ),
        "curl": "#!/bin/sh\nexit 1\n",
        "timeout": "#!/bin/sh\nexit 1\n",
    }
    for name, body in commands.items():
        path = fake_bin / name
        path.write_text(body, encoding="utf-8")
        path.chmod(0o755)
    env = dict(os.environ)
    env["PATH"] = f"{fake_bin}:/usr/bin:/bin"
    env["GO2W_RECONNECT_POLL_SECONDS"] = "0.01"

    result = subprocess.run(
        [
            "bash",
            str(ROOT / "scripts/go2w/wait_robot_reconnect.sh"),
            "--timeout",
            "2",
        ],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=5,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "[WAIT]" in result.stdout
    assert "[OK] robot reachable via 192.168.123.99" in result.stdout


def test_discard_is_scoped_to_active_mapping_files():
    source = DISCARD.read_text(encoding="utf-8")

    assert 'stop_remote_control_mapping.sh"' in source
    assert '--stop-slam-too' not in source
    assert 'stop_2d_localization.sh' in source
    assert 'slam_map_3d.json' in source
    assert 'semantic_objects.json' in source
    assert 'semantic_topology.json' in source
    assert 'target_watch_evidence' in source
    assert 'semantic_frame.jpg' in source
    assert 'unknown_object_vlm_evidence' in source
    assert 'live_perception_started_by_one_click' in source
    assert 'start_live_perception.sh' in source
    assert '/rosbags' not in source
    assert 'rm -rf' not in source
    assert 'outputs/remote_control_mapping"' not in source


def test_desktop_installer_points_to_foreground_owner():
    source = INSTALLER.read_text(encoding="utf-8")

    assert 'Type=Application' in source
    assert 'Terminal=true' in source
    assert 'one_click_remote_mapping.sh' in source
    assert 'metadata::trusted' in source
    assert 'GIO_EXTRA_MODULES' in source
    assert 'chmod 0755' in source
