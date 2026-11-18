"""Real local DDS process recovery; no robot, motion or Nav2 action servers."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest

pytest.importorskip('rclpy', reason='requires sourced ROS Python')
ROOT = Path(__file__).resolve().parents[1]


def test_future_poison_requires_rebuilding_both_monitor_and_pose_reader(tmp_path):
    out = tmp_path / 'dds_rebuild'
    fixture = ROOT / 'tests/fixtures/tf_consumer_rebuild_ros.py'
    env = dict(os.environ, ROS_DOMAIN_ID='98', ROS_LOCALHOST_ONLY='1',
        RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', GO2W_PROJECT_ROOT=str(ROOT))
    env.pop('CYCLONEDDS_URI', None)
    with subprocess.Popen([sys.executable, str(fixture), 'run', str(out)],
            env=env, cwd=ROOT, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE) as child:
        try:
            stdout, stderr = child.communicate(timeout=50)
        except subprocess.TimeoutExpired:
            # Let the supervisor's finally close its known children rather
            # than killing it and abandoning independent ROS process groups.
            child.send_signal(signal.SIGINT)
            stdout, stderr = child.communicate(timeout=30)
            pytest.fail('fixture timed out; supervisor cleanup requested: ' + stderr)
    (tmp_path / 'fixture.log').write_text(stdout + stderr)
    assert child.returncode == 0, (stdout, stderr, str(out))
    report = json.loads((out / 'report.json').read_text())
    assert report['passed'] and report['children_terminal']
    assert report['isolated_monitor_restart_incomplete_recovery_reproduced']
    assert report['two_consumer_rebuild_correct_before_poison_expiry']
    assert report['rebuild_with_bad_source_stays_closed']
    assert all(code == 0 for code in report['child_exit_codes'].values())
    assert all(not row['errors'] and row['action_sends'] == row['twist_messages'] == 0
        for row in report['cleanup'].values())
