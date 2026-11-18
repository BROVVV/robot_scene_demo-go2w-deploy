"""Exercise copied shutdown/reset scripts against fixture-only helpers/data."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest

REPO = Path(__file__).resolve().parents[1]


def copy_entry(root, name):
    target = root / 'scripts/go2w' / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(REPO / 'scripts/go2w' / name, target)
    if name == 'stop_all.sh':
        shutil.copy2(REPO / 'scripts/go2w/stop_owned_host_processes.py',
            target.with_name('stop_owned_host_processes.py'))
    return target


def helper(root, name, label, code=0, executable=True):
    target = root / 'scripts/go2w' / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text('#!/bin/bash\n'
        'printf "%s\\n" ' + label + ' >>"$RESET_FIXTURE_TRACE"\n'
        + ('printf "%s\\n" "$*" >"$RESET_FIXTURE_START_ARGS"\n' if label.startswith('start_') else '')
        + 'exit ' + str(code) + '\n')
    target.chmod(0o755 if executable else 0o644)


def run_entry(root, entry, args=()):
    result = subprocess.run(['bash', str(entry), *args], text=True, capture_output=True,
        timeout=12, cwd=root, env=dict(os.environ,
            RESET_FIXTURE_TRACE=str(root / 'trace.txt'),
            RESET_FIXTURE_START_ARGS=str(root / 'start_args.txt')))
    trace = (root / 'trace.txt').read_text().splitlines() if (root / 'trace.txt').exists() else []
    return result, trace


@pytest.mark.parametrize('problem', ['nav2_error', 'nav2_missing', 'localization_error'])
def test_full_stop_reports_incomplete_and_attempts_other_layer(tmp_path, problem):
    entry = copy_entry(tmp_path, 'stop_all.sh')
    if problem != 'nav2_missing':
        helper(tmp_path, 'stop_nav2_autonomous_search.sh', 'stop_nav2', 17 if problem == 'nav2_error' else 0)
    helper(tmp_path, 'stop_2d_localization.sh', 'stop_localization', 19 if problem == 'localization_error' else 0)
    result, trace = run_entry(tmp_path, entry)
    assert result.returncode != 0, result.stdout + result.stderr
    assert 'stop_localization' in trace
    if problem != 'nav2_missing':
        assert 'stop_nav2' in trace


def test_full_stop_runs_readable_helpers_without_executable_bit(tmp_path):
    entry = copy_entry(tmp_path, 'stop_all.sh')
    helper(tmp_path, 'stop_nav2_autonomous_search.sh', 'stop_nav2', executable=False)
    helper(tmp_path, 'stop_2d_localization.sh', 'stop_localization', executable=False)
    result, trace = run_entry(tmp_path, entry)
    assert result.returncode == 0, result.stdout + result.stderr
    assert trace == ['stop_nav2', 'stop_localization']


def search_fixture(root, *, failed='', action='map'):
    entry = copy_entry(root, 'reset_webui_runtime.sh')
    for name, label in [('stop_autonomous_search_web.sh', 'stop_web'),
                        ('stop_nav2_autonomous_search.sh', 'stop_nav2'),
                        ('stop_2d_localization.sh', 'stop_localization'),
                        ('start_nav2_autonomous_search.sh', 'start_nav2')]:
        helper(root, name, label, 23 if failed == label else 0)
    runtime = root / 'outputs/autonomous_search/runtime'
    runtime.mkdir(parents=True)
    contents = {}
    for index, name in enumerate(['slam_map_3d.json', 'slam_map_meta.json',
        'semantic_objects.json', 'semantic_topology.json', 'search_state.json']):
        contents[name] = ('{"fixture_only":true,"value":%d}\n' % index).encode()
        (runtime / name).write_bytes(contents[name])
    lock = root / 'runtime/go2w/webui_reset.lock'
    lock.parent.mkdir(parents=True)
    lock.write_text('fixture lock')
    args = ['--action', action, '--mode', 'autonomous_search', '--detector', 'yoloe_seg']
    return entry, args, runtime, contents, lock


@pytest.mark.parametrize('failed', ['stop_web', 'stop_nav2', 'stop_localization'])
def test_map_reset_stop_failure_preserves_data_and_never_restarts(tmp_path, failed):
    entry, args, runtime, contents, lock = search_fixture(tmp_path, failed=failed)
    result, trace = run_entry(tmp_path, entry, args)
    assert result.returncode != 0, result.stdout + result.stderr
    assert trace == ['stop_web', 'stop_nav2', 'stop_localization']
    assert not (tmp_path / 'start_args.txt').exists()
    for name, data in contents.items():
        assert (runtime / name).read_bytes() == data
    assert not lock.exists()


def test_successful_map_reset_archives_then_restarts_plan_only(tmp_path):
    entry, args, runtime, contents, lock = search_fixture(tmp_path)
    result, trace = run_entry(tmp_path, entry, args)
    assert result.returncode == 0, result.stdout + result.stderr
    assert trace == ['stop_web', 'stop_nav2', 'stop_localization', 'start_nav2']
    archive, = list((tmp_path / 'outputs/runtime_resets').iterdir())
    for name, data in contents.items():
        assert (archive / name).read_bytes() == data
        assert not (runtime / name).exists()
    start_args = (tmp_path / 'start_args.txt').read_text().split()
    assert start_args == ['--detector=yoloe_seg', '--no-browser']
    assert not lock.exists()


def test_object_reset_preserves_mapper_and_map_but_restarts_consumers(tmp_path):
    entry, args, runtime, contents, lock = search_fixture(tmp_path, action='objects')
    result, trace = run_entry(tmp_path, entry, args)
    assert result.returncode == 0, result.stdout + result.stderr
    assert trace == ['stop_web', 'stop_nav2', 'start_nav2']
    for name in ('slam_map_3d.json', 'slam_map_meta.json'):
        assert (runtime / name).read_bytes() == contents[name]
    for name in ('semantic_objects.json', 'semantic_topology.json', 'search_state.json'):
        assert not (runtime / name).exists()
    assert not lock.exists()


def test_teleop_object_failure_restores_ownership_markers(tmp_path):
    entry = copy_entry(tmp_path, 'reset_webui_runtime.sh')
    helper(tmp_path, 'stop_remote_control_mapping.sh', 'stop_mapping', 29)
    helper(tmp_path, 'start_remote_control_mapping.sh', 'start_mapping')
    runtime = tmp_path / 'outputs/remote_control_mapping/runtime'
    runtime.mkdir(parents=True)
    names = ['localization_2d_started_by_remote_mapping',
             'wheel_odom_started_by_remote_mapping', 'lidar_preprocessor_started_by_remote_mapping']
    for name in names:
        (runtime / name).write_text('owned fixture')
    (runtime / 'semantic_objects.json').write_text('{"fixture":true}')
    result, trace = run_entry(tmp_path, entry, ['--action', 'objects', '--mode',
        'teleop_mapping', '--detector', 'yoloe_seg'])
    assert result.returncode != 0
    assert trace == ['stop_mapping']
    assert not (tmp_path / 'start_args.txt').exists()
    for name in names:
        assert (runtime / name).read_text() == 'owned fixture'
        assert not (runtime / (name + '.reset_preserved')).exists()
    assert (runtime / 'semantic_objects.json').read_text() == '{"fixture":true}'
