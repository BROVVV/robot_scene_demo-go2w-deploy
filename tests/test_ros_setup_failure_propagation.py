"""Use temporary setup hooks; never source a real ROS or robot environment."""
from pathlib import Path
import shutil
import subprocess

import pytest


REPO = Path(__file__).resolve().parents[1]
OVERLAYS = (
    'GO2W_UNITREE_SETUP', 'GO2W_WORKSPACE_SETUP', 'GO2W_CONTROL_SETUP',
    'GO2W_NAV2_SAFETY_SETUP', 'GO2W_NAV2_RPP_SETUP', 'GO2W_NAV2_SPIN_SETUP',
    'GO2W_NAV2_BT_SETUP', 'GO2W_NAV2_CONTROLLER_SETUP', 'GO2W_LIDAR_HOTPATH_SETUP',
)


def fixture_environment(tmp_path):
    root = tmp_path / 'project'
    script = root / 'scripts/go2w/setup_environment.sh'
    script.parent.mkdir(parents=True)
    shutil.copy2(REPO / 'scripts/go2w/setup_environment.sh', script)
    config = root / 'configs/go2w/cyclonedds_go2w.xml'
    config.parent.mkdir(parents=True)
    config.write_text('<NetworkInterface name="fixture0" />\n')
    home = tmp_path / 'home'
    home.mkdir()
    env = dict(HOME=str(home), PATH='/usr/bin:/bin', LANG='C.UTF-8',
        GO2W_INTERFACE='fixture0', GO2W_HOST_IP='192.168.123.99',
        GO2W_CONTROL_ROOT=str(tmp_path / 'control'),
        SETUP_FIXTURE_TRACE=str(tmp_path / 'trace'))
    base = tmp_path / 'foxy/setup.bash'
    base.parent.mkdir()
    base.write_text('printf "base\\n" >>"$SETUP_FIXTURE_TRACE"\nexport ROS_DISTRO=foxy\n')
    env['GO2W_ROS_SETUP'] = str(base)
    for index, variable in enumerate(OVERLAYS):
        hook = tmp_path / ('overlay%d' % index) / 'setup.bash'
        hook.parent.mkdir()
        hook.write_text('printf "' + variable + '\\n" >>"$SETUP_FIXTURE_TRACE"\n')
        env[variable] = str(hook)
    return root, script, env


def run_setup(script, env, caller='conditional'):
    # Even an errexit caller suppresses it within `if source ...`. The sourced
    # entry must explicitly propagate errors, as diagnostic callers require.
    commands = {
        'conditional': 'if source "$1"; then result=0; else result=$?; fi; ',
        'plain': 'set +e; source "$1"; result=$?; ',
        'errexit': 'set -e; source "$1"; result=$?; ',
    }
    result = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c',
        commands[caller] + 'printf "source_result=%s\\n" "$result"; exit "$result"',
        'fixture', str(script)], env=env, text=True, capture_output=True, timeout=5)
    trace = Path(env['SETUP_FIXTURE_TRACE'])
    return result, trace.read_text().splitlines() if trace.exists() else []


@pytest.mark.parametrize('failed', ('GO2W_ROS_SETUP',) + OVERLAYS)
def test_setup_failure_is_returned_and_later_hooks_are_not_loaded(tmp_path, failed):
    root, script, env = fixture_environment(tmp_path)
    hook = Path(env[failed])
    with hook.open('a') as stream:
        stream.write('return 17\n')
    result, trace = run_setup(script, env)
    assert result.returncode == 17, result.stdout + result.stderr
    assert 'environment ready' not in result.stdout
    assert 'ERROR:' in result.stderr
    expected = ['base']
    if failed in OVERLAYS:
        expected += list(OVERLAYS[:OVERLAYS.index(failed) + 1])
    assert trace == expected
    assert not (root / 'runtime/go2w/cyclonedds_fixture0.xml').exists()


@pytest.mark.parametrize('failed', (False, True))
def test_incompatible_recorded_underlay_selects_local_hook_and_propagates_it(tmp_path, failed):
    root, script, env = fixture_environment(tmp_path)
    hook = Path(env['GO2W_UNITREE_SETUP'])
    hook.write_text('COLCON_CURRENT_PREFIX="/opt/ros/noetic"\n'
        'printf "WRONG_RECORDED_SETUP\\n" >>"$SETUP_FIXTURE_TRACE"\n')
    local_hook = hook.with_name('local_setup.bash')
    local_hook.write_text('printf "local_vendor\\n" >>"$SETUP_FIXTURE_TRACE"\n'
        + ('return 23\n' if failed else 'return 0\n'))
    result, trace = run_setup(script, env)
    assert result.returncode == (23 if failed else 0), result.stdout + result.stderr
    assert 'WRONG_RECORDED_SETUP' not in trace
    assert trace == ['base', 'local_vendor'] + ([] if failed else list(OVERLAYS[1:]))
    assert ('environment ready' in result.stdout) is (not failed)
    assert (root / 'runtime/go2w/cyclonedds_fixture0.xml').exists() is (not failed)


def test_success_sources_all_selected_hooks_in_original_order(tmp_path):
    root, script, env = fixture_environment(tmp_path)
    result, trace = run_setup(script, env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert trace == ['base'] + list(OVERLAYS)
    assert 'environment ready' in result.stdout
    assert '192.168.123.99' in (root / 'runtime/go2w/cyclonedds_fixture0.xml').read_text()


def test_desktop_optional_missing_overlays_remain_optional(tmp_path):
    root, script, env = fixture_environment(tmp_path)
    for variable in OVERLAYS:
        del env[variable]
    result, trace = run_setup(script, env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert trace == ['base']
    assert (root / 'runtime/go2w/cyclonedds_fixture0.xml').exists()


@pytest.mark.parametrize('caller', ('plain', 'errexit'))
@pytest.mark.parametrize('failed', ('GO2W_ROS_SETUP', 'GO2W_LIDAR_HOTPATH_SETUP'))
def test_failure_propagates_with_errexit_enabled_or_disabled(tmp_path, caller, failed):
    root, script, env = fixture_environment(tmp_path)
    with Path(env[failed]).open('a') as stream:
        stream.write('return 31\n')
    result, _ = run_setup(script, env, caller)
    assert result.returncode == 31, result.stdout + result.stderr
    assert 'environment ready' not in result.stdout
    assert not (root / 'runtime/go2w/cyclonedds_fixture0.xml').exists()


@pytest.mark.parametrize('failed', (False, True))
def test_vendor_alias_uses_full_setup_and_propagates_failure(tmp_path, failed):
    root, script, env = fixture_environment(tmp_path)
    # Preserve the existing special case: vendor /humble can export Foxy.
    alias = tmp_path / 'humble/setup.bash'
    alias.parent.mkdir()
    shutil.copy2(env['GO2W_ROS_SETUP'], alias)
    env['GO2W_ROS_SETUP'] = str(alias)
    hook = Path(env['GO2W_UNITREE_SETUP'])
    hook.write_text('COLCON_CURRENT_PREFIX="/opt/ros/noetic"\n'
        'printf "recorded_vendor\\n" >>"$SETUP_FIXTURE_TRACE"\n'
        + ('return 29\n' if failed else 'return 0\n'))
    hook.with_name('local_setup.bash').write_text(
        'printf "WRONG_LOCAL_SETUP\\n" >>"$SETUP_FIXTURE_TRACE"\n')
    result, trace = run_setup(script, env)
    assert result.returncode == (29 if failed else 0), result.stdout + result.stderr
    assert trace == ['base', 'recorded_vendor'] + ([] if failed else list(OVERLAYS[1:]))
    assert ('environment ready' in result.stdout) is (not failed)
    assert (root / 'runtime/go2w/cyclonedds_fixture0.xml').exists() is (not failed)


@pytest.mark.parametrize('distro', ('foxy', 'humble'))
def test_explicit_native_overlay_missing_or_wrong_distro_remains_fatal(tmp_path, distro):
    root, script, env = fixture_environment(tmp_path)
    if distro == 'foxy':
        Path(env['GO2W_LIDAR_HOTPATH_SETUP']).unlink()
    else:
        Path(env['GO2W_ROS_SETUP']).write_text('export ROS_DISTRO=humble\n')
    result, _ = run_setup(script, env)
    assert result.returncode == 2
    assert 'required native Foxy compatibility overlay missing' in result.stderr
    assert 'environment ready' not in result.stdout
    assert not (root / 'runtime/go2w/cyclonedds_fixture0.xml').exists()
