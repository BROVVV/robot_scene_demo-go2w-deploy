"""Stop only temporary fixture processes; never run real project supervisors."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]


def wait_file(path, child=None):
    deadline = time.monotonic() + 4
    while not path.exists():
        if child is not None:
            assert child.poll() is None
        assert time.monotonic() < deadline, str(path)
        time.sleep(.01)


def project(tmp_path):
    root = tmp_path / 'project'
    scripts = root / 'scripts/go2w'
    scripts.mkdir(parents=True)
    shutil.copy2(ROOT / 'scripts/go2w/stop_all.sh', scripts / 'stop_all.sh')
    helper = ROOT / 'scripts/go2w/stop_owned_host_processes.py'
    if helper.exists():
        shutil.copy2(helper, scripts / helper.name)
    for name in ('stop_nav2_autonomous_search.sh', 'stop_2d_localization.sh'):
        (scripts / name).write_text('#!/bin/bash\nprintf "%s\\n" ' + name
            + ' >>"' + str(root / 'trace') + '"\n')
    (root / 'runtime/go2w/pids').mkdir(parents=True)
    worker = scripts / 'fixture_worker.py'
    worker.write_text('import os,signal,subprocess,sys,time\nfrom pathlib import Path\n'
        'ready=Path(sys.argv[1])\n'
        'if "--ignore" in sys.argv: signal.signal(signal.SIGTERM,signal.SIG_IGN)\n'
        'if "--orphan" in sys.argv:\n'
        ' subprocess.Popen([sys.executable,__file__,str(ready),"--child"])\n'
        'else:\n'
        ' ready.write_text(str(os.getpid()))\n'
        ' time.sleep(60)\n')
    return root, worker


def record(root, pid, name='fixture'):
    path = root / 'runtime/go2w/pids' / (name + '.pid')
    path.write_text(str(pid) + '\n')
    return path


def run_stop(root):
    return subprocess.run(['/bin/bash', str(root / 'scripts/go2w/stop_all.sh')],
        cwd=root, text=True, capture_output=True, timeout=12)


def end(child):
    if child.poll() is None:
        child.kill()
    child.wait(timeout=3)


def run_helper(root, timeout=.4):
    return subprocess.run([sys.executable, str(root / 'scripts/go2w/stop_owned_host_processes.py'),
        str(root), '--timeout', str(timeout)], cwd=root, capture_output=True, text=True, timeout=5)


def load_helper():
    path = ROOT / 'scripts/go2w/stop_owned_host_processes.py'
    spec = importlib.util.spec_from_file_location('host_stop_test_helper', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_foreign_pid_is_refused_and_record_is_retained(tmp_path):
    root, _ = project(tmp_path)
    child = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'],
        start_new_session=True)
    pidfile = record(root, child.pid)
    try:
        result = run_stop(root)
        assert result.returncode != 0, result.stdout + result.stderr
        assert child.poll() is None
        assert pidfile.exists()
        assert (root / 'trace').read_text().splitlines() == [
            'stop_nav2_autonomous_search.sh', 'stop_2d_localization.sh']
    finally:
        end(child)


def test_orphan_owned_group_is_stopped_even_after_leader_exits(tmp_path):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    parent = subprocess.Popen([sys.executable, str(worker), str(ready), '--orphan'],
        start_new_session=True)
    pidfile = record(root, parent.pid)
    try:
        parent.wait(timeout=3)
        wait_file(ready)
        result = run_stop(root)
        assert result.returncode == 0, result.stdout + result.stderr
        child_pid = int(ready.read_text())
        stat = Path('/proc') / str(child_pid) / 'stat'
        assert not stat.exists() or stat.read_text().rsplit(')', 1)[1].split()[0] == 'Z'
        assert not pidfile.exists()
    finally:
        try:
            os.killpg(parent.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def test_alive_term_ignoring_process_keeps_record_and_reports_failure(tmp_path):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, str(worker), str(ready), '--ignore'],
        start_new_session=True)
    pidfile = record(root, child.pid)
    try:
        wait_file(ready, child)
        result = run_stop(root)
        assert result.returncode != 0, result.stdout + result.stderr
        assert child.poll() is None and pidfile.exists()
    finally:
        end(child)


@pytest.mark.parametrize('content', ['1', '0', '-42', 'abc42', '3\n4', '2147483648'])
def test_invalid_pid_record_is_retained_without_signal(tmp_path, content):
    root, _ = project(tmp_path)
    path = record(root, 42)
    path.write_text(content)
    result = run_helper(root)
    assert result.returncode == 1
    assert path.read_text() == content
    assert 'invalid pid' in result.stdout


def test_rooted_program_is_stopped_with_verified_exit(tmp_path):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, str(worker), str(ready)], start_new_session=True)
    path = record(root, child.pid)
    try:
        wait_file(ready, child)
        result = run_helper(root)
        assert result.returncode == 0, result.stdout + result.stderr
        assert child.wait(timeout=2) == -signal.SIGTERM
        assert not path.exists()
        assert json.loads(result.stdout)['exited'][0]['exited'] is True
    finally:
        end(child)


def test_nonleader_stop_does_not_signal_its_shared_group(tmp_path):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, str(worker), str(ready)])
    sibling = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'])
    path = record(root, child.pid)
    try:
        wait_file(ready, child)
        assert os.getpgid(child.pid) == os.getpgid(os.getpid())
        result = run_helper(root)
        assert result.returncode == 0, result.stdout + result.stderr
        assert child.wait(timeout=2) == -signal.SIGTERM
        assert sibling.poll() is None and not path.exists()
    finally:
        end(child)
        end(sibling)


def test_mixed_group_is_refused_before_any_member_is_signaled(tmp_path):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, str(worker), str(ready)], preexec_fn=os.setpgrp)
    sibling = None
    try:
        wait_file(ready, child)
        sibling = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'],
            preexec_fn=lambda: os.setpgid(0, child.pid))
        path = record(root, child.pid)
        result = run_helper(root)
        assert result.returncode == 1, result.stdout + result.stderr
        assert child.poll() is None and sibling.poll() is None and path.exists()
        assert 'refusing foreign member' in result.stdout
    finally:
        end(child)
        if sibling is not None:
            end(sibling)


def test_root_path_in_python_code_arguments_does_not_prove_ownership(tmp_path):
    root, worker = project(tmp_path)
    child = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)', str(worker)],
        start_new_session=True)
    path = record(root, child.pid)
    try:
        result = run_helper(root)
        assert result.returncode == 1
        assert child.poll() is None and path.exists()
    finally:
        end(child)


def test_other_project_program_is_not_owned(tmp_path):
    root, _ = project(tmp_path / 'current')
    _, worker = project(tmp_path / 'other')
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, str(worker), str(ready)], start_new_session=True)
    path = record(root, child.pid)
    try:
        wait_file(ready, child)
        result = run_helper(root)
        assert result.returncode == 1
        assert child.poll() is None and path.exists()
    finally:
        end(child)


def test_symlink_pidfile_is_refused(tmp_path):
    root, _ = project(tmp_path)
    outside = tmp_path / 'outside'
    outside.write_text('42\n')
    path = root / 'runtime/go2w/pids/fixture.pid'
    path.symlink_to(outside)
    result = run_helper(root)
    assert result.returncode == 1 and path.is_symlink()
    assert outside.read_text() == '42\n'


def test_replaced_pid_record_is_preserved_and_replacement_process_untouched(tmp_path):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    foreign = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'], start_new_session=True)
    path = root / 'runtime/go2w/pids/fixture.pid'
    worker.write_text('import signal,sys,time\nfrom pathlib import Path\n'
        'def stop(*args):\n Path(sys.argv[2]).write_text(sys.argv[3]);sys.exit(0)\n'
        'signal.signal(signal.SIGTERM,stop)\nPath(sys.argv[1]).touch()\ntime.sleep(60)\n')
    child = subprocess.Popen([sys.executable, str(worker), str(ready), str(path), str(foreign.pid)],
        start_new_session=True)
    record(root, child.pid)
    try:
        wait_file(ready, child)
        result = run_helper(root)
        assert result.returncode == 1, result.stdout + result.stderr
        assert child.wait(timeout=2) == 0 and foreign.poll() is None
        assert path.read_text() == str(foreign.pid)
        assert 'pidfile changed during stop' in result.stdout
    finally:
        end(child)
        end(foreign)


def test_owned_cleanup_removing_later_records_is_not_an_error(tmp_path):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    folder = root / 'runtime/go2w/pids'
    worker.write_text('import signal,sys,time\nfrom pathlib import Path\n'
        'def stop(*args):\n'
        ' for path in Path(sys.argv[2]).glob("*.pid"):path.unlink()\n'
        ' sys.exit(0)\n'
        'signal.signal(signal.SIGTERM,stop)\nPath(sys.argv[1]).touch()\ntime.sleep(60)\n')
    child = subprocess.Popen([sys.executable, str(worker), str(ready), str(folder)], start_new_session=True)
    record(root, child.pid, 'a')
    record(root, child.pid, 'b')
    try:
        wait_file(ready, child)
        result = run_helper(root)
        assert result.returncode == 0, result.stdout + result.stderr
        assert child.wait(timeout=2) == 0 and not list(folder.glob('*.pid'))
    finally:
        end(child)


def test_missing_host_helper_keeps_records_but_attempts_other_layers(tmp_path):
    root, _ = project(tmp_path)
    (root / 'scripts/go2w/stop_owned_host_processes.py').unlink()
    path = record(root, 42)
    result = run_stop(root)
    assert result.returncode == 1 and path.exists()
    assert (root / 'trace').read_text().splitlines() == [
        'stop_nav2_autonomous_search.sh', 'stop_2d_localization.sh']


def test_python38_identity_checked_fallback_without_pidfds(tmp_path, monkeypatch):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, str(worker), str(ready)], start_new_session=True)
    path = record(root, child.pid)
    module = load_helper()
    try:
        wait_file(ready, child)
        monkeypatch.setattr(module.os, 'pidfd_open', None, raising=False)
        monkeypatch.setattr(module.signal, 'pidfd_send_signal', None, raising=False)
        assert module.stop_record(path, root, timeout=.5)['exited']
        assert child.wait(timeout=2) == -signal.SIGTERM
    finally:
        end(child)


def test_identity_change_before_signal_is_refused(tmp_path, monkeypatch):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, str(worker), str(ready)], start_new_session=True)
    module = load_helper()
    try:
        wait_file(ready, child)
        original = module.table()[child.pid]
        monkeypatch.setattr(module, 'table', lambda: {
            child.pid: dict(original, start=original['start'] + 1)})
        with pytest.raises(RuntimeError, match='identity changed'):
            module.signal_one(original)
        assert child.poll() is None
    finally:
        end(child)


@pytest.mark.parametrize('selected', [True, False])
@pytest.mark.parametrize('binding', ['workspace', 'native_base'])
def test_explicit_install_program_ownership(tmp_path, monkeypatch, selected, binding):
    root, original_worker = project(tmp_path)
    install = tmp_path / 'native_install'
    worker = install / 'go2w_fixture/lib/go2w_fixture/worker.py'
    worker.parent.mkdir(parents=True)
    shutil.copy2(original_worker, worker)
    ready = tmp_path / 'ready'
    child = subprocess.Popen([sys.executable, str(worker), str(ready)], start_new_session=True)
    path = record(root, child.pid)
    prefix = install if selected else tmp_path / 'other_install'
    if binding == 'workspace':
        monkeypatch.setenv('GO2W_WORKSPACE_SETUP', str(prefix / 'setup.bash'))
        monkeypatch.delenv('GO2W_HOST_STOP_ADDITIONAL_INSTALLS', raising=False)
    else:
        monkeypatch.delenv('GO2W_WORKSPACE_SETUP', raising=False)
        monkeypatch.setenv('GO2W_HOST_STOP_ADDITIONAL_INSTALLS', str(prefix))
    try:
        wait_file(ready, child)
        result = run_helper(root)
        if selected:
            assert result.returncode == 0, result.stdout + result.stderr
            assert child.wait(timeout=2) == -signal.SIGTERM and not path.exists()
        else:
            assert result.returncode == 1 and child.poll() is None and path.exists()
    finally:
        end(child)


@pytest.mark.parametrize('value', ['relative/install', '/'])
def test_additional_install_trust_rejects_broad_or_relative_prefix(tmp_path, monkeypatch, value):
    root, _ = project(tmp_path)
    monkeypatch.setenv('GO2W_HOST_STOP_ADDITIONAL_INSTALLS', value)
    with pytest.raises(ValueError, match='absolute, specific'):
        load_helper().trusted_installs(root)


@pytest.mark.parametrize('correct_program', [True, False])
def test_search_web_stop_owns_only_its_http_capture(tmp_path, correct_program):
    root, worker = project(tmp_path)
    scripts = root / 'scripts/go2w'
    shutil.copy2(ROOT / 'scripts/go2w/stop_autonomous_search_web.sh', scripts)
    capture = scripts / 'robot_camera_http_capture.py'
    shutil.copy2(worker, capture)
    search_runtime = root / 'outputs/autonomous_search/runtime'
    manual_runtime = root / 'outputs/manual_web_demo/runtime'
    search_runtime.mkdir(parents=True); manual_runtime.mkdir(parents=True)
    search_ready, manual_ready = tmp_path / 'search_ready', tmp_path / 'manual_ready'
    search = subprocess.Popen([sys.executable, str(capture if correct_program else worker),
        str(search_ready)], start_new_session=True)
    manual = subprocess.Popen([sys.executable, str(capture), str(manual_ready)], start_new_session=True)
    search_record = search_runtime / 'camera_http_capture.pid'
    search_record.write_text(str(search.pid)+'\n')
    manual_record = manual_runtime / 'camera_http_capture.pid'
    manual_record.write_text(str(manual.pid)+'\n')
    try:
        wait_file(search_ready, search); wait_file(manual_ready, manual)
        result = subprocess.run(['/bin/bash', str(scripts / 'stop_autonomous_search_web.sh')],
            cwd=root, capture_output=True, text=True, timeout=12)
        assert manual.poll() is None and manual_record.read_text().strip() == str(manual.pid)
        if correct_program:
            assert result.returncode == 0, result.stdout+result.stderr
            assert search.wait(timeout=2) == -signal.SIGTERM and not search_record.exists()
        else:
            assert result.returncode != 0 and search.poll() is None and search_record.exists()
    finally:
        end(search); end(manual)


@pytest.mark.parametrize('selection', ['trusted', 'foreign_first', 'none', 'nonproject_package'])
def test_ros_wrapper_requires_project_package_from_trusted_resource_index(tmp_path, monkeypatch, selection):
    root, _ = project(tmp_path)
    module = load_helper()
    overlay, foreign = tmp_path / 'install', tmp_path / 'foreign_install'
    package = 'nav2_amcl' if selection == 'nonproject_package' else 'go2w_fixture'
    for prefix in (overlay, foreign):
        marker = prefix / 'share/ament_index/resource_index/packages' / package
        marker.parent.mkdir(parents=True)
        if selection != 'none':
            marker.touch()
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(overlay) if selection != 'foreign_first'
        else str(foreign) + os.pathsep + str(overlay))
    proc = Path('/proc/987654321')
    original_read = Path.read_bytes
    original_link = os.readlink
    monkeypatch.setattr(Path, 'read_bytes', lambda path: (
        b'/usr/bin/python3\0/opt/ros/humble/bin/ros2\0run\0' + package.encode() + b'\0worker\0'
        if path == proc / 'cmdline' else original_read(path)))
    def readlink(path, *args, **kwargs):
        if str(path) == str(proc / 'exe'):
            return '/usr/bin/python3.10'
        if str(path) == str(proc / 'cwd'):
            return str(root)
        return original_link(path, *args, **kwargs)
    monkeypatch.setattr(os, 'readlink', readlink)
    assert module.owns(dict(pid=987654321, uid=os.geteuid()), root, [overlay]) is (
        selection == 'trusted')


def test_owned_leader_descendants_are_stopped_without_rooted_child_argv(tmp_path):
    root, worker = project(tmp_path)
    ready = tmp_path / 'ready'
    worker.write_text('import subprocess,sys,time\nfrom pathlib import Path\n'
        'child=subprocess.Popen([sys.executable,"-c","import time;time.sleep(60)"])\n'
        'Path(sys.argv[1]).write_text(str(child.pid))\ntime.sleep(60)\n')
    parent = subprocess.Popen([sys.executable, str(worker), str(ready)], start_new_session=True)
    path = record(root, parent.pid)
    try:
        wait_file(ready, parent)
        result = run_helper(root)
        assert result.returncode == 0, result.stdout + result.stderr
        assert parent.wait(timeout=2) == -signal.SIGTERM and not path.exists()
        child_stat = Path('/proc') / ready.read_text() / 'stat'
        assert not child_stat.exists() or child_stat.read_text().rsplit(')', 1)[1].split()[0] == 'Z'
    finally:
        end(parent)
        try:
            os.killpg(parent.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def test_exited_pid_record_is_removed_without_signaling(tmp_path):
    root, _ = project(tmp_path)
    child = subprocess.Popen(['/usr/bin/true'], start_new_session=True)
    child.wait(timeout=2)
    path = record(root, child.pid)
    result = run_helper(root)
    assert result.returncode == 0 and not path.exists()
    assert json.loads(result.stdout)['exited'][0]['signaled_processes'] == 0


def test_caller_supervisor_cannot_be_stopped_by_pidfile(tmp_path):
    root, _ = project(tmp_path)
    path = record(root, os.getpid())
    result = run_helper(root)
    assert result.returncode == 1 and path.exists()
    assert 'supervisor' in result.stdout
