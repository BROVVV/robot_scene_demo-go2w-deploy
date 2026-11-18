#!/usr/bin/python3
"""Bounded host-worker stop from legacy pidfiles; no ROS, SDK or robot API."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import stat
import time


def table():
    result = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                continue
            result[int(path.name)] = dict(pid=int(path.name), ppid=int(fields[1]),
                pgid=int(fields[2]), start=int(fields[19]), uid=path.stat().st_uid)
        except (OSError, ValueError, IndexError):
            continue
    return result


def inside(path, root):
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def descendant(pid, leader, processes):
    seen = set()
    while pid in processes and pid not in seen:
        if pid == leader:
            return True
        seen.add(pid)
        pid = processes[pid]['ppid']
    return False


def trusted_installs(root):
    roots = [root / 'ros2_ws/install', root / 'unitree_go2w_control/ros2_ws/install']
    for name in ('GO2W_WORKSPACE_SETUP', 'GO2W_LIDAR_HOTPATH_SETUP'):
        if os.environ.get(name):
            roots.append(Path(os.path.abspath(os.environ[name])).parent)
    # Partial native overlays retain packages from explicitly declared bases.
    # Do not treat every AMENT_PREFIX_PATH entry as a trusted installation.
    for value in os.environ.get('GO2W_HOST_STOP_ADDITIONAL_INSTALLS', '').split(os.pathsep):
        if value:
            path = Path(value)
            if not path.is_absolute() or path == Path('/'):
                raise ValueError('additional host-stop installs must be absolute, specific prefixes')
            roots.append(path)
    return roots


def owns(record, root, installs, expected_program=None):
    if record['uid'] != os.geteuid():
        return False
    proc = Path('/proc') / str(record['pid'])
    try:
        argv = [a.decode(errors='replace') for a in (proc / 'cmdline').read_bytes().split(b'\0') if a]
        executable = Path(os.readlink(str(proc / 'exe'))).name
        cwd = Path(os.readlink(str(proc / 'cwd')))
    except OSError:
        return False
    if not argv:
        return False
    index = 0
    if re.fullmatch(r'python(?:\d+(?:\.\d+)*)?|bash|dash|sh|zsh', executable):
        index = 1
        while index < len(argv) and argv[index] in ('-u', '-s', '-E', '-B', '-I', '-S', '--'):
            index += 1
        if index >= len(argv) or argv[index].startswith('-'):
            return False  # Do not interpret -c strings, -m, or arbitrary flags.
    program = Path(argv[index])
    if not program.is_absolute():
        program = cwd / program
    program = Path(os.path.abspath(str(program)))
    if expected_program is not None and program.resolve() != Path(expected_program).resolve():
        return False
    if program.is_file() and (inside(program.resolve(), root)
            or any(inside(program, install) for install in installs)):
        return True
    # ros2 run/launch wrappers execute outside the project. Accept only a
    # project package selected from an explicitly trusted install resource.
    if str(program) not in ('/opt/ros/humble/bin/ros2', '/opt/ros/foxy/bin/ros2'):
        return False
    command = argv[index + 1:]
    if len(command) < 3 or command[0] not in ('run', 'launch'):
        return False
    package = command[1]
    if not re.fullmatch(r'(?:go2w_|robot_scene_)[A-Za-z0-9_]+|hesai_ros_driver', package):
        return False
    search = [Path(p) for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
    search += [p for install in installs for p in (install, install / package)]
    for prefix in search:
        if (prefix / 'share/ament_index/resource_index/packages' / package).is_file():
            return any(inside(Path(os.path.abspath(str(prefix))), install) for install in installs)
    return False


def unchanged(path, original):
    try:
        descriptor = os.open(str(path), os.O_RDONLY | os.O_NOFOLLOW)
    except FileNotFoundError:
        return True  # An owned supervisor may remove its own record on exit.
    except OSError:
        return False
    try:
        info = os.fstat(descriptor)
        return (info.st_dev, info.st_ino, os.read(descriptor, 129)) == original
    finally:
        os.close(descriptor)


def signal_one(record):
    """Use a pidfd when available; Python3.8 retains explicit identity checks."""
    pid = record['pid']
    pidfd_open = getattr(os, 'pidfd_open', None)
    pidfd_send = getattr(signal, 'pidfd_send_signal', None)
    descriptor = None
    try:
        if callable(pidfd_open) and callable(pidfd_send):
            descriptor = pidfd_open(pid)
        current = table().get(pid)
        if current is None:
            return
        if (current['start'], current['uid']) != (record['start'], record['uid']):
            raise RuntimeError('process identity changed before signal: %s' % pid)
        if descriptor is not None:
            pidfd_send(descriptor, signal.SIGTERM)
        else:
            os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    finally:
        if descriptor is not None:
            os.close(descriptor)


def stop_record(path, root, timeout=8., expected_program=None):
    descriptor = os.open(str(path), os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid():
            raise RuntimeError('pidfile is not a regular file owned by this user')
        raw = os.read(descriptor, 128)
        if os.read(descriptor, 1):
            raise RuntimeError('pidfile is too long')
    finally:
        os.close(descriptor)
    text = raw.decode('ascii').strip()
    if not re.fullmatch(r'[0-9]+', text) or not 1 < int(text) <= 2147483647:
        raise RuntimeError('invalid pid; decimal integer in the host PID range required')
    pid = int(text)
    original = (info.st_dev, info.st_ino, raw)
    processes = table()
    protected, parent = set(), os.getpid()
    while parent in processes and parent not in protected:
        protected.add(parent)
        parent = processes[parent]['ppid']
    installs = trusted_installs(root)
    leader = processes.get(pid)
    if leader is not None and (pid in protected or not owns(leader, root, installs, expected_program)):
        raise RuntimeError('refusing foreign or supervisor pid %s' % pid)
    is_group = leader is None or leader['pgid'] == pid
    tracked, signaled = {}, set()
    deadline = time.monotonic() + timeout
    while True:
        processes = table()
        if leader is not None and pid in processes and processes[pid]['start'] != leader['start']:
            raise RuntimeError('leader PID was reused; record retained')
        members = {p: record for p, record in processes.items() if
            (is_group and record['pgid'] == pid)
            or (not is_group and descendant(p, pid, processes))
            or p in tracked}
        # Validate the complete observed scope before sending any new signal.
        for member, record in members.items():
            if member in protected or record['uid'] != os.geteuid():
                raise RuntimeError('refusing supervisor/foreign member %s' % member)
            if member in tracked and tracked[member] != record['start']:
                raise RuntimeError('member PID was reused; record retained')
            from_leader = leader is not None and pid in processes and descendant(member, pid, processes)
            if member not in tracked and not from_leader and not owns(record, root, installs, expected_program):
                raise RuntimeError('refusing foreign member %s; record retained' % member)
        if not members:
            if not unchanged(path, original):
                raise RuntimeError('pidfile changed during stop; replacement retained')
            if path.exists():
                path.unlink()
            return dict(pid=pid, signaled_processes=len(signaled), exited=True)
        if time.monotonic() >= deadline:
            raise RuntimeError('processes still alive after graceful stop; record retained: %s' % sorted(members))
        for member, record in members.items():
            key = (member, record['start'])
            tracked[member] = record['start']
            if key not in signaled:
                signal_one(record)
                signaled.add(key)
        time.sleep(.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--timeout', type=float, default=8.)
    parser.add_argument('--camera-pidfile', type=Path,
        help='stop only this search-owned HTTP capture, with an exact program check')
    args = parser.parse_args()
    if not .05 <= args.timeout <= 30:
        parser.error('timeout must be between 0.05 and 30 seconds')
    root = args.root.resolve()
    results, errors = [], []
    expected = None
    if args.camera_pidfile is not None:
        path = args.camera_pidfile.absolute()
        owner = root / 'outputs/autonomous_search/runtime'
        if path.parent.resolve() != owner.resolve() or path.name != 'camera_http_capture.pid':
            parser.error('camera pidfile must belong to the autonomous-search runtime')
        paths = [path]
        expected = root / 'scripts/go2w/robot_camera_http_capture.py'
    else:
        paths = sorted((root / 'runtime/go2w/pids').glob('*.pid'))
    for path in paths:
        try:
            results.append(dict(file=path.name, **stop_record(path, root, args.timeout, expected)))
        except FileNotFoundError:
            # Another owned cleanup can remove later records while earlier
            # workers are being stopped. Their disappearance is not a failure.
            continue
        except Exception as error:
            errors.append(dict(file=path.name, error=str(error)))
    print(json.dumps(dict(exited=results, errors=errors), ensure_ascii=False))
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
