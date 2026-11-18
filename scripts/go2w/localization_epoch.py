#!/usr/bin/env python3
"""Bind reuse of a mapping process to its robot, odometry and geometry epoch."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.go2w.prepare_nav2_trial import live_robot_boot_id


def process_identity(pid):
    # Split after comm: a Linux process name may contain spaces or parentheses.
    stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    return {'pid': int(pid), 'start_ticks': int(stat[19])}


def snapshot(root, pid, robot_boot_id):
    # Discover the exact node if the launch predates pidfile ownership. An
    # ambiguous or missing owner cannot be certified by an arbitrary PID.
    matches = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = entry.joinpath('cmdline').read_bytes().split(b'\0')
            if any(a.endswith(b'/go2w_lio_bringup/go2w_wheel_odom') for a in argv):
                matches.append(int(entry.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    if len(matches) != 1:
        raise ValueError(f'expected one wheel odometry process, found {len(matches)}')
    return {'schema_version': 1,
            'robot_boot_id': str(uuid.UUID(robot_boot_id)),
            'host_boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'localization': process_identity(pid),
            'wheel_odometry': process_identity(matches[0]),
            'sensor_profile_sha256': hashlib.sha256(
                (root/'configs/go2w/navigation_sensor_profile.yaml').read_bytes()).hexdigest()}


def require_matching_epoch(saved, current):
    for key in ('schema_version', 'robot_boot_id', 'host_boot_id', 'localization',
                'wheel_odometry', 'sensor_profile_sha256'):
        if key not in saved or saved[key] != current[key]:
            raise ValueError(f'localization epoch changed or unrecorded: {key}; '
                             'archive the old map/posegraph and explicitly recover localization')


def require_fresh_wheel_after_robot_reboot(saved, current):
    """A new map may not inherit wheel integrator state across a robot boot."""
    if (saved.get('robot_boot_id') != current.get('robot_boot_id')
            and saved.get('wheel_odometry') == current.get('wheel_odometry')):
        raise ValueError('wheel odometry process crosses robot reboot; stop the owned upstreams '
                         'and restart wheel odometry before recording a new localization epoch')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=('record', 'verify'))
    p.add_argument('--pid', required=True, type=int)
    args = p.parse_args()
    root = Path(__file__).resolve().parents[2]
    path = root/'outputs/autonomous_search/runtime/localization_epoch.json'
    current = snapshot(root, args.pid, live_robot_boot_id())
    if args.action == 'record':
        if path.exists():
            try:
                previous = json.loads(path.read_text())
            except (OSError, ValueError) as exc:
                raise ValueError('existing localization epoch is unreadable; refuse to overwrite') from exc
            require_fresh_wheel_after_robot_reboot(previous, current)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(current, indent=2)+'\n')
        temporary.replace(path)
    else:
        try:
            saved = json.loads(path.read_text())
        except (OSError, ValueError) as exc:
            raise ValueError('localization epoch is unrecorded; archive and recover the old map first') from exc
        require_matching_epoch(saved, current)
    print(json.dumps({'localization_epoch': args.action, **current}))


if __name__ == '__main__':
    main()
