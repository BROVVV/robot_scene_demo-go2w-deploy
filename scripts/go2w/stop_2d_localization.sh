#!/usr/bin/env bash
# Stop only the exact project-owned 2D localization process group, including
# orphan children after a ros2 launch parent exits. Never sends motion.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/../.." && pwd)"
/usr/bin/python3 - "$project_root" <<'PY'
import os
from pathlib import Path
import signal
import sys
import time

root=Path(sys.argv[1])
workspace_install=Path(os.environ.get('GO2W_WORKSPACE_SETUP',
    str(root/'ros2_ws/install/setup.bash'))).parent
workspace_candidates=(root/'ros2_ws/install', workspace_install,
    root.parents[1]/'robotscene_runtime/install/ros2_ws')
# An explicitly selected native overlay may provide the same owned mapping
# executables while the unchanged base packages retain their old prefix.
if os.environ.get('GO2W_LIDAR_HOTPATH_SETUP'):
    workspace_candidates+=(Path(os.environ['GO2W_LIDAR_HOTPATH_SETUP']).parent,)
pidfiles=[root/'outputs/autonomous_search/runtime'/name for name in
          ('localization_2d.pid','localization_floor_overlay.pid')]
if not any(p.exists() for p in pidfiles):
    print('No owned 2D localization pidfile; nothing stopped.')
for pidfile in pidfiles:
    if not pidfile.exists():continue
    pid=int(pidfile.read_text())
    if pid<=1:raise RuntimeError('invalid localization pid')

    def members():
        found=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():continue
            try:
                if os.getpgid(int(entry.name))!=pid:continue
                if entry.joinpath('stat').read_text().split()[2]=='Z':continue
                argv=[a.decode() for a in entry.joinpath('cmdline').read_bytes().split(b'\0') if a]
                found.append((int(entry.name),argv))
            except (FileNotFoundError,ProcessLookupError,PermissionError):continue
        return found

    owned=members()
    for member,argv in owned:
        exact_launch='localization_2d.launch.py' in argv
        observed_floor_wrapper=(len(argv)>=5 and argv[1] in (
            '/opt/ros/humble/bin/ros2','/opt/ros/foxy/bin/ros2') and
            argv[2:5]==['run','robot_scene_nav_bringup','observed_floor_map.py'])
        project_node=any(
            str(install/'robot_scene_nav_bringup/lib/robot_scene_nav_bringup')+'/' in a
            for install in workspace_candidates for a in argv)
        ros_localizer=bool(argv and argv[0] in (
            '/opt/ros/humble/lib/nav2_amcl/amcl','/opt/ros/humble/lib/nav2_map_server/map_server',
            '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager',
            '/opt/ros/foxy/lib/nav2_amcl/amcl','/opt/ros/foxy/lib/nav2_map_server/map_server',
            '/opt/ros/foxy/lib/nav2_lifecycle_manager/lifecycle_manager') and
            any(a in ('__node:=amcl','__node:=map_server','__node:=lifecycle_manager_localization_2d') for a in argv))
        if not (exact_launch or observed_floor_wrapper or project_node or ros_localizer):
            raise RuntimeError(f'refusing foreign process in localization group: {member} {argv}')
    if owned:
        os.killpg(pid,signal.SIGTERM)
        deadline=time.monotonic()+8
        while members() and time.monotonic()<deadline:time.sleep(.1)
        remaining=members()
        if remaining:raise RuntimeError(f'owned localization group still alive; pidfile retained: {remaining}')
    if pidfile.exists() and int(pidfile.read_text())==pid:pidfile.unlink()
    print(f'Stopped owned 2D localization group {pid}; {len(owned)} processes.')
PY
