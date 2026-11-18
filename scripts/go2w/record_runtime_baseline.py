#!/usr/bin/python3
"""记录可复现基线（计划书 §P0.3）：Git 状态 + 文件 hash + 运行参数 + 进程快照。

只读，不启停任何东西，也不写工作树里的源码。产物是一个 JSON；下一轮接手时
用同一个脚本再跑一次，两份 JSON 逐字段对比就知道"到底哪里变了"。

为什么单独要文件 hash：`git status` 只说"这个文件被改过"，说不出"改成了什么"。
本项目的工作树是**故意保留脏的**（计划书 §1.3），同一个文件在不同接手人手里
都显示 M，光靠 git 分不出是不是同一份内容。hash 才分得出。

用法：
    /usr/bin/python3 scripts/go2w/record_runtime_baseline.py \
        --out artifacts/go2w_2d_nav_20261015/baseline_<轮次>.json
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]

# 与本链路直接相关、需要留 hash 的文件：Nav2 单链路的判据、启动/停止入口、
# 审计工具，以及它们的配置与测试。接口变了但 hash 没变 = 有人改了别的副本。
BASELINE_FILES = [
    'app/navigation/corridor_analyzer.py',
    'app/navigation/nav2_search_backend.py',
    'scripts/go2w/start_nav2_autonomous_search.sh',
    'scripts/go2w/stop_nav2_autonomous_search.sh',
    'scripts/go2w/stop_all.sh',
    'scripts/go2w/audit_navigation_runtime.py',
    'scripts/go2w/start_2d_localization.sh',
    'tests/test_corridor_analyzer.py',
    'tests/test_nav2_search_backend.py',
    'tests/test_search_session_recovery.py',
    'ros2_ws/src/robot_scene_nav_bringup/launch/robot_scene_nav2.launch.py',
    'ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml',
    'ros2_ws/src/robot_scene_nav_bringup/config/collision_monitor.yaml',
    'ros2_ws/src/go2w_cmd_vel_bridge/go2w_cmd_vel_bridge/bridge_core.py',
]


def run(args, **kwargs):
    """跑一条命令，返回 (returncode, stdout)。绝不抛，命令挂了也要留下证据。"""
    try:
        proc = subprocess.run(args, cwd=str(ROOT), capture_output=True,
                              text=True, timeout=60, **kwargs)
        return proc.returncode, proc.stdout.strip()
    except Exception as exc:  # noqa: BLE001 - 记进产物比抛出去有用
        return -1, 'ERROR: %s' % exc


def sha256(path):
    target = ROOT / path
    if not target.is_file():
        return None
    digest = hashlib.sha256()
    with target.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--test-command', default='')
    parser.add_argument('--test-result', default='')
    args = parser.parse_args()

    _, head = run(['git', 'rev-parse', 'HEAD'])
    _, branch = run(['git', 'rev-parse', '--abbrev-ref', 'HEAD'])
    _, status = run(['git', 'status', '--porcelain'])
    _, diff = run(['git', 'diff'])
    _, diff_cached = run(['git', 'diff', '--cached'])

    modified, untracked = [], []
    for line in status.splitlines():
        code, _, path = line[:2], line[2:3], line[3:]
        if code.strip() == '??':
            untracked.append(path)
        else:
            modified.append(path)

    # 工作树的 diff 摘要。`git diff | sha256` 是**单个**摘要：它能证明"两轮之间
    # 有没有任何改动"，但不能定位；定位靠下面逐文件的 hash。
    worktree_digest = hashlib.sha256((diff + diff_cached).encode('utf-8')).hexdigest()

    _, processes = run(['ps', '-eo', 'pid,ppid,pgid,etimes,cmd', '--no-headers'])
    interesting = [line for line in processes.splitlines()
                   if any(needle in line for needle in (
                       'ros2 launch', 'ros2 run', 'nav2_', 'lifecycle_manager',
                       'go2w_', 'slam_toolbox', 'uvicorn', 'hesai'))]

    payload = {
        'recorded_at_unix': time.time(),
        'recorded_at_local': time.strftime('%Y-%m-%d %H:%M:%S'),
        'git': {
            'head': head,
            'branch': branch,
            'dirty_tracked_count': len(modified),
            'untracked_count': len(untracked),
            'worktree_diff_sha256': worktree_digest,
            'dirty_tracked_files': sorted(modified),
            'untracked_files': sorted(untracked),
        },
        'file_sha256': {path: sha256(path) for path in BASELINE_FILES},
        'processes': interesting,
        'test_command': args.test_command,
        'test_result': args.test_result,
    }

    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
                   encoding='utf-8')
    print('baseline written: %s' % out)
    print('  head=%s branch=%s dirty=%d untracked=%d diff_sha256=%s'
          % (head[:12], branch, len(modified), len(untracked), worktree_digest[:16]))
    missing = [path for path, digest in payload['file_sha256'].items() if digest is None]
    if missing:
        print('  WARNING: 以下文件不存在，hash 为空（路径写错了？）：%s' % ', '.join(missing))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
