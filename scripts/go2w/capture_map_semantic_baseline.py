#!/usr/bin/env python3
"""Capture a read-only P0 baseline without changing robot or ROS state."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time
from typing import Any


def _run(command: list[str], *, cwd: Path, timeout: float = 10.0) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            command, cwd=cwd, capture_output=True, text=True, timeout=timeout,
            check=False,
        )
        return {"command": command, "returncode": completed.returncode,
                "stdout": completed.stdout, "stderr": completed.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"command": command, "returncode": None, "stdout": "", "stderr": str(exc)}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def capture(repo: Path, robot: str) -> dict[str, Any]:
    git_head = _run(["git", "rev-parse", "HEAD"], cwd=repo)
    git_status = _run(["git", "status", "--short", "--branch"], cwd=repo, timeout=30)
    processes = _run([
        "ps", "-eo", "pid,lstart,args", "--sort=pid"
    ], cwd=repo)
    relevant_processes = [
        line for line in processes["stdout"].splitlines()
        if any(term in line.casefold() for term in (
            "ros2", "go2w", "slam", "nav2", "realsense", "pandar", "unitree"
        ))
    ]
    key = Path.home() / ".ssh" / "id_ed25519_unitree_robotscene"
    remote_command = (
        "hostname; uname -a; printf 'boot_id='; "
        "sed -n '1p' /proc/sys/kernel/random/boot_id; "
        "printf 'uptime_seconds='; cut -d' ' -f1 /proc/uptime; "
        "ps -eo pid,lstart,args --sort=pid | "
        "grep -E 'image_server|realsense|sport|lidar|unitree|ros' | grep -v grep"
    )
    remote = _run([
        "ssh", "-F", "/dev/null", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
        "-o", "IdentitiesOnly=yes", "-o", "StrictHostKeyChecking=yes",
        "-o", f"UserKnownHostsFile={Path.home() / '.ssh' / 'known_hosts'}",
        "-i", str(key), robot, remote_command,
    ], cwd=repo, timeout=10)
    config_paths = [
        repo / "configs/go2w/navigation_sensor_profile.yaml",
        repo / "configs/go2w/lidar_preprocess.yaml",
        repo / "configs/go2w/time_sync.yaml",
        repo / "configs/go2w/autonomous_search_web.yaml",
        repo / "ros2_ws/src/robot_scene_nav_bringup/config/slam_2d_mapping.yaml",
        repo / "ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_humble.yaml",
    ]
    return {
        "captured_at": time.time(),
        "repo": str(repo),
        "git_head": git_head["stdout"].strip() or None,
        "git_status": git_status,
        "host": {
            "platform": platform.platform(),
            "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
            "ros_environment": {
                key: os.environ.get(key) for key in (
                    "ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"
                )
            },
            "relevant_processes": relevant_processes,
        },
        "robot": {"target": robot, "read_only_probe": remote},
        "resolved_configuration": {
            "process_arguments": relevant_processes,
            "files": {
                str(path.relative_to(repo)): {
                    "sha256": _sha256(path), "bytes": path.stat().st_size
                }
                for path in config_paths if path.is_file()
            },
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--robot", default="unitree@192.168.123.18")
    args = parser.parse_args()
    repo = args.repo.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    baseline = capture(repo, args.robot)
    (output / "runtime_identity.json").write_text(
        json.dumps({"host": baseline["host"], "robot": baseline["robot"],
                    "captured_at": baseline["captured_at"]},
                   ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (output / "resolved_configuration.json").write_text(
        json.dumps(baseline["resolved_configuration"], ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output / "workspace_state.txt").write_text(
        f"captured_at={baseline['captured_at']}\n"
        f"repo={repo}\n"
        f"git_head={baseline['git_head']}\n"
        + baseline["git_status"]["stdout"],
        encoding="utf-8",
    )
    print(json.dumps({"output_dir": str(output), "git_head": baseline["git_head"],
                      "robot_probe_ok": baseline["robot"]["read_only_probe"]["returncode"] == 0}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
