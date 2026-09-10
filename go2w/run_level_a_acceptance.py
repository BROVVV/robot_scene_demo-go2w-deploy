#!/usr/bin/env python3
"""Run the stationary Level-A transport soak without starting motion nodes."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FORBIDDEN_PROCESS_MARKERS = (
    "go2w_cmd_vel_bridge",
    "go2w_control_arbiter",
    "controller_server",
    "bt_navigator",
    "velocity_smoother",
    "collision_monitor",
    "motion_server",
    "sport_client",
)


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def process_snapshot(root_pid: int) -> dict:
    parents: dict[int, int] = {}
    commands: dict[int, str] = {}
    rss_mib: dict[int, float] = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            stat = (entry / "stat").read_text(encoding="utf-8")
            remainder = stat[stat.rfind(")") + 2 :].split()
            parents[pid] = int(remainder[1])
            raw = (entry / "cmdline").read_bytes().replace(b"\0", b" ").strip()
            commands[pid] = raw.decode("utf-8", errors="replace")
            status = (entry / "status").read_text(encoding="utf-8")
            rss_line = next(
                (line for line in status.splitlines() if line.startswith("VmRSS:")),
                "VmRSS: 0 kB",
            )
            rss_mib[pid] = float(rss_line.split()[1]) / 1024.0
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
            continue
    owned = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, parent in parents.items():
            if parent in owned and pid not in owned:
                owned.add(pid)
                changed = True
    return {
        "pids": sorted(pid for pid in owned if pid in parents or pid == root_pid),
        "rss_mib": sum(rss_mib.get(pid, 0.0) for pid in owned),
        "commands": [commands[pid] for pid in sorted(owned) if commands.get(pid)],
    }


def spool_size_bytes(root: Path) -> int:
    total = 0
    if not root.exists():
        return 0
    for path in root.rglob("*"):
        try:
            if path.is_file() and not path.is_symlink():
                total += path.stat().st_size
        except FileNotFoundError:
            pass
    return total


def latest_payload(spool: Path) -> tuple[Path, dict] | None:
    try:
        directory = (spool / "latest").resolve(strict=True)
        payload = json.loads((directory / "frame_bundle.json").read_text(encoding="utf-8"))
        if not (directory / "READY").is_file():
            return None
        return directory, payload
    except (OSError, ValueError, KeyError, json.JSONDecodeError):
        return None


def bundle_count(spool: Path) -> int:
    root = spool / "bundles"
    try:
        return sum(path.is_dir() for path in root.iterdir())
    except FileNotFoundError:
        return 0


def ethernet_carrier_up(interface: str = "enp6s0") -> bool:
    try:
        return Path(f"/sys/class/net/{interface}/carrier").read_text().strip() == "1"
    except OSError:
        return False


def slope_per_second(samples: list[dict], key: str) -> float:
    if len(samples) < 3:
        return math.inf
    usable = samples[max(1, len(samples) // 2) :]
    xs = [float(item["elapsed_seconds"]) for item in usable]
    ys = [float(item[key]) for item in usable]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    denominator = sum((value - x_mean) ** 2 for value in xs)
    if denominator <= 0.0:
        return math.inf
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator


def solid_green_fraction(image_path: Path) -> float:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return 1.0
    values = image.astype(np.int16)
    blue, green, red = (values[:, :, index] for index in range(3))
    return float(
        np.mean(
            (green >= 80)
            & (green >= blue * 3)
            & (green >= red * 3)
            & (blue <= 12)
            & (red <= 12)
        )
    )


def stop_owned_launcher(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGTERM)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def pid_is_running(pid: int) -> bool:
    try:
        status = Path(f"/proc/{pid}/status").read_text(encoding="utf-8")
    except (FileNotFoundError, ProcessLookupError):
        return False
    state = next((line for line in status.splitlines() if line.startswith("State:")), "")
    return "Z (zombie)" not in state


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-seconds", type=float, default=600.0)
    parser.add_argument("--minimum-level-a-seconds", type=float, default=600.0)
    parser.add_argument("--sample-period-seconds", type=float, default=5.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "outputs/go2w_acceptance/level_a_stationary_soak",
    )
    args = parser.parse_args()
    if args.duration_seconds <= 0.0 or args.sample_period_seconds <= 0.0:
        raise SystemExit("duration and sample period must be positive")
    if args.minimum_level_a_seconds < 600.0:
        raise SystemExit("minimum-level-a-seconds cannot be below the plan's 600 seconds")

    output = args.output_dir.resolve()
    run_id = f"run_{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}"
    # Each run owns a unique spool. Older acceptance evidence is preserved but
    # cannot become this run's first frame or inflate its retention count.
    spool = output / "spool" / run_id
    output.mkdir(parents=True, exist_ok=True)
    stdout_path = output / "launcher.stdout.log"
    stderr_path = output / "launcher.stderr.log"
    environment = os.environ.copy()
    environment["GO2W_FRAME_SPOOL_DIR"] = str(spool)
    command = ["bash", str(PROJECT_ROOT / "scripts/go2w/start_live_perception.sh")]
    samples: list[dict] = []
    observed_commands: set[str] = set()
    owned_pids: set[int] = set()
    start = time.monotonic()
    first_bundle = None
    last_bundle = None
    last_frame_id = None
    last_frame_change_time = None
    observation_end = None
    observation_end_wall = None
    process = None
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        process = subprocess.Popen(
            command,
            cwd=PROJECT_ROOT,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            start_new_session=True,
        )
        try:
            startup_deadline = time.monotonic() + 30.0
            while time.monotonic() < startup_deadline:
                if process.poll() is not None:
                    raise RuntimeError(f"read-only launcher exited early: {process.returncode}")
                current = latest_payload(spool)
                if current is not None:
                    first_bundle = current
                    last_bundle = current
                    last_frame_id = int(current[1]["frame_id"])
                    last_frame_change_time = time.monotonic()
                    break
                time.sleep(0.2)
            if first_bundle is None:
                raise RuntimeError("no complete frame bundle arrived within 30 seconds")

            next_sample = time.monotonic()
            while time.monotonic() - start < args.duration_seconds:
                if process.poll() is not None:
                    raise RuntimeError(f"read-only launcher exited early: {process.returncode}")
                now = time.monotonic()
                if now >= next_sample:
                    process_data = process_snapshot(process.pid)
                    owned_pids.update(process_data["pids"])
                    observed_commands.update(process_data["commands"])
                    current = latest_payload(spool)
                    if current is not None:
                        last_bundle = current
                        current_frame_id = int(current[1]["frame_id"])
                        if current_frame_id != last_frame_id:
                            last_frame_id = current_frame_id
                            last_frame_change_time = now
                    samples.append(
                        {
                            "elapsed_seconds": now - start,
                            "owned_rss_mib": process_data["rss_mib"],
                            "spool_bytes": spool_size_bytes(spool),
                            "retained_bundles": bundle_count(spool),
                            "ethernet_carrier_up": ethernet_carrier_up(),
                            "latest_frame_id": (
                                int(last_bundle[1]["frame_id"]) if last_bundle else None
                            ),
                        }
                    )
                    next_sample = now + args.sample_period_seconds
                time.sleep(min(0.2, max(0.01, next_sample - time.monotonic())))
            observation_end = time.monotonic()
            observation_end_wall = time.time()
            current = latest_payload(spool)
            if current is not None:
                last_bundle = current
        finally:
            if process is not None:
                final_snapshot = process_snapshot(process.pid)
                owned_pids.update(final_snapshot["pids"])
                observed_commands.update(final_snapshot["commands"])
                stop_owned_launcher(process)

    elapsed = time.monotonic() - start
    cleanup_deadline = time.monotonic() + 5.0
    residual_pids = sorted(pid for pid in owned_pids if pid_is_running(pid))
    while residual_pids and time.monotonic() < cleanup_deadline:
        time.sleep(0.1)
        residual_pids = sorted(pid for pid in owned_pids if pid_is_running(pid))
    if last_bundle is None:
        raise RuntimeError("no final frame bundle is available")
    first_payload = first_bundle[1]
    last_directory, final_payload = last_bundle
    stamp_span = (
        int(final_payload["image_receive_time_ns"])
        - int(first_payload["image_receive_time_ns"])
    ) / 1e9
    written_span = int(final_payload["frame_id"]) - int(first_payload["frame_id"])
    bundle_rate_hz = written_span / stamp_span if stamp_span > 0.0 else 0.0
    final_bundle_age = (
        max(0.0, observation_end_wall - (last_directory / "READY").stat().st_mtime)
        if observation_end_wall is not None
        else math.inf
    )
    stream_covered_duration = stamp_span >= max(0.0, args.duration_seconds - 5.0)
    rss_slope = slope_per_second(samples, "owned_rss_mib")
    rss_growth_check_applicable = elapsed >= 120.0
    forbidden_seen = sorted(
        marker
        for marker in FORBIDDEN_PROCESS_MARKERS
        if any(marker in command for command in observed_commands)
    )
    health = final_payload.get("sensor_health") or {}
    green_fraction = solid_green_fraction(last_directory / "image.jpg")
    transport_checks = {
        "camera_fresh": health.get("camera") is True,
        "lidar_fresh": health.get("lidar") is True,
        "motion_commands_false": (
            (final_payload.get("motion_state") or {}).get("commanded_by_bridge") is False
        ),
        "no_motion_or_nav_process_started": not forbidden_seen,
        "bundle_rate_0_8_to_1_2_hz": 0.8 <= bundle_rate_hz <= 1.2,
        "bundle_stream_covered_requested_duration": stream_covered_duration,
        "latest_bundle_age_below_2_5_seconds": final_bundle_age <= 2.5,
        "ethernet_carrier_remained_up": all(
            bool(item["ethernet_carrier_up"]) for item in samples
        ),
        "retained_bundle_count_bounded": max(
            (int(item["retained_bundles"]) for item in samples), default=0
        ) <= 30,
        "spool_below_20_mib": max(
            (int(item["spool_bytes"]) for item in samples), default=0
        ) <= 20 * 1024 * 1024,
        "rss_growth_below_0_05_mib_per_sec": (
            not rss_growth_check_applicable or rss_slope <= 0.05
        ),
        "image_content_not_solid_green": green_fraction < 0.95,
        "owned_processes_cleaned_up": not residual_pids,
    }
    duration_passed = elapsed >= args.minimum_level_a_seconds - 1.0
    camera_info_calibrated = health.get("camera_info_calibrated") is True
    camera_tf_valid = health.get("tf") is True
    transport_passed = all(transport_checks.values())
    level_a_passed = (
        duration_passed
        and transport_passed
        and camera_info_calibrated
        and camera_tf_valid
    )
    result = {
        "schema_version": "1.0",
        "run_id": run_id,
        "spool_path": str(spool),
        "level_a_passed": level_a_passed,
        "transport_soak_passed": duration_passed and transport_passed,
        "duration_seconds": elapsed,
        "required_duration_seconds": args.minimum_level_a_seconds,
        "duration_passed": duration_passed,
        "camera_info_calibrated": camera_info_calibrated,
        "camera_tf_valid": camera_tf_valid,
        "level_a_blockers": (
            []
            if level_a_passed
            else [
                reason
                for reason, blocked in (
                    ("ten_minute_duration_not_met", not duration_passed),
                    ("transport_soak_failed", not transport_passed),
                    ("camera_intrinsics_not_calibrated", not camera_info_calibrated),
                    ("camera_tf_not_validated", not camera_tf_valid),
                )
                if blocked
            ]
        ),
        "transport_checks": transport_checks,
        "metrics": {
            "bundle_rate_hz": bundle_rate_hz,
            "bundle_stamp_span_seconds": stamp_span,
            "final_bundle_age_seconds": final_bundle_age,
            "first_frame_id": int(first_payload["frame_id"]),
            "last_frame_id": int(final_payload["frame_id"]),
            "retained_bundles_final": bundle_count(spool),
            "spool_bytes_final": spool_size_bytes(spool),
            "owned_rss_mib_min": min(item["owned_rss_mib"] for item in samples),
            "owned_rss_mib_max": max(item["owned_rss_mib"] for item in samples),
            "owned_rss_growth_mib_per_second": rss_slope,
            "rss_growth_check_applicable": rss_growth_check_applicable,
            "solid_green_fraction": green_fraction,
        },
        "final_sensor_health": health,
        "forbidden_process_markers_seen": forbidden_seen,
        "residual_owned_pids": residual_pids,
        "sample_count": len(samples),
        "motion_commands_sent": False,
    }
    atomic_json(output / "result.json", result)
    atomic_json(output / "samples.json", {"samples": samples})
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if level_a_passed else (3 if result["transport_soak_passed"] else 1)


if __name__ == "__main__":
    raise SystemExit(main())
