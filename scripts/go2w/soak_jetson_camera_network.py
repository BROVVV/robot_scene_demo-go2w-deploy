#!/usr/bin/env python3
"""Read-only D435/Wi-Fi soak sampler; records health without image payloads."""

import argparse
import json
import subprocess
import time
from pathlib import Path
from urllib.request import urlopen


def _json(url):
    with urlopen(url, timeout=3) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=1800)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-wifi", action="store_true")
    parser.add_argument("--wifi-interface", default="wlan0")
    parser.add_argument("--max-frame-age", type=float, default=0.5)
    parser.add_argument("--min-camera-fps", type=float, default=25.0)
    args = parser.parse_args()
    if args.duration <= 0 or args.interval <= 0:
        parser.error("duration and interval must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    end = time.monotonic() + args.duration
    sample_count = 0
    failures = 0
    previous = None
    with args.output.open("w", encoding="utf-8") as stream:
        while time.monotonic() < end:
            sample = {"time": time.time(), "monotonic": time.monotonic()}
            try:
                sample["health"] = _json("http://127.0.0.1:8080/health")
                frame = _json("http://127.0.0.1:8080/rgbd/latest.json")
                sample["frame_id"] = frame.get("frame_id")
                sample["intrinsics_ok"] = all(
                    key in (frame.get("intrinsics") or {})
                    for key in ("fx", "fy", "cx", "cy")
                )
                sample["ok"] = (bool(sample["health"].get("ok"))
                    and sample["intrinsics_ok"]
                    and float(sample["health"].get("age_s", 999)) <= args.max_frame_age)
                if previous is not None:
                    elapsed = sample["monotonic"] - previous[0]
                    sample["camera_fps"] = (int(sample["frame_id"]) - previous[1]) / elapsed
                    sample["ok"] = sample["ok"] and sample["camera_fps"] >= args.min_camera_fps
                previous = (sample["monotonic"], int(sample["frame_id"]))
            except Exception as exc:  # hardware failures are evidence, not fatal
                sample["ok"] = False
                sample["error"] = "%s: %s" % (type(exc).__name__, exc)
            ip = subprocess.run(
                ["ip", "-br", "addr", "show", args.wifi_interface],
                text=True, capture_output=True, check=False, timeout=3,
            )
            sample["wlan"] = ip.stdout.strip()
            link = subprocess.run(
                ["iw", "dev", args.wifi_interface, "link"],
                text=True, capture_output=True, check=False, timeout=3,
            )
            sample["wifi_connected"] = link.returncode == 0 and "Connected to " in link.stdout
            sample["wifi_link"] = link.stdout.strip()
            if args.require_wifi:
                sample["ok"] = sample["ok"] and sample["wifi_connected"] and " UP " in sample["wlan"]
            if not sample["ok"]:
                failures += 1
            sample_count += 1
            stream.write(json.dumps(sample, ensure_ascii=False) + "\n")
            stream.flush()
            time.sleep(max(0.1, args.interval))
    print("samples=%d failures=%d" % (sample_count, failures))
    return 0 if failures == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
