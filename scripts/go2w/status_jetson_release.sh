#!/usr/bin/env bash
# Read-only status for a staged Jetson release.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
printf 'release: %s\n' "$root"
printf 'manifest: %s\n' "$([[ -f "$root/release_manifest.json" ]] && echo present || echo missing)"
printf 'web: '
curl -sS --max-time 3 -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8876/ || true
printf 'camera: '
curl -sS --max-time 3 http://127.0.0.1:8080/health || true
printf '\nwlan: '
ip -br addr show wlan0 2>/dev/null || true
printf 'point-cloud Web bridges: '
pgrep -af 'scripts/go2w/(plain_slam_web_bridge|nav2_web_map_bridge)\.py' || true
printf 'active production link: '
readlink /home/unitree/robotscene_current 2>/dev/null || echo none
