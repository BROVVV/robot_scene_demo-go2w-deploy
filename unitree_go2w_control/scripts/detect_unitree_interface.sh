#!/usr/bin/env bash
set -Eeuo pipefail

ROBOT_IP="${UNITREE_ROBOT_IP:-192.168.123.18}"
route_line="$(ip route get "$ROBOT_IP" 2>/dev/null | head -n 1)"
interface="$(awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}' <<<"$route_line")"

if [[ -z "$interface" || "$interface" == "lo" ]]; then
  printf 'ERROR: cannot resolve a usable interface for %s\n' "$ROBOT_IP" >&2
  exit 1
fi

printf '%s\n' "$interface"
