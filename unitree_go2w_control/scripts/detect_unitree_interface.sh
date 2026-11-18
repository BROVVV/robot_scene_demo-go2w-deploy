#!/usr/bin/env bash
set -Eeuo pipefail

ROBOT_IP="${UNITREE_ROBOT_IP:-192.168.123.18}"
route_line="$(ip route get "$ROBOT_IP" 2>/dev/null | head -n 1)"
interface="$(awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}' <<<"$route_line")"

if [[ -z "$interface" || "$interface" == "lo" ]]; then
  # 同机部署（机器狗 Ubuntu 即本机，ROBOT_IP 是本机地址时路由走 lo）：
  # 回退到带 192.168.123.x 地址的物理网卡。
  interface="$(ip -4 -o addr show 2>/dev/null | awk '$4 ~ /^192[.]168[.]123[.]/ {print $2; exit}')"
fi
if [[ -z "$interface" || "$interface" == "lo" ]]; then
  printf 'ERROR: cannot resolve a usable interface for %s\n' "$ROBOT_IP" >&2
  exit 1
fi

printf '%s\n' "$interface"
