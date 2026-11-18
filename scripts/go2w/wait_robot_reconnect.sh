#!/usr/bin/env bash
# Wait for the robot-facing IPv4 route and one real robot endpoint after an
# Ethernet replug.  This helper is intentionally side-effect free so the
# reconnect transition can be regression-tested without starting ROS/WebUI.
set -euo pipefail

robot_ip="${ROBOT_IP:-192.168.123.18}"
timeout_s="${REMOTE_MAPPING_RECONNECT_TIMEOUT_SECONDS:-180}"
poll_s="${GO2W_RECONNECT_POLL_SECONDS:-2}"

while (( $# )); do
  case "$1" in
    --robot-ip) robot_ip="${2:?--robot-ip requires an address}"; shift 2 ;;
    --timeout) timeout_s="${2:?--timeout requires seconds}"; shift 2 ;;
    *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done

[[ "${timeout_s}" =~ ^[1-9][0-9]*$ ]] || {
  echo 'ERROR: reconnect timeout must be a positive integer.' >&2
  exit 2
}
[[ "${poll_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo 'ERROR: GO2W_RECONNECT_POLL_SECONDS must be non-negative.' >&2
  exit 2
}

route_source() {
  ip route get "${robot_ip}" 2>/dev/null \
    | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}'
}

endpoint_reachable() {
  ping -c 1 -W 1 "${robot_ip}" >/dev/null 2>&1 \
    || curl --noproxy '*' -fsS --connect-timeout 1 --max-time 2 \
      "http://${robot_ip}:8080/health" >/dev/null 2>&1 \
    || timeout 2 bash -c 'exec 3<>"/dev/tcp/${1}/22"' _ "${robot_ip}" \
      >/dev/null 2>&1
}

deadline=$((SECONDS + timeout_s))
next_report=0
while :; do
  source_ip="$(route_source || true)"
  if [[ "${source_ip}" =~ ^192[.]168[.]123[.][0-9]+$ ]] \
     && endpoint_reachable; then
    printf '[OK] robot reachable via %s\n' "${source_ip}"
    exit 0
  fi
  if (( SECONDS >= deadline )); then
    printf 'ERROR: waited %ss but robot %s is still unreachable.\n' \
      "${timeout_s}" "${robot_ip}" >&2
    echo '       Check robot power, Ethernet carrier and the host 192.168.123.x address.' >&2
    exit 3
  fi
  if (( SECONDS >= next_report )); then
    remaining=$((deadline - SECONDS))
    printf '[WAIT] 等待网线/网卡/机器狗恢复（本机源地址=%s，最多再等 %ss）……\n' \
      "${source_ip:-无}" "${remaining}"
    next_report=$((SECONDS + 10))
  fi
  sleep "${poll_s}"
done
