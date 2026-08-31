#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
ensure_layout
require_cmd ip

ROBOT_IP=192.168.123.18
selected=""
reason=""

route_line="$(ip route get "$ROBOT_IP" 2>/dev/null | head -n1 || true)"
route_dev="$(awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}' <<<"$route_line")"
route_src="$(awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}' <<<"$route_line")"
if [[ -n "$route_dev" && "$route_src" == 192.168.123.* && "$route_dev" != "lo" ]]; then
  selected="$route_dev"
  reason=route-existing
fi

if [[ -z "$selected" ]]; then
  mapfile -t candidates < <(
    for path in /sys/class/net/*; do
      iface="${path##*/}"
      [[ "$iface" =~ ^(lo|docker.*|veth.*|br-.*|virbr.*|tailscale.*)$ ]] && continue
      [[ -d "$path/device" ]] || continue
      [[ "$(cat "$path/type")" == 1 ]] || continue
      [[ "$(cat "$path/carrier" 2>/dev/null || printf 0)" == 1 ]] || continue
      printf '%s\n' "$iface"
    done
  )
  if [[ "${#candidates[@]}" -eq 1 ]]; then
    selected="${candidates[0]}"
    reason=single-carrier
    if ! ip route show default | grep -Eq "(^| )dev ${selected}( |$)"; then
      reason=carrier-no-default-route
    fi
  fi
fi

if [[ -z "$selected" ]]; then
  report="$REPORT_ROOT/NEEDS_NETWORK_SELECTION.md"
  {
    printf '# 需要选择机器人网卡\n\n'
    printf '未能唯一识别专用 Ethernet 接口。未修改任何网卡。\n\n'
    printf '```text\n'
    ip -br link
    ip -br addr
    ip route
    printf '```\n'
  } >"$report"
  die "interface selection is ambiguous; see $report"
fi

runtime="$PROJECT_ROOT/config/runtime.env"
{
  printf 'UNITREE_IFACE=%q\n' "$selected"
  printf 'SELECTION_REASON=%q\n' "$reason"
  printf 'UNITREE_HOST_IPV4=%q\n' '192.168.123.99/24'
  printf 'UNITREE_ROBOT_IP=%q\n' "$ROBOT_IP"
} >"$runtime"
chmod 600 "$runtime"
printf 'UNITREE_IFACE=%s\nSELECTION_REASON=%s\n' "$selected" "$reason"
