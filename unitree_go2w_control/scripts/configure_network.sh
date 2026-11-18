#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
new_log_dir
# shellcheck source=/dev/null
source "$PROJECT_ROOT/config/runtime.env"
require_cmd ip
require_cmd nmcli

PROFILE=unitree-go2w
if nmcli -g NAME connection show | grep -Fxq "$PROFILE"; then
  nmcli connection modify "$PROFILE" \
    connection.interface-name "$UNITREE_IFACE" \
    ipv4.method manual ipv4.addresses "$UNITREE_HOST_IPV4" \
    ipv4.gateway "" ipv4.dns "" ipv4.never-default yes \
    ipv4.ignore-auto-dns yes ipv6.method disabled
else
  nmcli connection add type ethernet ifname "$UNITREE_IFACE" con-name "$PROFILE" \
    ipv4.method manual ipv4.addresses "$UNITREE_HOST_IPV4" \
    ipv4.gateway "" ipv4.dns "" ipv4.never-default yes \
    ipv4.ignore-auto-dns yes ipv6.method disabled
fi
nmcli connection up "$PROFILE"

{
  ip -br addr show dev "$UNITREE_IFACE"
  ip route get "$UNITREE_ROBOT_IP"
  ping -I "$UNITREE_IFACE" -c 4 -W 1 "$UNITREE_ROBOT_IP" || true
  ip neigh show dev "$UNITREE_IFACE"
  ip route show default
  nmcli connection show "$PROFILE"
} 2>&1 | tee "$LOG_DIR/network_after.txt"

ip -4 addr show dev "$UNITREE_IFACE" | grep -Fq '192.168.123.99/24' || die "static IPv4 missing"
ip route get "$UNITREE_ROBOT_IP" | grep -Fq "dev $UNITREE_IFACE" || die "robot route uses wrong interface"
