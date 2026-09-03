#!/usr/bin/env bash

_go2w_setup_fail() {
  printf 'go2w setup error: %s\n' "$1" >&2
  return 1 2>/dev/null || exit 1
}

_GO2W_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GO2W_CONTROL_ROOT="$(cd -- "$_GO2W_SCRIPT_DIR/.." && pwd)"
_GO2W_PROJECT_ROOT="$(cd -- "$GO2W_CONTROL_ROOT/.." && pwd)"
if [[ -z "${GO2W_UNITREE_ROOT:-}" ]]; then
  if [[ -d "$_GO2W_PROJECT_ROOT/external/unitree_ros2/cyclonedds_ws/install" ]]; then
    GO2W_UNITREE_ROOT="$_GO2W_PROJECT_ROOT/external/unitree_ros2"
  else
    GO2W_UNITREE_ROOT="$HOME/unitree_ros2"
  fi
fi
export GO2W_UNITREE_ROOT
export GO2W_ROBOT_IP="${GO2W_ROBOT_IP:-192.168.123.18}"
export GO2W_ROBOT_INTERFACE="$("$_GO2W_SCRIPT_DIR/detect_unitree_interface.sh")" || \
  _go2w_setup_fail "cannot resolve the robot interface"
if [[ -z "${GO2W_ROBOT_HOST_IP:-}" ]]; then
  GO2W_ROBOT_HOST_IP="$(ip -4 -o address show dev "$GO2W_ROBOT_INTERFACE" 2>/dev/null \
    | awk '$4 ~ /^192[.]168[.]123[.][0-9]+\// {sub("/.*", "", $4); print $4; exit}')"
fi
if [[ -z "${GO2W_ROBOT_HOST_IP:-}" ]]; then
  GO2W_ROBOT_HOST_IP="$(ip route get "$GO2W_ROBOT_IP" 2>/dev/null \
    | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
fi
[[ "$GO2W_ROBOT_HOST_IP" =~ ^192[.]168[.]123[.][0-9]+$ ]] || \
  _go2w_setup_fail "cannot resolve a 192.168.123.x host address on $GO2W_ROBOT_INTERFACE"
export GO2W_ROBOT_HOST_IP

GO2W_ROS_SETUP="${GO2W_ROS_SETUP:-}"
if [[ -z "$GO2W_ROS_SETUP" ]]; then
  # Jetson 真机：foxy 为官方 apt 包（humble 是移植版，DDS 栈不稳定）；
  # 工作站（22.04）：humble 官方。自动探测。
  if [[ -f /opt/ros/foxy/setup.bash ]]; then
    GO2W_ROS_SETUP=/opt/ros/foxy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    GO2W_ROS_SETUP=/opt/ros/humble/setup.bash
  fi
fi
if [[ ! -f "$GO2W_ROS_SETUP" ]]; then
  _go2w_setup_fail "ROS 2 setup missing: ${GO2W_ROS_SETUP:-<none>} (set GO2W_ROS_SETUP)"
fi
[[ -f "$GO2W_UNITREE_ROOT/cyclonedds_ws/install/setup.bash" ]] || \
  _go2w_setup_fail "Unitree message workspace missing"
[[ -f "$GO2W_CONTROL_ROOT/ros2_ws/install/setup.bash" ]] || \
  _go2w_setup_fail "control workspace is not built"

# The launcher is often invoked from a Conda/ROS1 shell.  Keep the control
# process on the Ubuntu Humble toolchain and let the project venv provide the
# SDK dependencies without inheriting ROS1 paths.
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_ROOT ROS_PACKAGE_PATH
unset ROS_MASTER_URI ROSLISP_PACKAGE_DIRECTORIES AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

set +u
# shellcheck source=/dev/null
source "$GO2W_ROS_SETUP"
# shellcheck source=/dev/null
source "$GO2W_UNITREE_ROOT/cyclonedds_ws/install/setup.bash"
# shellcheck source=/dev/null
source "$GO2W_CONTROL_ROOT/ros2_ws/install/setup.bash"
set -u

# Some ROS 2 setup files preserve legacy ROS 1 variables when this script is
# sourced from a mixed Noetic/ROS 2 shell. Clear them after all ROS 2 overlays
# have been sourced so rclpy and DDS do not resolve the ROS 1 environment.
unset ROS_ETC_DIR ROS_MASTER_URI ROS_PACKAGE_PATH ROS_ROOT

export GO2W_CONTROL_PYTHON="${GO2W_CONTROL_PYTHON:-$GO2W_CONTROL_ROOT/.venv/bin/python}"
export PYTHONPATH="$GO2W_CONTROL_ROOT/vendor/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"

# This workstation also carries ROS 1 Noetic.  Its Python/library paths must
# never precede Humble packages, otherwise rclpy resolves the ROS 1 std_msgs
# and crashes with "type object 'type' has no attribute '_TYPE_SUPPORT'".
# Noetic is absent on the robot, so this filter is a no-op there.
_GO2W_STRIP_NOETIC() {
  local _var="$1" _out="" _entry _old_ifs="$IFS"
  IFS=":"
  for _entry in ${!_var}; do
    case "$_entry" in
      /opt/ros/noetic|/opt/ros/noetic/*) ;;
      *) _out="${_out:+${_out}:}${_entry}" ;;
    esac
  done
  IFS="$_old_ifs"
  printf '%s' "$_out"
}
export PYTHONPATH="$(_GO2W_STRIP_NOETIC PYTHONPATH)"
export LD_LIBRARY_PATH="$(_GO2W_STRIP_NOETIC LD_LIBRARY_PATH)"
unset -f _GO2W_STRIP_NOETIC
# 注意：不能在此全局前置 ~/cyclonedds_0.10.2/lib —— ROS 层
# rmw_cyclonedds_cpp 0.7.11 与 libddsc 0.10.2 不兼容（rmw_create_node 失败）。
# SDK 进程（hold_sport_lease，依赖 python cyclonedds 0.10.2 的 ddsi_sertype_v0
# 符号）由 go2w_motion_control.launch.py 单独注入该库路径。

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
_GO2W_CYCLONE_FILE="/tmp/go2w_cyclonedds_${UID}.xml"
if [[ "$(uname -m)" == "aarch64" ]]; then
  # Jetson 真机（arm64）：foxy CycloneDDS 0.7.0 在默认多播模式下解析 SPDP
  # 报文会段错误（ddsi_plist_init_frommsg）；且其 CycloneDDS 不识别
  # <Interfaces> 元素。本机部署所有 ROS 节点都在本机，SDK 走 DDS RPC——
  # 用 localhost-only 环回通信（talker 实测稳定），并生成无 Interfaces 的配置。
  export ROS_LOCALHOST_ONLY=1
  {
    printf '%s\n' '<?xml version="1.0" encoding="UTF-8"?>'
    printf '%s\n' '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
  } >"$_GO2W_CYCLONE_FILE"
else
  # 工作站（x86_64，Ubuntu 22.04 官方 humble，CycloneDDS 0.10）：多播直连
  # 狗主控 DDS 域（/lf/sportmodestate、/lf/lowstate 等），用 name 形式绑定网卡。
  export ROS_LOCALHOST_ONLY=0
  {
    printf '%s\n' '<?xml version="1.0" encoding="UTF-8"?>'
    printf '%s\n' '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><Interfaces>'
    printf '  <NetworkInterface name="%s" priority="default" multicast="default"/>\n' "$GO2W_ROBOT_INTERFACE"
    printf '%s\n' '</Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
  } >"$_GO2W_CYCLONE_FILE"
fi
chmod 600 "$_GO2W_CYCLONE_FILE"
export CYCLONEDDS_URI="file://$_GO2W_CYCLONE_FILE"

printf 'Go2-W ROS 2 ready: interface=%s domain=%s rmw=%s\n' \
  "$GO2W_ROBOT_INTERFACE" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"
unset _GO2W_SCRIPT_DIR _GO2W_PROJECT_ROOT _GO2W_CYCLONE_FILE
