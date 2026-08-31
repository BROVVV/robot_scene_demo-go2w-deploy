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

[[ -f /opt/ros/humble/setup.bash ]] || _go2w_setup_fail "ROS 2 Humble missing"
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
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "$GO2W_UNITREE_ROOT/cyclonedds_ws/install/setup.bash"
# shellcheck source=/dev/null
source "$GO2W_CONTROL_ROOT/ros2_ws/install/setup.bash"
set -u

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

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
_GO2W_CYCLONE_FILE="/tmp/go2w_cyclonedds_${UID}.xml"
{
  printf '%s\n' '<?xml version="1.0" encoding="UTF-8"?>'
  printf '%s\n' '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><Interfaces>'
  printf '  <NetworkInterface address="%s" priority="default" multicast="default"/>\n' "$GO2W_ROBOT_HOST_IP"
  printf '%s\n' '</Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
} >"$_GO2W_CYCLONE_FILE"
chmod 600 "$_GO2W_CYCLONE_FILE"
export CYCLONEDDS_URI="file://$_GO2W_CYCLONE_FILE"

printf 'Go2-W ROS 2 ready: interface=%s domain=%s rmw=%s\n' \
  "$GO2W_ROBOT_INTERFACE" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"
unset _GO2W_SCRIPT_DIR _GO2W_PROJECT_ROOT _GO2W_CYCLONE_FILE
