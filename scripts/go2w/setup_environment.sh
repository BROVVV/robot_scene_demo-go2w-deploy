#!/usr/bin/env bash

# Source this file from a ROS 2 shell. It intentionally does not source the
# Conda application environment into ROS workers.
# This file is sourced by launch/check scripts.  Do not enable errexit in the
# caller: a timed ROS probe is allowed to return non-zero and be recorded as a
# degraded readiness check instead of terminating the whole diagnostic script.
set -uo pipefail

GO2W_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GO2W_PROJECT_ROOT="$(cd -- "$GO2W_SCRIPT_DIR/../.." && pwd)"
# Jetson 真机：humble 为移植版（DDS 栈崩溃），foxy 为官方 apt 包（稳定）。
# Prefer an explicitly selected ROS installation.  Otherwise select the
# installation that actually exists on this machine: the Jetson deployment
# uses Foxy, while the x86_64 host uses the official Humble installation.
GO2W_ROS_SETUP="${GO2W_ROS_SETUP:-}"
if [[ -z "$GO2W_ROS_SETUP" ]]; then
  if [[ -f /opt/ros/foxy/setup.bash ]]; then
    GO2W_ROS_SETUP=/opt/ros/foxy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    GO2W_ROS_SETUP=/opt/ros/humble/setup.bash
  fi
fi
GO2W_WORKSPACE_SETUP="${GO2W_WORKSPACE_SETUP:-$GO2W_PROJECT_ROOT/ros2_ws/install/setup.bash}"
GO2W_CONTROL_ROOT="${GO2W_CONTROL_ROOT:-$GO2W_PROJECT_ROOT/unitree_go2w_control}"
GO2W_UNITREE_ROOT="${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}"
GO2W_CONTROL_SETUP="${GO2W_CONTROL_SETUP:-$GO2W_CONTROL_ROOT/ros2_ws/install/setup.bash}"
GO2W_UNITREE_SETUP="${GO2W_UNITREE_SETUP:-}"

if [[ -z "${GO2W_INTERFACE:-}" ]]; then
  for candidate in eth0 enp6s0 enp3s0 enp4s0 enp5s0; do
    if [[ -r "/sys/class/net/${candidate}/carrier" ]] \
      && [[ "$(< "/sys/class/net/${candidate}/carrier")" == "1" ]] \
      && ip -4 -o address show dev "$candidate" 2>/dev/null \
        | awk '$4 ~ /^192[.]168[.]123[.][0-9]+\// { found=1 } END { exit !found }'; then
      GO2W_INTERFACE="$candidate"
      break
    fi
  done
fi
GO2W_INTERFACE="${GO2W_INTERFACE:-enp6s0}"

if [[ ! "$GO2W_INTERFACE" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
  printf 'ERROR: invalid GO2W_INTERFACE: %s\n' "$GO2W_INTERFACE" >&2
  return 2 2>/dev/null || exit 2
fi

# This host can have more than one IPv4 address on the robot-facing NIC.  A
# name-only CycloneDDS interface selector may then choose the wrong address
# (for example 192.168.1.10 instead of the robot network's 192.168.123.99).
# Pin DDS to the address used to reach the robot.  Address-only selection is
# intentional: CycloneDDS rejects a name+address pair when the NIC has
# multiple addresses.
if [[ -z "${GO2W_HOST_IP:-}" ]]; then
  # 网卡不存在时 ip 会返回非零；调用方多半开了 pipefail，这里不能让整段静默退出，
  # 必须走到下面那条明确的 ERROR。
  GO2W_HOST_IP="$(ip -4 -o address show dev "$GO2W_INTERFACE" 2>/dev/null \
    | awk '$4 ~ /^192[.]168[.]123[.][0-9]+\// {sub("/.*", "", $4); print $4; exit}' || true)"
fi
if [[ -z "${GO2W_HOST_IP:-}" ]]; then
  GO2W_HOST_IP="$(ip route get 192.168.123.18 2>/dev/null \
    | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}' || true)"
fi
if [[ ! "${GO2W_HOST_IP:-}" =~ ^192[.]168[.]123[.][0-9]+$ ]]; then
  printf 'ERROR: cannot resolve a 192.168.123.x host address on %s (got: %s)\n' \
    "$GO2W_INTERFACE" "${GO2W_HOST_IP:-<none>}" >&2
  return 2 2>/dev/null || exit 2
fi

if [[ -z "${GO2W_UNITREE_SETUP}" ]]; then
  for candidate in \
    "$HOME/cyclonedds_ws/install/setup.bash" \
    "$GO2W_UNITREE_ROOT/cyclonedds_ws/install/setup.bash" \
    "$GO2W_PROJECT_ROOT/external/unitree_ros2/cyclonedds_ws/install/setup.bash" \
    "$GO2W_PROJECT_ROOT/external/unitree_ros2/cyclonedds_ws/install_system/setup.bash" \
    "$HOME/robot/unitree_ros2/cyclonedds_ws/install/setup.bash" \
    "/home/brov/robot/unitree_ros2/cyclonedds_ws/install/setup.bash"; do
    if [[ -f "$candidate" ]]; then
      GO2W_UNITREE_SETUP="$candidate"
      break
    fi
  done
fi

if [[ -n "${GO2W_UNITREE_SETUP:-}" && \
      "$GO2W_UNITREE_SETUP" == */cyclonedds_ws/install/setup.bash ]]; then
  GO2W_UNITREE_ROOT="${GO2W_UNITREE_SETUP%/cyclonedds_ws/install/setup.bash}"
elif [[ -n "${GO2W_UNITREE_SETUP:-}" && \
        "$GO2W_UNITREE_SETUP" == */cyclonedds_ws/install_system/setup.bash ]]; then
  GO2W_UNITREE_ROOT="${GO2W_UNITREE_SETUP%/cyclonedds_ws/install_system/setup.bash}"
fi

if [[ ! -f "$GO2W_ROS_SETUP" ]]; then
  printf 'ERROR: ROS 2 setup not found: %s\n' "$GO2W_ROS_SETUP" >&2
  return 2 2>/dev/null || exit 2
fi

# This workspace is ROS 2-only. Clear inherited ROS 1 variables so a shell
# opened from a ROS Noetic development environment cannot mix paths.
unset ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ETC_DIR
unset ROS_MASTER_URI ROS_ROOT ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH PYTHONPATH

set +u
# shellcheck disable=SC1090
source "$GO2W_ROS_SETUP"
set -u

# A colcon setup.bash records every underlay that was active when that
# workspace was built.  Avoid an incompatible recorded underlay, but retain
# the full vendor chain on the DCU where /opt/ros/humble is intentionally a
# Foxy-compatible vendor environment (ROS_DISTRO differs from the path name).
GO2W_BASE_SETUP_LABEL="$(basename "$(dirname "$GO2W_ROS_SETUP")")"
GO2W_VENDOR_CHAIN_MODE=0
if [[ "${ROS_DISTRO:-}" != "$GO2W_BASE_SETUP_LABEL" ]]; then
  GO2W_VENDOR_CHAIN_MODE=1
fi

source_overlay_safely() {
  local setup_file="$1"
  local local_setup="${setup_file%/setup.bash}/local_setup.bash"
  set +u
  if [[ "$GO2W_VENDOR_CHAIN_MODE" == "0" ]] \
    && grep -Eq 'COLCON_CURRENT_PREFIX="/opt/ros/[^"/]+' "$setup_file" \
    && grep -E 'COLCON_CURRENT_PREFIX="/opt/ros/[^"/]+' "$setup_file" \
      | grep -Fvq "/opt/ros/${ROS_DISTRO}" \
    && [[ -f "$local_setup" ]]; then
    # shellcheck disable=SC1090
    source "$local_setup"
  else
    # shellcheck disable=SC1090
    source "$setup_file"
  fi
  set -u
}

if [[ -n "${GO2W_UNITREE_SETUP:-}" && -f "$GO2W_UNITREE_SETUP" ]]; then
  source_overlay_safely "$GO2W_UNITREE_SETUP"
fi
if [[ -f "$GO2W_WORKSPACE_SETUP" ]]; then
  source_overlay_safely "$GO2W_WORKSPACE_SETUP"
fi
if [[ -z "${GO2W_CONTROL_SETUP:-}" ]]; then
  for candidate in \
    "$GO2W_CONTROL_ROOT/ros2_ws/install/setup.bash" \
    "$HOME/robot/unitree_go2w_control/ros2_ws/install/setup.bash" \
    "$HOME/unitree_go2w_control/ros2_ws/install/setup.bash" \
    "/home/brov/robot/unitree_go2w_control/ros2_ws/install/setup.bash"; do
    if [[ -f "$candidate" ]]; then
      GO2W_CONTROL_SETUP="$candidate"
      break
    fi
  done
fi
if [[ -z "${GO2W_CONTROL_SETUP:-}" ]]; then
  :
fi
if [[ -n "${GO2W_CONTROL_SETUP:-}" && -f "$GO2W_CONTROL_SETUP" ]]; then
  source_overlay_safely "$GO2W_CONTROL_SETUP"
fi
unset -f source_overlay_safely

export GO2W_CONTROL_ROOT GO2W_UNITREE_ROOT GO2W_CONTROL_SETUP GO2W_UNITREE_SETUP
export GO2W_CONTROL_PYTHON="${GO2W_CONTROL_PYTHON:-$GO2W_CONTROL_ROOT/.venv/bin/python}"
if [[ -d "$GO2W_CONTROL_ROOT/vendor/unitree_sdk2_python" ]]; then
  export PYTHONPATH="$GO2W_CONTROL_ROOT/vendor/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export GO2W_PROJECT_ROOT
export GO2W_RUNTIME_ROOT="${GO2W_RUNTIME_ROOT:-$GO2W_PROJECT_ROOT/runtime/go2w}"
mkdir -p "$GO2W_RUNTIME_ROOT"

# Rebuild the project-owned configuration on every source.  This avoids a
# stale CYCLONEDDS_URI inherited from an older shell/session silently winning.
# An explicitly managed external DDS setup can opt out.
if [[ "${GO2W_USE_EXTERNAL_CYCLONEDDS_URI:-0}" != "1" ]]; then
  GO2W_CYCLONEDDS_CONFIG="$GO2W_RUNTIME_ROOT/cyclonedds_${GO2W_INTERFACE}.xml"
  sed -E "s#<NetworkInterface[^>]*/>#        <NetworkInterface address=\"$GO2W_HOST_IP\" priority=\"default\" multicast=\"default\" />#" \
    "$GO2W_PROJECT_ROOT/configs/go2w/cyclonedds_go2w.xml" \
    > "$GO2W_CYCLONEDDS_CONFIG"
  CYCLONEDDS_URI="file://$GO2W_CYCLONEDDS_CONFIG"
fi
export CYCLONEDDS_URI GO2W_INTERFACE GO2W_HOST_IP GO2W_UNITREE_SETUP GO2W_CONTROL_SETUP

if [[ "${CONDA_PREFIX:-}" == *go2_robot_scene_demo* ]]; then
  printf '%s\n' \
    'WARNING: Conda is active. ROS workers still use /usr/bin/python3.' >&2
fi

printf 'GO2-W ROS environment ready: distro=%s rmw=%s domain=%s\n' \
  "${ROS_DISTRO:-unknown}" "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID"
