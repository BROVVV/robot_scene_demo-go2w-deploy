#!/usr/bin/env bash
set -euo pipefail

# Idempotent dependency installation. This script never starts ROS nodes or
# communicates with the robot.

if [[ ! -r /etc/os-release ]]; then
  printf 'ERROR: /etc/os-release is unavailable\n' >&2
  exit 2
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 ]]; then
  printf 'ERROR: Go2-W ROS deployment requires Ubuntu 22.04; found %s %s\n' \
    "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 2
fi

packages=(
  build-essential
  cmake
  git
  jq
  python3-pip
  python3-venv
  libyaml-cpp-dev
  libeigen3-dev
  python3-opencv
  python3-av
  python3-yaml
  python3-numpy
  python3-colcon-common-extensions
  ros-humble-rmw-cyclonedds-cpp
  ros-humble-rosidl-generator-dds-idl
  ros-humble-rclcpp
  ros-humble-rclpy
  ros-humble-sensor-msgs
  ros-humble-geometry-msgs
  ros-humble-nav-msgs
  ros-humble-diagnostic-msgs
  ros-humble-vision-msgs
  ros-humble-std-msgs
  ros-humble-std-srvs
  ros-humble-tf2
  ros-humble-tf2-ros
  ros-humble-tf2-geometry-msgs
  ros-humble-tf2-sensor-msgs
  ros-humble-image-transport
  ros-humble-cv-bridge
  ros-humble-camera-info-manager
  ros-humble-camera-calibration
  ros-humble-robot-state-publisher
  ros-humble-xacro
  ros-humble-message-filters
  ros-humble-pcl-conversions
  ros-humble-pcl-ros
  ros-humble-laser-geometry
  ros-humble-pointcloud-to-laserscan
  ros-humble-rko-lio
  ros-humble-slam-toolbox
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-nav2-collision-monitor
  ros-humble-nav2-velocity-smoother
  # plain_slam_ros2 + go2w_plain_slam_bridge build/test dependencies.
  ros-humble-ament-cmake-gtest
  ros-humble-ament-lint-auto
  ros-humble-ament-lint-common
)

missing=()
for package in "${packages[@]}"; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null \
      | grep -q '^install ok installed$'; then
    missing+=("$package")
  fi
done

if (( ${#missing[@]} == 0 )); then
  printf 'All %d Go2-W host dependencies are already installed.\n' \
    "${#packages[@]}"
else
  printf 'Installing %d missing packages:\n' "${#missing[@]}"
  printf '  %s\n' "${missing[@]}"
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends "${missing[@]}"
fi

# A 1920x1080 bgr8 Image serializes to more than 6 MiB. CycloneDDS needs
# enough kernel socket buffering to receive its fragment burst reliably.
minimum_dds_buffer=16777216
current_rmem="$(sysctl -n net.core.rmem_max)"
current_wmem="$(sysctl -n net.core.wmem_max)"
if (( current_rmem < minimum_dds_buffer )); then
  sudo sysctl -w "net.core.rmem_max=$minimum_dds_buffer"
fi
if (( current_wmem < minimum_dds_buffer )); then
  sudo sysctl -w "net.core.wmem_max=$minimum_dds_buffer"
fi

printf 'Dependency installation complete.\n'
