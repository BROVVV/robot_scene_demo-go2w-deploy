#!/usr/bin/env bash
set -euo pipefail
. /etc/os-release
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  echo "仅支持 Ubuntu 22.04 + ROS2 Humble，当前 ${VERSION_ID:-unknown}" >&2
  exit 2
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "humble" ]]; then
  echo "当前 ROS_DISTRO=$ROS_DISTRO，不会安装其他版本。" >&2
  exit 2
fi
# A fresh Ubuntu installation does not contain the ROS 2 APT repository.
# Bootstrap the official repository before looking up ros-humble-* packages.
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe
sudo install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
architecture="$(dpkg --print-architecture)"
ros2_apt_repository="${ROS2_APT_REPOSITORY:-http://packages.ros.org/ros2/ubuntu}"
echo "deb [arch=${architecture} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ros2_apt_repository} ${UBUNTU_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-nav2-collision-monitor \
  ros-humble-nav2-velocity-smoother ros-humble-tf2-ros \
  ros-humble-tf-transformations python3-colcon-common-extensions

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "安装结束但 /opt/ros/humble/setup.bash 不存在，请检查 APT 输出。" >&2
  exit 3
fi
echo "ROS2 Humble/Nav2 安装完成：/opt/ros/humble/setup.bash"
