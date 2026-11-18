#!/usr/bin/env bash
# Source for ROS builds/tests with no robot: isolated domain, loopback only.
# This does not rewrite the production DDS file or inspect a robot interface.
go2w_offline_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GO2W_PROJECT_ROOT="$(cd -- "$go2w_offline_script_dir/../.." && pwd)"
go2w_offline_domain="${ROS_DOMAIN_ID:-89}"
if [[ ! "$go2w_offline_domain" =~ ^([1-9][0-9]?|100)$ ]]; then
  printf '%s\n' 'Offline tests require ROS_DOMAIN_ID between 1 and 100; domain 0 is refused.' >&2
  return 2 2>/dev/null || exit 2
fi
go2w_offline_base="${GO2W_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "$go2w_offline_base" ]]; then
  printf 'ROS setup missing: %s\n' "$go2w_offline_base" >&2
  return 2 2>/dev/null || exit 2
fi
unset ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_ETC_DIR ROS_MASTER_URI ROS_ROOT
unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH CYCLONEDDS_URI
# colcon's top-level setup can replay a recorded Noetic underlay. Only source
# each workspace's own hooks after explicitly selecting the ROS 2 base.
go2w_offline_source() {
  local go2w_offline_result=0 go2w_offline_nounset=0
  [[ "$-" == *u* ]] && go2w_offline_nounset=1
  set +u
  source "$1" || go2w_offline_result=$?
  if [[ "$go2w_offline_nounset" == 1 ]]; then set -u; fi
  return "$go2w_offline_result"
}
go2w_offline_source "$go2w_offline_base" || return
for go2w_offline_overlay in \
  "$GO2W_PROJECT_ROOT/external/unitree_ros2/cyclonedds_ws/install_system/local_setup.bash" \
  "$GO2W_PROJECT_ROOT/ros2_ws/install/local_setup.bash" \
  "$GO2W_PROJECT_ROOT/unitree_go2w_control/ros2_ws/install/local_setup.bash"; do
  if [[ -f "$go2w_offline_overlay" ]]; then
    go2w_offline_source "$go2w_offline_overlay" || return
  fi
done
export ROS_DOMAIN_ID="$go2w_offline_domain" ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
printf 'Offline ROS environment: distro=%s domain=%s localhost_only=1\n' "${ROS_DISTRO:-unknown}" "$ROS_DOMAIN_ID"
unset go2w_offline_script_dir go2w_offline_domain go2w_offline_base go2w_offline_overlay
unset -f go2w_offline_source
