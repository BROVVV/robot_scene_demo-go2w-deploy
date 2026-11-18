#!/usr/bin/env bash
# Only native component gtests, on a loopback-only domain. No SDK or robot bridge.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || {
  echo 'This entry requires native aarch64 ROS 2 Foxy.' >&2
  exit 2
}
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH CYCLONEDDS_URI
export PYTHONNOUSERSITE=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/foxy/setup.bash
source "$runtime/install/20261106_nav2_compat/local_setup.bash"
set -u
mkdir -p "$runtime/tests"
dds="$runtime/tests/nav2_compat_loopback99.xml"
cat > "$dds" <<'XML'
<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
XML
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$dds"
source_root="$root/compat/foxy_nav2/src"
cd "$root/compat/foxy_nav2"
colcon --log-base "$runtime/logs/nav2_compat_test" test \
  --executor sequential \
  --base-paths "$source_root/go2w_nav2_compat" "$source_root/nav2_velocity_smoother" "$source_root/nav2_collision_monitor" \
  --packages-select go2w_nav2_compat nav2_velocity_smoother nav2_collision_monitor \
  --build-base "$runtime/build/20261106_nav2_compat" \
  --install-base "$runtime/install/20261106_nav2_compat" \
  --return-code-on-test-failure --event-handlers console_direct+ \
  --ctest-args -R '^(test_required_parameters|test_footprint_timestamp|kinematics_test|sources_test|polygons_test|collision_monitor_node_test|velocity_smoother_tests)$'
colcon --log-base "$runtime/logs/nav2_compat_test_result" test-result \
  --test-result-base "$runtime/build/20261106_nav2_compat" --verbose
