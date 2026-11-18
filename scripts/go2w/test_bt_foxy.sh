#!/usr/bin/env bash
# Native BT action tests on loopback domain 99; no controller server or SDK.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || exit 2
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH CYCLONEDDS_URI
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/foxy/setup.bash
source "$runtime/install/20261106_bt_compat/local_setup.bash"
set -u
mkdir -p "$runtime/tests"
dds="$runtime/tests/bt_compat_loopback99.xml"
cat > "$dds" <<'XML'
<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
XML
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$dds"
cd "$root/compat/foxy_nav2"
colcon --log-base "$runtime/logs/bt_compat_tests" test --executor sequential \
  --base-paths "$root/compat/foxy_nav2/src/go2w_nav2_bt_compat" \
  --packages-select go2w_nav2_bt_compat \
  --build-base "$runtime/build/20261106_bt_compat" \
  --install-base "$runtime/install/20261106_bt_compat" \
  --return-code-on-test-failure --event-handlers console_direct+ \
  --ctest-args -R '^test_bt_action_lifecycle$'
colcon --log-base "$runtime/logs/bt_compat_test_result" test-result \
  --test-result-base "$runtime/build/20261106_bt_compat" --verbose
