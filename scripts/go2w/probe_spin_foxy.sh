#!/usr/bin/env bash
# Synthetic duration-bounded Spin test. No robot profile or SDK.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || exit 2
[[ $# == 1 ]] || { echo 'Usage: probe_spin_foxy.sh NEW_OUTPUT_DIRECTORY' >&2; exit 2; }
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH CYCLONEDDS_URI
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/foxy/setup.bash
source "$runtime/install/ros2_ws/local_setup.bash"
source "$runtime/install/20261104_01/local_setup.bash"
source "$runtime/install/20261106_nav2_compat/local_setup.bash"
source "$runtime/install/20261106_spin_compat/local_setup.bash"
set -u
mkdir -p "$runtime/tests"
dds="$runtime/tests/spin_chain_loopback99.xml"
cat > "$dds" <<'XML'
<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
XML
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$dds"
exec /usr/bin/python3 "$root/compat/foxy_nav2/probes/spin.py" --output-dir "$1"
