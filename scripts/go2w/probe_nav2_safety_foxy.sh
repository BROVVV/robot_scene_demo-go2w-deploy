#!/usr/bin/env bash
# Synthetic smoother -> collision -> final guard; pinned profile, no SDK/bridge.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || exit 2
[[ $# == 1 || ( $# == 2 && "$2" == --with-controller ) ]] || { echo 'Usage: probe_nav2_safety_foxy.sh NEW_OUTPUT_DIRECTORY [--with-controller]' >&2; exit 2; }
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH CYCLONEDDS_URI
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set -a
source "$root/configs/go2w/jetson_foxy.env"
set +a
source "$root/scripts/go2w/setup_environment.sh"
export PYTHONPATH="$root:$GO2W_ROS_APP_SITE_PACKAGES:${PYTHONPATH:-}"
mkdir -p "$runtime/tests"
dds="$runtime/tests/nav2_chain_loopback99.xml"
cat > "$dds" <<'XML'
<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
XML
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$dds"
exec /usr/bin/python3 "$root/compat/foxy_nav2/probes/velocity_chain.py" --output-dir "$1" "${@:2}"
