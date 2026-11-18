#!/usr/bin/env bash
# Native Nav2/health/pose-reader TF rebuild; loopback synthetic inputs only.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
runtime="${GO2W_RUNTIME_ROOT:-/home/unitree/robotscene_runtime}"
[[ "$(uname -m)" == aarch64 && -f /opt/ros/foxy/setup.bash ]] || exit 2
[[ $# == 1 ]] || { echo 'Usage: probe_tf_chain_rebuild_foxy.sh NEW_OUTPUT_DIRECTORY' >&2; exit 2; }
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH CYCLONEDDS_URI
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
# The release profile pins reused base and compatibility overlays; their
# directory names need not match this source release. No ROS node is created
# until after the domain and DDS interface are replaced below.
set -a
source "$root/configs/go2w/jetson_foxy.env"
set +a
source "$root/scripts/go2w/setup_environment.sh"
export PYTHONPATH="$root:$GO2W_ROS_APP_SITE_PACKAGES:${PYTHONPATH:-}"
mkdir -p "$runtime/tests"
dds="$runtime/tests/tf_chain_loopback99.xml"
cat > "$dds" <<'XML'
<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
XML
export GO2W_BT_ACK_DIAGNOSTICS=1
export GO2W_FOXY_SPIN_ACTION=1
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$dds"
exec /usr/bin/python3 "$root/compat/foxy_nav2/probes/tf_chain_rebuild.py" --output-dir "$1"
