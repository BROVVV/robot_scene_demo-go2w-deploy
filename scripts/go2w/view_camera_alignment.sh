#!/usr/bin/env bash
set -eo pipefail

# Read-only camera preview. It starts no Sport, lease, cmd_vel, or Nav2 node.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
camera_pid=""

set +u
source /opt/ros/humble/setup.bash
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
source "${project_root}/ros2_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${project_root}/configs/go2w/cyclonedds_go2w.xml"
export GDK_BACKEND="${GDK_BACKEND:-x11}"

if [[ ! -r /sys/class/net/enp6s0/carrier ]] \
  || [[ "$(< /sys/class/net/enp6s0/carrier)" != "1" ]]; then
  printf '%s\n' 'ERROR: enp6s0 has no Ethernet carrier.' >&2
  exit 2
fi

cleanup() {
  if [[ -n "$camera_pid" ]] && kill -0 -- "-$camera_pid" 2>/dev/null; then
    kill -INT -- "-$camera_pid" 2>/dev/null || true
    for _ in {1..20}; do kill -0 -- "-$camera_pid" 2>/dev/null || break; sleep 0.1; done
    kill -TERM -- "-$camera_pid" 2>/dev/null || true
    for _ in {1..20}; do kill -0 -- "-$camera_pid" 2>/dev/null || break; sleep 0.1; done
    kill -KILL -- "-$camera_pid" 2>/dev/null || true
    wait "$camera_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ros2 node list 2>/dev/null | grep -qx '/go2w_camera_bridge'; then
  printf '%s\n' 'Using existing read-only camera bridge.'
else
  setsid ros2 run go2w_camera_bridge camera_bridge --ros-args \
    -p source:=rpc -p interface:=enp6s0 \
    -p "calibration_file:=${project_root}/configs/go2w/camera_intrinsics.yaml" \
    >"${project_root}/runtime/go2w/camera_alignment_bridge.log" 2>&1 &
  camera_pid=$!
fi

publisher_ready=false
for _ in {1..150}; do
  topic_info="$(ros2 topic info /camera/front/image_raw 2>/dev/null || true)"
  if grep -Eq '^Publisher count: [1-9][0-9]*$' <<<"$topic_info"; then
    publisher_ready=true
    break
  fi
  sleep 0.1
done
if [[ "$publisher_ready" != true ]]; then
  printf '%s\n' 'ERROR: camera publisher was not discovered.' >&2
  exit 2
fi

printf '%s\n' 'Opening read-only alignment viewer. Press Q or Esc in the window to close.'
/usr/bin/python3 "${script_dir}/view_camera_alignment_ros.py" --board 9x6
