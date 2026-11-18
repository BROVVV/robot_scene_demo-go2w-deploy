#!/usr/bin/env bash
# Independent 2D SLAM/AMCL. This entry point never starts motion nodes.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/../.." && pwd)"
mode=mapping
map_file=""
current_pose=()
while (($#)); do
  case "$1" in
    --mode) mode="${2:?--mode requires mapping or localization}"; shift ;;
    --map) map_file="${2:?--map requires an absolute YAML path}"; shift ;;
    --initial-pose)
      (($# >= 4)) || { printf '%s\n' '--initial-pose requires X Y YAW in map frame (radians)' >&2; exit 2; }
      current_pose=("$2" "$3" "$4"); shift 3 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
  shift
done
[[ "$mode" == mapping || "$mode" == localization ]] || exit 2
if [[ "$mode" == localization ]]; then
  if [[ ${#current_pose[@]} != 3 ]]; then
    printf '%s\n' 'Localization requires --initial-pose X Y YAW for the current robot position. The pose at map save time is historical.' >&2
    exit 2
  fi
  if ! pose="$(/usr/bin/python3 - "${current_pose[@]}" <<'PY'
import math, sys
try:
    values = [float(v) for v in sys.argv[1:]]
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite initial pose')
except ValueError:
    print('Initial pose must contain three finite numbers.', file=sys.stderr)
    sys.exit(2)
print(*values)
PY
  )"; then
    exit 2
  fi
  read -r initial_pose_x initial_pose_y initial_pose_yaw <<<"$pose"
fi
source "$script_dir/setup_environment.sh"
runtime="$project_root/outputs/autonomous_search/runtime"
logs="$project_root/outputs/autonomous_search/logs"
mkdir -p "$runtime" "$logs"
sensor_profile="$project_root/configs/go2w/navigation_sensor_profile.yaml"
sensor_profile_id=""
if [[ -r "$sensor_profile" ]]; then
  sensor_profile_id="sha256:$(sha256sum "$sensor_profile" | awk '{print $1}')"
fi
# Reuse only our exact launch. Never erase a map to make a startup check green.
if [[ -f "$runtime/localization_2d.pid" ]]; then
  pid="$(<"$runtime/localization_2d.pid")"
  if [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]]; then
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline")"
    mapfile -d '' -t process_arguments <"/proc/$pid/cmdline"
    has_owned_argument() {
      local argument
      for argument in "${process_arguments[@]}"; do
        [[ "$argument" == "$1" ]] && return 0
      done
      return 1
    }
    if [[ "$cmd" == *"localization_2d.launch.py mode:=$mode"* ]]; then
      if [[ -n "$sensor_profile_id" ]] && ! has_owned_argument "sensor_profile_id:=$sensor_profile_id"; then
        printf 'A different or unversioned sensor profile is active; stop its owned launch before switching.\n' >&2
        exit 2
      fi
      /usr/bin/python3 "$script_dir/localization_epoch.py" verify --pid "$pid"
      if [[ "$mode" == localization ]] && ! has_owned_argument "map:=$map_file"; then
        printf 'A different saved map is active; stop its owned launch before switching.\n' >&2
        exit 2
      fi
      if [[ "$mode" == localization ]]; then
        for assignment in "initial_pose_x:=$initial_pose_x" "initial_pose_y:=$initial_pose_y" "initial_pose_yaw:=$initial_pose_yaw"; do
          if ! has_owned_argument "$assignment"; then
            printf 'A different initial pose is active; stop its owned launch before reseeding AMCL.\n' >&2
            exit 2
          fi
        done
      fi
      printf 'Reusing project-owned 2D localization (%s, pid %s).\n' "$mode" "$pid"
      exit 0
    fi
    printf 'Localization pidfile points to another active process; inspect it before switching.\n' >&2
    exit 2
  fi
fi
[[ -n "$sensor_profile_id" ]] || { printf 'Missing navigation sensor profile: %s\n' "$sensor_profile" >&2; exit 2; }
# A remaining map publisher may belong to another operator. Do not take it over.
count="$(timeout 5 ros2 topic info /map 2>/dev/null | awk '/Publisher count:/ {print $3}' || true)"
if [[ "${count:-0}" != 0 ]]; then
  printf 'Existing /map publisher found; refusing a second mapping/localization owner.\n' >&2
  exit 2
fi
# Wheel odometry must own exactly the navigation TF; LIO correction stays isolated.
for pair in 'publish_tf True' 'lio_enabled False'; do
  read -r parameter expected <<<"$pair"
  # Foxy CLI startup on Jetson can exceed five seconds under camera/ML load.
  value="$(timeout 12 ros2 param get /go2w_wheel_odom "$parameter" 2>/dev/null || true)"
  if [[ "$value" != *": $expected" ]]; then
    printf 'Wheel odometry requires %s=%s; launch wheel_odom.launch.py first.\n' "$parameter" "$expected" >&2
    exit 2
  fi
done
map_authorized=false
initial_pose_args=()
map_args=()
if [[ "$mode" == localization ]]; then
  # Authorization is bound to the saved artifacts, never just a launch flag.
  /usr/bin/python3 "$script_dir/save_navigation_map.py" --verify "$map_file" --require-authorized \
    --sensor-profile "$project_root/configs/go2w/navigation_sensor_profile.yaml"
  map_authorized=true
  map_args=("map:=$map_file")
  # A saved pose records where the robot WAS. Teleoperation, carrying or a
  # restart may change it. Only an explicit current estimate seeds AMCL.
  printf 'Current initial pose estimate: x=%s y=%s yaw=%s\n' \
    "$initial_pose_x" "$initial_pose_y" "$initial_pose_yaw"
  initial_pose_args=("use_initial_pose:=true"
                     "initial_pose_x:=$initial_pose_x"
                     "initial_pose_y:=$initial_pose_y"
                     "initial_pose_yaw:=$initial_pose_yaw")
fi
setsid ros2 launch robot_scene_nav_bringup localization_2d.launch.py \
  "mode:=$mode" "${map_args[@]}" "map_authorized:=$map_authorized" \
  "sensor_profile_id:=$sensor_profile_id" \
  "${initial_pose_args[@]}" \
  >"$logs/localization_2d.log" 2>&1 &
launch_pid=$!
printf '%s\n' "$launch_pid" >"$runtime/localization_2d.pid"
# Catch immediate launch/argument failures before reporting success. Readiness
# (map/TF and localization evidence) is checked by the caller's runtime audit.
for _ in {1..5}; do
  sleep .2
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid" || true
    rm -f "$runtime/localization_2d.pid"
    printf '2D %s launch exited during startup; inspect %s\n' "$mode" "$logs/localization_2d.log" >&2
    tail -n 20 "$logs/localization_2d.log" >&2
    exit 2
  fi
done
if ! /usr/bin/python3 "$script_dir/localization_epoch.py" record --pid "$launch_pid"; then
  printf 'Failed to bind localization to the current robot/odometry epoch; stopping owned localization.\n' >&2
  bash "$script_dir/stop_2d_localization.sh"
  exit 2
fi
printf 'Started independent 2D %s, pid %s.\n' "$mode" "$launch_pid"
