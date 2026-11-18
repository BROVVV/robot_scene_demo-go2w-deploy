#!/usr/bin/env bash
# Detached owner for the WebUI "clear objects" / "clear map" operations.
# It never sends a motion command.  Active data is archived before a restart.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
cd "${project_root}"

action=""
mode=""
detector=""
lock_file="${project_root}/runtime/go2w/webui_reset.lock"
while (( $# )); do
  case "$1" in
    --action) action="${2:?--action requires objects or map}"; shift 2 ;;
    --mode) mode="${2:?--mode requires autonomous_search or teleop_mapping}"; shift 2 ;;
    --detector) detector="${2:?--detector requires grounded_sam or yoloe_seg}"; shift 2 ;;
    --lock-file) lock_file="${2:?--lock-file requires a path}"; shift 2 ;;
    *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done

[[ "$action" == objects || "$action" == map ]] || { echo 'ERROR: invalid action' >&2; exit 2; }
[[ "$mode" == autonomous_search || "$mode" == teleop_mapping ]] || { echo 'ERROR: invalid mode' >&2; exit 2; }
[[ "$detector" == grounded_sam || "$detector" == yoloe_seg ]] || { echo 'ERROR: invalid detector' >&2; exit 2; }

mapping_runtime="${project_root}/outputs/remote_control_mapping/runtime"
normal_markers=(
  "${mapping_runtime}/localization_2d_started_by_remote_mapping"
  "${mapping_runtime}/wheel_odom_started_by_remote_mapping"
  "${mapping_runtime}/lidar_preprocessor_started_by_remote_mapping"
)
preserved_markers=(
  "${normal_markers[0]}.reset_preserved"
  "${normal_markers[1]}.reset_preserved"
  "${normal_markers[2]}.reset_preserved"
)

restore_preserved_markers() {
  local index
  for index in "${!normal_markers[@]}"; do
    if [[ -f "${preserved_markers[$index]}" && ! -e "${normal_markers[$index]}" ]]; then
      mv "${preserved_markers[$index]}" "${normal_markers[$index]}" || true
    fi
  done
}

cleanup() {
  # If an object-only restart failed after temporarily hiding ownership, put
  # every marker back so the next explicit stop still owns the same 2D stack.
  restore_preserved_markers
  rm -f -- "$lock_file"
}
trap cleanup EXIT

sleep 1.0  # let the accepting HTTP response reach the browser first

reset_stamp="$(date +%Y%m%d_%H%M%S)"
archive_dir="${project_root}/outputs/runtime_resets/${reset_stamp}_${mode}_${action}"
mkdir -p "$archive_dir"

archive_file() {
  local source="$1"
  if [[ -f "$source" ]]; then
    cp -a -- "$source" "$archive_dir/"
  fi
}

archive_mapping_objects() {
  local base="${project_root}/outputs/remote_control_mapping/runtime"
  archive_file "$base/semantic_objects.json"
  archive_file "$base/semantic_topology.json"
  archive_file "$base/target_watch_request.json"
  archive_file "$base/target_watch_state.json"
  archive_file "$base/events.jsonl"
  archive_file "$base/status.json"
}

archive_mapping_map() {
  local base="${project_root}/outputs/remote_control_mapping/runtime"
  archive_file "$base/slam_map_3d.json"
  archive_file "$base/slam_map_meta.json"
  archive_mapping_objects
}

archive_search_state() {
  local base="${project_root}/outputs/autonomous_search/runtime"
  archive_file "$base/slam_map_3d.json"
  archive_file "$base/slam_map_meta.json"
  archive_file "$base/semantic_objects.json"
  archive_file "$base/semantic_topology.json"
  archive_file "$base/search_state.json"
}

clear_mapping_objects() {
  local base="${project_root}/outputs/remote_control_mapping/runtime"
  rm -f -- \
    "$base/semantic_objects.json" \
    "$base/semantic_topology.json" \
    "$base/target_watch_request.json" \
    "$base/target_watch_state.json" \
    "$base/events.jsonl" \
    "$base/status.json"
}

clear_mapping_map() {
  local base="${project_root}/outputs/remote_control_mapping/runtime"
  rm -f -- "$base/slam_map_3d.json" "$base/slam_map_meta.json"
  clear_mapping_objects
}

clear_search_objects() {
  local base="${project_root}/outputs/autonomous_search/runtime"
  rm -f -- \
    "$base/semantic_objects.json" \
    "$base/semantic_topology.json" \
    "$base/search_state.json"
}

clear_search_map() {
  local base="${project_root}/outputs/autonomous_search/runtime"
  rm -f -- "$base/slam_map_3d.json" "$base/slam_map_meta.json"
  clear_search_objects
}

printf '\n[%s] reset action=%s mode=%s detector=%s\n' \
  "$(date -Is)" "$action" "$mode" "$detector"

if [[ "$mode" == teleop_mapping ]]; then
  if [[ "$action" == objects ]]; then
    # Keep the exact 2D SLAM, wheel odometry and lidar-preprocessor owners
    # alive while only semantic perception and the WebUI restart.
    for preserved_marker in "${preserved_markers[@]}"; do
      if [[ -e "$preserved_marker" ]]; then
        echo "ERROR: stale preserved ownership marker exists: $preserved_marker" >&2
        exit 3
      fi
    done
    for index in "${!normal_markers[@]}"; do
      if [[ -f "${normal_markers[$index]}" ]]; then
        mv "${normal_markers[$index]}" "${preserved_markers[$index]}"
      fi
    done
    bash "${script_dir}/stop_remote_control_mapping.sh"
    archive_mapping_objects
    clear_mapping_objects
    bash "${script_dir}/start_remote_control_mapping.sh" \
      --map-mode 2d --detector "$detector" --no-browser
    restore_preserved_markers
  else
    bash "${script_dir}/stop_remote_control_mapping.sh"
    # The mapper may have been reused rather than started by teleop mode.
    # Stop only the exact project-owned localization group if it still exists.
    bash "${script_dir}/stop_2d_localization.sh"
    archive_mapping_map
    clear_mapping_map
    # Let DDS remove the just-stopped wheel-odom/lidar/map endpoints before the
    # launcher decides whether to reuse or own them.  Without this window a
    # dead endpoint can be counted as live and the fresh mapping scan waits for
    # odometry forever.
    sleep 3
    bash "${script_dir}/start_remote_control_mapping.sh" \
      --map-mode 2d --detector "$detector" --no-browser
  fi
else
  stop_failed=0
  bash "${script_dir}/stop_autonomous_search_web.sh" || {
    printf 'ERROR: WebUI stop failed.\n' >&2
    stop_failed=1
  }
  bash "${script_dir}/stop_nav2_autonomous_search.sh" || {
    printf 'ERROR: Nav2 stop failed.\n' >&2
    stop_failed=1
  }
  if [[ "$action" == map ]]; then
    bash "${script_dir}/stop_2d_localization.sh" || {
      printf 'ERROR: 2D localization stop failed.\n' >&2
      stop_failed=1
    }
  fi
  # Try every layer in this reset's scope, but never erase data or launch a
  # replacement while an old Web/TF consumer may still be alive.
  if (( stop_failed )); then
    printf 'ERROR: runtime stop incomplete; current data retained, restart refused.\n' >&2
    exit 1
  fi
  archive_search_state
  if [[ "$action" == map ]]; then
    clear_search_map
  else
    clear_search_objects
  fi
  # Runtime resets intentionally return to plan-only.  Re-authorizing physical
  # motion after a process restart must remain an explicit operator action.
  bash "${script_dir}/start_nav2_autonomous_search.sh" \
    "--detector=${detector}" --no-browser
fi

printf '[%s] reset complete; archive=%s\n' "$(date -Is)" "$archive_dir"
