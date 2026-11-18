#!/usr/bin/env bash
set -eo pipefail

# Stop project-owned host workers. This script sends host process signals,
# not robot API commands; an externally managed Sport lease is outside scope.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
pid_root="${project_root}/runtime/go2w/pids"
mkdir -p "${pid_root}"
touch "${project_root}/runtime/go2w/CANCEL_SEARCH" \
  "${project_root}/runtime/go2w/CANCEL_NAV2" \
  "${project_root}/runtime/go2w/EXECUTION_DISABLED"

stop_failed=0
host_stop="${script_dir}/stop_owned_host_processes.py"
if [[ -r "${host_stop}" ]]; then
  /usr/bin/python3 "${host_stop}" "${project_root}" || {
    printf 'ERROR: owned host-worker stop is incomplete; live or unverified records retained.\n' >&2
    stop_failed=1
  }
else
  printf 'ERROR: %s not readable; host workers were NOT stopped.\n' "${host_stop}" >&2
  stop_failed=1
fi

# The three flag files above are written by this script and read by nothing:
# `grep -rn CANCEL_NAV2\|CANCEL_SEARCH\|EXECUTION_DISABLED` only ever finds this
# file. They are kept for external supervisors, but this script must not claim
# they cancel anything. What actually stops the Nav2 layer is the dedicated stop
# script: this loop only reads runtime/go2w/pids/, while Nav2 bringup, the
# control arbiter and the cmd_vel bridge keep their pidfiles under
# outputs/autonomous_search/runtime/ and are invisible to it.
printf '%s\n' 'Cancel flags written to runtime/go2w/ (advisory: no process in this repo reads them).'

nav2_stop="${script_dir}/stop_nav2_autonomous_search.sh"
if [[ -r "${nav2_stop}" ]]; then
  printf '%s\n' 'Stopping the Nav2 layer (bringup / arbiter / cmd_vel bridge) ...'
  bash "${nav2_stop}" || {
    printf 'ERROR: %s reported a problem; full-stack stop is incomplete.\n' "${nav2_stop}" >&2
    stop_failed=1
  }
else
  printf 'ERROR: %s not readable; Nav2 bringup / arbiter / cmd_vel bridge were NOT stopped.\n' \
    "${nav2_stop}" >&2
  stop_failed=1
fi

# The 2D localizer is intentionally outside stop_nav2_autonomous_search.sh's
# scope and keeps its own pidfiles under outputs/autonomous_search/runtime/.
# A full-stack stop must therefore call its ownership-checking stop entry too;
# otherwise slam_toolbox/map overlay processes survive and contaminate the next
# mapping epoch even though this script reported that everything was stopped.
localization_stop="${script_dir}/stop_2d_localization.sh"
# Repository scripts are invoked through bash and may intentionally lack the
# executable bit. Existence/readability is the contract used by every caller.
if [[ -r "${localization_stop}" ]]; then
  printf '%s\n' 'Stopping the owned 2D localization / mapping layer ...'
  bash "${localization_stop}" || {
    printf 'ERROR: %s reported a problem; full-stack stop is incomplete.\n' \
      "${localization_stop}" >&2
    stop_failed=1
  }
else
  printf 'ERROR: %s not found; 2D localization / mapping was NOT stopped.\n' \
    "${localization_stop}" >&2
  stop_failed=1
fi

if (( stop_failed )); then
  exit 1
fi

printf '%s\n' 'This host-only script sent no direct robot motion/StopMove command.'
printf '%s\n' 'This script does not claim or stop an externally managed Sport lease.'
printf '%s\n' 'Session logs were preserved.'
