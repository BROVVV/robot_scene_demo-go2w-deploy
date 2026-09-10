#!/usr/bin/env bash
set -eo pipefail

# This deployment never acquired a motion lease, so stopping means canceling
# project-owned host workers and keeping every execution gate disabled. It does
# not transmit StopMove or any other robot command.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
pid_root="${project_root}/runtime/go2w/pids"
mkdir -p "${pid_root}"
touch "${project_root}/runtime/go2w/CANCEL_SEARCH" \
  "${project_root}/runtime/go2w/CANCEL_NAV2" \
  "${project_root}/runtime/go2w/EXECUTION_DISABLED"

stopped=0
for pid_file in "${pid_root}"/*.pid; do
  [[ -e "${pid_file}" ]] || continue
  process_id="$(tr -cd '0-9' < "${pid_file}")"
  if [[ -n "${process_id}" ]] && kill -0 "${process_id}" 2>/dev/null; then
    kill -TERM -- "-${process_id}" 2>/dev/null || kill -TERM "${process_id}" 2>/dev/null || true
    stopped=$((stopped + 1))
  fi
  rm -f "${pid_file}"
done

printf 'Stopped %d project-owned host process group(s).\n' "${stopped}"
printf '%s\n' 'Search/Nav2 cancellation flags written; execution remains disabled.'
printf '%s\n' 'No robot motion/StopMove command was transmitted by this host-only script.'
printf '%s\n' 'This script does not claim or stop an externally managed Sport lease.'
printf '%s\n' 'Session logs were preserved.'
