#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${UNITREE_PROJECT_ROOT:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
LOG_ROOT="$PROJECT_ROOT/logs"
REPORT_ROOT="$PROJECT_ROOT/reports"

ensure_layout() {
  mkdir -p "$PROJECT_ROOT/scripts" "$PROJECT_ROOT/config" "$LOG_ROOT" "$REPORT_ROOT"
  chmod 700 "$LOG_ROOT"
}

new_log_dir() {
  ensure_layout
  RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  LOG_DIR="$LOG_ROOT/$RUN_ID"
  mkdir -p "$LOG_DIR"
  chmod 700 "$LOG_DIR"
  export RUN_ID LOG_DIR
}

log() {
  printf '[%s] %s\n' "$(date --iso-8601=seconds)" "$*"
}

die() {
  log "ERROR: $*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

official_example_bin() {
  local name="$1"
  local unitree_root="${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}"
  local base="$unitree_root/example/install/unitree_ros2_example"
  if [[ -x "$base/bin/$name" ]]; then
    printf '%s\n' "$base/bin/$name"
  elif [[ -x "$base/$name" ]]; then
    printf '%s\n' "$base/$name"
  else
    return 1
  fi
}
