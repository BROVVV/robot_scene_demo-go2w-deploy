#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export UNITREE_PROJECT_ROOT="$SCRIPT_DIR"
# shellcheck source=scripts/common.sh
source "$SCRIPT_DIR/scripts/common.sh"
new_log_dir

log "Starting non-motion Unitree Go2-W validation" | tee "$LOG_DIR/main.txt"
[[ "$(. /etc/os-release; printf '%s' "$VERSION_ID")" == 22.04 ]] || die "Ubuntu 22.04 required"
[[ "$(dpkg --print-architecture)" =~ ^(amd64|arm64)$ ]] || die "unsupported architecture"

"$SCRIPT_DIR/scripts/detect_interface.sh" | tee -a "$LOG_DIR/main.txt"
"$SCRIPT_DIR/scripts/configure_network.sh" | tee -a "$LOG_DIR/main.txt"

for cmd in git curl ros2 colcon rosdep; do
  command -v "$cmd" >/dev/null 2>&1 || die "dependency missing: $cmd; install host dependencies first"
done
[[ -f /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble is not installed"

REPO="${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}"
[[ -d "$REPO/.git" ]] || die "official repository is missing: $REPO"
origin="$(git -C "$REPO" remote get-url origin)"
[[ "$origin" == 'https://github.com/unitreerobotics/unitree_ros2.git' ]] || die "repository origin is not official: $origin"

set +u
source /opt/ros/humble/setup.bash
set -u
(
  cd "$REPO/cyclonedds_ws"
  colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
) 2>&1 | tee "$LOG_DIR/cyclonedds_ws_build.txt"

set +u
source "$REPO/cyclonedds_ws/install/setup.bash"
set -u
(
  cd "$REPO/example"
  colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
) 2>&1 | tee "$LOG_DIR/example_build.txt"

"$SCRIPT_DIR/scripts/diagnose.sh" | tee -a "$LOG_DIR/main.txt"
"$SCRIPT_DIR/scripts/read_state.sh" | tee -a "$LOG_DIR/main.txt"
"$SCRIPT_DIR/scripts/stop_robot.sh" | tee -a "$LOG_DIR/main.txt"

log "Non-motion validation passed. No stand or motion command was sent." | tee -a "$LOG_DIR/main.txt"
