#!/usr/bin/env bash
set -euo pipefail

# Reproducibly builds the official Unitree Point-LIO fallback without sudo and
# without sourcing the host ROS 2 installation into the ROS 1 environment.
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
CONDA_BIN=${CONDA_BIN:-/home/brov/miniconda3/bin/conda}
ENVIRONMENT=go2w_point_lio_noetic
WORKSPACE="$PROJECT_ROOT/point_lio_ws"
SOURCE="$WORKSPACE/src/point_lio_unilidar"
PATCH_FILE="$PROJECT_ROOT/patches/go2w/point_lio_noetic_pcl115.patch"
COMMIT=18ed5976d8fab2bd8a5148c26a40692bd3c0dc91

if [[ ! -x "$CONDA_BIN" ]]; then
  printf 'conda executable not found: %s\n' "$CONDA_BIN" >&2
  exit 1
fi
CONDA_ROOT=$(cd -- "$(dirname -- "$CONDA_BIN")/.." && pwd)

if ! "$CONDA_BIN" env list | awk '{print $1}' | grep -Fxq "$ENVIRONMENT"; then
  "$CONDA_BIN" create -y -n "$ENVIRONMENT" --override-channels \
    -c conda-forge -c robostack-noetic \
    ros-noetic-ros-base ros-noetic-pcl-ros ros-noetic-tf \
    ros-noetic-cv-bridge ros-noetic-eigen-conversions catkin_tools \
    compilers cmake make ninja
else
  "$CONDA_BIN" install -y -n "$ENVIRONMENT" --override-channels \
    -c conda-forge -c robostack-noetic ros-noetic-eigen-conversions
fi

mkdir -p "$WORKSPACE/src"
if [[ ! -d "$SOURCE/.git" ]]; then
  git clone https://github.com/unitreerobotics/point_lio_unilidar.git "$SOURCE"
fi
if [[ $(git -C "$SOURCE" remote get-url origin) != \
      https://github.com/unitreerobotics/point_lio_unilidar.git ]]; then
  printf 'unexpected Point-LIO origin; refusing to alter %s\n' "$SOURCE" >&2
  exit 1
fi
git -C "$SOURCE" fetch --quiet origin "$COMMIT"
git -C "$SOURCE" checkout --detach "$COMMIT"

if git -C "$SOURCE" apply --reverse --check "$PATCH_FILE" >/dev/null 2>&1; then
  printf 'PCL 1.15 compatibility patch already applied\n'
elif git -C "$SOURCE" apply --check "$PATCH_FILE"; then
  git -C "$SOURCE" apply "$PATCH_FILE"
else
  printf 'Point-LIO source is not clean or does not match the pinned commit\n' >&2
  exit 1
fi

"$CONDA_BIN" run -n "$ENVIRONMENT" bash --noprofile --norc -c \
  "set -e; cd '$WORKSPACE'; catkin config --extend '$CONDA_ROOT/envs/$ENVIRONMENT' --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5; catkin build point_lio_unilidar --no-status"

"$CONDA_BIN" run -n "$ENVIRONMENT" rosversion -d
git -C "$SOURCE" rev-parse HEAD
