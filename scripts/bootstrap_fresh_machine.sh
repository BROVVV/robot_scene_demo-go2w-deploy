#!/usr/bin/env bash
set -Eeuo pipefail

# Reproducible, non-motion bootstrap for a fresh checkout.
# Profiles:
#   core  - Python/WebUI/mock/API vision
#   go2w  - core + ROS 2 Humble + Unitree/Hesai + project ROS workspaces
#   full  - go2w + Grounded-SAM-2 and public checkpoints (default)

profile="full"
skip_system_packages=0
for arg in "$@"; do
  case "$arg" in
    --profile=core|--profile=go2w|--profile=full) profile="${arg#*=}" ;;
    --skip-system-packages) skip_system_packages=1 ;;
    -h|--help)
      sed -n '1,18p' "$0"
      exit 0
      ;;
    *) printf 'ERROR: unknown argument: %s\n' "$arg" >&2; exit 2 ;;
  esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
venv="${project_root}/.venv"
external_root="${project_root}/external"

die() { printf 'ERROR: %s\n' "$*" >&2; exit 2; }
step() { printf '\n==> %s\n' "$*"; }

command -v git >/dev/null 2>&1 || die 'git is required'
[[ -r /etc/os-release ]] || die '/etc/os-release is unavailable'
# shellcheck disable=SC1091
source /etc/os-release
host_python="$(command -v python3)"
[[ -x /usr/bin/python3 ]] && host_python=/usr/bin/python3
if [[ "$profile" != core && ("${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04) ]]; then
  die "Go2-W profiles require Ubuntu 22.04 (found ${ID:-unknown} ${VERSION_ID:-unknown})"
fi

install_base_packages() {
  (( skip_system_packages == 1 )) && return
  step 'Installing base host packages'
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    ca-certificates curl git build-essential pkg-config \
    python3 python3-dev python3-pip python3-venv python3-setuptools python3-wheel
}

install_ros_humble_repository() {
  [[ -f /opt/ros/humble/setup.bash ]] && return
  step 'Configuring the official ROS 2 Humble apt repository'
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl gnupg lsb-release locales
  sudo add-apt-repository universe -y
  sudo locale-gen en_US en_US.UTF-8
  sudo mkdir -p /usr/share/keyrings
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
  arch="$(dpkg --print-architecture)"
  codename="$(. /etc/os-release && printf '%s' "$UBUNTU_CODENAME")"
  printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
    "$arch" "$codename" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y ros-humble-desktop python3-rosdep python3-colcon-common-extensions
}

ensure_repo_at_commit() {
  local url="$1" destination="$2" commit="$3"
  if [[ ! -d "$destination/.git" ]]; then
    [[ ! -e "$destination" ]] || die "$destination exists but is not a Git repository"
    git clone "$url" "$destination"
  fi
  origin="$(git -C "$destination" remote get-url origin 2>/dev/null || true)"
  [[ "$origin" == "$url" || "$origin" == "${url%.git}" ]] \
    || die "unexpected origin for $destination: ${origin:-missing}"
  if [[ -n "$(git -C "$destination" status --porcelain)" ]]; then
    die "$destination has local changes; refusing to replace third-party source"
  fi
  git -C "$destination" fetch --depth 1 origin "$commit"
  git -C "$destination" checkout --detach "$commit"
}

install_base_packages

step 'Creating the project Python environment'
venv_args=()
if [[ "$profile" == go2w || "$profile" == full ]]; then
  # ROS 2 Humble installs rclpy into Ubuntu's system site-packages.  Let the
  # same interpreter see both those ROS modules and the project's pip
  # dependencies so the autonomous worker cannot end up with only half of
  # its dependency set.
  venv_args+=(--system-site-packages)
fi
"$host_python" -m venv "${venv_args[@]}" "$venv"
pip_install() {
  if ! "$venv/bin/python" -m pip "$@"; then
    printf 'pip failed with the inherited proxy; retrying without proxy variables.\n' >&2
    env -u ALL_PROXY -u all_proxy -u HTTPS_PROXY -u https_proxy \
      -u HTTP_PROXY -u http_proxy \
      "$venv/bin/python" -m pip "$@"
  fi
}
pip_install install --upgrade pip setuptools wheel
pip_install install -r "${project_root}/requirements.txt"

if [[ ! -f "${project_root}/.env" ]]; then
  cp "${project_root}/.env.example" "${project_root}/.env"
  printf 'Created .env from .env.example; add SILICONFLOW_API_KEY locally when needed.\n'
fi
mkdir -p \
  "${project_root}/outputs" \
  "${project_root}/runtime/go2w/spool" \
  "${project_root}/runtime/go2w/sessions" \
  "${project_root}/runtime/go2w/status" \
  "${project_root}/runtime/go2w/locks" \
  "${project_root}/runtime/go2w/pids"

if [[ "$profile" == go2w || "$profile" == full ]]; then
  install_ros_humble_repository
  step 'Installing Go2-W/ROS host dependencies'
  bash "${project_root}/scripts/go2w/install_dependencies.sh"

  mkdir -p "$external_root"
  step 'Provisioning the pinned Hesai ROS 2 driver'
  ensure_repo_at_commit \
    https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git \
    "${project_root}/ros2_ws/src/hesai_ros_driver" \
    e7e112f0809f0eed5e3c81c55a1a0376474db234

  step 'Provisioning pinned Unitree ROS 2 support'
  ensure_repo_at_commit \
    https://github.com/unitreerobotics/unitree_ros2.git \
    "${external_root}/unitree_ros2" \
    668d1ec5a05d1c38d3306bdca7d59f2ba3581a88

  step 'Building pinned Unitree messages and bundled Go2-W motion control'
  GO2W_UNITREE_ROOT="${external_root}/unitree_ros2" \
    bash "${project_root}/scripts/go2w/setup_go2w_control.sh"

  step 'Resolving and building the project ROS 2 workspace'
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${external_root}/unitree_ros2/cyclonedds_ws/install/setup.bash"
  # shellcheck disable=SC1091
  source "${project_root}/unitree_go2w_control/ros2_ws/install/setup.bash"
  set -u
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
  rosdep install --from-paths "${project_root}/ros2_ws/src" --ignore-src -r -y \
    --rosdistro humble
  (
    cd "${project_root}/ros2_ws"
    colcon build --symlink-install --event-handlers console_direct+
  )
fi

if [[ "$profile" == full ]]; then
  step 'Provisioning pinned Grounded-SAM-2 source'
  mkdir -p "$external_root"
  grounded_root="${external_root}/Grounded-SAM-2"
  ensure_repo_at_commit \
    https://github.com/IDEA-Research/Grounded-SAM-2.git \
    "$grounded_root" \
    b7a9c29f196edff0eb54dbe14588d7ae5e3dde28
  pip_install install -e "$grounded_root"
  pip_install install --no-build-isolation -e \
    "$grounded_root/grounding_dino"
  mkdir -p "$grounded_root/checkpoints" "$grounded_root/gdino_checkpoints"
  [[ -s "$grounded_root/checkpoints/sam2.1_hiera_tiny.pt" ]] || \
    curl -fL --retry 3 -o "$grounded_root/checkpoints/sam2.1_hiera_tiny.pt" \
      https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
  [[ -s "$grounded_root/gdino_checkpoints/groundingdino_swint_ogc.pth" ]] || \
    curl -fL --retry 3 -o "$grounded_root/gdino_checkpoints/groundingdino_swint_ogc.pth" \
      https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/groundingdino_swint_ogc.pth
fi

step 'Running deployment verification'
bash "${project_root}/scripts/verify_fresh_deployment.sh" --profile="$profile"

printf '\nBootstrap complete (profile=%s).\n' "$profile"
printf 'Next: edit %s/.env, then start mock UI with:\n' "$project_root"
printf '  bash %q --mock\n' "${project_root}/scripts/go2w/start_autonomous_search_web.sh"
printf 'Real robot motion is never started by this bootstrap.\n'
