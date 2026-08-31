#!/usr/bin/env bash
set -Eeuo pipefail

# Rebuild the missing Go2-W motion workspace on a fresh Ubuntu workstation.
# This script is intentionally non-motion: it only clones/builds software and
# never starts a ROS node or sends a command to the robot.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
control_root="${GO2W_CONTROL_ROOT:-${project_root}/unitree_go2w_control}"
unitree_root="${GO2W_UNITREE_ROOT:-${HOME}/unitree_ros2}"
unitree_url="https://github.com/unitreerobotics/unitree_ros2.git"

# ROSIDL/colcon must run with the Ubuntu Humble system Python.  A parent
# Conda/ROS1 shell otherwise makes CMake invoke the wrong interpreter (and
# hides Humble's python3-empy module).
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_ROOT ROS_PACKAGE_PATH
unset ROS_MASTER_URI ROSLISP_PACKAGE_DIRECTORIES AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL PYTHONHOME PYTHONPATH
unset LD_LIBRARY_PATH CMAKE_PREFIX_PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

[[ -f /opt/ros/humble/setup.bash ]] || die \
  'ROS 2 Humble is required; install it before running this script.'
command -v git >/dev/null 2>&1 || die 'git is required.'
command -v colcon >/dev/null 2>&1 || die 'colcon is required; run install_dependencies.sh first.'
[[ -x /usr/bin/python3 ]] || die '/usr/bin/python3 is required for ROS 2 Humble.'
[[ -d "${control_root}/ros2_ws/src/go2w_motion_control" ]] || die \
  "bundled control source is missing: ${control_root}"

if [[ ! -d "${unitree_root}/.git" ]]; then
  if [[ -e "${unitree_root}" ]]; then
    die "${unitree_root} exists but is not the official unitree_ros2 Git repository."
  fi
  printf 'Cloning official Unitree ROS 2 support into %s ...\n' "${unitree_root}"
  git clone --depth 1 "${unitree_url}" "${unitree_root}"
fi

origin="$(git -C "${unitree_root}" remote get-url origin 2>/dev/null || true)"
case "${origin}" in
  https://github.com/unitreerobotics/unitree_ros2.git|\
  https://github.com/unitreerobotics/unitree_ros2|\
  git@github.com:unitreerobotics/unitree_ros2.git) ;;
  *) die "unexpected unitree_ros2 origin: ${origin:-missing}" ;;
esac

export GO2W_CONTROL_ROOT="${control_root}"
export GO2W_UNITREE_ROOT="${unitree_root}"
export PYTHONPATH="${control_root}/vendor/unitree_sdk2_python${PYTHONPATH:+:${PYTHONPATH}}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

printf 'Building official Unitree ROS 2 message packages ...\n'
(
  cd "${unitree_root}/cyclonedds_ws"
  colcon build --symlink-install --cmake-clean-cache --event-handlers console_direct+
)

set +u
# shellcheck disable=SC1090
source "${unitree_root}/cyclonedds_ws/install/setup.bash"
set -u

printf 'Building bundled Go2-W motion interfaces and controller ...\n'
(
  cd "${control_root}/ros2_ws"
  colcon build --symlink-install --cmake-clean-cache --event-handlers console_direct+ \
    --packages-select go2w_motion_interfaces go2w_motion_control \
    --cmake-args \
      -DPython3_EXECUTABLE=/usr/bin/python3 \
      -DPYTHON_EXECUTABLE=/usr/bin/python3
)

# The Python SDK is bundled under the BSD-licensed upstream source tree. Keep
# it in a project-local venv so it does not alter the workstation's system
# Python. The launch file can also use PYTHONPATH when this venv is absent.
venv="${control_root}/.venv"
venv_needs_recreate="false"
if [[ ! -x "${venv}/bin/python" || ! -x "${venv}/bin/pip" || \
      ! -f "${venv}/pyvenv.cfg" || \
      "$(grep -E '^include-system-site-packages = ' "${venv}/pyvenv.cfg" 2>/dev/null || true)" \
        != "include-system-site-packages = true" ]]; then
  venv_needs_recreate="true"
fi
if [[ "${venv_needs_recreate}" == true ]]; then
  if [[ -e "${venv}" ]]; then
    rm -rf "${venv}"
  fi
  /usr/bin/python3 -m venv --system-site-packages "${venv}"
fi

# Some development hosts export a SOCKS proxy without installing the Python
# SOCKS adapter. Try the normal environment first, then retry without proxy
# variables so a direct connection can still bootstrap the SDK. A Python 3.10
# wheel is available for cyclonedds; wheel itself is installed first because
# the upstream SDK uses a legacy setup.py editable install.
pip_install() {
  if ! "${venv}/bin/python" -m pip "$@"; then
    env -u ALL_PROXY -u all_proxy -u HTTPS_PROXY -u https_proxy \
      -u HTTP_PROXY -u http_proxy \
      "${venv}/bin/python" -m pip "$@"
  fi
}
pip_install install wheel
pip_install install --no-build-isolation -e \
  "${control_root}/vendor/unitree_sdk2_python"

printf '\nGo2-W control deployment complete.\n'
printf 'Control root: %s\n' "${control_root}"
printf 'Unitree ROS 2 root: %s\n' "${unitree_root}"
printf 'Action/service package is built; no robot process was started.\n'
printf '\nTo start the real control server, with the robot area cleared and the remote in hand:\n'
printf '  source %q\n' "${control_root}/scripts/setup_go2w_ros2.sh"
printf '  bash %q\n' "${project_root}/scripts/go2w/start_motion_control.sh"
