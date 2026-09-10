#!/usr/bin/env bash
# Vendor plain_slam_ros2 into ros2_ws/src and pin its commit (plan §3.2 / §19).
#
# Also ensures the build dependencies (Sophus, nanoflann >= 1.6.0) are present
# in an idempotent way.  Run BEFORE build_ros2.sh:
#
#   bash scripts/go2w/vendor_plain_slam_ros2.sh            # latest upstream
#   bash scripts/go2w/vendor_plain_slam_ros2.sh --commit <SHA>  # pinned
#
# LICENSE: plain_slam_ros2 is free for academic/personal use; commercial use
# requires the author's written permission (docs/go2w/plain_slam_license_note.md).
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
src_dir="${project_root}/ros2_ws/src"
plain_slam_dir="${src_dir}/plain_slam_ros2"
lock_file="${project_root}/configs/go2w/plain_slam_lock.yaml"
repo_url="https://github.com/NaokiAkai/plain_slam_ros2.git"

pin_commit=""
sophus_tag="v0.9.5"
nanoflann_tag="v1.6.3"
while (( $# )); do
  case "$1" in
    --commit) pin_commit="${2:-}"; shift 2 ;;
    *) printf 'ERROR: unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if ! command -v git >/dev/null 2>&1; then
  printf 'ERROR: git is required.\n' >&2
  exit 2
fi

# sudo helper: SUDO_PASSWORD (plain text) enables passwordless-ish installs
# in non-interactive shells; otherwise sudo runs interactively.
run_sudo() {
  if [[ -n "${SUDO_PASSWORD:-}" ]]; then
    printf '%s\n' "${SUDO_PASSWORD}" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}

# ---------------------------------------------------------------------------
# 1. fetch / update the upstream source
# ---------------------------------------------------------------------------
if [[ ! -d "${plain_slam_dir}/.git" ]]; then
  printf 'Cloning %s ...\n' "${repo_url}"
  git clone "${repo_url}" "${plain_slam_dir}"
fi

commit=""
if [[ -n "${pin_commit}" ]]; then
  git -C "${plain_slam_dir}" fetch origin --quiet
  git -C "${plain_slam_dir}" checkout --quiet "${pin_commit}"
  commit="$(git -C "${plain_slam_dir}" rev-parse HEAD)"
else
  git -C "${plain_slam_dir}" fetch origin --quiet || true
  git -C "${plain_slam_dir}" pull --ff-only --quiet 2>/dev/null || true
  commit="$(git -C "${plain_slam_dir}" rev-parse HEAD)"
fi
printf 'plain_slam_ros2 pinned at %s\n' "${commit}"

# Apply recorded upstream patches (idempotent; plan §3.1 requires patches to
# be tiny and documented under patches/plain_slam_ros2/).
patch_dir="${project_root}/patches/plain_slam_ros2"
if [[ -d "${patch_dir}" ]]; then
  for patch in "${patch_dir}"/*.patch; do
    [[ -f "${patch}" ]] || continue
    if git -C "${plain_slam_dir}" apply --check "${patch}" 2>/dev/null; then
      git -C "${plain_slam_dir}" apply "${patch}"
      printf '[OK] applied patch %s\n' "$(basename "${patch}")"
    else
      printf '[OK] patch %s already applied (skipped)\n' "$(basename "${patch}")"
    fi
  done
fi

# ---------------------------------------------------------------------------
# 2. record the lock (fill placeholders; never guess a SHA)
# ---------------------------------------------------------------------------
if grep -q 'PENDING_VENDOR_FILL_ME' "${lock_file}"; then
  sed -i \
    -e "s|^  commit: PENDING_VENDOR_FILL_ME|  commit: ${commit}|" \
    -e "s|tag_or_commit: PENDING_VENDOR_FILL_ME|tag_or_commit: ${sophus_tag}|" \
    -e "s|vendored_at: PENDING_VENDOR_FILL_ME|vendored_at: $(date -Iseconds)|" \
    "${lock_file}"
fi
printf 'Lock file: %s\n' "${lock_file}"

# ---------------------------------------------------------------------------
# 3. Sophus (idempotent)
# ---------------------------------------------------------------------------
if [[ -d /usr/local/include/sophus ]] || [[ -d /usr/include/sophus ]]; then
  printf '[OK] Sophus already installed; skipping.\n'
else
  printf 'Installing Sophus %s into /usr/local ...\n' "${sophus_tag}"
  tmp_dir="$(mktemp -d)"
  git clone --depth 1 --branch "${sophus_tag}" \
    https://github.com/strasdat/Sophus.git "${tmp_dir}/Sophus"
  cmake -S "${tmp_dir}/Sophus" -B "${tmp_dir}/Sophus/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_EXAMPLES=OFF \
    -DBUILD_SOPHUS_TESTS=OFF
  run_sudo cmake --install "${tmp_dir}/Sophus/build"
  rm -rf "${tmp_dir}"
  printf '[OK] Sophus installed.\n'
fi

# ---------------------------------------------------------------------------
# 4. nanoflann >= 1.6 (idempotent; apt on 22.04 may be too old)
# ---------------------------------------------------------------------------
installed_header=""
for candidate in /usr/local/include/nanoflann.hpp /usr/include/nanoflann.hpp; do
  if [[ -f "${candidate}" ]]; then
    version_hex="$(grep -o 'define NANOFLANN_VERSION 0x[0-9a-fA-F]*' \
      "${candidate}" | grep -o '0x[0-9a-fA-F]*' | head -1)"
    version_hex="${version_hex:-0x0}"
    if (( version_hex >= 0x160 )); then
      installed_header="${candidate}"
      printf '[OK] nanoflann >= 1.6 found: %s (0x%x)\n' \
        "${candidate}" "${version_hex}"
      break
    fi
    printf 'nanoflann %s is too old (< 1.6); upgrading to %s in /usr/local\n' \
      "${candidate}" "${nanoflann_tag}"
  fi
done
if [[ -n "${installed_header}" ]]; then
  printf '[OK] nanoflann header found: %s (skipping)\n' "${installed_header}"
else
  printf 'Installing nanoflann %s into /usr/local ...\n' "${nanoflann_tag}"
  tmp_dir="$(mktemp -d)"
  git clone --depth 1 --branch "${nanoflann_tag}" \
    https://github.com/jlblancoc/nanoflann.git "${tmp_dir}/nanoflann"
  run_sudo install -D -m 644 "${tmp_dir}/nanoflann/include/nanoflann.hpp" \
    /usr/local/include/nanoflann.hpp
  rm -rf "${tmp_dir}"
  printf '[OK] nanoflann installed.\n'
fi

printf '%s\n' 'Vendoring complete. Run: bash scripts/go2w/build_ros2.sh'