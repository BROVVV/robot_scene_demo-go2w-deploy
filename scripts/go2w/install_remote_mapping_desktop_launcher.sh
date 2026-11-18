#!/usr/bin/env bash
# Install/refresh the clickable launcher on this Ubuntu user's desktop.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
[[ -n "${desktop_dir}" ]] || desktop_dir="${HOME}/Desktop"
mkdir -p "${desktop_dir}"
launcher="${desktop_dir}/GO2W一键遥操建图.desktop"

template="$(mktemp)"
trap 'rm -f -- "${template}"' EXIT
cat >"${template}" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=GO2W 一键遥操建图
Comment=每次创建新二维地图；关闭窗口即停止并丢弃本轮地图
Exec=${project_root}/scripts/go2w/one_click_remote_mapping.sh
Path=${project_root}
Icon=utilities-terminal
Terminal=true
StartupNotify=true
Categories=Science;Robotics;
EOF
install -m 0755 "${template}" "${launcher}"
if command -v gio >/dev/null 2>&1; then
  # On this Ubuntu image libgvfsdbus exists but is not present in gio's
  # automatically discovered module set.  Without it, `gio set` misleadingly
  # leaves the file untrusted and DING silently refuses a desktop double-click.
  gio_modules="/usr/lib/x86_64-linux-gnu/gio/modules"
  if [[ -d "${gio_modules}" ]]; then
    GIO_EXTRA_MODULES="${gio_modules}" \
      gio set -t string "${launcher}" metadata::trusted true \
      >/dev/null 2>&1 || true
  else
    gio set -t string "${launcher}" metadata::trusted true \
      >/dev/null 2>&1 || true
  fi
fi
chmod 0755 "${launcher}"
printf '桌面启动器已安装：%s\n' "${launcher}"

