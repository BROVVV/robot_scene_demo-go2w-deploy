#!/usr/bin/env bash
# Foreground owner for one disposable teleoperated-mapping session.
# Closing this terminal (or Ctrl+C) stops the stack and discards the map.
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_dir="${project_root}/outputs/remote_control_mapping/runtime"
log_dir="${project_root}/outputs/remote_control_mapping/logs"
lock_file="${project_root}/runtime/go2w/one_click_remote_mapping.lock"
web_port="${REMOTE_MAPPING_WEB_PORT:-8766}"
detector="${REMOTE_MAPPING_DETECTOR:-yoloe_seg}"
cleanup_started=0

mkdir -p "$(dirname -- "${lock_file}")" "${runtime_dir}" "${log_dir}"
exec 9>"${lock_file}"
if ! flock -n 9; then
  printf '\n已有一个“一键遥操建图”启动器正在运行。\n' >&2
  printf '请关闭原来的启动器窗口，或双击原窗口后按 Ctrl+C。\n' >&2
  printf '按 Enter 关闭本窗口……'
  read -r _ || true
  exit 3
fi

# Keep the exact terminal transcript.  If a future launch still fails after a
# cable reconnect, the cause is immediately available instead of disappearing
# together with the terminal window.
launcher_log="${log_dir}/one_click_launcher.log"
exec > >(tee -a "${launcher_log}") 2>&1
printf '\n[%s] ===== new one-click mapping launch =====\n' "$(date '+%F %T')"

cleanup() {
  if [[ "${cleanup_started}" == "1" ]]; then
    return
  fi
  cleanup_started=1
  # Finish cleanup even when the terminal emulator has already sent SIGHUP.
  trap '' HUP INT TERM
  printf '\n\n==========================================================\n'
  printf '正在停止遥操建图并丢弃本轮地图，请稍候……\n'
  bash "${script_dir}/discard_remote_control_mapping_session.sh" || true
  printf '清理完成。现在可以重新双击启动器创建一张全新的地图。\n'
  printf '==========================================================\n'
  return 0
}
trap cleanup EXIT
trap 'exit 0' HUP INT TERM

cd "${project_root}"
clear 2>/dev/null || true
cat <<'EOF'
==========================================================
             GO2W 一键遥操建图（一次性会话）
==========================================================
  • 本窗口保持开启期间，遥操建图持续运行。
  • 先用 Unitree 实体遥控器把机器狗停稳，再关闭本窗口。
  • 关闭窗口或按 Ctrl+C：停止进程并丢弃本轮地图。
  • 下次双击：一定从一张新的二维地图开始。
  • 本软件只读感知，不向机器狗发送运动命令。
==========================================================
EOF

printf '\n[1/2] 清理可能残留的上一轮会话……\n'
bash "${script_dir}/discard_remote_control_mapping_session.sh"

record_channels='三路录像'
if [[ "${GO2W_WEB_3D_MAP_ENABLED:-1}" == "0" && "${GO2W_WEB_2D_MAP_ENABLED:-0}" != "1" ]]; then
  record_channels='相机+二维拓扑录像'
fi
printf '\n[2/2] 启动激光桥、二维 SLAM、YOLOE、VLM、%s和 WebUI……\n' "$record_channels"
# bootstrap_webui owns the cold-start path from Unitree's /utlidar/cloud to
# /go2w/sensors/cloud. Calling start_remote_control_mapping.sh directly works
# only when that bridge already happens to be alive after an earlier session.
set +e
GO2W_BOOTSTRAP_OWNER_MARKER="${runtime_dir}/live_perception_started_by_one_click" \
REMOTE_MAPPING_WEB_PORT="${web_port}" \
  bash "${script_dir}/bootstrap_webui.sh" \
    --mode mapping \
    --detector "${detector}"
bootstrap_rc=$?
set -e
if [[ "${bootstrap_rc}" != "0" ]]; then
  printf '\n[FAIL] 启动失败（rc=%s），正在自动清理本轮半启动状态。\n' \
    "${bootstrap_rc}"
  printf '详细日志：%s\n' "${launcher_log}"
  cleanup
  if [[ -t 0 ]]; then
    printf '\n请按 Enter 关闭窗口；处理提示中的硬件问题后再次双击即可。'
    read -r _ || true
  fi
  exit "${bootstrap_rc}"
fi

web_url="http://127.0.0.1:${web_port}"
printf '\n==========================================================\n'
printf 'WebUI 已启动：%s\n' "${web_url}"
printf '不要关闭本窗口；需要重建地图时直接关闭本窗口，再次双击图标。\n'
printf '==========================================================\n\n'

# The real services are deliberately detached by the production launcher.
# Keep this foreground owner alive and watch the exact pidfiles so a broken
# connection/process is visible instead of leaving a deceptively live window.
while :; do
  sleep 2
  web_pid="$(tr -cd '0-9' <"${runtime_dir}/web.pid" 2>/dev/null || true)"
  mapper_pid="$(tr -cd '0-9' <"${runtime_dir}/semantic_mapper.pid" 2>/dev/null || true)"
  if [[ -z "${web_pid}" || -z "${mapper_pid}" ]] \
     || ! kill -0 "${web_pid}" 2>/dev/null \
     || ! kill -0 "${mapper_pid}" 2>/dev/null; then
    printf '\n检测到 WebUI 或语义建图进程已经断开。\n' >&2
    printf '本轮将被清理；关闭此窗口后重新双击即可重建。\n' >&2
    exit 4
  fi
done

