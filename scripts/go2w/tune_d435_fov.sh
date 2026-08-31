#!/usr/bin/env bash
# tune_d435_fov.sh — 通过 SSH 连接机器狗上的 RealSense D435，诊断/调整取景配置。
#
# 重要物理事实（请先知道）：
#   D435 的镜头视场是硬件固定值（彩色约 69.4°x42.5°，深度约 87°x58°）。
#   SSH 改设置**不能**把镜头视场“拉宽”。脚本能做的是：
#     1) 确认当前流没有意外处于裁剪/低清晰模式（用满分辨率 = 用满硬件视场）；
#     2) 切换到最大分辨率 1280x720（像素更多、更清晰，方便算法看到更多小物体）；
#     3) 打印实时的 HFOV/VFOV，让你核对当前模式确实用满了视场。
#   想真正看到更大范围：调整相机安装角度/高度、或让机器狗转头/换位扫视（不要只盯正前方）。
#
# 用法：
#   bash scripts/go2w/tune_d435_fov.sh status
#   bash scripts/go2w/tune_d435_fov.sh wide          # 重启为 1280x720 满分辨率模式
#   bash scripts/go2w/tune_d435_fov.sh mode 1280 720 30   # 自定义宽x高x帧率
#   bash scripts/go2w/tune_d435_fov.sh reset         # 恢复默认 640x480@30
#
# 环境变量（可选）：ROBOT_IP（默认 192.168.123.18）、ROBOT_USER（默认 unitree）、
#   ROBOT_PASS（默认 123）、D435_PORT（默认 8080）。
# 需要本机装了 sshpass（或手动输入密码交互：把 rssh 里的 sshpass 换成 ssh）。

set -uo pipefail

ROBOT_IP="${ROBOT_IP:-192.168.123.18}"
ROBOT_USER="${ROBOT_USER:-unitree}"
ROBOT_PASS="${ROBOT_PASS:-123}"
D435_PORT="${D435_PORT:-8080}"
REMOTE_SCRIPT="/home/${ROBOT_USER}/realsense_stream.py"
SERVICE="realsense-stream"
BASE="http://${ROBOT_IP}:${D435_PORT}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_SCRIPT="${script_dir}/realsense_stream.py"

# 通用 ssh 封装：优先 sshpass，否则普通 ssh（手动输密码）
if command -v sshpass >/dev/null 2>&1; then
  rssh() { sshpass -p "${ROBOT_PASS}" ssh -o StrictHostKeyChecking=accept-new "${ROBOT_USER}@${ROBOT_IP}" "$@"; }
  rscp() { sshpass -p "${ROBOT_PASS}" scp -o StrictHostKeyChecking=accept-new "$@"; }
else
  rssh() { ssh "${ROBOT_USER}@${ROBOT_IP}" "$@"; }
  rscp() { scp "$@"; }
fi

_restart_service() {
  local runtime_args="$1"
  # 先把脚本更新传到机器狗（含 FOV 诊断），再重启服务
  rscp "${LOCAL_SCRIPT}" "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_SCRIPT}" || { echo "  ERROR: 上传脚本失败" >&2; exit 1; }
  rssh "echo '${ROBOT_PASS}' | sudo -S -p '' bash -c 'cat > /etc/systemd/system/${SERVICE}.service <<EOF
[Unit]
Description=Intel RealSense D435 RGB-D stream server (Go2-W)
After=network-online.target
[Service]
Type=simple
User=${ROBOT_USER}
ExecStart=/usr/bin/python3 ${REMOTE_SCRIPT} ${runtime_args}
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload && systemctl restart ${SERVICE}'"
}

_fetch_fov() {
  # 等几秒让服务起来，然后从 /fov 和 /info.json 拉取 FOV 与当前模式
  sleep 2
  echo "  GET ${BASE}/fov"
  if curl -s -m 5 "${BASE}/fov"; then
    echo
  fi
  echo "  当前模式(info.json 节选):"
  curl -s -m 5 "${BASE}/info.json" \
    | python3 -c 'import json,sys
d=json.load(sys.stdin)
c=d.get("color") or {}; dp=d.get("depth") or {}
print("    彩色 %sx%s@%sfps  深度 %sx%s@%sfps" % (c.get("width"),c.get("height"),c.get("fps"),dp.get("width"),dp.get("height"),dp.get("fps")))
f=d.get("fov_deg") or {}
print("    FOV(彩色) %s" % (f.get("color") or "n/a"))
' 2>/dev/null || echo "    (info.json 暂不可用；服务可能还在启动)"
}

case "${1:-status}" in
  status)
    echo "== D435 当前状态（SSH: ${ROBOT_USER}@${ROBOT_IP}） =="
    rssh "systemctl is-active ${SERVICE} || echo '服务未运行'; curl -s -m 5 ${BASE}/health || true" 2>/dev/null
    echo
    echo "== 实测 FOV / 分辨率 =="
    _fetch_fov
    echo
    echo "== 物理说明 =="
    echo "  D435 满宽度视场：彩色约 69°x42°、深度约 87°x58°。"
    echo "  关键：640x480 是中心裁剪模式，HFOV 只有约 55°，会明显觉得'取景小'。"
    echo "  切换到 848x480（默认）或 1280x720（--wide）即可真正拉宽取景范围。"
    echo "  想再进一步：加高相机/调倾角增大俯视范围、让机器狗转头扫视、或硬件换广角镜头。"
    ;;
  wide)
    echo "== 重启为 1280x720 满分辨率/满视场模式（FOV不变，像素更多） =="
    _restart_service "--wide --port ${D435_PORT}"
    echo "  重启完成，等待服务起来："
    _fetch_fov
    ;;
  mode)
    W="${2:-1280}"; H="${3:-720}"; FPS="${4:-30}"
    echo "== 重启为自定义模式 ${W}x${H}@${FPS} =="
    _restart_service "--width ${W} --height ${H} --fps ${FPS} --port ${D435_PORT}"
    echo "  重启完成："
    _fetch_fov
    ;;
  reset)
    echo "== 恢复默认 1280x720@30（满宽度满分辨率模式） =="
    _restart_service "--width 1280 --height 720 --fps 30 --port ${D435_PORT}"
    echo "  重启完成："
    _fetch_fov
    ;;
  *)
    echo "用法: $0 {status|wide|mode W H FPS|reset}" >&2
    exit 1
    ;;
esac
