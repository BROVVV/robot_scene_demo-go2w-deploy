#!/usr/bin/env bash
# Go2-W 机载 Intel RealSense D435 RGB-D 流服务 部署/管理脚本。
#
# 用法：
#   bash scripts/go2w/start_realsense_stream.sh install    # 上传脚本到机器狗 + 安装 systemd 服务（开机自启）
#   bash scripts/go2w/start_realsense_stream.sh start      # 启动服务
#   bash scripts/go2w/start_realsense_stream.sh stop       # 停止服务
#   bash scripts/go2w/start_realsense_stream.sh status     # 查看状态（服务 + /health）
#   bash scripts/go2w/start_realsense_stream.sh logs       # 跟踪服务日志
#   bash scripts/go2w/start_realsense_stream.sh view       # 打印本机查看地址
#
# 环境变量（可选）：ROBOT_IP（默认 192.168.123.18）、ROBOT_USER（默认 unitree）、
#   ROBOT_PASS（默认 123，可用 sshpass 或手动交互）。
# 依赖：sshpass（本机）、机器狗上 pyrealsense2（见 docs/REALSENSE_D435_DEPLOYMENT.md）。

set -uo pipefail

ROBOT_IP="${ROBOT_IP:-192.168.123.18}"
ROBOT_USER="${ROBOT_USER:-unitree}"
ROBOT_PASS="${ROBOT_PASS:-123}"
REMOTE_SCRIPT="/home/${ROBOT_USER}/realsense_stream.py"
SERVICE="realsense-stream"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
LOCAL_SCRIPT="${script_dir}/realsense_stream.py"

rssh() { sshpass -p "${ROBOT_PASS}" ssh -o StrictHostKeyChecking=accept-new "${ROBOT_USER}@${ROBOT_IP}" "$@"; }

case "${1:-}" in
  install)
    [[ -f "${LOCAL_SCRIPT}" ]] || { echo "ERROR: 缺少 ${LOCAL_SCRIPT}" >&2; exit 1; }
    echo ">> 上传 ${LOCAL_SCRIPT} -> ${ROBOT_IP}:${REMOTE_SCRIPT}"
    sshpass -p "${ROBOT_PASS}" scp -o StrictHostKeyChecking=accept-new \
      "${LOCAL_SCRIPT}" "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_SCRIPT}"
    echo ">> 安装 systemd 服务 ${SERVICE}"
    rssh "echo '${ROBOT_PASS}' | sudo -S -p '' bash -c '
cat > /etc/systemd/system/${SERVICE}.service <<EOF
[Unit]
Description=Intel RealSense D435 RGB-D stream server (Go2-W)
After=network-online.target

[Service]
Type=simple
User=${ROBOT_USER}
ExecStart=/usr/bin/python3 ${REMOTE_SCRIPT} --width 1280 --height 720 --fps 30 --port 8080
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload && systemctl enable ${SERVICE} && systemctl restart ${SERVICE}'"
    echo ">> 完成"
    ;;
  start)  rssh "echo '${ROBOT_PASS}' | sudo -S -p '' systemctl start ${SERVICE} && systemctl is-active ${SERVICE}" ;;
  stop)   rssh "echo '${ROBOT_PASS}' | sudo -S -p '' systemctl stop ${SERVICE}" ;;
  status)
    rssh "systemctl is-active ${SERVICE} && systemctl is-enabled ${SERVICE}"
    curl -s -m 5 "http://${ROBOT_IP}:8080/health" || echo "  (服务未响应 /health)"
    ;;
  logs)   rssh "journalctl -u ${SERVICE} -f" ;;
  view)
    echo "  浏览器:  http://${ROBOT_IP}:8080/"
    echo "  OpenCV:  ${script_dir}/view_realsense.py"
    ;;
  *)
    echo "用法: $0 {install|start|stop|status|logs|view}" >&2
    exit 1
    ;;
esac
