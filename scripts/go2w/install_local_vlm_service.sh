#!/usr/bin/env bash
# 把机器狗上的本地 VLM（llama-server + LFM2.5-VL-3B）装成开机自起。
#
# 为什么不是把 start_local_vlm.sh 塞进 rc.local：
#   - rc.local 起不了它。这台机的 /etc/rc.local 只做 `hwclock -s`（RTC 电池没电，
#     每次开机把系统钟拉回一个错的点）和几个 GPIO，跟本服务无关；
#   - 用 nohup 起的进程没有重启、没有日志轮转、也不在 systemd 的账上，
#     挂了没人知道。
#
# 幂等：重复跑只会覆盖单元文件并重启服务。装的单元见 scripts/go2w/robot_local_vlm.service。
#
# 用法：scripts/go2w/install_local_vlm_service.sh [--check-only]
#   --check-only  只做体检（连不上/端口/pidfile/GPU），不装不改
set -euo pipefail

ROBOT_HOST="${ROBOT_HOST:-192.168.123.18}"
ROBOT_USER="${ROBOT_USER:-unitree}"
KEY="${KEY:-$HOME/.ssh/id_ed25519_unitree_robotscene}"
UNIT_NAME="local-vlm.service"
UNIT_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/robot_local_vlm.service"
UNIT_DST="/etc/systemd/system/$UNIT_NAME"
SUDO_PASS="${SUDO_PASS:-123}"

# -F /dev/null：宿主的 ~/.ssh/config 里 Host 段挂着一条 RemoteForward 17891，
# 那个端口常被僵尸监听占着，复用默认 config 会在建连阶段就失败。
SSH=(ssh -F /dev/null -o BatchMode=yes -o IdentitiesOnly=yes
     -o StrictHostKeyChecking=yes -o UserKnownHostsFile="$HOME/.ssh/known_hosts"
     -o ConnectTimeout=10 -i "$KEY" "$ROBOT_USER@$ROBOT_HOST")

check_only=0
[[ "${1:-}" == "--check-only" ]] && check_only=1

echo "== 1. 机器狗可达性 =="
if ! "${SSH[@]}" true 2>/dev/null; then
  cat >&2 <<'EOF'
连不上机器狗。先确认物理链路：
  ip -br addr | grep enp3s0        # 应有 192.168.123.99
  cat /sys/class/net/enp3s0/carrier  # 应是 1
2026-10-12 实测过一次 NO-CARRIER（网线掉线）：此时宿主会改走默认网关，
到 192.168.123.18 的包被丢掉，ssh 卡在 SYN-SENT。
EOF
  exit 1
fi
echo "OK"

echo "== 2. 体检 =="
"${SSH[@]}" 'bash -s' <<'REMOTE'
set -u
unit="local-vlm.service"
echo "systemd: $(systemctl --version | head -1)"
echo "单元文件: $(ls "$unit" 2>/dev/null; ls /etc/systemd/system/$unit 2>/dev/null || echo 未安装)"
echo "is-enabled: $(systemctl is-enabled $unit 2>&1 || true)"
echo "llama-server 进程:"
ps -eo pid,ppid,etime,cmd | grep "[/]home/unitree/local_vlm/build/bin/llama-server" || echo "  （无）"
echo "pidfile: $(cat /home/unitree/local_vlm/server.pid 2>/dev/null || echo 无)"
echo "18081 监听:"; ss -ltn 2>/dev/null | grep 18081 || echo "  （无）"
REMOTE

if [[ $check_only -eq 1 ]]; then
  echo "--check-only：到此为止，未做任何改动"
  exit 0
fi

echo "== 3. 停掉手工起的实例（否则单元起不来：18081 会撞） =="
"${SSH[@]}" 'bash -s' <<'REMOTE'
set -u
unit="local-vlm.service"
if systemctl is-active --quiet "$unit"; then
  echo "单元已在跑，交给 systemctl restart"
  exit 0
fi
pidfile=/home/unitree/local_vlm/server.pid
if [[ -f "$pidfile" ]]; then
  pid="$(cat "$pidfile" 2>/dev/null || true)"
  # 机器狗一重启 pid 就从头再分配，pidfile 里存的那个号可能已经属于**别的进程**
  # （今天实测：重启后 pidfile 写着 127131，而当时全场最大 pid 只有 4436）。
  # 所以杀之前先确认它真是 llama-server，对不上就只清 pidfile、绝不 kill。
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q 'llama-server'; then
      echo "停掉 nohup 起的 llama-server pid=$pid"
      kill "$pid"
      for _ in $(seq 1 40); do kill -0 "$pid" 2>/dev/null || break; sleep 0.25; done
      kill -0 "$pid" 2>/dev/null && { echo "pid=$pid 没退，强杀"; kill -9 "$pid"; sleep 1; }
    else
      echo "pidfile 里的 pid=$pid 不是 llama-server（重启后 pid 复用），只清 pidfile、不动进程"
    fi
  fi
fi
rm -f "$pidfile"
REMOTE

echo "== 4. 安装单元 =="
b64="$(base64 -w0 "$UNIT_SRC")"
# 每一次 sudo 都必须自己带口令：无 tty 时 sudo 不做凭据缓存，漏一次就是
# "a terminal is required to read the password"（2026-10-12 实测踩到，
# 后果是单元装进去了却没 daemon-reload、也没 enable/start）。
"${SSH[@]}" "printf '%s' '$b64' | base64 -d > /tmp/$(basename "$UNIT_SRC") && \
  printf '%s\n' '$SUDO_PASS' | sudo -S install -m 0644 /tmp/$(basename "$UNIT_SRC") '$UNIT_DST' && \
  printf '%s\n' '$SUDO_PASS' | sudo -S systemctl daemon-reload"

echo "== 5. 开机自起 + 现在起 =="
# enable 才是"开机自起"；这里只 enable + restart，**绝不 reboot** 去验证 ——
# 机器狗一重启会打断宿主侧的 LIO 且不能自愈（要重起 SLAM+wheel_odom+Nav2），
# 而这只 15W 的盒子重启一次也不便宜。自起由 is-enabled 证明。
"${SSH[@]}" "printf '%s\n' '$SUDO_PASS' | sudo -S systemctl enable '$UNIT_NAME' && \
  printf '%s\n' '$SUDO_PASS' | sudo -S systemctl restart '$UNIT_NAME'"

echo "== 6. 等模型加载完（3B 要十几秒） =="
for i in $(seq 1 60); do
  if curl -s --noproxy '*' -m 3 http://127.0.0.1:18081/health 2>/dev/null | grep -q '"status":"ok"'; then
    echo "18081 就绪（第 ${i} 次探测）"; break
  fi
  sleep 2
  [[ $i -eq 60 ]] && { echo "18081 一直没就绪，去看机器狗上的 server.log 与 journalctl -u $UNIT_NAME" >&2; exit 1; }
done

echo "== 7. 验收 =="
"${SSH[@]}" 'bash -s' <<REMOTE
set -u
echo "is-enabled（= 开机自起，不靠 reboot 证明）: \$(sudo -n systemctl is-enabled local-vlm.service 2>/dev/null || systemctl is-enabled local-vlm.service 2>&1)"
mid=\$(systemctl show local-vlm.service -p MainPID --value)
pf=\$(cat /home/unitree/local_vlm/server.pid 2>/dev/null)
echo "MainPID: \$mid"
echo "pidfile: \$pf"
# 把"两者应相同"做成断言而不是给人看的一句话：2026-10-12 就是靠这一行才发现
# 单元里写 \$\$ 被 systemd 折叠成字面 \$、pidfile 里只有一个 "\$"。
if [ -n "\$mid" ] && [ "\$mid" != "0" ] && [ "\$mid" = "\$pf" ]; then
  echo "pidfile 与 MainPID 一致 ✅ start_local_vlm.sh 的判重有效"
else
  echo "pidfile 与 MainPID 不一致 ❌ start_local_vlm.sh 会判成没在跑、再起一个实例去撞 18081" >&2
  exit 1
fi
echo "18081: \$(ss -ltn 2>/dev/null | grep -c 18081) 个监听"
REMOTE

echo
echo "还要看一件事：**日志里没有 CUDA 行不代表跑在 CPU 上**。"
echo "这只狗上 server.log 通篇没有 cuda/gpu/offload 字样，但 /proc/<pid>/maps 映射了"
echo "libggml-cuda，且请求期间 GR3D_FREQ 有 99% —— 判据用这两个，别用 grep。"
echo "现成的离线自检：scripts/go2w/validation/run_local_vlm_static.sh"
echo
echo "装好了。顺带一提，宿主侧的端口转发是常驻单元 robotscene-local-vlm-forward.service，"
echo "它自带重试，网线恢复后会自己接上，不必手动重开。"
