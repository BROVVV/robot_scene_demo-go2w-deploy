#!/usr/bin/env bash
# 把机器狗的钟对齐到宿主。**只碰时间，不碰任何运动栈。**
#
# 为什么需要它：D435 的 HTTP 服务跑在机器狗上，帧时间戳用的是**机器狗的钟**；
# 而落图要用"拍摄时刻的 SLAM 位姿"，位姿查询按时间戳找最近的 odom/TF 样本。
# remote_control_semantic_mapping_worker.py 有个时钟一致性门
# （--max-clock-skew-seconds，默认 5s）：偏差超过门限就**放弃拍摄时间戳**、
# 改用接收时刻。2026-09-18 实测机器狗快 13.67s ⇒ 每一帧都降级，
# 状态里 timestamp_source 恒为 receive_time_clock_skew、每个物体都带
# clock_skew_seconds≈13.7。功能不报错，但时间戳系统性偏移一个网络往返，
# 而且这个 WARNING 会把真正的问题盖住。
#
# 机器狗**没有外网**（NTP/HTTPS 都是 no route to host）、RTC 读出来是 1970，
# 所以它的 systemd-timesyncd 永远同步不上；宿主也没有在 123/udp 上对外服务
# （只装了客户端 timesyncd、且本机不能免密 sudo，装服务端这条路走不通）。
# 因此唯一可用的方向是**从宿主把时间推过去**——本脚本做的事。
#
# 用法：
#   bash scripts/go2w/sync_robot_clock.sh            # status：只读测量并报告
#   bash scripts/go2w/sync_robot_clock.sh status     # 同上
#   bash scripts/go2w/sync_robot_clock.sh apply      # 读取 → 下推 → 复测
#
# status 的退出码：偏差在容差内 0，超出 3，测不到 4。方便被别的脚本当闸门用。
#
# 环境变量（可选）：ROBOT_IP（默认 192.168.123.18）、ROBOT_USER（默认 unitree）、
#   ROBOT_PASS（默认 123）。
#
# 安全性：apply 默认**拒绝在 plain_slam 还在跑的时候改钟**。原因见下面
# require_slam_stopped() 的注释。确实要在跑的时候改，加 --force，并自己承担
# 地图可能重影的后果。
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
pid_root="${project_root}/runtime/go2w/pids"

ROBOT_IP="${ROBOT_IP:-192.168.123.18}"
ROBOT_USER="${ROBOT_USER:-unitree}"
ROBOT_PASS="${ROBOT_PASS:-123}"

# 容差**不写死**：直接取 worker 里那个门的默认值，保证两边永远一致。
# 两处各写一个 5.0，改了一处忘了另一处，就会出现"脚本说没问题、worker 却在降级"。
worker_py="${script_dir}/remote_control_semantic_mapping_worker.py"
tolerance="$(grep -m1 -E '^DEFAULT_MAX_CLOCK_SKEW_S' "${worker_py}" 2>/dev/null \
  | sed -E 's/.*=[[:space:]]*([0-9.]+).*/\1/' || true)"
tolerance="${tolerance:-5.0}"

rssh() {
  sshpass -p "${ROBOT_PASS}" ssh -F /dev/null \
    -o ControlMaster=no -o ControlPath=none -o ClearAllForwardings=yes \
    -o StrictHostKeyChecking=accept-new \
    -o UserKnownHostsFile="${HOME}/.ssh/known_hosts" \
    -o ConnectTimeout=8 "${ROBOT_USER}@${ROBOT_IP}" "$@"
}

# 机器狗的钟 vs 宿主：用一次 ssh 的往返时间做中点估计，别拿"发之前/收之后"平均
# 就完事——ssh 登录本身要几百毫秒，那点误差虽然进不了 5 秒的量级，但既然
# 往返时间是现成的，就没理由不用。
measure_skew() {
  local before after robot mid host
  before="$(date +%s.%N)"
  robot="$(rssh 'date +%s.%N' 2>/dev/null || true)"
  after="$(date +%s.%N)"
  [[ -n "${robot}" ]] || return 1
  mid="$(awk -v a="${before}" -v b="${after}" 'BEGIN{printf "%.3f", (a+b)/2}')"
  host="${mid}"
  SKEW="$(awk -v r="${robot}" -v h="${host}" 'BEGIN{printf "%.3f", r-h}')"
  RTT="$(awk -v a="${before}" -v b="${after}" 'BEGIN{printf "%.3f", b-a}')"
  return 0
}

report() {
  printf '宿主  %s  (epoch %s)\n' "$(date '+%F %T %Z')" "$(date +%s)"
  printf '机器狗 %s  (epoch %s)\n' "${robot_local}" "${robot_epoch}"
  printf '偏差  %+.3f s   (ssh 往返 %s s，容差 ±%s s)\n' "${SKEW}" "${RTT}" "${tolerance}"
  if awk -v s="${SKEW}" -v t="${tolerance}" 'BEGIN{exit !(s<0?-s<=t:s<=t)}'; then
    echo '  判定：在容差内 —— 帧时间戳会被当作拍摄时刻使用。'
    return 0
  fi
  echo '  判定：**超出容差** —— 落图会降级成 receive_time_clock_skew：'
  echo '        每个物体的时间戳改用接收时刻，状态里带 clock_skew_seconds。'
  return 3
}

# 改钟的时机很要紧：机器狗的钟一旦跳变，任何**跨机对齐时间轴**的环节都会看到
# 一个大台阶。宿主侧的 IMU 兜底适配器带步长限制（--max-offset-step-s），
# 台阶会被卡住并留下时间轴脱钩——2026-09-18 之前已经吃过一次亏，表现是
# 地图重影，而查错方向会被引到点云上去。所以默认要求先停 SLAM。
require_slam_stopped() {
  local force="$1" alive=()
  local name pid_file pid
  for name in plain_slam_go2w lidar_delay_buffer plain_slam_web_cloud_relay plain_slam_disk_guard; do
    pid_file="${pid_root}/${name}.pid"
    [[ -f "${pid_file}" ]] || continue
    pid="$(tr -cd '0-9' < "${pid_file}")"
    [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && alive+=("${name}(pid ${pid})")
  done
  # pid 文件可能过期（launch 崩了但文件还在），所以再按进程名兜一层。
  if pgrep -f 'plain_slam_go2w\.launch\.py|lio_3d_node|slam_3d_node' >/dev/null 2>&1; then
    alive+=('plain_slam 节点(按进程名)')
  fi
  if [[ "${#alive[@]}" -eq 0 ]]; then
    return 0
  fi
  if [[ "${force}" == "1" ]]; then
    echo "WARNING: plain_slam 仍在运行（${alive[*]}），--force 已指定，继续改钟。"
    echo "         地图可能因此重影；改完建议重开一轮建图。"
    return 0
  fi
  echo "拒绝执行：plain_slam 还在运行：${alive[*]}" >&2
  echo "  改钟会让跨机对齐的时间轴出现台阶（宿主侧 IMU 适配器有步长限制，会被卡住），" >&2
  echo "  已知后果是地图重影。请先停栈再同步：" >&2
  echo "    bash scripts/go2w/stop_plain_slam_mapping.sh" >&2
  echo "    bash scripts/go2w/sync_robot_clock.sh apply" >&2
  echo "    bash scripts/go2w/start_remote_control_mapping.sh" >&2
  echo "  （确实要带着 SLAM 改，加 --force。）" >&2
  return 1
}

action="status"
force=0
for arg in "$@"; do
  case "${arg}" in
    status|apply) action="${arg}" ;;
    --force) force=1 ;;
    -h|--help) sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数：${arg}" >&2; exit 2 ;;
  esac
done

command -v sshpass >/dev/null 2>&1 || { echo "缺少 sshpass（apt install sshpass）" >&2; exit 2; }

echo "== 机器狗时钟 vs 宿主 =="
if ! measure_skew; then
  echo "测不到机器狗时间：${ROBOT_USER}@${ROBOT_IP} ssh 不通。" >&2
  exit 4
fi
robot_epoch="$(rssh 'date +%s' 2>/dev/null || echo '?')"
robot_local="$(rssh "date '+%F %T %Z'" 2>/dev/null || echo '?')"
report && rc=0 || rc=$?

if [[ "${action}" == "status" ]]; then
  exit "${rc}"
fi

# ---- apply ----
if [[ "${rc}" -eq 0 ]]; then
  echo '已在容差内，无需改动。'
  exit 0
fi

require_slam_stopped "${force}"

target="$(date +%s)"
printf '下推：机器狗 -> %s (epoch %s)\n' "$(date -d "@${target}" '+%F %T %Z')" "${target}"
# 用 `date -s @epoch` 而不是 `timedatectl set-time`：后者在 NTP 同步开启时会直接
# 拒绝执行（"Automatic time synchronization is enabled"），而机器狗的
# systemd-timesyncd 恰恰是开着的（只是永远同步不上）。`date -s @epoch` 写的是
# UTC epoch，跟时区无关，也绕开了那条拒绝。
rssh "echo '${ROBOT_PASS}' | sudo -S -p '' date -s '@${target}'" >/dev/null

# ---- 重启能不能自己回来 ----
#
# 这一段**不能看退出码**。这台机器狗的 RTC 是坏的（备份电池没电：上电清零后
# 从 0 开始数，实测 RTC 计数 10189s ≈ uptime 10180s），而
# `hwclock --systohc` 在这种情况下**照样返回 0**、`hwclock --show` 也会静默
# 回落到系统时钟把正确时间打印出来 —— 两个都是假绿，会让人以为写进去了。
# 唯一可信的读法是 `timedatectl` 的 RTC time 行。
rtc_line() { rssh 'timedatectl 2>/dev/null' | awk -F': ' '/RTC time/{print $2; exit}'; }

rtc_now="$(rtc_line || true)"
if [[ -z "${rtc_now}" ]]; then
  echo '  读不到 RTC（timedatectl 无输出）；重启后的行为未知。'
else
  rssh "echo '${ROBOT_PASS}' | sudo -S -p '' hwclock --systohc" >/dev/null 2>&1 || true
  rtc_after="$(rtc_line || true)"
  # 只看年份：写成功后 RTC 应该落在今年，而不是 1970。
  if [[ "${rtc_after}" == *"$(date +%Y)"* ]]; then
    echo "  RTC 已写入（${rtc_after##* }），重启后能自己回来。"
  else
    echo "  RTC 写不进去：写前「${rtc_now##* }」、写后「${rtc_after##* }」没变。"
    echo '  这台机器狗的 RTC 备份电池没电（上电清零），软件修不了 —— 不是本脚本能解决的。'
    echo '  后果：每次重启后系统钟会回到 1970，要靠下面那个落点文件恢复。'
  fi
fi

# systemd-timesyncd 从没同步成功过时，启动会用这个文件里存的时间戳把钟**往前拨**
# （它只前拨、不回拨，这是 systemd 自带机制，不是我们发明的旁路）。机器狗既没有
# 外网、RTC 又是坏的，写它等于给重启留个落点。
# 注意落点是**上一次 apply 的时刻**、不是当前时刻：关机期间它不会自己走，
# 所以断电越久越慢，重启后要重新跑一次 apply。
if rssh 'test -e /var/lib/systemd/timesync' >/dev/null 2>&1; then
  if rssh "echo '${ROBOT_PASS}' | sudo -S -p '' sh -c 'date +%s > /var/lib/systemd/timesync/clock'" >/dev/null 2>&1; then
    echo '  已写 timesyncd 落点文件：重启后钟会回到**本次同步的时刻**（关机时长会丢，需重跑 apply）。'
  else
    echo '  timesyncd 落点文件没写成功；重启后需要重新跑一次 apply。'
  fi
fi

echo
echo '== 复测 =='
measure_skew || { echo '复测失败（ssh 断了？）' >&2; exit 4; }
robot_epoch="$(rssh 'date +%s' 2>/dev/null || echo '?')"
robot_local="$(rssh "date '+%F %T %Z'" 2>/dev/null || echo '?')"
report && rc=0 || rc=$?
exit "${rc}"
