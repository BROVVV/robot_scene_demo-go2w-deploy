#!/usr/bin/env bash
set -euo pipefail

# start_nav2_autonomous_search.sh 的对应停止入口（计划书 §P0）。
#
# 为什么需要单独一个脚本：那一轮起的三套东西（Nav2 bringup / 控制仲裁器 /
# cmd_vel bridge）的 pidfile 写在 outputs/autonomous_search/runtime/ 下，而
# 在此之前**没有任何脚本读它们** —— stop_all.sh 读的是另一个目录
# （runtime/go2w/pids/）。结果是这三套只能靠手工 kill 停，而手工 kill 不留
# pidfile，下一轮 start 就会在活着的同名节点旁边再起一套。
#
#   bash scripts/go2w/stop_nav2_autonomous_search.sh
#       # 只停**正常入口**起的那一套。隔离验收链路（isolated_velocity_chain +
#       # robot_scene_nav2.launch.py nav2_cmd_vel_topic:=/go2w/acceptance/...）
#       # 不碰，只在结尾点名说它还活着。
#
#   bash scripts/go2w/stop_nav2_autonomous_search.sh --include-isolated-acceptance
#       # 连隔离验收链路一起、**整条**停掉。
#
# 只停这三套。感知 / 激光雷达 / 轮式里程计 / 二维定位 / WebUI 都不在本脚本
# 范围内（分别由 start_live_perception.sh、start_2d_localization.sh、
# stop_autonomous_search_web.sh 负责），避免把现场其他流程一起带走。
#
# 为什么要区分这两套（2026-10-15 补）：两套 Nav2 的 cmdline 里都有
# `robot_scene_nav2.launch.py`，按 cmdline 扫是**分不开**的，唯一逐字可靠的
# 判据是隔离那套多带了 `nav2_cmd_vel_topic:=/go2w/acceptance/`（launch 自己的
# 约定，见 robot_scene_nav2.launch.py 的 DeclareLaunchArgument 默认值
# `/go2w/nav2_cmd_vel` —— 不带这个覆盖的就是物理入口那一套）。
# 不区分会同时坏两头：
#   * 兜底扫描会把正在服务验收的隔离链路当"Nav2 bringup"顺手停掉，而操作员
#     读到的字面是正常入口的停止；
#   * 更糟的是**只停一半**：隔离链路还有另一半（`isolated_velocity_chain.launch.py`
#     起的 acceptance_velocity_smoother / acceptance_velocity_guard /
#     acceptance_collision_monitor / acceptance_lifecycle）没有任何 needle 匹配得到，
#     于是它们留在图上继续往 `/go2w/acceptance/*` 发 —— 下一轮验收的"发布者是
#     不是唯一"证据就被上一轮的孤儿污染了，正是本脚本要消灭的那类假绿。
# 所以带上 `--include-isolated-acceptance` 时会**两半一起**停。
#
# 本脚本不发任何机器人运动指令，也不写 runtime/go2w/ 下的 CANCEL_* 标志位：
# 那三个标志全仓没有任何读取方（只有 stop_all.sh 写），写了只会让人以为
# "搜索已经被取消了"，而真正让搜索停下来的是下面这些进程被停掉。

# 隔离验收链路的逐字判据。**只认这一个**：launch 的默认值是物理入口
# `/go2w/nav2_cmd_vel`，隔离那套必须显式覆盖它。
ISOLATED_NAV2_MARKER="nav2_cmd_vel_topic:=/go2w/acceptance/"
# 隔离链路的另一半。这三个是 `isolated_velocity_chain.launch.py` 起的节点名，
# 上面的 marker 一个都匹配不到，所以必须单独列出来，否则永远只停一半。
ISOLATED_CHAIN_NEEDLES=(
  "isolated_velocity_chain.launch.py"
  "acceptance_velocity_smoother"
  "acceptance_velocity_guard"
  "acceptance_collision_monitor"
  "acceptance_lifecycle"
)

INCLUDE_ISOLATED=0
for arg in "$@"; do
  case "$arg" in
    --include-isolated-acceptance) INCLUDE_ISOLATED=1 ;;
    -h|--help)
      sed -n '3,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      printf 'ERROR: unknown argument: %s\n' "$arg" >&2
      printf '只接受 --include-isolated-acceptance（默认不碰隔离验收链路）。\n' >&2
      exit 2
      ;;
  esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_root="${project_root}/outputs/autonomous_search/runtime"

nav2_pidfile="${runtime_root}/nav2_bringup.pid"
bridge_pidfile="${runtime_root}/cmd_vel_bridge.pid"
arbiter_pidfile="${runtime_root}/control_arbiter.pid"

stopped=0
warned=0
alive_pids=()
# 扫到但按旗标选择不动的隔离验收进程。结束时必须点名，否则操作员会以为
# "stop 跑过了 = 图上干净了"，而 start 下一轮照样拒绝。
skipped_isolated=()

# 停 pidfile 记的那一套。pidfile 里存的是 setsid 出来的会话首进程，pgid == pid，
# Nav2 先作生命周期关闭、仅通知 launch 转发一次 SIGINT；其他组件对进程组发信号。
# 10 秒内没退干净就 SIGKILL 掉父进程（子节点可能被 reparent 到 pid 1，由后面的
# 兜底扫描收拾）。
#
# pidfile 存在但 pid 已经死了（僵尸指针）时**不静默删除就算了**：那正是"上一轮在
# 写 pidfile 之后自己退了"或者"被人手工 kill 过"的现场，说明真相不在 pidfile 里。
# 打印出来并计入 warned，再交给兜底扫描去找真正在跑的那套。
# 这一条 cmdline 是不是隔离验收链路的 Nav2 半。判据见文件头。
is_isolated_nav2_cmd() {
  [[ "$1" == *"$ISOLATED_NAV2_MARKER"* ]]
}

interrupt_owned_process() {
  local pid="$1" cmd="$2"
  if [[ "$cmd" == *robot_scene_nav2.launch.py* ]]; then
    # The helper verifies the launcher's child manager and uses its own DDS
    # environment. Foxy costmap plugins need lifecycle cleanup before SIGINT
    # invalidates their ROS context. Other distributions retain their behavior.
    if ! /usr/bin/python3 "${script_dir}/shutdown_owned_nav2.py" --launch-pid "$pid"; then
      printf 'WARNING: Nav2 pid %s lifecycle shutdown failed; continuing process cleanup.\n' "$pid" >&2
      warned=$((warned + 1))
    fi
    # ros2 launch forwards SIGINT. A group signal would interrupt each child
    # twice and can arrive during its destructor.
    kill -INT "$pid" 2>/dev/null || true
  else
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  fi
}

stop_from_pidfile() {
  local label="$1" pidfile="$2" needle="$3" pid cmd
  if [[ ! -f "$pidfile" ]]; then
    printf '%s：没有 pidfile，跳过。\n' "$label" >&2
    return 0
  fi
  pid="$(tr -cd '0-9' <"$pidfile")"
  if [[ -z "$pid" ]]; then
    printf '%s：pidfile 为空，跳过。\n' "$label" >&2
    rm -f "$pidfile"
    return 0
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    printf '%s：pidfile 记的 pid %s 已经不在了（僵尸指针），改由兜底扫描处理。\n' \
      "$label" "$pid" >&2
    rm -f "$pidfile"
    warned=$((warned + 1))
    return 0
  fi
  cmd="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  if [[ "$cmd" != *"$needle"* ]]; then
    printf 'ERROR: 拒绝停 %s 的 pid %s：它不是本项目的进程（cmdline 不含 %s）。\n' \
      "$label" "$pid" "$needle" >&2
    warned=$((warned + 1))
    return 0
  fi
  # pidfile 里记的也是隔离那一套时，同样受旗标管辖：默认不动，免得"停正常入口"
  # 这个动作把正在服务验收的链路带下去（今天没有这个 pidfile，是防下一次有人
  # 用 start 脚本以外的方式给隔离链路记账）。
  if (( ! INCLUDE_ISOLATED )) && is_isolated_nav2_cmd "$cmd"; then
    printf '%s：pidfile 里记的 pid %s 是隔离验收链路，未传 --include-isolated-acceptance，跳过。\n' \
      "$label" "$pid" >&2
    skipped_isolated+=("$pid")
    return 0
  fi
  printf 'Stopping %s (pid %s, pgid %s) ...\n' "$label" "$pid" "$pid" >&2
  interrupt_owned_process "$pid" "$cmd"
  for _ in $(seq 1 40); do
    kill -0 -- "-${pid}" 2>/dev/null || break
    sleep 0.25
  done
  if kill -0 -- "-${pid}" 2>/dev/null; then
    printf 'WARNING: %s pid %s did not exit after SIGINT; forcing its process group to stop.\n' "$label" "$pid" >&2
    warned=$((warned + 1))
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$pidfile"
  stopped=$((stopped + 1))
}

# 兜底：pidfile 不可信时按 cmdline 直接扫 /proc。两种情况都靠它收拾——
#   (1) pidfile 是僵尸指针（上一轮自己退了，或被人手工 kill 过）；
#   (2) 上一轮的 launch 有子节点被 reparent 到 pid 1，pidfile 的进程组碰不到。
# 按 /proc/$pid/exe 排除 shell：本项目的每个节点都是被 `exec` 成 python3/ELF 的
# （`setsid bash -c 'exec ...'`），而任何"命令行里恰好含这段文字"的外层 shell
# 都会被这一步滤掉。不用 `pkill -f`：它会把调用它的那个 shell 自己也匹配上杀掉。
# 调用前用 `SWEEP_ISOLATED_MODE=... sweep_orphans ...` 指定怎么处理隔离链路：
#   normal   —— 按 marker 筛：隔离那一套跳过并登记，其余照停
#   isolated —— 只停带 marker 的（用于两套共用的那个 needle）
#   all      —— 不筛，命中就停（needle 本身已经唯一时用它）
#   report   —— 不筛，命中只登记不停（点名用）
# 注意 `isolated` 与 `report` 都**只能**用在两套共用的那个 needle
# （`robot_scene_nav2.launch.py`）上。隔离链路的另一半
# （`isolated_velocity_chain.launch.py` 与 acceptance_* 节点）**没有** marker
# ——它是速度/碰撞链，不是 Nav2 —— 对它们用 `isolated` 会一个都匹配不到，
# 于是"两半一起停"退化成"只停一半"，正是本脚本要消灭的那个缺陷。那一半的
# needle 本身已经唯一，所以用 `all` / `report`。
# 用变量而不是加参数：`sweep_orphans` 的 needle 是变参，再插一个位置参数会让
# 三个调用点都得记住顺序，而顺序记错的表现是"多停了东西"，不报错。
SWEEP_ISOLATED_MODE=all
sweep_orphans() {
  local label="$1" tmp d pid exe cmd hit needle alive isolated
  shift
  tmp="$(mktemp)"
  for d in /proc/[0-9]*; do
    pid="${d#/proc/}"
    [[ "$pid" == "$$" || "$pid" == "${PPID:-0}" ]] && continue
    exe="$(readlink -f "$d/exe" 2>/dev/null)" || continue
    case "${exe##*/}" in bash|sh|dash|zsh) continue ;; esac
    cmd="$(tr '\0' ' ' <"$d/cmdline" 2>/dev/null)" || continue
    hit=0
    for needle in "$@"; do
      [[ "$cmd" == *"$needle"* ]] && { hit=1; break; }
    done
    (( hit )) || continue
    isolated=0
    is_isolated_nav2_cmd "$cmd" && isolated=1
    case "$SWEEP_ISOLATED_MODE" in
      normal)   (( isolated )) && { skipped_isolated+=("$pid"); continue ;} ;;
      isolated) (( isolated )) || continue ;;
      report)   skipped_isolated+=("$pid"); continue ;;
    esac
    printf '%s %s\n' "$pid" "$cmd" >>"$tmp"
  done
  if [[ ! -s "$tmp" ]]; then
    rm -f "$tmp"
    return 0
  fi
  printf '兜底清理 %s：以下进程不在 pidfile 的覆盖范围内，即将停止——\n' "$label" >&2
  while read -r pid cmd; do
    printf '  pid %s  %s\n' "$pid" "$cmd" >&2
  done <"$tmp"
  alive_pids=()
  while read -r pid cmd; do
    [[ "$pid" != "$$" ]] && alive_pids+=("$pid")
  done <"$tmp"
  rm -f "$tmp"
  while read -r pid cmd; do
    interrupt_owned_process "$pid" "$cmd"
  done < <(for pid in "${alive_pids[@]}"; do
    printf '%s %s\n' "$pid" "$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  done)
  for _ in $(seq 1 40); do
    alive=0
    for pid in "${alive_pids[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    (( alive )) || break
    sleep 0.25
  done
  for pid in "${alive_pids[@]}"; do
    kill -0 "$pid" 2>/dev/null || continue
    printf 'WARNING: orphan pid %s did not exit after SIGINT; forcing cleanup.\n' "$pid" >&2
    warned=$((warned + 1))
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  done
  stopped=$((stopped + ${#alive_pids[@]}))
}

# 顺序：先停 Nav2 bringup（不再产生新目标），再停 bridge（速度通路），最后仲裁器。
# 三者都 fail-closed，顺序不改变安全性，但按这个顺序停下后图上不会留下
# "有消费者、没有发布者"的中间态。
stop_from_pidfile "Nav2 bringup" "$nav2_pidfile" "robot_scene_nav2.launch.py"
stop_from_pidfile "cmd_vel bridge" "$bridge_pidfile" "go2w_cmd_vel_bridge"
stop_from_pidfile "控制仲裁器" "$arbiter_pidfile" "go2w_control_arbiter"

# Nav2 只扫正常入口那一套：`robot_scene_nav2.launch.py` 这个 needle 两套都有，
# 不加筛就会把正在服务验收的隔离链路当"Nav2 bringup"停掉。
SWEEP_ISOLATED_MODE=normal \
  sweep_orphans "Nav2 bringup" "robot_scene_nav2.launch.py"
# bridge 的两个 needle 合起来才覆盖完整：`ros2 run` 是两层进程，外层包装的
# cmdline 是 `ros2 run go2w_cmd_vel_bridge bridge`（空格），内层节点的是
# `.../lib/go2w_cmd_vel_bridge/bridge`（斜杠）。只写一个都会漏掉一半。
# 也不能用 `go2w_cmd_vel_bridge` 本身：同一个包还装了 velocity_guard，而那个是
# Nav2 bringup 正常起的，拿它当 needle 会把正常链路判成待清理。
# 这两条 needle 对隔离链路也唯一（隔离那半用的是 velocity_guard 节点名），所以
# 不需要筛。
SWEEP_ISOLATED_MODE=all \
  sweep_orphans "cmd_vel bridge" "go2w_cmd_vel_bridge bridge" "go2w_cmd_vel_bridge/bridge"
SWEEP_ISOLATED_MODE=all sweep_orphans "控制仲裁器" "go2w_control_arbiter"

if (( INCLUDE_ISOLATED )); then
  # 两半必须一起停，理由见文件头：只停 Nav2 半会留下 acceptance_* 继续发。
  SWEEP_ISOLATED_MODE=isolated \
    sweep_orphans "隔离验收链路（Nav2 半）" "robot_scene_nav2.launch.py"
  SWEEP_ISOLATED_MODE=all \
    sweep_orphans "隔离验收链路（速度/碰撞半）" "${ISOLATED_CHAIN_NEEDLES[@]}"
else
  # 另一半点名用：它们一条 needle 都不含 Nav2 marker，只能靠节点名单独找。
  SWEEP_ISOLATED_MODE=report \
    sweep_orphans "隔离验收链路（速度/碰撞半）" "${ISOLATED_CHAIN_NEEDLES[@]}"
fi

printf '已停 %d 个进程/进程组' "$stopped" >&2
if (( warned )); then
  printf '，%d 项需要人工确认（见上）' "$warned" >&2
fi
printf '。\n' >&2
printf '%s\n' '本脚本没有向机器人发送任何运动指令，也没有启用/禁用执行闸门。' >&2

if (( ${#skipped_isolated[@]} )) && (( ! INCLUDE_ISOLATED )); then
  # 必须点名并把出口写出来。操作员读到的若是"已停 N 个"，会以为图上干净了，
  # 而 start 下一轮照样以 exit 2 拒绝（它扫到的是同一批 cmdline）。
  printf '\n' >&2
  printf '注意：隔离验收链路还有 %d 个进程在跑，本次**没有**动它们：\n' \
    "${#skipped_isolated[@]}" >&2
  for pid in "${skipped_isolated[@]}"; do
    printf '  pid %-8s %s\n' \
      "$pid" "$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || echo '<已退出>')" >&2
  done
  printf '%s\n' '  它们是验收用的那条隔离链路（出口 /go2w/acceptance/），不是正常入口。' >&2
  printf '%s\n' '  要用正常入口起一套，先整条停掉它们：' >&2
  printf '    bash %s --include-isolated-acceptance\n' "${BASH_SOURCE[0]}" >&2
  # **不计入 warned**：隔离验收链路活着是它该有的状态，不是故障。计进去会让
  # 只跑验收的那些天里 stop_all.sh 每次都报"有问题"，警告被稀释成噪音。
  # 真正让下一轮 start 起不来这件事，由 start 脚本自己以 exit 2 说清楚。
fi

if (( warned )); then
  exit 1
fi
exit 0
