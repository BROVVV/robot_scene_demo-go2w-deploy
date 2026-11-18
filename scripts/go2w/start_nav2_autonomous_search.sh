#!/usr/bin/env bash
set -euo pipefail

# GO2-W Nav2 自主搜索唯一入口（计划书 §9）。
#
#   bash scripts/go2w/start_nav2_autonomous_search.sh
#       # 默认：只规划、不执行。坐标桥 + planner_server + bt_navigator 起来，
#       # 地图、TF、/go2w/nav/localization_valid 都在，规划链路可查；但控制器/
#       # 平滑器/碰撞监视器一个都不起，任何速度都不可能进入机器人。
#
#   bash scripts/go2w/start_nav2_autonomous_search.sh --enable-motion
#       # 现场验收通过后才用。即便传了它，真正的运动仍要再过
#       # go2w_cmd_vel_bridge 的九道闸门（运动授权/急停/租约/雷达/定位/…）。
#
#   bash scripts/go2w/start_nav2_autonomous_search.sh --enable-motion \
#       --operator-supervised-rotation
#       # 操作员在场时的转向放行（2026-10-10 用户批准，方案见
#       # docs/GO2W_操作员在场转向放行_方案_20261010.md）。**只**豁免 bridge
#       # 的净空那一条（雷达看不见自身近场，持久证明在软件上开不了），其余
#       # 闸门照旧。必须与 --enable-motion 同时用。
#       # 所有真机 Nav2 执行都必须显式传这一项。RPP 即使沿直线路径也会输出
#       # 小幅角速度修正；缺少现场监督转向放行时，bridge 会把整条速度命令按
#       # rotation_clearance_unvalidated 归零，账本却已经完成预约。
#       # 放行时 /go2w/cmd_vel_bridge/status 写成
#       # allowed_operator_supervised_rotation —— 看得出是靠豁免，不是净空证据。
#
#   bash scripts/go2w/start_nav2_autonomous_search.sh --reuse-only
#       # 上游只复用、绝不代起（计划书 §P4-2）。给"感知另有流程在管"的现场用：
#       # 缺上游时保留旧的严格行为 —— 报出来、退出，让你自己去起。
#
# 没有 --enable-motion 一律只规划；不接受环境变量或隐藏参数绕过。
# --bounded-trial 另选经用户授权的固定次数验收包线；须先 prepare_nav2_trial.py
# 初始化一次 runtime/nav2_acceptance_trial.json。此档独立于大场地放行开关，
# 执行前核对狗的 boot_id、剩余预约数；不豁免现场确认、几何或急停闸门。
# 历史窗口仍是每步 0.2m；新前向窗口为一次、累计最多 3m，并有方向/时间限制。
#
# 分层与归属（改这里的逻辑前先看这一段，否则很容易造出第二套发布者）：
#   §1  感知 / 轮式里程计 —— 复用健康的，缺了按所有权起（见上面 --reuse-only）
#   §2  二维定位        —— start_2d_localization.sh（它自己幂等，且要求 wheel
#                          已就绪、/map 无发布者）
#   §3  控制仲裁器 + cmd_vel bridge
#   §4  4 项就绪证据 + 运行时审计
#   §5  WebUI 与 ROS 侧车 —— start_autonomous_search_web.sh
# plain_slam 那条链不在本脚本范围里（现场自己管）。
#
# 对应的停止入口是 stop_nav2_autonomous_search.sh。两者必须成对使用：本脚本
# 起的三套东西（Nav2 bringup / 控制仲裁器 / cmd_vel bridge）的 pidfile 只有它
# 自己读，删掉 pidfile 或手工 kill 都会让下一轮起出第二套同名节点。

ENABLE_MOTION=0
OPERATOR_SUPERVISED_ROTATION=0
START_UPSTREAMS=1
BOUNDED_TRIAL=0
TRIAL_LEDGER_PATH=""
DETECTOR="${AUTONOMOUS_SEARCH_DEFAULT_DETECTOR:-grounded_sam}"
OPEN_BROWSER=1
BOUNDED_CORE_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --enable-motion) ENABLE_MOTION=1 ;;
    --operator-supervised-rotation) OPERATOR_SUPERVISED_ROTATION=1 ;;
    --reuse-only) START_UPSTREAMS=0 ;;
    --bounded-trial) BOUNDED_TRIAL=1 ;;
    --trial-ledger=*) TRIAL_LEDGER_PATH="${arg#--trial-ledger=}" ;;
    --detector=*) DETECTOR="${arg#--detector=}" ;;
    --no-browser) OPEN_BROWSER=0 ;;
    --bounded-core-only) BOUNDED_CORE_ONLY=1 ;;
    -h|--help)
      # 用 awk 按"文件开头那段注释"取，不再写死行号：写死过一次，加两行说明
      # 就静默截断了（--reuse-only 的说明当时就露不出来）。
      awk 'NR>3 { if (/^#/) { sub(/^# ?/, ""); print } else exit }' "${BASH_SOURCE[0]}"
      exit 0
      ;;
    *)
      printf 'ERROR: unknown argument: %s\n' "$arg" >&2
      printf '只接受 --enable-motion、--operator-supervised-rotation、--reuse-only、--bounded-trial、--bounded-core-only、--detector=...、--no-browser（默认只规划）。\n' >&2
      exit 2
      ;;
  esac
done
if [[ "$DETECTOR" != "grounded_sam" && "$DETECTOR" != "yoloe_seg" ]]; then
  printf 'ERROR: --detector 只接受 grounded_sam 或 yoloe_seg。\n' >&2
  exit 2
fi
if [[ -n "$TRIAL_LEDGER_PATH" && "$BOUNDED_TRIAL" != 1 ]]; then
  printf '%s\n' 'ERROR: --trial-ledger=PATH requires --bounded-trial; never creates or refunds a window.' >&2
  exit 2
fi
if [[ "$BOUNDED_CORE_ONLY" == 1 && "$BOUNDED_TRIAL" != 1 ]]; then
  printf '%s\n' 'ERROR: --bounded-core-only 只能与 --bounded-trial 同用。' >&2
  exit 2
fi

# 转向放行只在执行链路里有意义：单独传它而不传 --enable-motion，bridge 会以
# execution_enabled:=false 起来，豁免根本用不上。这里宁可报错，也不要让它看起来
# 被接受了却没接线 —— "参数写了但没生效"正是本项目反复踩的坑。
if [[ "$OPERATOR_SUPERVISED_ROTATION" == 1 && "$ENABLE_MOTION" == 0 ]]; then
  printf '%s\n' 'ERROR: --operator-supervised-rotation 必须与 --enable-motion 同时使用。' >&2
  exit 2
fi
if [[ "$ENABLE_MOTION" == 1 && "$OPERATOR_SUPERVISED_ROTATION" == 0 ]]; then
  printf '%s\n' 'ERROR: 真机 Nav2 执行必须显式传 --operator-supervised-rotation。' >&2
  printf '%s\n' 'RPP 的直线路径也会产生小幅航向修正；缺少该参数时 bridge 会以' >&2
  printf '%s\n' 'rotation_clearance_unvalidated 拦住命令，并可能先消耗一次性试验额度。' >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
runtime_root="${project_root}/outputs/autonomous_search/runtime"
log_root="${project_root}/outputs/autonomous_search/logs"
mkdir -p "${runtime_root}" "${log_root}" "${runtime_root}/nav2"

nav2_pidfile="${runtime_root}/nav2_bringup.pid"
bridge_pidfile="${runtime_root}/cmd_vel_bridge.pid"
arbiter_pidfile="${runtime_root}/control_arbiter.pid"
nav2_extra_args=()
trial_bridge_max_speed=""
unset GO2W_NAV2_TRIAL_LEDGER
if [[ "$BOUNDED_TRIAL" == 1 ]]; then
  export GO2W_NAV2_TRIAL_LEDGER="$(realpath -e -- "${TRIAL_LEDGER_PATH:-${runtime_root}/nav2_acceptance_trial.json}")"
  trial_review_args=()
  if [[ "$ENABLE_MOTION" == 1 ]]; then trial_review_args+=(--verify-live-robot); fi
  /usr/bin/python3 "${script_dir}/prepare_nav2_trial.py" \
    --ledger "$GO2W_NAV2_TRIAL_LEDGER" \
    --params-output "${runtime_root}/nav2/bounded_trial_params.yaml" "${trial_review_args[@]}"
  trial_bridge_max_speed="$(/usr/bin/python3 - "$GO2W_NAV2_TRIAL_LEDGER" <<'PY'
import json,sys
value=float(json.load(open(sys.argv[1])) .get('maximum_speed_mps',.05))
if value not in (.05,.12,.30): raise SystemExit('unsupported bounded-trial bridge speed')
print(value)
PY
)"
  nav2_extra_args+=("params_file:=${runtime_root}/nav2/bounded_trial_params.yaml")
  nav2_extra_args+=("velocity_smoother_params_file:=${runtime_root}/nav2/bounded_trial_params.yaml")
  # A centimetre-scale goal can move inside the robot footprint before the
  # regular 1 Hz replan fires.  Replanning that already-audited short path then
  # fails although FollowPath is progressing.  The bounded tree computes once;
  # production search keeps its dynamic replanning tree.
  nav2_extra_args+=("bounded_trial:=true")
fi

# ---- 0. 参数与前置检查 ---------------------------------------------------- #
if [[ "$ENABLE_MOTION" == 0 ]]; then
  printf '%s\n' '=============================================================='
  printf '%s\n' ' GO2-W Nav2 自主搜索：只规划、不执行'
  printf '%s\n' ' 未传 --enable-motion。控制器/平滑器/碰撞监视器不会启动，'
  printf '%s\n' ' 不会有速度进入机器人。'
  printf '%s\n' '=============================================================='
else
  printf '%s\n' '=============================================================='
  printf '%s\n' ' GO2-W Nav2 自主搜索：执行链路已启用（--enable-motion）'
  printf '%s\n' ' 现场验收尚未由本项目代为完成；运动仍由 cmd_vel bridge 的'
  printf '%s\n' ' 九道闸门把关。确认场地无人、急停在手后再继续。'
  if [[ "$OPERATOR_SUPERVISED_ROTATION" == 1 ]]; then
    printf '%s\n' ' 转向放行：已开启（--operator-supervised-rotation）'
    printf '%s\n' ' 桥的「净空验证」那一条被豁免，其余闸门照旧。'
    printf '%s\n' ' 近场盲区是真实存在的（实测最差 0.291 m 不可观测）——'
    printf '%s\n' ' 这条通道的安全性依赖现场的人手持即停，不是"已证明安全"。'
  fi
  printf '%s\n' '=============================================================='

  # 计划书 §10：执行授权记录在 configs/go2w/navigation_gate.yaml 的
  # nav2_allow_execute。只有设备所有者明确授权并留下可审查记录后才可设为 true。
  # --bounded-trial 是另行授权的短距验收档，不能扩大为大场地资格；它已在任何
  # 启动动作前核对固定账本与实时 boot_id。
  allow_execute="$(sed -n 's/^[[:space:]]*nav2_allow_execute:[[:space:]]*\([A-Za-z]*\).*/\1/p' \
    "${project_root}/configs/go2w/navigation_gate.yaml" | head -n 1)"
  if [[ "${allow_execute}" != "true" && "$BOUNDED_TRIAL" != 1 ]]; then
    printf '%s\n' 'ERROR: 执行链路未获授权（计划书 §10）。' >&2
    printf '%s\n' '设备所有者明确授权并完成现场验收后，把 configs/go2w/navigation_gate.yaml' >&2
    printf '%s\n' '的 nav2_allow_execute 设为 true，再传 --enable-motion。' >&2
    exit 3
  fi
  if [[ "$BOUNDED_TRIAL" == 1 ]]; then
    printf '%s\n' '仅启用所选固定账本内的包线与剩余额度；不授予大场地连续执行资格。'
  fi

  # 计划书 §9：--enable-motion 仍须过「现有的人工运动确认」。这条确认必须
  # 发生在**拉起任何东西之前**——否则在提示符上按 Ctrl-C 会留下一台已经
  # execution_enabled:=true 的 bridge 和一套起着的 Nav2 控制器。
  # 与 start_autonomous_search_web.sh / start_manual_web_demo.sh 用同一个
  # 口令，不另立第二套。
  if [[ "${GO2W_AREA_CLEARED:-}" != "I_HAVE_CLEARED_THE_AREA" ]]; then
    printf '%s\n' '执行链路需要现场清场确认。' >&2
    printf '%s\n' '清场后可以：export GO2W_AREA_CLEARED=I_HAVE_CLEARED_THE_AREA' >&2
    if [[ ! -t 0 ]]; then
      printf '%s\n' '当前不是交互终端，无法口头确认，退出。' >&2
      exit 2
    fi
    read -r -p 'Type I_CONFIRM to authorize autonomous motion: ' answer
    if [[ "$answer" != "I_CONFIRM" ]]; then
      printf '%s\n' '未确认，退出。' >&2
      exit 2
    fi
  fi
fi

source "${script_dir}/setup_environment.sh"

# Reject unsupported native execution before starting sensors, localization or
# a bridge. Only the fixed, one-use supervised commissioning envelope is
# available on Foxy; regular production execution remains closed.
if [[ "${ROS_DISTRO:-}" == foxy && "$ENABLE_MOTION" == 1 ]]; then
  if [[ "$BOUNDED_TRIAL" != 1 ]]; then
    printf '%s\n' 'ERROR: Foxy production execution is not qualified; only a fresh single 20cm supervised trial is supported.' >&2
    exit 3
  fi
  PYTHONPATH="${project_root}:${PYTHONPATH:-}" /usr/bin/python3 - "$GO2W_NAV2_TRIAL_LEDGER" <<'PY'
from pathlib import Path
import sys
from app.navigation.foxy_supervised_trial import validate_supervised_trial
validate_supervised_trial(sys.argv[1], boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
PY
  nav2_extra_args+=("foxy_supervised_trial_ledger:=${GO2W_NAV2_TRIAL_LEDGER}")
fi

# Geometry is an engineering estimate with a bounded live posture, not a
# manufacturer calibration. Bind the bridge to this exact validated file.
geometry_contract="$(/usr/bin/python3 - "${project_root}/configs/go2w/navigation_sensor_profile.yaml" <<'PY'
import sys
from go2w_description.sensor_profile import load_sensor_profile
p=load_sensor_profile(sys.argv[1])
print(p['execution_scope'],p['profile_sha256'])
PY
)"
read -r geometry_scope geometry_sha256 <<<"$geometry_contract"
if [[ "$ENABLE_MOTION" == 1 && "$BOUNDED_TRIAL" == 0 ]]; then
  if [[ "$geometry_scope" != operator_supervised_field || "$OPERATOR_SUPERVISED_ROTATION" != 1 ]]; then
    printf '%s\n' 'ERROR: 大场地执行需要 operator_supervised_field 几何档和 --operator-supervised-rotation；三步档不能隐式扩大。' >&2
    exit 3
  fi
fi

# 不要再 source ros2_ws/install/setup.bash —— setup_environment.sh 已经装好了 ROS
# 与整个工作区，而且它是**故意绕开 setup.bash** 的：这个工作区的 setup.bash 里记着
# 两个链式前缀（/opt/ros/noetic 与 /opt/ros/humble），直接 source 会把 ROS 1 的
# dist-packages 顶到 PYTHONPATH 最前面，之后任何 rclpy 节点
# `from nav_msgs.msg import Odometry` 拿到的是 ROS 1 的消息类。
# setup_environment.sh 的 source_overlay_safely() 检测到这种「记录了别的 distro」
# 的 setup.bash 时会改 source local_setup.bash，正是为此。
# 顺带：那一遍裸 source 还没被 `set +u` 包住，会在 colcon 那段无引号保护的
# `[ -n "$COLCON_TRACE" ]` 上以 "unbound variable" 直接终止本脚本——症状看着像
# 缺环境变量，真因是这里多 source 了一遍。

owns_pidfile() {
  # 只认本项目自己起的进程：pidfile + cmdline 双重核对，避免误杀。
  local pidfile="$1" needle="$2" pid cmd
  [[ -f "$pidfile" ]] || return 1
  pid="$(<"$pidfile")"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  cmd="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  [[ "$cmd" == *"$needle"* ]]
}

stop_owned() {
  # pidfile 里存的是 setsid 出来的会话首进程，它的 pgid 就等于 pid，所以要对着
  # **整个进程组**发信号，不能只发那一个 pid。
  # `ros2 launch` 收到 SIGINT 后只是尽力把信号转发给子节点；只要有子节点没在下面
  # 这 10 秒里退出，本函数就会 SIGKILL 掉父进程，而那些子节点会被 reparent 到
  # pid 1 继续活着。上一轮留下的坐标桥和 bt_navigator 于是和新一轮的同名节点并存：
  # `/map` 出现两个发布者（§13.2 第 2 项直接失败），lifecycle_manager 的
  # change_state 也会打到旧的那个上——症状是新的 planner_server 永远停在
  # unconfigured，bringup 日志里连一句报错都没有。
  local pidfile="$1" needle="$2" pid
  if owns_pidfile "$pidfile" "$needle"; then
    pid="$(<"$pidfile")"
    printf 'Stopping previous project-owned %s (pid %s, pgid %s) ...\n' "$needle" "$pid" "$pid" >&2
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 40); do
      kill -0 -- "-${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$pidfile"
}

find_running_matching() {
  # 按 cmdline 扫 /proc 找出所有匹配的进程，**不看 pidfile**。
  # 为什么需要它：pidfile 只能证明「本脚本上一轮起过什么」，证明不了「现在有什么」。
  # 手工起过、或者上一轮在写完 pidfile 之后自己退了（实测：2026-10-15 的
  # nav2_bringup.pid=3224461 与 cmd_vel_bridge.pid=2439210 都是死 pid，而真正的
  # 那一套在跑），pidfile 就成了僵尸指针 —— owns_pidfile 判 false，stop_owned
  # 静默 rm 掉它，然后在活着的同名节点旁边再起一套。症状是 /cmd_vel_nav_raw
  # 出现两个发布者、lifecycle_manager 的 change_state 打到旧的那个上，新的
  # planner_server 永远停在 unconfigured，日志里一句报错都没有。
  # 用 /proc/*/cmdline 枚举而不是 `pkill -f`：`pkill -f` 会匹配到调用它的那个
  # shell 自己的 cmdline，把 shell 一起杀掉。
  # 同理，这里按 /proc/$pid/exe 把 shell 排除掉：本脚本起的每一条都用
  # `setsid bash -c 'exec ...'` 把 bash 换成 ros2/节点本体，所以真正要找的进程
  # 的 exe 一定是 python3 或 ELF，绝不可能是 shell。而任何「命令行里恰好含有
  # 这段文字」的外层 shell（例如从别的 shell 里直接贴命令跑）都会被这一步滤掉。
  local needle d pid cmd exe hit found=()
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
    (( hit )) && found+=("$pid")
  done
  (( ${#found[@]} )) && printf '%s\n' "${found[@]}"
  return 0
}

refuse_if_already_running() {
  # 起新一套之前必须确认「没有第二套已经在跑」。宁可停下来让人看见是谁在跑，
  # 也不要静默并存 —— 并存的症状（planner 永远 unconfigured、日志无报错）比
  # 起不来难查得多。
  # 连 cmdline 一起打出来，不只是 pid：本机同时可能有**两套**看起来同名的东西
  # ——正常入口起的那一套，和隔离验收链路（isolated_velocity_chain +
  # robot_scene_nav2.launch.py nav2_cmd_vel_topic:=/go2w/acceptance/...）起的那
  # 一套。光看 pid 分不出是谁，看不出该不该停、由谁停。
  local label="$1" hint="$2" pids pid cmd isolated=0
  shift 2
  pids="$(find_running_matching "$@")"
  [[ -n "${pids//$'\n'/}" ]] || return 0
  printf 'ERROR: %s 已经在跑，本脚本不再起第二套（并存的症状是 planner_server\n' "$label" >&2
  printf '%s\n' '       永远停在 unconfigured，日志里一句报错都没有）：' >&2
  while read -r pid; do
    [[ -n "$pid" ]] || continue
    cmd="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || echo '<读取失败>')"
    # 同一条 cmdline 可能是两套里任意一套。判据与 stop 脚本同一处、逐字一致：
    # 只有隔离那套带 `nav2_cmd_vel_topic:=/go2w/acceptance/`（launch 默认值
    # 是物理入口 /go2w/nav2_cmd_vel）。光看 pid 分不出是谁，提示也就指不对 ——
    # 下面那句 hint 会让操作员去跑 stop 脚本，而 stop 脚本默认**不碰**隔离链路，
    # 于是"按提示做了却还是起不来"。
    [[ "$cmd" == *"nav2_cmd_vel_topic:=/go2w/acceptance/"* ]] && isolated=1
    printf '  pid %-8s %s\n' "$pid" "$cmd" >&2
  done <<<"$pids"
  if (( isolated )); then
    printf '%s\n' '  上面这一套是**隔离验收链路**（出口 /go2w/acceptance/），不是正常入口。' >&2
    printf '%s\n' '  要换成正常入口，先整条停掉它：' >&2
    printf '    bash %s/stop_nav2_autonomous_search.sh --include-isolated-acceptance\n' \
      "${script_dir}" >&2
  else
    printf '%s\n' "$hint" >&2
  fi
  exit 2
}

topic_publisher_count() {
  local topic="$1"
  timeout 5 ros2 topic info "$topic" -v 2>/dev/null \
    | awk '/Publisher count:/ { print $3; exit }'
}

wait_for() {
  local label="$1" timeout_s="$2"
  shift 2
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if "$@"; then
      printf '[OK] %s\n' "$label"
      return 0
    fi
    sleep 0.5
  done
  printf '[FAIL] %s（等待 %ss 超时）\n' "$label" "$timeout_s" >&2
  return 1
}

# 等一组话题各自出现**唯一**发布者。与上面的 wait_for 分开，因为它多了两件
# 事：一次判多个话题，以及**监督进程中途死掉就立刻带日志退出**——否则现场看到
# 的是一次没有归因的 60 s 空等，而原因是监督脚本自己在第一行就 exit 2 了。
wait_for_upstream_topics() {
  local label="$1" supervisor_pid="$2" log_file="$3" timeout_s="$4"
  shift 4
  local deadline=$((SECONDS + timeout_s)) topic ok
  while (( SECONDS < deadline )); do
    ok=1
    for topic in "$@"; do
      [[ "$(topic_publisher_count "$topic")" == "1" ]] || { ok=0; break; }
    done
    if (( ok )); then
      printf '[OK] %s 已就绪（唯一发布者）\n' "$label"
      return 0
    fi
    if [[ -n "$supervisor_pid" ]] && ! kill -0 "$supervisor_pid" 2>/dev/null; then
      printf 'ERROR: %s 的监督进程 %s 已经退出，起不来。日志尾部（%s）：\n' \
        "$label" "$supervisor_pid" "$log_file" >&2
      tail -n 15 "$log_file" >&2 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
  printf 'ERROR: %s 等了 %ss，以下话题仍没有唯一发布者：\n' "$label" "$timeout_s" >&2
  for topic in "$@"; do
    printf '  %-38s pubs=%s\n' "$topic" "$(topic_publisher_count "$topic")" >&2
  done
  return 1
}

# ---- 1. 上游：健康的复用，缺了的按所有权起 ------------------------------- #
# 计划书 §P4-2：一步入口要能"明确复用健康的已有上游，或按所有权启动缺失的
# wheel/传感器/二维 SLAM"。原来的做法是缺一个就 exit 2 让人手工去起——那不叫
# 一步入口。三条规则，按顺序判：
#   * 话题有 **>1** 个发布者 —— 先拒绝。那是两套并存，起新的只会更糟，而且起
#     什么都修不好，必须有人看现场。
#   * 恰好 1 个 —— 复用，并把复用了谁打出来（"明确复用"）。
#   * 0 个 —— 按所有权起，各自记 pidfile，沿用 runtime/go2w/pids/ 那一套
#     （stop_all.sh 认它）。注意它按**进程组**发信号，所以起的时候必须
#     `setsid`，让 pgid == pid；用普通 `&` 起的话 stop 只会打到包装层，
#     节点本身会 reparent 到 pid 1 继续跑 —— 正是"手工重启造出重复发布者"
#     那条老坑。
#
# 感知的**半活状态**（一部分话题有发布者、一部分没有）单独拒绝，见下。
upstream_pid_root="${project_root}/runtime/go2w/pids"
mkdir -p "$upstream_pid_root"

# 0=健康 1=缺 2=有第二家。话题不存在时 topic_publisher_count 返回空串。
upstream_state() {
  case "$(topic_publisher_count "$1")" in
    1) return 0 ;;
    ''|0) return 1 ;;
    *) return 2 ;;
  esac
}

# 默认必须与 Nav2 消费的那条话题逐字一致（config/nav2_params_humble.yaml 的
# obstacle_layer 与 collision_monitor.yaml 都是 /go2w/lidar/scan，由
# start_live_perception.sh 起的 lidar_preprocessor 发布）。写成 /scan 会让前置
# 检查永远失败——那条话题全仓没有发布者。
perception_topics=(
  /camera/front/image_raw
  "${GO2W_SCAN_TOPIC:-/go2w/lidar/navigation_scan}"
  /go2w/lidar/mapping_scan
)
duplicated=()
perception_alive=()
perception_missing=()
for topic in "${perception_topics[@]}"; do
  state=0
  # `set -e` 下不能直接 `state=$?`：函数返回非 0 会在这一行就把脚本带走。
  upstream_state "$topic" || state=$?
  case "$state" in
    0) perception_alive+=("$topic") ;;
    2) duplicated+=("$topic") ;;
    *) perception_missing+=("$topic") ;;
  esac
done

if [[ "${#duplicated[@]}" -gt 0 ]]; then
  printf '%s\n' 'ERROR: 以下上游话题有**不止一个**发布者，说明有两套在并存：' >&2
  printf '  - %s\n' "${duplicated[@]}" >&2
  printf '%s\n' '本脚本不替你决定停哪一套。先看清是谁在发，再停本项目记账的那些：' >&2
  printf '%s\n' '  bash scripts/go2w/stop_all.sh' >&2
  printf '%s\n' '  ros2 topic info <话题> -v | sed -n "/Node name/p"' >&2
  exit 2
fi

if [[ "${#perception_missing[@]}" -gt 0 ]]; then
  # 半活状态**不能**直接调感知监督脚本：start_live_perception.sh 会把那 7 个
  # 节点整套起一遍（它自己**没有任何幂等检查**，见该脚本第 108-140 行是无条件
  # 的一串 start_read_only_node），于是活着的那些（比如相机桥）立刻多出第二个
  # 发布者 —— 正是本脚本要消灭的那类重复。这种状态只能让人先把剩下的停干净。
  #
  # 判据不能只看那三个话题：那 7 个节点里 description / time / lidar_watchdog /
  # fusion / live_bridge 都不发这三条话题，所以"三个话题全是 0"完全可能是
  # "上一轮只剩这几条半死不活"。真正的账本是监督脚本自己写的那组 pidfile
  # （runtime/go2w/pids/{description,camera,time,lidar,lidar_watchdog,fusion,
  # live_bridge}.pid，它 exit 时整个删掉）。只要有**任意一个**还活着，就说明
  # 现场已经有一份感知，起第二份必然重复 —— 一律拒绝。
  #
  # `start_live_perception` 这个名字不是新起的：start_semantic_exploration.sh:189
  # 冷启动感知时就用的它。两边必须逐字一致 —— 否则那条路径起的监督进程在这边
  # 看不见（反过来也一样），"半活"门就漏了。
  perception_owned_leftovers=()
  for name in description camera time lidar lidar_watchdog fusion live_bridge \
    start_live_perception; do
    owner_pidfile="${upstream_pid_root}/${name}.pid"
    [[ -f "$owner_pidfile" ]] || continue
    owner_pid="$(tr -cd '0-9' <"$owner_pidfile")"
    [[ -n "$owner_pid" ]] && kill -0 "$owner_pid" 2>/dev/null \
      && perception_owned_leftovers+=("${name}(pid ${owner_pid})")
  done

  if [[ "${#perception_alive[@]}" -gt 0 || "${#perception_owned_leftovers[@]}" -gt 0 ]]; then
    printf '%s\n' 'ERROR: 感知处于"半活"状态，本脚本不接管：' >&2
    [[ "${#perception_alive[@]}" -gt 0 ]] \
      && printf '  已有发布者：%s\n' "${perception_alive[*]}" >&2
    [[ "${#perception_missing[@]}" -gt 0 ]] \
      && printf '  没有发布者：%s\n' "${perception_missing[*]}" >&2
    [[ "${#perception_owned_leftovers[@]}" -gt 0 ]] \
      && printf '  本项目记账的感知进程还活着：%s\n' "${perception_owned_leftovers[*]}" >&2
    printf '%s\n' '  整条起一遍会让活着的那些多出第二个发布者。先停干净：' >&2
    printf '%s\n' '    bash scripts/go2w/stop_all.sh' >&2
    printf '%s\n' '  （start_live_perception.sh 自己没有任何幂等检查，别直接再跑一遍）' >&2
    exit 2
  fi
  if [[ "$START_UPSTREAMS" == 0 ]]; then
    printf '%s\n' 'ERROR: 以下上游话题没有发布者，且传了 --reuse-only（只复用、不代起）：' >&2
    printf '  - %s\n' "${perception_missing[@]}" >&2
    printf '%s\n' '去掉 --reuse-only，或手工起：bash scripts/go2w/start_live_perception.sh' >&2
    exit 2
  fi
  perception_log="${log_root}/start_live_perception.log"
  printf '缺 %d 个感知上游话题，按所有权启动感知监督（start_live_perception.sh）...\n' \
    "${#perception_missing[@]}"
  setsid bash "${script_dir}/start_live_perception.sh" >"$perception_log" 2>&1 &
  perception_pid=$!
  # 记账。stop_all.sh 是遍历这个目录、按**进程组**发信号的，而监督脚本给它那 7 个
  # 节点各自 `setsid`（见 start_live_perception.sh:60），每个都是**独立会话**，
  # 不在监督进程的组里 —— 所以两者都要记：7 个节点靠监督脚本自己写的 pidfile，
  # 监督进程本身靠这一条（名字与 start_semantic_exploration.sh:189 一致）。
  # 漏了这条，stop_all.sh 停完 7 个节点后监督进程还在 `wait` 上挂着，下一轮现场
  # 再手工跑一次 start_live_perception.sh 就会照着那 7 个已死的 pidfile 起出第二套。
  printf '%s\n' "$perception_pid" >"${upstream_pid_root}/start_live_perception.pid"
  wait_for_upstream_topics "感知上游" "$perception_pid" "$perception_log" 120 \
    "${perception_missing[@]}" || exit 2
else
  printf '[OK] 复用已有感知上游：%s\n' "${perception_alive[*]}"
fi

# 轮式里程计。它在 start_2d_localization.sh 里是硬前置（那条脚本查
# publish_tf=True / lio_enabled=False，不符就 exit 2），所以必须在 §2 调它
# **之前**处理好。起法沿用 start_semantic_exploration.sh 的做法。
state=0
upstream_state /go2w/odom/fused || state=$?
if [[ "$state" == 2 ]]; then
  printf '%s\n' 'ERROR: /go2w/odom/fused 有不止一个发布者，有两套轮式里程计在并存。' >&2
  printf '%s\n' '先停干净：bash scripts/go2w/stop_all.sh' >&2
  exit 2
elif [[ "$state" == 0 ]]; then
  printf '[OK] 复用已有上游 /go2w/odom/fused\n'
elif [[ "$START_UPSTREAMS" == 0 ]]; then
  printf '%s\n' 'ERROR: /go2w/odom/fused 没有发布者，且传了 --reuse-only。' >&2
  printf '%s\n' '手工起：ros2 launch go2w_lio_bringup wheel_odom.launch.py' >&2
  exit 2
else
  odom_log="${log_root}/wheel_odom.log"
  printf '%s\n' '缺轮式里程计，按所有权启动（wheel_odom.launch.py）...'
  setsid /usr/bin/python3 "${script_dir}/exec_ros2.py" launch go2w_lio_bringup wheel_odom.launch.py >"$odom_log" 2>&1 &
  odom_pid=$!
  printf '%s\n' "$odom_pid" >"${upstream_pid_root}/wheel_odom.pid"
  wait_for_upstream_topics "轮式里程计" "$odom_pid" "$odom_log" 60 /go2w/odom/fused || exit 2
fi

# ---- 2. Nav2 bringup ------------------------------------------------------ #
stop_owned "$nav2_pidfile" "robot_scene_nav2.launch.py"
refuse_if_already_running "Nav2 bringup" \
  "  先停掉它：bash ${script_dir}/stop_nav2_autonomous_search.sh" \
  "robot_scene_nav2.launch.py"
# A validated saved-map AMCL owner is a first-class production upstream.  The
# old unconditional call requested the default ``mapping`` mode and therefore
# refused every healthy localization-mode process as "another active
# process".  Reuse only our pidfile-bound exact launch; foreign /map owners are
# still rejected by start_2d_localization.sh when no owned launch is present.
localization_mode=mapping
localization_pidfile="${runtime_root}/localization_2d.pid"
reuse_saved_map_localization=0
if [[ -f "$localization_pidfile" ]]; then
  localization_pid="$(tr -cd '0-9' <"$localization_pidfile")"
  if [[ -n "$localization_pid" && -r "/proc/${localization_pid}/cmdline" ]]; then
    localization_cmd="$(tr '\0' ' ' <"/proc/${localization_pid}/cmdline")"
    if [[ "$localization_cmd" == *"localization_2d.launch.py mode:=localization"* ]]; then
      /usr/bin/python3 "${script_dir}/localization_epoch.py" verify --pid "$localization_pid"
      localization_mode=localization
      reuse_saved_map_localization=1
      printf 'Reusing project-owned validated saved-map localization (pid %s).\n' "$localization_pid"
    fi
  fi
fi
if [[ "$reuse_saved_map_localization" != 1 ]]; then
  bash "${script_dir}/start_2d_localization.sh"
fi
if [[ "$ENABLE_MOTION" == 1 ]]; then
  execution_enabled=true
else
  execution_enabled=false
fi
printf 'Starting Nav2 bringup (execution_enabled:=%s) ...\n' "$execution_enabled"
# Bash background jobs inherit ignored SIGINT. Reset it before exec so the
# launcher can forward one graceful shutdown signal to its children.
setsid /usr/bin/python3 "${script_dir}/exec_ros2.py" launch robot_scene_nav_bringup robot_scene_nav2.launch.py \
  "execution_enabled:=${execution_enabled}" localization_source:=external_2d "${nav2_extra_args[@]}" \
  >"${log_root}/nav2_bringup.log" 2>&1 &
printf '%s\n' "$!" >"$nav2_pidfile"

# ---- 3. 控制仲裁与 cmd_vel bridge ---------------------------------------- #
# 即便只规划也要起：它们自身 fail-closed，且「速度链只有一个预期发布者」
# 这条只规划证据要靠它们在图上出现才能核对。
stop_owned "$arbiter_pidfile" "go2w_control_arbiter"
stop_owned "$bridge_pidfile" "go2w_cmd_vel_bridge"
# bridge 的两个 needle 合起来才覆盖完整：`ros2 run` 是两层进程，外层包装的
# cmdline 是 `ros2 run go2w_cmd_vel_bridge bridge`（空格），内层节点的是
# `.../lib/go2w_cmd_vel_bridge/bridge`（斜杠）。只写一个都会漏掉一半。
# 注意不能用 `go2w_cmd_vel_bridge` 本身当 needle：同一个包还装了 velocity_guard，
# 而那个是 nav2 bringup 正常起的，拿它当 needle 会把正常链路判成重复。
refuse_if_already_running "控制仲裁器" \
  "  先停掉它：bash ${script_dir}/stop_nav2_autonomous_search.sh" \
  "go2w_control_arbiter"
refuse_if_already_running "cmd_vel bridge" \
  "  先停掉它：bash ${script_dir}/stop_nav2_autonomous_search.sh" \
  "go2w_cmd_vel_bridge bridge" "go2w_cmd_vel_bridge/bridge"
setsid /usr/bin/python3 "${script_dir}/exec_ros2.py" run go2w_control_arbiter arbiter \
  >"${log_root}/control_arbiter.log" 2>&1 &
printf '%s\n' "$!" >"$arbiter_pidfile"
bridge_args=(--ros-args -p "geometry_execution_scope:=${geometry_scope}" -p "geometry_profile_sha256:=${geometry_sha256}")
if [[ "$ENABLE_MOTION" == 1 ]]; then
  bridge_args+=(-p execution_enabled:=true)
  if [[ "$OPERATOR_SUPERVISED_ROTATION" == 1 ]]; then
    bridge_args+=(-p operator_supervised_rotation:=true)
  fi
fi
if [[ "$BOUNDED_TRIAL" == 1 ]]; then
  bridge_args+=(-p "trial_ledger:=${GO2W_NAV2_TRIAL_LEDGER}")
  bridge_args+=(-p "maximum_linear_x_mps:=${trial_bridge_max_speed}")
fi
setsid /usr/bin/python3 "${script_dir}/exec_ros2.py" run go2w_cmd_vel_bridge bridge "${bridge_args[@]}" \
  >"${log_root}/cmd_vel_bridge.log" 2>&1 &
printf '%s\n' "$!" >"$bridge_pidfile"

# ---- 4. 等齐就绪证据 ------------------------------------------------------ #
# 任何一项没到就不启动搜索：宁可让人看见「哪一项没到」，也不要让搜索跑在
# 半个链路上然后把失败推给下游。
ready_ok=1
wait_for "/map 有唯一发布者" 60 \
  bash -c '[[ "$(timeout 5 ros2 topic info /map -v 2>/dev/null | awk "/Publisher count:/ { print \$3; exit }")" == "1" ]]' || ready_ok=0
# 只确认话题名存在是假绿：bridge 在定位无效时照样以 ~15 Hz 发布 false，
# `grep -qx /go2w/nav/localization_valid` 会照常打 [OK]，搜索却跑在一个
# 被判无效的定位上。必须读值，且必须等到 true 为止（bridge 只在回调里发，
# 冷启动头几帧为 false 是正常的，所以要等而不是查一次）。
#
# 这里不用 `ros2 topic echo`：这台 Foxy 上它对 VOLATILE 话题收不到任何消息
# （实测 /go2w/nav/localization_valid、/go2w/safety/lidar_fresh 全空，
# 而同一条命令对 latched 的 /map 正常），空输出会被误读成「话题没在发」。
wait_for "/go2w/nav/localization_valid 为 true" 60 \
  /usr/bin/python3 -c '
import sys, time
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)
    from std_msgs.msg import Bool
except Exception:
    sys.exit(1)
rclpy.init()
node = Node("nav2_startup_localization_valid_probe")
got = []
node.create_subscription(
    Bool, "/go2w/nav/localization_valid", lambda m: got.append(bool(m.data)),
    QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
               reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.VOLATILE))
deadline = time.time() + 8.0
while time.time() < deadline and not got:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got and got[-1] else 1)
' || ready_ok=0
# TF 连通必须在同一个 rclpy 进程里查，否则每个 ros2 CLI 调用都是一次新的发现。
wait_for "map -> odom_fused -> base_link TF 连通" 60 \
  /usr/bin/python3 -c '
import sys
try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
except Exception:
    sys.exit(1)
rclpy.init()
node = Node("nav2_startup_tf_probe")
buffer = Buffer()
TransformListener(buffer, node)
ok = False
for _ in range(40):
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        buffer.lookup_transform("map", "base_link", rclpy.time.Time(), Duration(seconds=0.2))
        ok = True
        break
    except Exception:
        continue
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
' || ready_ok=0
# 直接调用 lifecycle 服务，不经 ros2 CLI/daemon。Jetson 高负载冷启动时曾实测
# managed nodes 早已 active，而 `ros2 lifecycle get` 的 CLI 发现进程连续超时，
# 造成 60 秒假失败。仍逐字比较 label，不能把 inactive 当成 active。
wait_for "planner_server 生命周期 active" 60 \
  /usr/bin/python3 -c '
import sys
try:
    import rclpy
    from lifecycle_msgs.srv import GetState
except Exception:
    sys.exit(1)
rclpy.init()
node = rclpy.create_node("nav2_planner_lifecycle_probe")
client = node.create_client(GetState, "/planner_server/get_state")
ok = False
if client.wait_for_service(timeout_sec=4.0):
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=4.0)
    try:
        ok = future.done() and future.result().current_state.label == "active"
    except Exception:
        ok = False
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
' || ready_ok=0

if [[ "$ready_ok" != 1 ]]; then
  printf 'ERROR: 就绪证据不齐，未启动搜索。日志：%s/nav2_bringup.log\n' "${log_root}" >&2
  # map -> odom_fused -> base_link 断链最常见的成因是轮式里程计没起。走到这里
  # 说明 §1 已经查过它的发布者：要么是复用的那份（那它就在跑，断链另有原因），
  # 要么是本脚本按所有权起的那份（那它起过、但 TF 仍不通）。所以这里不再说
  # "我没管它"，而是给出**顺着查**的下一步 —— 否则现场看到的是一个没有归因的
  # TF [FAIL]，最容易误判成"启动顺序不对，再跑一次"。
  printf '%s\n' '提示：若失败项是 map -> odom_fused -> base_link TF，按顺序查（§1 已确认它有唯一发布者）：' >&2
  printf '%s\n' '  1) ros2 topic info /go2w/odom/fused -v | sed -n "/Publisher count/p"   # 必须是 1' >&2
  printf '%s\n' '  2) ros2 run tf2_ros tf2_echo odom_fused base_link                       # 有没有在动' >&2
  printf '%s\n' '  3) ros2 topic echo /go2w/odom/fused --once | head -20                   # 帧 id 对不对' >&2
  printf '%s\n' '  起它的日志：%s/wheel_odom.log（本脚本起的）或现场自己那份' "${log_root}" >&2
  exit 3
fi

# Live-data audit complements the checks above: cached messages cannot pass it.
# 它查的是上面那些 wait_for 查不到的东西：速度链**每一级发布者的身份**（只数
# count==1 证明不了是谁发的）、物理入口有没有发布者、以及同名节点有没有出现
# 两次。退出码非 0 必须在这里接住并说清楚是哪一条 —— 上面那些 wait_for 失败时
# 会打印 [FAIL] 和出口，这里如果让它直接以 exit 2 冒泡，现场看到的就只是一个
# 没有归因的退出码，而原因全锁在 JSON 里（实测：遗留的
# lifecycle_manager_navigation 与新的 _2d 并存，就是靠这一条查出来的）。
audit_json="${runtime_root}/nav2/startup_audit.json"
audit_extra_args=(--mode "$localization_mode")
if [[ "$ENABLE_MOTION" == 1 ]]; then
  audit_extra_args+=(--require-execution)
fi
if [[ "$BOUNDED_TRIAL" == 1 ]]; then
  audit_extra_args+=(--trial-ledger "$GO2W_NAV2_TRIAL_LEDGER")
fi
if ! /usr/bin/python3 "${script_dir}/audit_navigation_runtime.py" --duration 5 \
  --json "${audit_json}" "${audit_extra_args[@]}"; then
  printf 'ERROR: 运行时审计未通过，未启动搜索。明细：%s\n' "${audit_json}" >&2
  printf '%s\n' '  常见两条：' >&2
  printf '%s\n' '  - lifecycle_manager_count:2 —— 有第二套 lifecycle_manager 在管同一批节点，' >&2
  printf '%s\n' '    按名字做的检查看不出来。先确认是不是遗留进程：' >&2
  printf '%s\n' '      for d in /proc/[0-9]*; do tr "\\0" " " <$d/cmdline 2>/dev/null | grep -q lifecycle_manager && echo "$d"; done' >&2
  printf '%s\n' '  - publisher:<话题> —— 该级发布者不是预期节点，或有第二家。' >&2
  exit 4
fi

# 有界真机验收只需要上面已审计通过的 Nav2/速度/安全链。WebUI、
# 搜索 worker 和识别器不参与单次短距离目标；等待它们会让现场确认窗口
# 在低功耗机器上过期。该选项在前置参数检查中被限定为 --bounded-trial，
# 不得改变正常自主搜索启动语义。
if [[ "$BOUNDED_CORE_ONLY" == 1 ]]; then
  printf '%s\n' '[OK] 有界验收 Nav2 核心链已就绪；按要求不启动 WebUI/搜索 worker。'
  exit 0
fi

# ---- 5. WebUI + 搜索 worker（后端固定 nav2） ------------------------------ #
# 复用旧入口，避免维护第二套 WebUI 启动流程；它负责启 worker，而 worker 的
# 默认后端已经是 nav2（见 autonomous_search_worker.build_argv）。
web_args=(--detector "$DETECTOR")
if [[ "$OPEN_BROWSER" == 0 ]]; then
  web_args+=(--no-browser)
fi
export GO2W_NAV2_EXECUTION_AUTHORIZED=0
if [[ "$ENABLE_MOTION" == 1 ]]; then
  export GO2W_NAV2_EXECUTION_AUTHORIZED=1
  web_args+=(--enable-autonomous-motion)
fi
printf 'Starting WebUI + search worker (backend=nav2, detector=%s) ...\n' "$DETECTOR"
exec bash "${script_dir}/start_autonomous_search_web.sh" "${web_args[@]}"
