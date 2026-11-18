#!/usr/bin/env bash
# 离线自检：五道闸门接线是否真的通 —— 网关 → Unix socket → 网络代理 → 五个 Bool 话题。
#
#   bash scripts/go2w/check_motion_gate_chain.sh
#
# 不需要机器狗，**也不碰正在跑的宿主 ROS 图**：脚本把两个进程关在一个隔离域里
# （RMW 换 fastrtps + ROS_DOMAIN_ID=77 + ROS_LOCALHOST_ONLY=1），所以它发的
# 发现报文只在本机 domain 77 内，与宿主那套 cyclonedds domain 0 互不可见。
#
# 为什么要这么一个脚本：`motion_gate_state_core` 的 33 个用例只覆盖折算规则，
# 覆盖不到"节点真的建了发布者、真的按周期发"这一层。2026-10-10 第一次跑这个脚本
# 就抓到 `create_timer` 被当成一次性用——自检每 3 秒重跑一次、把**自己**数成
# "已存在的发布者"，刷五条假冲突告警。那种缺陷任何纯逻辑用例都照不出来。
#
# 退出码：0 = 全过；非 0 = 有断言失败（失败点会打印出来）。
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
ros_python="${GO2W_ROS_PYTHON:-/usr/bin/python3}"

# 运动栈跑在宿主（交接书 §5「运动栈改在 x86_64 宿主跑」），所以这里要的是宿主那套
# Humble 工作区；狗上是 Foxy，跑这个脚本没有意义，直接说清楚而不是报一堆 import 错。
[[ -f /opt/ros/humble/setup.bash ]] || {
  printf 'ERROR: 需要宿主 Humble 环境（运动栈跑在宿主，不在机器狗上）\n' >&2
  exit 2
}
work="$(mktemp -d /tmp/go2w_gate_check.XXXXXX)"
lease_dir="${work}/lease"
mkdir -p "${lease_dir}"

gateway_pid=""
proxy_pid=""
lease_refresh_pid=""
arbiter_pid=""
cleanup() {
  [[ -n "${lease_refresh_pid}" ]] && kill "${lease_refresh_pid}" 2>/dev/null || true
  [[ -n "${proxy_pid}" ]] && kill "${proxy_pid}" 2>/dev/null || true
  [[ -n "${gateway_pid}" ]] && kill "${gateway_pid}" 2>/dev/null || true
  [[ -n "${arbiter_pid}" ]] && kill "${arbiter_pid}" 2>/dev/null || true
  wait 2>/dev/null || true
  rm -rf "${work}"
}
trap cleanup EXIT

failures=0
ok()   { printf '  [OK]   %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; failures=$((failures + 1)); }

# --------------------------------------------------------------------------- #
# 隔离环境                                                                     #
# --------------------------------------------------------------------------- #
cat >"${work}/env.sh" <<'EOF'
# 必须**先** source 再开 set -u：Humble 的 setup.bash 在 nounset 下会直接退出。
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source external/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source unitree_go2w_control/ros2_ws/install/setup.bash >/dev/null 2>&1
# 场景 6 要起 go2w_control_arbiter，它在项目自己的 ros2_ws 里。这个 overlay 的
# setup.bash 记了 Noetic underlay，下面的 strip_noetic 会清掉。
source ros2_ws/install/setup.bash >/dev/null 2>&1

# 这两个 workspace 的 setup.bash 记了 Noetic underlay；不过滤的话 rclpy 会拿到
# ROS 1 的 std_srvs（报 _TYPE_SUPPORT）。与 setup_go2w_ros2.sh 同法。
strip_noetic() {
  local out="" entry old_ifs="$IFS"
  IFS=":"
  for entry in $1; do
    case "$entry" in
      /opt/ros/noetic|/opt/ros/noetic/*) ;;
      *) out="${out:+${out}:}${entry}" ;;
    esac
  done
  IFS="$old_ifs"
  printf '%s' "$out"
}
export PYTHONPATH="$(strip_noetic "${PYTHONPATH:-}")"
export LD_LIBRARY_PATH="$(strip_noetic "${LD_LIBRARY_PATH:-}")"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
EOF
cd "${project_root}"

# --------------------------------------------------------------------------- #
# 订阅端：QoS 照抄 bridge_node（默认 depth 10 = VOLATILE）                      #
# --------------------------------------------------------------------------- #
cat >"${work}/listen.py" <<'EOF'
"""订阅五道闸门，等期望值出现；打印实测值，按期望判定退出码。

QoS 刻意用 bridge_node 的默认（depth 10、VOLATILE）：只有与桥同款的订阅者能收到，
才证明接线对桥有效。用 TRANSIENT_LOCAL 订阅反而更严（收不到 VOLATILE 的那条）。
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

TOPICS = (
    "/go2w/software_arm",
    "/go2w/sport_lease/alive",
    "/go2w/robot_error_zero",
    "/go2w/emergency_stop",
    "/go2w/remote_override",
)
expected = json.loads(sys.argv[1])
seconds = float(sys.argv[2])

rclpy.init()
node = Node("go2w_gate_check_listener")
values: dict[str, bool] = {}

for topic in TOPICS:
    def callback(message: Bool, topic: str = topic) -> None:
        values[topic] = bool(message.data)

    node.create_subscription(Bool, topic, callback, 10)

deadline = time.time() + seconds
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

rc = 0
for topic in TOPICS:
    got = values.get(topic)
    want = expected.get(topic)
    mark = "OK  " if got == want else "FAIL"
    if got != want:
        rc = 1
    print(f"  [{mark}] {topic:26s} 实测={got} 期望={want} pubs={node.count_publishers(topic)}")
node.destroy_node()
rclpy.shutdown()
raise SystemExit(rc)
EOF

start_gateway() {
  rm -f "${work}/gateway.sock"
  bash -c "source '${work}/env.sh'; exec '${ros_python}' scripts/go2w/motion_local_gateway.py \
    --socket '${work}/gateway.sock' --lease-status-dir '${lease_dir}'" \
    >"${work}/gateway.log" 2>&1 &
  gateway_pid=$!
  for _ in $(seq 1 60); do
    [[ -S "${work}/gateway.sock" ]] && return 0
    kill -0 "${gateway_pid}" 2>/dev/null || break
    sleep 0.5
  done
  printf '网关没起来，日志：\n'; cat "${work}/gateway.log"; return 1
}

start_proxy() {
  bash -c "source '${work}/env.sh'; exec '${ros_python}' scripts/go2w/motion_network_proxy.py \
    --socket '${work}/gateway.sock' $1" >"${work}/proxy.log" 2>&1 &
  proxy_pid=$!
  for _ in $(seq 1 60); do
    grep -q "gate state publishers:" "${work}/proxy.log" && return 0
    kill -0 "${proxy_pid}" 2>/dev/null || break
    sleep 0.5
  done
  printf '代理没起来，日志：\n'; cat "${work}/proxy.log"; return 1
}

stop_proxy() {
  [[ -n "${proxy_pid}" ]] && kill "${proxy_pid}" 2>/dev/null || true
  wait "${proxy_pid}" 2>/dev/null || true
  proxy_pid=""
  sleep 1
}

# 租约是**心跳**不是一次性标志：`hold_sport_lease.py` 会持续 touch，
# 网关按 1.5 s 无心跳判失效。所以自检也必须持续 touch —— 只 touch 一次的话，
# 监听窗口跑到 1.5 s 之后就自己过期了，会把"接线断了"和"心跳停了"混为一谈。
start_lease_refresh() {
  printf '1\n' >"${lease_dir}/alive"
  printf '4242\n' >"${lease_dir}/id"
  touch "${lease_dir}/heartbeat"
  ( while true; do touch "${lease_dir}/heartbeat"; sleep 0.2; done ) &
  lease_refresh_pid=$!
}
stop_lease_refresh() {
  [[ -n "${lease_refresh_pid}" ]] && kill "${lease_refresh_pid}" 2>/dev/null || true
  wait "${lease_refresh_pid}" 2>/dev/null || true
  lease_refresh_pid=""
}
make_lease_stale() {
  stop_lease_refresh
  touch -d '10 seconds ago' "${lease_dir}/heartbeat"
}

expect() {  # expect <说明> <期望 JSON> [秒]
  local label="$1" want="$2" seconds="${3:-6}"
  printf '%s\n' "${label}"
  if bash -c "source '${work}/env.sh'; exec '${ros_python}' '${work}/listen.py' '${want}' '${seconds}'"; then
    ok "${label}"
  else
    bad "${label}"
  fi
}

printf '\n== 五道闸门接线自检（隔离域 domain 77）==\n\n'

start_gateway || { bad "网关启动"; exit 1; }
ok "网关起来了（socket 权限 $(stat -c '%a' "${work}/gateway.sock")）"

# --- 场景 1：租约新鲜、代理默认（不许操作员断言） ---------------------------- #
start_lease_refresh
start_proxy "" || { bad "代理启动"; exit 1; }
expect "租约新鲜 + 默认参数：lease 放行、remote_override 仍拦" \
  '{"/go2w/software_arm": false, "/go2w/sport_lease/alive": true,
    "/go2w/robot_error_zero": false, "/go2w/emergency_stop": false,
    "/go2w/remote_override": true}'

# --- 场景 1b：第一次读数到手之前，一个值都不许发 ----------------------------- #
# ``gateway unavailable`` 这个 note 是**专属指纹**：它只在 ``_gate_payload is None``
# 时产生（从没读到过）。网关后来挂掉走的是另一个分支（``gateway error: ...``，
# 见场景 5），不会误伤这条断言。
#
# 为什么必须断言：那一组 fail-closed 默认值里 ``emergency_stop=True`` 是**在断言
# "急停被按下"**，而控制仲裁器那边是 ``self._emergency = self._emergency or data``
# —— **锁存**。2026-10-10 真机上就是这么把仲裁器永久卡死在 emergency_stop_active 的
# （重启后立刻变成 remote_override_active_or_unverified，实证）。消费端各自的初值
# 本来就是拦住，所以"沉默"不会让任何一道闸门变松。
#
# 反向校准过：把 ``if payload is None: return`` 去掉，本断言立即失败。
if grep -q "gateway unavailable" "${work}/proxy.log"; then
  bad "代理在读到网关之前就发布了 fail-closed 默认值（会被锁存型消费者永久卡死）"
else
  ok "读到网关之前不发任何值（首发前保持沉默）"
fi

# --- 场景 2：发布者只建一次（自检定时器必须是"一次性"的） -------------------- #
sleep 7
repeats="$(grep -c "gate state publishers:" "${work}/proxy.log" || true)"
conflicts="$(grep -c "already has" "${work}/proxy.log" || true)"
if [[ "${repeats}" == "1" && "${conflicts}" == "0" ]]; then
  ok "发布者只建一次（7 秒内无重复自检、无假冲突告警）"
else
  bad "发布者自检重复执行：'gate state publishers' x${repeats}，'already has' x${conflicts}"
fi

# --- 场景 3：租约过期 -> 闸门收回 ------------------------------------------- #
make_lease_stale
sleep 1
expect "租约过期：lease 闸门收回" \
  '{"/go2w/software_arm": false, "/go2w/sport_lease/alive": false,
    "/go2w/robot_error_zero": false, "/go2w/emergency_stop": false,
    "/go2w/remote_override": true}' 4

# --- 场景 4：操作员显式断言遥控器已清 -> remote_override 放行 ---------------- #
stop_proxy
start_lease_refresh
start_proxy "--remote-override-cleared" || { bad "代理重启"; exit 1; }
expect "显式断言 remote_override 已清：该闸门放行" \
  '{"/go2w/software_arm": false, "/go2w/sport_lease/alive": true,
    "/go2w/robot_error_zero": false, "/go2w/emergency_stop": false,
    "/go2w/remote_override": false}'

# --- 场景 5：网关整个不在 -> 五个闸门全回 fail-closed ------------------------ #
stop_proxy
kill "${gateway_pid}" 2>/dev/null || true
wait "${gateway_pid}" 2>/dev/null || true
gateway_pid=""
rm -f "${work}/gateway.sock"
start_proxy "--remote-override-cleared" || { bad "代理重启（无网关）"; exit 1; }
expect "网关不可达：即使操作员断言过，五个闸门仍全部拦住" \
  '{"/go2w/software_arm": false, "/go2w/sport_lease/alive": false,
    "/go2w/robot_error_zero": false, "/go2w/emergency_stop": true,
    "/go2w/remote_override": true}'

# --- 场景 6：控制仲裁器的接线 ----------------------------------------------- #
# 仲裁器原先那两道闸门（`software_control_armed` / `remote_override_verified`）是
# **节点参数**，而全仓没有任何启动器设置它们 —— `start_nav2_autonomous_search.sh:204`
# 是裸命令 `ros2 run ... arbiter`，不带 `-p`。于是它**恒拦所有运动**：那是一道没有
# 输入的死门，不是一道收紧的门。现在改成订阅 `/go2w/software_arm` 与
# `/go2w/remote_override`，与 `go2w_cmd_vel_bridge` 消费的是同一组话题；
# `remote_override` 的唯一断言行留在代理侧（`--remote-override-cleared`），
# 不在仲裁器再设第二个开关。
#
# 阶段②是**分辨点**：把话题发成 False，旧码仍答 `remote_override_active_or_unverified`。
# 阶段①测的是"一条读数都没收到"时的 fail-closed 默认值。
# 阶段⑤刻意保留：急停是**锁存**的，本轮**没有**动它。
# 场景 5 那个代理还活着，而且**正按 5 Hz 发着 fail-closed 默认值**
# （含 `emergency_stop=True`）—— 第一次跑这一节就是这么被抓的：仲裁器一启动就被
# 那一帧锁存，阶段① 直接是 emergency_stop_active。这既是本节的必要前置，
# 也是"锁存 + 上游谎报"这条因果链在隔离域里的**可复现复现**。
stop_proxy

cat >"${work}/arbiter_probe.py" <<'PYEOF'
"""控制仲裁器接线探针：那两道闸门是读话题，还是读参数。"""
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

BOOL_TOPICS = ("/go2w/remote_override", "/go2w/software_arm", "/go2w/emergency_stop")

rclpy.init()
node = Node("go2w_arbiter_wiring_probe")
values = {
    "/go2w/remote_override": True,  # 没有断言行 -> 拦
    "/go2w/software_arm": False,    # 没有武装 -> 拦
    "/go2w/emergency_stop": False,
}
publishers = {topic: node.create_publisher(Bool, topic, 10) for topic in BOOL_TOPICS}
cmd_pub = node.create_publisher(Twist, "/go2w/nav2_cmd_vel", 10)
publishing = [False]
send_command = [False]
status = {}
selected = {}

node.create_subscription(
    String, "/go2w/control_status", lambda m: status.__setitem__("v", m.data), 10
)
node.create_subscription(
    Twist,
    "/go2w/cmd_vel_selected",
    lambda m: selected.__setitem__("v", (m.linear.x, m.angular.z)),
    10,
)


def pump() -> None:
    while True:
        if publishing[0]:
            for topic in BOOL_TOPICS:
                publishers[topic].publish(Bool(data=values[topic]))
            if send_command[0]:
                message = Twist()
                message.linear.x = 0.1
                cmd_pub.publish(message)
        time.sleep(0.05)


threading.Thread(target=pump, daemon=True).start()


def settle(seconds: float) -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


failures = 0


def check(label: str, got: object, want: object) -> None:
    global failures
    good = got == want
    failures += 0 if good else 1
    print(f"  [{'OK  ' if good else 'FAIL'}] {label}: 实测={got!r} 期望={want!r}")


settle(2.0)
check("① 一条读数都没收到：默认拦住", status.get("v"), "remote_override_active_or_unverified")

publishing[0] = True
values["/go2w/remote_override"] = False
settle(1.5)
check(
    "② 话题说遥控已清 -> 立即读它（旧码答 remote_override_active_or_unverified）",
    status.get("v"),
    "software_control_not_armed",
)

values["/go2w/software_arm"] = True
send_command[0] = True
settle(1.5)
velocity = selected.get("v") or (0.0, 0.0)
check("③ 话题说已武装 + 新鲜 nav2 指令 -> 放行（选中速度非零）", abs(velocity[0]) > 0.01, True)

values["/go2w/emergency_stop"] = True
settle(1.0)
check("④ 急停拉高 -> 立即拦", status.get("v"), "emergency_stop_active")

values["/go2w/emergency_stop"] = False
settle(1.5)
check("⑤ 急停回落仍拦住（锁存刻意保留、无复位路径）", status.get("v"), "emergency_stop_active")

node.destroy_node()
rclpy.shutdown()
print(f"  arbiter wiring: {'PASS' if failures == 0 else f'{failures} FAIL'}")
raise SystemExit(0 if failures == 0 else 1)
PYEOF

setsid bash -c "source '${work}/env.sh'; exec ros2 run go2w_control_arbiter arbiter" \
  >"${work}/arbiter.log" 2>&1 &
arbiter_pid=$!
sleep 8
if bash -c "source '${work}/env.sh'; exec '${ros_python}' '${work}/arbiter_probe.py'"; then
  ok "控制仲裁器接线（读话题、不读参数；急停锁存保持）"
else
  bad "控制仲裁器接线"
  printf '    仲裁器日志尾部：\n'
  tail -5 "${work}/arbiter.log" | sed 's/^/      /'
fi
# `ros2 run` 是**两层进程**（包装层 + 节点）。setsid 让整组同属一个进程组，
# 所以按进程组杀才不会留下孤儿节点继续发话题。
kill -TERM -- "-${arbiter_pid}" 2>/dev/null || kill -TERM "${arbiter_pid}" 2>/dev/null || true
wait "${arbiter_pid}" 2>/dev/null || true
arbiter_pid=""

printf '\n== 结果：%s ==\n' "$([[ ${failures} -eq 0 ]] && echo '全部通过' || echo "${failures} 项失败")"
exit $(( failures == 0 ? 0 : 1 ))
