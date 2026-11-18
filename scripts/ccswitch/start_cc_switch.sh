#!/usr/bin/env bash
# start_cc_switch.sh —— 启动 cc-switch，并给它一份「不被本地代理拖死」的出站环境。
#
# 背景（2026-09-09 实测定位）：
#   cc-switch 的 reqwest 默认跟随系统代理（本机 ALL_PROXY=socks5://127.0.0.1:17891）。
#   当 17891 那个代理节点对某些站点不通时——实测 github 200，但 google /
#   api.justwoker.icu / ps.air-outer.com 一律 5s 超时——cc-switch 转发上游就报
#   「上游连接失败: error sending request」，Codex/Claude 侧表现为
#   `unexpected status 502 Bad Gateway: CC Switch local proxy failed ...`。
#   而这些上游**直连正常**（api.justwoker.icu 直连 200 / 0.3s），所以把上游域名
#   加进 NO_PROXY 让 cc-switch 直连即可——与 ~/.claude/settings.json 里已有的
#   `NO_PROXY=...,api.justwoker.icu,.justwoker.icu` 完全同一口径。
#
# 用法：
#   bash scripts/ccswitch/start_cc_switch.sh              # 停掉旧的、按修正环境重启
#   bash scripts/ccswitch/start_cc_switch.sh --status     # 只看状态
#   CC_SWITCH_NO_PROXY="..." bash ...                     # 覆盖 NO_PROXY 清单
#   CC_SWITCH_USE_SYSTEM_PROXY=1 bash ...                 # 不注入 NO_PROXY（回到跟随系统代理）
set -euo pipefail

BIN=${CC_SWITCH_BIN:-/usr/bin/cc-switch}
PORT=${CC_SWITCH_PORT:-15721}
DEFAULT_NO_PROXY="localhost,127.0.0.1,::1,api.justwoker.icu,.justwoker.icu,ps.air-outer.com,.air-outer.com,agentrouter.org,.agentrouter.org"
NO_PROXY_LIST="${CC_SWITCH_NO_PROXY:-$DEFAULT_NO_PROXY}"

pids() { pgrep -f '[/]usr/bin/cc-switch' || true; }

status() {
  local p; p=$(pids | head -1)
  echo "进程: ${p:-未运行}"
  echo "端口: $(ss -tln 2>/dev/null | grep -c ":$PORT " ) 个监听于 $PORT"
  if [ -n "$p" ]; then
    echo "NO_PROXY(进程内): $(tr '\0' '\n' < "/proc/$p/environ" | grep -m1 '^NO_PROXY=' || echo '(未设置 → 跟随系统代理)')"
    echo "代理变量(进程内): $(tr '\0' '\n' < "/proc/$p/environ" | grep -ciE '^(ALL_PROXY|all_proxy|HTTP_PROXY|http_proxy|HTTPS_PROXY|https_proxy)=' ) 个（0 = 已剥离，直连）"
  fi
  curl -s -o /dev/null -w "路由自检: http=%{http_code} t=%{time_total}s\n" "http://127.0.0.1:$PORT/v1/models" || true
}

if [ "${1:-}" = "--status" ]; then status; exit 0; fi

# 1) 从正在运行的实例继承图形环境（DISPLAY / Wayland / D-Bus），否则用兜底值
PID=$(pids | head -1)
if [ -n "$PID" ]; then
  for v in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS XAUTHORITY; do
    val=$(tr '\0' '\n' < "/proc/$PID/environ" 2>/dev/null | grep -m1 "^$v=" || true)
    if [ -n "$val" ]; then export "${val?}"; fi
  done
fi
export DISPLAY=${DISPLAY:-:0}

# 2) 出站环境修正：默认剥离 6 个代理变量（与 ~/.local/share/applications/cc-switch.desktop
#    覆盖里的 `env -u HTTP_PROXY ...` 完全一致），并注入 NO_PROXY 兜底。
#    保留代理：CC_SWITCH_KEEP_PROXY=1
if [ "${CC_SWITCH_KEEP_PROXY:-0}" != "1" ]; then
  unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
fi
if [ "${CC_SWITCH_USE_SYSTEM_PROXY:-0}" != "1" ]; then
  export NO_PROXY="$NO_PROXY_LIST" no_proxy="$NO_PROXY_LIST"
fi

# 3) 停掉旧实例
if [ -n "$PID" ]; then
  echo "停止旧 cc-switch (pid $PID)"
  pkill -f '[/]usr/bin/cc-switch' || true
  for _ in $(seq 1 40); do pids | grep -q . || break; sleep 0.25; done
  pids | grep -q . && { pkill -9 -f '[/]usr/bin/cc-switch' || true; sleep 1; }
fi

# 4) 启动（cc-switch 会按 proxy_config(codex).enabled=1 自动恢复 Codex 路由接管）
setsid nohup "$BIN" >/dev/null 2>&1 &
sleep 6

# 5) 自检
if ss -tln 2>/dev/null | grep -q ":$PORT "; then
  echo "OK: cc-switch 已启动，127.0.0.1:$PORT 监听中"
  echo "    NO_PROXY=$NO_PROXY"
  echo "    提示：切换供应商后需在 VS Code 里 Developer: Reload Window 才生效"
else
  echo "FAIL: $PORT 未监听，请检查 cc-switch 是否启动成功" >&2
  exit 1
fi
