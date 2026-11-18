#!/usr/bin/env bash
# 核验"当前这一轮搜索"里 detector 真的发了什么（感知提速阶段一报告 §6.3 第二层）。
#
#   bash scripts/go2w/check_perception_rollback_session.sh                 核验当前会话（严格）
#   bash scripts/go2w/check_perception_rollback_session.sh --history [N]   **明确要求**才浏览历史
#
# 退出码：0 = 读到至少一条 detector_diagnostics；1 = 尚无法验证；2 = 环境错。
#
# 三条"不冒充"的规矩（前两条是第二次复审 F2，第三条是第三次复审）：
#   1. 核验当前会话就只核验当前会话 —— 当前会话的事件流还没落盘时**不退回**历史目录，
#      否则会拿旧实例的诊断当本次回退生效的证据。
#   2. `jq` 零行输出不算通过。
#   3. **会话必须还在跑**：搜索结束后 WebUI 仍保留 `session_id` 与终态 `status`
#      （`current_session()` 只要 `_info` 不为 None 就一直返回），日志也还在。
#      只看 `session_id` 会把一个 OPERATOR_STOP 的旧会话当成"正在跑的实例"，
#      于是报告里"核验跑着的实例"这句话就不成立。
#
# PERCEPTION_CHECK_ROOT 只为隔离测试覆盖仓库根，正常使用不要设。

set -u

cd "${PERCEPTION_CHECK_ROOT:-/home/mxt/robotscene}" || exit 2
command -v jq >/dev/null 2>&1 || {
  echo "找不到 jq（应在 /home/mxt/anaconda3/bin/jq，检查 PATH）" >&2
  exit 2
}

# 只有这些状态才叫"正在跑的这一轮"。**刻意用白名单而不是"排除终态"**：
# 将来 WebUI 多出一个新的终态时，白名单会落回"尚无法验证"（安全的假红），
# 排除法会静默放行（正是本轮要修的假绿）。终态命名见
# app/manual_web_demo/search_session_archive.py 的 TERMINAL_STATUSES。
# STOPPING 不在内：它正在收尾，不算"跑着的实例"。
LIVE_STATUSES=" STARTING RUNNING PAUSED "

if [ "${1:-}" = "--history" ]; then
  EV=$(ls -1dt outputs/live_runs/*/events.jsonl 2>/dev/null | sed -n "${2:-1}p")
  [ -n "$EV" ] || { echo "没有历史会话的事件流可读。" >&2; exit 1; }
  SESSION=${EV%/events.jsonl}
  echo "注意：--history 是明确要求浏览历史，读到的不代表正在跑的实例。"
else
  STATE=$(curl -fsS http://127.0.0.1:8765/api/search/state 2>/dev/null)
  if [ -z "$STATE" ]; then
    echo "尚无法验证：取不到 /api/search/state（WebUI 没起？先跑 start_autonomous_search_web.sh）。" >&2
    exit 1
  fi
  SESSION_ID=$(printf '%s' "$STATE" | jq -r '.session_id // empty' 2>/dev/null)
  STATUS=$(printf '%s' "$STATE" | jq -r '.status // empty' 2>/dev/null)
  if [ -z "$SESSION_ID" ]; then
    echo "尚无法验证：WebUI 没在跑搜索会话（session_id 为空，status=$STATUS）。" >&2
    echo "要浏览历史会话请显式加参数：bash $0 --history" >&2
    exit 1
  fi
  case "$LIVE_STATUSES" in
  *" $STATUS "*)
    echo "会话 ID: $SESSION_ID（status=$STATUS）"
    ;;
  *)
    echo "尚无法验证：会话 $SESSION_ID 的 status=$STATUS，不是正在跑的这一轮。" >&2
    echo "（搜索结束后 WebUI 仍留着 session_id 与日志 —— 终端态会话只是历史证据，" >&2
    echo "  拿它当'跑着的实例'正是第三次复审复现出的问题。）" >&2
    exit 1
    ;;
  esac
  SESSION="outputs/live_runs/$SESSION_ID"
  if [ ! -f "$SESSION/events.jsonl" ]; then
    echo "尚无法验证：当前会话 $SESSION_ID 的事件流还没落盘。" >&2
    echo "（缺 $SESSION/events.jsonl）等几秒重跑；**不会**退回历史会话。" >&2
    exit 1
  fi
fi

echo "事件流 : $SESSION/events.jsonl"

# -R + fromjson? ：事件流是边跑边追加的，尾巴可能只有半行；按整行解析会在这里炸。
JQ_FILTER='fromjson?
| select(.event=="detector_diagnostics")
| {frame_id, available: .diagnostics_available,
   attempts: [.attempts[]
     | {frame: .frame_id, request_id,
        cache: .diagnostics.frame_feature_cache,
        rle: .diagnostics.mask_rle_encoder,
        requested: .diagnostics.mask_rle_encoder_requested,
        backbone: .diagnostics.backbone_count,
        terms: .diagnostics.term_count}]}'

OUT=$(jq -c -R "$JQ_FILTER" "$SESSION/events.jsonl" 2>/dev/null)
N=$(printf '%s' "$OUT" | grep -c .)

if [ "$N" -eq 0 ]; then
  echo "尚无法验证：该会话还没有任何 detector_diagnostics 事件（检测未发生或日志未写完）。" >&2
  exit 1
fi

echo "detector_diagnostics 条数: $N"
printf '%s\n' "$OUT"
exit 0
