#!/usr/bin/env bash
# Fix #3（原地打转闸门）这一轮的 A/B/C 三连跑批，一次跑完，产出 §2.9 那张台账。
#
#   A. 注错校准（18 条注入，每条都要让指定的断言变红）—— 见 calibrate_explorer_injections.py
#   B. 定向回归（6 个文件：闸门、穷尽 vs 卡死、前沿记忆、卡死分类器、预算、清障 E2E）
#   C. 全量（--continue-on-collection-errors）
#
# 刻意**不并发**：本仓库有一批时延敏感的用例（LLM/视频/AppTest），并发负载会把它们带偏。
#
# 两条硬要求（仓库的坑，见 memory: stale-pyc-invalidates-injection-calibration /
# host-test-env-and-llm-latency）：
#   - `env -u PYTHONNOUSERSITE`：.bashrc 里的这个变量会挡住 /usr/bin/python3 的
#     ~/.local 依赖（历史上让 WebUI 起搜索 0.5 秒就挂）。
#   - 每次运行换 PYTHONPYCACHEPREFIX：陈旧 .pyc 会让"已还原"的副本继续跑注入版。
#
# 收尾的**纯追加才还原**：全量套件会往 data/memory/observational_memory.jsonl 追加
# mock 行（test_streamlit_video_mode 干的）。还原它**不能**用 git checkout --（那会连
# 索引一起动，仓库带着不许动的未提交改动）；先 `cmp` 前 N 字节，确认只是追加，再整份
# 写回。若不是纯追加就原样保留，交给人判断。
#
# 用法：bash scripts/go2w/run_exploration_gate_checks.sh [产物目录]
# 退出码：0 = 三段全过；任一阶段失败则返回非零，完整输出保存在产物目录。
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
cd "$ROOT" || exit 2

PY=${GO2W_TEST_PYTHON:-$ROOT/.venv/bin/python3}
OBS=data/memory/observational_memory.jsonl
TMP=${TMPDIR:-/tmp}
ART=${1:-$(mktemp -d "$ROOT/artifacts/exploration_checks_XXXXXXXX")}
mkdir -p "$ART" || exit 2
WORK=$(mktemp -d "$TMP/exploration_checks_XXXXXXXX") || exit 2
echo "ARTIFACTS=$ART"

TARGETED="tests/test_in_place_turn_streak.py tests/test_search_exhaustion_vs_stuck.py
tests/test_frontier_memory_lifecycle.py tests/test_stuck_detector.py
tests/test_exploration_budget.py tests/test_go2w_clearance_viewpoint_e2e.py"

echo "=== 工作树快照（跑之前）==="
git status --porcelain | md5sum
sha256sum "$OBS"
cp "$OBS" "$WORK/obs_before_run.jsonl" || exit 2
OBS_BYTES=$(stat -c%s "$WORK/obs_before_run.jsonl")
echo

cleanup() {
  local rc=$?
  trap - EXIT
  if head -c "$OBS_BYTES" "$OBS" | cmp -s - "$WORK/obs_before_run.jsonl"; then
    if ! cmp -s "$OBS" "$WORK/obs_before_run.jsonl"; then
      cp "$WORK/obs_before_run.jsonl" "$OBS" || rc=2
    fi
  else
    cp "$WORK/obs_before_run.jsonl" "$ART/observational_memory.before.jsonl"
    echo "!! 观测文件不是纯追加，保留现场和还原副本：$ART"
    rc=2
  fi
  rm -rf "$WORK"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

run_stage() {
  local name=$1 rc
  shift
  echo "=== $name ==="
  env -u PYTHONNOUSERSITE PYTHONPYCACHEPREFIX="$WORK/pyc_$name" \
    "$PY" "$@" >"$ART/$name.log" 2>&1
  rc=$?
  tail -12 "$ART/$name.log"
  echo "$name=$rc" | tee -a "$ART/status.txt"
  return "$rc"
}

# 注错校准失败后不继续：被测源码可能尚未还原，不能拿后续结果作证。
run_stage A_injections scripts/go2w/calibrate_explorer_injections.py || exit $?
result=0
run_stage B_targeted -m pytest -q -p no:cacheprovider $TARGETED || result=$?
run_stage C_full -m pytest -q -p no:cacheprovider \
  --continue-on-collection-errors || result=$?
git status --porcelain | md5sum
sha256sum "$OBS"
echo "ALL_CHECKS_DONE exit=$result"
exit "$result"
