#!/usr/bin/env bash
# 注入校准：证明 tests/test_async_verifier.py 抓得住它声称要抓的缺陷。
#
# 八刀，每刀把代码改回**缺陷版**，跑判据，断言**指定的那几条**变红，
# 然后立即还原并用 LC_ALL=C sha256sum -c 复核。
#
# 为什么用 LC_ALL=C：中文 locale 下 `sha256sum -c` 打的是「成功」而不是 OK，
# 用 `grep ': OK$'` 判会整份清单漏判 —— 那是"还原校验"自己变成假绿。
#
#
# **脚本放版本控制、日志放 artifacts**：`artifacts/` 被 .gitignore 忽略，
# 报告让读者跑的脚本跟着它一起丢，就是阶段一第四次复审点出的那个交付缺口。
# 证据（日志、还原副本）留 artifacts，脚本留在 scripts/go2w/。
# 用法：bash calibrate_async_verify.sh
# 退出码：0 = 八刀都按预期变红且已还原；非 0 = 校准本身不成立。

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
cd "$ROOT" || exit 2

PY=$ROOT/.venv/bin/python
ASYNC=app/live_robot/async_verifier.py
EXPL=app/live_robot/autonomous_explorer.py
ENTRY=scripts/go2w/run_semantic_exploration.py
JUDGE=tests/test_async_verifier.py

ART=${GO2W_PHASE2_ARTIFACTS:-$ROOT/artifacts/perception_optimization/20261013_phase2}
LOGDIR=$ART/injection_calibration
mkdir -p "$LOGDIR"
CAL=$LOGDIR/restore_p2t5
mkdir -p "$CAL"
for f in "$ASYNC" "$EXPL" "$ENTRY"; do
  cp -f "$f" "$CAL/$(basename "$f").orig"
done
( cd "$ROOT" && sha256sum "$ASYNC" "$EXPL" "$ENTRY" ) >"$CAL/SHA256SUMS"

restore() {
  cp -f "$CAL/async_verifier.py.orig" "$ASYNC"
  cp -f "$CAL/autonomous_explorer.py.orig" "$EXPL"
  cp -f "$CAL/run_semantic_exploration.py.orig" "$ENTRY"
}

trap 'restore' EXIT

FAILED_CASES=0

verify_restored() {
  if ( cd "$ROOT" && LC_ALL=C sha256sum -c "$CAL/SHA256SUMS" >/dev/null 2>&1 ); then
    echo "    [还原校验] 与校准前逐字节相同"
  else
    echo "    [还原校验] **失败** —— 注入留在文件里了" >&2
    exit 9
  fi
}

run_case() {
  local name="$1"
  shift
  local log="$LOGDIR/injection_p2t5_${name}.log"
  local cache
  cache=$(mktemp -d)
  PYTHONPYCACHEPREFIX="$cache" "$PY" -m pytest "$JUDGE" -q >"$log" 2>&1
  local rc=$?
  rm -rf "$cache"

  echo "  退出码 rc=$rc（非 0 才说明判据抓到了）"
  if [ "$rc" -eq 0 ]; then
    echo "    **判据没变红** —— 这一刀没被抓住，校准失败" >&2
    FAILED_CASES=1
  fi
  local missing=0
  for t in "$@"; do
    if grep -q "FAILED.*${t}" "$log"; then
      echo "    抓到：$t"
    else
      echo "    **没抓到**：$t（期望它红）" >&2
      missing=1
    fi
  done
  [ "$missing" -eq 0 ] || FAILED_CASES=1
  echo "    日志：${log#"$ROOT"/}"
}

# --------------------------------------------------------------------------- #
echo "=== 刀 A：删掉序号丢弃（乱序的旧结论会覆盖新结论） ==="
"$PY" - "$ASYNC" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "            elif result.sequence <= self._applied_sequence.get(\n"
    "                _candidate_key(request.task_generation, request.candidate_id), 0\n"
    "            ):\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "            elif False:\n"), encoding="utf-8")
PY
[ $? -eq 0 ] || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case A_no_sequence_discard \
  test_an_out_of_order_result_cannot_confirm_an_already_rejected_candidate
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 B：没有判定也当否定（UNKNOWN 落 ABSENT） ==="
"$PY" - "$EXPL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = '    return verification.status in ("", VERIFY_STATUS_REJECTED)\n'
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "    return True\n"), encoding="utf-8")
PY
[ $? -eq 0 ] || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case B_unknown_is_negative \
  test_exhausted_possible_with_an_unknown_verdict_keeps_possible \
  test_a_verifier_that_raises_is_not_a_negative
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 C：客户端不再判超时（挂住的 worker 永远没有结论） ==="
"$PY" - "$ASYNC" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "                if now - request.submitted_at < self.client_timeout_s:\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "                if True:\n"), encoding="utf-8")
PY
[ $? -eq 0 ] || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case C_no_client_timeout \
  test_a_hung_worker_becomes_unknown_at_the_client_deadline \
  test_await_submission_gives_up_at_the_deadline_with_an_unknown
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 D：超时之后晚到的结果照样落地（假『找到了』） ==="
"$PY" - "$ASYNC" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
# 前导换行不能省：12 空格那行是 16 空格那行（poll_completed 里同名判断）的
# 子串，用纯 substring 计数会命中 2 次。替换时换行也要带上，否则两行会被接成一行。
needle = "\n            if request.resolved:\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "\n            if False:\n"), encoding="utf-8")
PY
[ $? -eq 0 ] || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case D_late_result_applied \
  test_the_late_result_after_the_client_timeout_is_discarded
restore; verify_restored


# --------------------------------------------------------------------------- #
echo
echo "=== 刀 E：取值不再绑序号（按候选取最近一条 ⇒ 新帧消费旧结论） ==="
"$PY" - "$ASYNC" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "    def get_result_for(self, sequence: int) -> VerificationResult | None:\n"
    "        \"\"\"按序号取结论 —— 身份绑定的取值口。\"\"\"\n"
    "        with self._lock:\n"
    "            return self._by_sequence.get(int(sequence))\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
buggy = (
    "    def get_result_for(self, sequence: int) -> VerificationResult | None:\n"
    "        with self._lock:\n"
    "            request = self._inflight.get(int(sequence))\n"
    "            if request is not None:\n"
    "                return self._latest.get(\n"
    "                    _candidate_key(request.task_generation, request.candidate_id)\n"
    "                )\n"
    "            return self._by_sequence.get(int(sequence))\n"
)
p.write_text(s.replace(needle, buggy), encoding="utf-8")
PY
[ $? -eq 0 ] || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case E_identity_not_bound \
  test_a_new_frame_cannot_consume_the_previous_frames_verdict
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 F：客户端超时顺手把物理在途名额也放掉（max_inflight 成摆设） ==="
"$PY" - "$ASYNC" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "                request.resolved = True\n"
    "                self._client_timed_out.add(request.sequence)\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
buggy = needle + "                del self._inflight[request.sequence]\n"
p.write_text(s.replace(needle, buggy), encoding="utf-8")
PY
[ $? -eq 0 ] || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case F_timeout_frees_the_slot \
  test_a_client_timeout_does_not_release_the_physical_slot
restore; verify_restored


# --------------------------------------------------------------------------- #
echo
echo "=== 刀 G：入口退回按候选取结论（模块改对了、入口没跟着改） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        outcome = async_verifier.await_submission(submission)\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(
    s.replace(needle, "        outcome = async_verifier.await_result(candidate_id)\n"),
    encoding="utf-8",
)
PY
[ $? -eq 0 ] || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case G_entry_awaits_by_candidate \
  test_the_web_entry_awaits_by_submission_and_verifies_identity
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 H：当面核验恒真（谁来都放行） ==="
"$PY" - "$ASYNC" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "    if result is None:\n        return False\n    return (\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(
    s.replace(needle, "    if result is None:\n        return True\n    return (\n"),
    encoding="utf-8",
)
PY
[ $? -eq 0 ] || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case H_identity_check_always_true \
  test_the_entry_identity_check_refuses_any_other_submission
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
if [ "$FAILED_CASES" -eq 0 ]; then
  echo "八刀都按预期变红，且每次都逐字节还原。判据有分辨力。"
else
  echo "有刀没被抓住 —— 判据对那条缺陷没有分辨力。" >&2
fi
exit "$FAILED_CASES"
