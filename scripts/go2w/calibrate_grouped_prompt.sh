#!/usr/bin/env bash
# 注入校准：证明 tests/test_grouped_prompt_quality.py 抓得住它声称要抓的缺陷。
#
# 六刀，每刀把代码改成**缺陷版**，跑判据，断言**指定的那几条**变红，然后立即
# 还原并用 LC_ALL=C sha256sum -c 复核。
#
# 为什么用 LC_ALL=C：中文 locale 下 `sha256sum -c` 打的是「成功」而不是 OK，
# 用 `grep ': OK$'` 判会整份清单漏判 —— 那是"还原校验"自己变成假绿。
#
# 为什么每刀先落盘还原副本（restore_p2t6/）：`SIGKILL` 绕过 `trap ... EXIT`，
# 注入会留在文件里；只有落过盘的副本能把它救回来。
#
# 用法：bash scripts/go2w/calibrate_grouped_prompt.sh
# 退出码：0 = 六刀都按预期变红且已还原；非 0 = 校准本身不成立。
#
# **脚本放版本控制、日志放 artifacts**：`artifacts/` 被 .gitignore 忽略，
# 报告让读者跑的脚本跟着它一起丢，就是阶段一第四次复审点出的那个交付缺口。
# 证据（日志、还原副本）留 artifacts，脚本留在 scripts/go2w/。

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
cd "$ROOT" || exit 2

PY=$ROOT/.venv/bin/python
WORKER=app/detectors/grounded_sam_worker.py
BENCH=scripts/go2w/perception_replay_benchmark.py
JUDGE=tests/test_grouped_prompt_quality.py

ART=${GO2W_PHASE2_ARTIFACTS:-$ROOT/artifacts/perception_optimization/20261013_phase2}
LOGDIR=$ART/injection_calibration
mkdir -p "$LOGDIR"
CAL=$LOGDIR/restore_p2t6
mkdir -p "$CAL"
for f in "$WORKER" "$BENCH"; do
  cp -f "$f" "$CAL/$(basename "$f").orig"
done
( cd "$ROOT" && sha256sum "$WORKER" "$BENCH" ) >"$CAL/SHA256SUMS"

restore() {
  cp -f "$CAL/grounded_sam_worker.py.orig" "$WORKER"
  cp -f "$CAL/perception_replay_benchmark.py.orig" "$BENCH"
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
  local log="$LOGDIR/injection_p2t6_${name}.log"
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

inject() {  # inject <file> <python-body>  —— body 里用 s/needle 两个变量
  local target="$1"
  shift
  "$PY" - "$target" "$@" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1])
s = p.read_text(encoding="utf-8")
needle, replacement = sys.argv[2], sys.argv[3]
count = s.count(needle)
assert count == 1, f"锚点命中 {count} 次（应为 1）：{needle!r}"
p.write_text(s.replace(needle, replacement), encoding="utf-8")
PY
}

# --------------------------------------------------------------------------- #
echo "=== 刀 A（计划书点名的那一刀）：串词检测恒返 0 —— 人造串词输出不再变红 ==="
inject "$BENCH" \
  '        "label_flips": sum(1 for item in matches if item["label_flip"]),' \
  '        "label_flips": 0,' \
  || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case A_crosstalk_always_zero \
  test_a_box_that_landed_on_the_right_object_with_the_wrong_word_is_cross_talk \
  test_cross_talk_rejects_adoption_even_when_nothing_is_lost
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 B：关联不唯一也猜一个词（把串词重新引进来） ==="
inject "$WORKER" \
  '    if len(contained) == 1:
        return contained[0]
    return None' \
  '    if len(contained) == 1:
        return contained[0]
    return terms[0] if terms else None' \
  || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case B_guess_instead_of_refusing \
  test_an_ambiguous_phrase_is_refused_rather_than_guessed \
  test_a_box_whose_phrase_matches_no_term_is_dropped_and_counted \
  test_the_switch_reaches_the_dino_call_through_the_real_detect
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 C：配对改成多对一（重复框把召回刷成 1.0，漏检被藏起来） ==="
inject "$BENCH" \
  '        if best_index in used:' \
  '        if False:' \
  || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case C_many_to_one_matching \
  test_matching_is_one_to_one_so_a_double_detection_cannot_fake_recall
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 D：分组不再要求 token 跨度（退回串词那条老路） ==="
inject "$WORKER" \
  '            remove_combined=True,' \
  '            remove_combined=False,' \
  || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case D_no_token_span \
  test_grouped_caption_uses_the_period_separator_and_asks_for_token_spans \
  test_the_switch_reaches_the_dino_call_through_the_real_detect
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 E：词表同一性断言不按 prompt 模式分支（分组那侧被无条件判死） ==="
inject "$BENCH" \
  '        prompts[(item["mode"], item["prompt_mode"], item["frame_id"])].add(' \
  '        prompts[(item["mode"], item["frame_id"])].add(' \
  || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
inject "$BENCH" \
  '        prompts[("ALL", item["prompt_mode"], item["frame_id"])].add(' \
  '        prompts[("ALL", item["frame_id"])].add(' \
  || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case E_identity_not_scoped_per_prompt_mode \
  test_the_prompt_identity_assertion_is_scoped_to_one_prompt_mode
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 F：单一轴的轮次顺序把元组写反（与阶段一的顺序不再可比） ==="
inject "$BENCH" \
  '            [(name, prompt_modes[0]) for name in names]' \
  '            [(prompt_modes[0], name) for name in names]' \
  || { echo "    **注入没打上** —— 锚点失效，校准不成立" >&2; exit 3; }
run_case F_single_axis_tuple_swapped \
  test_one_prompt_mode_reproduces_the_phase_one_round_order_exactly
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
if [ "$FAILED_CASES" -eq 0 ]; then
  echo "六刀都按预期变红，且每次都逐字节还原。判据有分辨力。"
else
  echo "有刀没被抓住 —— 判据对那条缺陷没有分辨力。" >&2
fi
exit "$FAILED_CASES"
