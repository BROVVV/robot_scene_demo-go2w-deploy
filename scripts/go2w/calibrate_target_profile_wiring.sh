#!/usr/bin/env bash
# 注入校准：证明 tests/test_web_entry_target_profile_wiring.py 抓得住
# 「画像没接线」这个缺陷，而不只是"碰巧是绿的"。
#
# 三刀，每刀都把代码改回**缺陷版**，跑判据，断言**指定的那几条**变红，
# 然后立即还原并用 LC_ALL=C sha256sum -c 复核（中文 locale 下 sha256sum 打印
# "成功"不是 "OK"，比对会漏判）。
#
#
# **脚本放版本控制、日志放 artifacts**：`artifacts/` 被 .gitignore 忽略，
# 报告让读者跑的脚本跟着它一起丢，就是阶段一第四次复审点出的那个交付缺口。
# 证据（日志、还原副本）留 artifacts，脚本留在 scripts/go2w/。
# 用法：bash calibrate_target_profile_wiring.sh
# 退出码：0 = 三刀都按预期变红且已还原；非 0 = 校准本身不成立。

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
cd "$ROOT" || exit 2

PY=$ROOT/.venv/bin/python
ENTRY=scripts/go2w/run_semantic_exploration.py
SUBP=app/detectors/grounded_sam_subprocess.py
JUDGE=tests/test_web_entry_target_profile_wiring.py

ART=${GO2W_PHASE2_ARTIFACTS:-$ROOT/artifacts/perception_optimization/20261013_phase2}
LOGDIR=$ART/injection_calibration
mkdir -p "$LOGDIR"
CAL=$LOGDIR/restore_p2t1
mkdir -p "$CAL"
cp -f "$ENTRY" "$CAL/entry.orig"
cp -f "$SUBP" "$CAL/subp.orig"
( cd "$ROOT" && sha256sum "$ENTRY" "$SUBP" ) >"$CAL/SHA256SUMS"

restore() {
  cp -f "$CAL/entry.orig" "$ENTRY"
  cp -f "$CAL/subp.orig" "$SUBP"
}

# SIGKILL 会绕过 trap，所以每刀**跑完立刻**还原，不靠退出时兜底。
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

# run_case <刀名> <期望变红的测试名…>
run_case() {
  local name="$1"
  shift
  local log="$LOGDIR/injection_${name}.log"
  local cache
  cache=$(mktemp -d) # 陈旧 .pyc 会让注入校准说谎：每次换 PYTHONPYCACHEPREFIX
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
echo "=== 刀 A：删掉接线块里的赋值行（回到"画像从不落到 node 上"） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        node._target_profile = profile\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, ""), encoding="utf-8")
PY
run_case A_wiring_removed \
  test_the_wiring_block_hands_the_profile_to_the_node
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 B：把 use_llm 改回硬编码 False（回到"开关形同虚设"） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "use_llm=bool(settings.go2w_web_target_profile_use_llm)"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "use_llm=False"), encoding="utf-8")
PY
run_case B_use_llm_hardcoded \
  test_the_profile_is_resolved_with_the_switch_not_a_hardcoded_value
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 C：给"画像存在"再加一个发射理由（架空回退核验的零条诊断门） ==="
"$PY" - "$SUBP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "    if not attempts and not (available and diagnostics):\n        return None\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
patched = (
    "    if (\n"
    "        not attempts\n"
    "        and not (available and diagnostics)\n"
    "        and not getattr(getattr(detector, \"target_profile\", None),"
    " \"raw_query\", None)\n"
    "    ):\n        return None\n"
)
p.write_text(s.replace(needle, patched), encoding="utf-8")
PY
run_case C_profile_emits_event \
  test_a_profile_alone_does_not_make_the_event_get_written
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
if [ "$FAILED_CASES" -eq 0 ]; then
  echo "三刀都按预期变红，且每次都逐字节还原。判据有分辨力。"
else
  echo "有刀没被抓住 —— 判据对那条缺陷没有分辨力。" >&2
fi
exit "$FAILED_CASES"
