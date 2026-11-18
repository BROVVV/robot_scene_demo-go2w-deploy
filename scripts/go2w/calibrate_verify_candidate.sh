#!/usr/bin/env bash
# 注入校准：证明 tests/test_verify_candidate_selection.py 抓得住它声称要抓的缺陷。
#
# 三刀，每刀把代码改回**缺陷版**，跑判据，断言**指定的那几条**变红，
# 然后立即还原并用 LC_ALL=C sha256sum -c 复核。
#
#
# **脚本放版本控制、日志放 artifacts**：`artifacts/` 被 .gitignore 忽略，
# 报告让读者跑的脚本跟着它一起丢，就是阶段一第四次复审点出的那个交付缺口。
# 证据（日志、还原副本）留 artifacts，脚本留在 scripts/go2w/。
# 用法：bash calibrate_verify_candidate.sh
# 退出码：0 = 三刀都按预期变红且已还原；非 0 = 校准本身不成立。

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
cd "$ROOT" || exit 2

PY=$ROOT/.venv/bin/python
ROLES=app/perception/entity_roles.py
ENTRY=scripts/go2w/run_semantic_exploration.py
JUDGE=tests/test_verify_candidate_selection.py

ART=${GO2W_PHASE2_ARTIFACTS:-$ROOT/artifacts/perception_optimization/20261013_phase2}
LOGDIR=$ART/injection_calibration
mkdir -p "$LOGDIR"
CAL=$LOGDIR/restore_p2t3
mkdir -p "$CAL"
cp -f "$ROLES" "$CAL/roles.orig"
cp -f "$ENTRY" "$CAL/entry.orig"
( cd "$ROOT" && sha256sum "$ROLES" "$ENTRY" ) >"$CAL/SHA256SUMS"

restore() {
  cp -f "$CAL/roles.orig" "$ROLES"
  cp -f "$CAL/entry.orig" "$ENTRY"
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
  local log="$LOGDIR/injection_p2t3_${name}.log"
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
echo "=== 刀 A：verifier 退回在**全量** detections 上取最高分 ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "        best, authoritative = select_candidate(\n"
    "            observation.target_candidates,\n"
    "            observation.detections,\n"
    "            scope=settings.go2w_verify_candidate_scope,\n"
    "            candidates_authoritative=settings.go2w_result_role_separation,\n"
    "        )\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
patched = (
    "        best = max(\n"
    "            observation.detections,\n"
    '            key=lambda item: item.get("score", 0.0),\n'
    "            default=None,\n"
    "        )\n"
    "        authoritative = False\n"
)
p.write_text(s.replace(needle, patched), encoding="utf-8")
PY
run_case A_verifier_argmax_over_detections \
  test_the_real_verifier_crops_the_target_candidate_not_the_chair \
  test_scene_objects_alone_are_refused_and_the_reason_says_why \
  test_no_bare_argmax_over_detections_remains_in_the_entry
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 B：权威候选为空时回退到 detections（把"确实没有"变成"去复核椅子"） ==="
"$PY" - "$ROLES" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        pool = [item for item in (candidates or ()) if isinstance(item, dict)]\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
patched = (
    "        pool = [item for item in (candidates or detections or ())"
    " if isinstance(item, dict)]\n"
)
p.write_text(s.replace(needle, patched), encoding="utf-8")
PY
run_case B_empty_view_falls_back \
  test_an_empty_authoritative_view_is_not_a_fallback_signal \
  test_scene_objects_alone_are_refused_and_the_reason_says_why
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 C：observe 的锁框退回全量 argmax（第二处调用点没跟上） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "        best_feed, _ = select_candidate(\n"
    "            target_candidates,\n"
    "            detections,\n"
    "            scope=settings.go2w_verify_candidate_scope,\n"
    "            candidates_authoritative=settings.go2w_result_role_separation,\n"
    "        )\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
patched = (
    "        best_feed = (\n"
    '            max(detections, key=lambda item: item["score"]) if detections'
    " else None\n"
    "        )\n"
)
p.write_text(s.replace(needle, patched), encoding="utf-8")
PY
run_case C_second_site_regressed \
  test_no_bare_argmax_over_detections_remains_in_the_entry \
  test_both_call_sites_go_through_select_candidate
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
if [ "$FAILED_CASES" -eq 0 ]; then
  echo "三刀都按预期变红，且每次都逐字节还原。判据有分辨力。"
else
  echo "有刀没被抓住 —— 判据对那条缺陷没有分辨力。" >&2
fi
exit "$FAILED_CASES"
