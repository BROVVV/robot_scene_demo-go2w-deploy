#!/usr/bin/env bash
# 注入校准：证明 tests/test_entity_roles.py 抓得住它声称要抓的缺陷。
#
# 七刀，每刀把代码改回**缺陷版**，跑判据，断言**指定的那几条**变红，
# 然后立即还原并用 LC_ALL=C sha256sum -c 复核（中文 locale 下 sha256sum 打印
# "成功"不是 "OK"，比对会漏判）。
#
#
# **脚本放版本控制、日志放 artifacts**：`artifacts/` 被 .gitignore 忽略，
# 报告让读者跑的脚本跟着它一起丢，就是阶段一第四次复审点出的那个交付缺口。
# 证据（日志、还原副本）留 artifacts，脚本留在 scripts/go2w/。
# 用法：bash calibrate_entity_roles.sh
# 退出码：0 = 七刀都按预期变红且已还原；非 0 = 校准本身不成立。

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
cd "$ROOT" || exit 2

PY=$ROOT/.venv/bin/python
ROLES=app/perception/entity_roles.py
ENTRY=scripts/go2w/run_semantic_exploration.py
JUDGE=tests/test_entity_roles.py

ART=${GO2W_PHASE2_ARTIFACTS:-$ROOT/artifacts/perception_optimization/20261013_phase2}
LOGDIR=$ART/injection_calibration
mkdir -p "$LOGDIR"
CAL=$LOGDIR/restore_p2t2
mkdir -p "$CAL"
cp -f "$ROLES" "$CAL/roles.orig"
cp -f "$ENTRY" "$CAL/entry.orig"
( cd "$ROOT" && sha256sum "$ROLES" "$ENTRY" ) >"$CAL/SHA256SUMS"

restore() {
  cp -f "$CAL/roles.orig" "$ROLES"
  cp -f "$CAL/entry.orig" "$ENTRY"
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
  local log="$LOGDIR/injection_p2t2_${name}.log"
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
echo "=== 刀 A：实体 ID 从哈希换成序号（回到"返回顺序一变就换号"） ==="
"$PY" - "$ROLES" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = '            entry["entity_id"] = entity_id_for(entry)\n'
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(
    s.replace(needle, '            entry["entity_id"] = f"det_{len(out):03d}"\n'),
    encoding="utf-8",
)
PY
run_case A_index_id \
  test_ids_are_stable_when_the_detector_returns_objects_in_another_order
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 B：删掉 POSSIBLE 分支（回到二值判定） ==="
"$PY" - "$ROLES" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "    return TARGET_PRESENT if best >= float(target_score_min) "
    "else TARGET_POSSIBLE\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "    return TARGET_PRESENT\n"), encoding="utf-8")
PY
run_case B_no_possible_branch \
  test_a_target_candidate_below_the_threshold_is_possible_not_present \
  test_possible_is_not_positive_evidence \
  test_a_nan_score_does_not_sneak_past_the_threshold
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 C：入口不再把两份视图交给适配层（回到字段恒为空表） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "            target_candidates=target_candidates,\n"
    "            relation_anchors=relation_anchors,\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, ""), encoding="utf-8")
PY
# 只有守卫那条会红：适配层本身的回退行为由 `_to_live` 直接喂参另测，
# 入口不传时适配层照旧工作（这正是"字段没接线"最难自己暴露的地方）。
run_case C_views_not_passed \
  test_the_entry_feeds_the_two_views_into_the_adapter
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 D：入口又把 detections 从三件套重建（丢掉 entity_id / term） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "                detections = partition.detections\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
rebuild = (
    "                detections = [\n"
    "                    {\n"
    "                        \"label\": str(item.get(\"label\", \"object\")),\n"
    "                        \"score\": float(item.get(\"score\", 0.0)),\n"
    "                        \"bbox_2d\": [float(v) for v in item.get(\"bbox_2d\", [0.0, 0.0, 1.0, 1.0])],\n"
    "                    }\n"
    "                    for item in objects\n"
    "                ]\n"
)
p.write_text(s.replace(needle, rebuild), encoding="utf-8")
PY
run_case D_entry_rebuilds_detections \
  test_the_entrys_three_views_share_one_entity_id_per_box
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 E：分类时复制一份（三份视图不再是同一批 dict） ==="
"$PY" - "$ROLES" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "        entry[\"roles\"] = roles\n"
    "        partition.detections.append(entry)\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 复现旧写法：detections 收原件（不带 roles），三份视图各收一份拷贝。
bug = (
    "        partition.detections.append(entry)\n"
    "        entry = dict(entry, roles=roles)\n"
)
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case E_views_are_copies \
  test_the_entrys_three_views_share_one_entity_id_per_box
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 F：认同一实体时不比源帧（回到“bbox 哈希相同就是同一件”） ==="
"$PY" - "$ROLES" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "    frame_a = str(a.get(\"entity_source_frame_id\") or \"\")\n"
    "    frame_b = str(b.get(\"entity_source_frame_id\") or \"\")\n"
    "    return bool(frame_a) and frame_a == frame_b\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "    return True\n"), encoding="utf-8")
PY
run_case F_identity_ignores_frame \
  test_the_same_id_in_two_different_frames_is_not_the_same_entity
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 G：入口不把来源帧交给适配层（下游只剩 bbox 哈希可猜） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "            views_source_frame_id=views_source_frame_id,\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, ""), encoding="utf-8")
PY
run_case G_frame_not_passed \
  test_the_entry_tells_the_adapter_which_frame_the_views_came_from
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
if [ "$FAILED_CASES" -eq 0 ]; then
  echo "七刀都按预期变红，且每次都逐字节还原。判据有分辨力。"
else
  echo "有刀没被抓住 —— 判据对那条缺陷没有分辨力。" >&2
fi
exit "$FAILED_CASES"
