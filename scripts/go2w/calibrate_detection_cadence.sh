#!/usr/bin/env bash
# 注入校准：证明 tests/test_detection_scheduler.py 抓得住它声称要抓的缺陷。
#
# 二十刀（本项改动面最大，刀数相应多），每刀把代码改回**缺陷版**，
# 跑判据，断言**指定的那几条**变红，然后立即还原并用 LC_ALL=C sha256sum -c 复核。
#
# 刀 I/J/K 是 2026-10-14 补充复审「必修 1」（物体层槽归属）的校准：I 拆掉
# 生产端的"这条对哪些槽还算最新"标记，K 拆掉消费端按标记过滤的那道门，
# J 把去重方向掰回"先到先得"。三者分属两处代码、抓的是不同的判据。
#
# 刀 L/M 是 2026-10-14 第三次补充复审「必修」的校准：L 把默认回退路径
# （职责分离**关着**、分频开着）上的新鲜度过滤整段拿掉 —— 这正是复审用生产
# 函数复现出来的那个缺陷；M 反向砍一刀，把"没有标记"也当成"过期"一律丢，
# 那是本仓真踩过的错误口径（墙钟夹具第一版就是这么把分频**关着**的臂整臂染红的），
# 抓的是"分频关着时一条都不许丢"那条 containment。
#
# 刀 N~S 是 2026-10-14 第四次补充复审「必修」的校准：N 拆掉消费端按**词槽来源**
# 过滤的那道门（旧帧的上下文词 paper tray 又赢回来）；O 反向砍一刀，把"来源未知"
# 也当成过期一律丢（那是过度过滤，会把 quick 路径与旧缓存的框一起清掉）；
# P 让词→槽归属表**恒空**（上游白记）；Q 让 `_to_detected_object` 不把归属写进
# `DetectedObject`；R 让检测主路径不再传归属；S 把 static base 词表**无条件**记成
# 场景槽的词（目标槽问出来的词借场景槽的新鲜度留下来）—— S 是本轮实现中真写出来
# 又被自查改掉的版本，留一刀防止它回来。
# 刀 T 把来源归属的完整短语相等改回子串匹配：RAM 场景词 `tray` 不许
# 冒领目标槽的 `paper tray` 框。判据必须从检测器一路抓到默认入口锁框。
#
#
# **脚本放版本控制、日志放 artifacts**：`artifacts/` 被 .gitignore 忽略，
# 报告让读者跑的脚本跟着它一起丢，就是阶段一第四次复审点出的那个交付缺口。
# 证据（日志、还原副本）留 artifacts，脚本留在 scripts/go2w/。
# 用法：bash calibrate_detection_cadence.sh
# 退出码：0 = 十九刀都按预期变红且已还原；非 0 = 校准本身不成立。

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
cd "$ROOT" || exit 2

PY=$ROOT/.venv/bin/python
SCHED=app/perception/detection_scheduler.py
ROLES=app/perception/entity_roles.py
SUBP=app/detectors/grounded_sam_subprocess.py
ENTRY=scripts/go2w/run_semantic_exploration.py
JUDGE=tests/test_detection_scheduler.py

ART=${GO2W_PHASE2_ARTIFACTS:-$ROOT/artifacts/perception_optimization/20261013_phase2}
LOGDIR=$ART/injection_calibration
mkdir -p "$LOGDIR"
CAL=$LOGDIR/restore_p2t4
mkdir -p "$CAL"
for f in "$SCHED" "$ROLES" "$SUBP" "$ENTRY"; do
  cp -f "$f" "$CAL/$(basename "$f").orig"
done
( cd "$ROOT" && sha256sum "$SCHED" "$ROLES" "$SUBP" "$ENTRY" ) >"$CAL/SHA256SUMS"

restore() {
  cp -f "$CAL/detection_scheduler.py.orig" "$SCHED"
  cp -f "$CAL/entity_roles.py.orig" "$ROLES"
  cp -f "$CAL/grounded_sam_subprocess.py.orig" "$SUBP"
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
  local log="$LOGDIR/injection_p2t4_${name}.log"
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
echo "=== 刀 A：删掉 starvation 硬顶（场景词可以被无限期不刷） ==="
"$PY" - "$SCHED" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "        if not refresh_scene and waited >= float(scene_max_stale_s):\n"
    "            refresh_scene = True\n"
    "            forced_scene = True\n"
    "            scene_wait_s = waited\n"
    '            reasons.append("scene_max_stale")\n'
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, ""), encoding="utf-8")
PY
run_case A_no_starvation \
  test_starvation_max_wait_forces_a_scene_refresh \
  test_the_max_wait_is_reported_with_its_actual_duration
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 B：去掉地点切换判据（换了地方还沿用旧场景词） ==="
"$PY" - "$SCHED" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "    place_changed = (\n"
    "        place_id is not None\n"
    "        and last_place_id is not None\n"
    '        and str(place_id) != str(last_place_id)\n'
    "    )\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "    place_changed = False\n"), encoding="utf-8")
PY
run_case B_no_place_gate \
  test_a_place_switch_refreshes_the_scene_even_when_nothing_else_is_due \
  test_a_changed_view_forces_every_slot
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 C：缓存键丢掉词槽（子集结果会喂给全量那一轮） ==="
"$PY" - "$SUBP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        None if slots is None else tuple(sorted({str(item) for item in slots})),\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "        None,\n"), encoding="utf-8")
PY
run_case C_cache_key_drops_slots \
  test_the_same_prompt_under_different_slots_gets_different_keys
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 D：入口前台调用点不再传槽集（开关变空转） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "                fresh = node._detect(stable_image, prompt, env,\n"
    "                                     frame_id=frame_id, slots=slots)\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
patched = (
    "                fresh = node._detect(stable_image, prompt, env,\n"
    "                                     frame_id=frame_id)\n"
)
p.write_text(s.replace(needle, patched), encoding="utf-8")
PY
run_case D_entry_drops_slots \
  test_the_entry_hands_the_slot_set_to_the_detector
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 E：退回整体覆盖（只刷一个槽就把别的槽的结果丢掉） ==="
"$PY" - "$SCHED" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "        if best is None:\n"
    "            continue\n"
    "        if not any(best is item for item in picked):\n"
    "            picked.append(best)\n"
    "    picked.sort(key=lambda item: float(item.at))\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 旧行为：只取最新那一条 —— 别的槽的物体随之消失。
bug = (
    "        if best is None:\n"
    "            continue\n"
    "    picked = [max(entries, key=lambda item: float(item.at))] if entries else []\n"
)
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case E_whole_cache_overwrite \
  test_refreshing_one_slot_does_not_drop_the_other_slots_objects
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 F：不给组进来的物体标来源帧（旧框冒充当前帧） ==="
"$PY" - "$SCHED" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "            if frame:\n"
    "                # 真实来源帧写在物体自己身上。跨帧的同一 bbox 哈希**不**构成\n"
    "                # 同一实体（见 `entity_roles.entities_are_the_same`）。\n"
    '                item["entity_source_frame_id"] = frame\n'
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, ""), encoding="utf-8")
PY
run_case F_no_frame_stamp \
  test_a_reused_object_carries_its_own_source_frame
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 G：混帧时按当前帧号上报（过期结果冒充当前帧事实） ==="
"$PY" - "$SCHED" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        frame_id=frames.pop() if len(frames) == 1 else \"\",\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(
    s.replace(needle, '        frame_id=frames.pop() if frames else "",\n'),
    encoding="utf-8",
)
PY
run_case G_mixed_frames_claim_one_frame \
  test_a_mixed_frame_assembly_reports_an_unknown_frame_not_the_current_one
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 H：入口不再按槽组装（回到整块覆盖） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "                    assembly = assemble_slot_objects(\n"
    "                        cadence_entries,\n"
    "                        now=time.monotonic(),\n"
    "                        ttl_by_slot=cadence_ttl_by_slot,\n"
    "                    )\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 只留本次刷新的那批：即"整块覆盖"的等价物（代码仍可跑，只是行为退回旧版）。
bug = (
    "                    assembly = SlotAssembly(\n"
    "                        objects=list(fresh),\n"
    "                        entries=(),\n"
    '                        frame_id=str(frame_id or ""),\n'
    "                    )\n"
)
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case H_entry_stops_assembling \
  test_a_target_only_round_still_sees_the_scene_objects \
  test_a_scene_only_round_still_sees_the_target_candidate
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 I：物体不再记「我属于哪些槽还算最新」（旧全量条目的目标框自称还是最新） ==="
"$PY" - "$SCHED" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "        fresh = [s for s in wanted\n"
    "                 if s in entry.slots and latest_by_slot.get(s) is entry]\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 缺陷版：条目覆盖哪些槽就算哪些槽最新 —— 旧全量条目的 target 框于是自称
# 对 target 槽也还作数，重新压过新帧的真框。
bug = "        fresh = [s for s in wanted if s in entry.slots]\n"
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case I_no_per_object_freshness \
  test_a_stale_full_entry_cannot_overwrite_the_fresher_target_verdict \
  test_a_disappeared_target_is_cleared_by_the_newest_target_slot \
  test_a_word_in_both_word_lists_does_not_come_back_as_a_scene_object \
  test_the_verification_lock_goes_to_the_new_box_not_the_stale_one \
  test_a_round_whose_target_vanished_does_not_lock_on_the_old_box
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 J：跨条目去重掰回「先到先得」（旧帧那个框先入 seen，新框被当重复丢掉） ==="
"$PY" - "$SCHED" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "            prev = best_by_key.get(key)\n"
    "            if prev is None or (float(entry.at), index) > prev[:2]:\n"
    "                best_by_key[key] = (float(entry.at), index, item)\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
bug = (
    "            if key not in best_by_key:\n"
    "                best_by_key[key] = (float(entry.at), index, item)\n"
)
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case J_dedup_keeps_the_older_copy \
  test_the_newest_box_wins_even_when_the_old_box_sits_at_the_same_coordinates
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 K：消费端不看槽归属标记（两半里少了后一半） ==="
"$PY" - "$ROLES" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        if fresh is not None:\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "        if False:\n"), encoding="utf-8")
PY
run_case K_consumer_ignores_freshness \
  test_a_stale_full_entry_cannot_overwrite_the_fresher_target_verdict \
  test_a_disappeared_target_is_cleared_by_the_newest_target_slot \
  test_a_word_in_both_word_lists_does_not_come_back_as_a_scene_object \
  test_the_verification_lock_goes_to_the_new_box_not_the_stale_one \
  test_a_round_whose_target_vanished_does_not_lock_on_the_old_box
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 L：默认回退路径整段不新鲜度过滤（职责分离关着时旧框回到选框池） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "                pool = objects\n"
    "                if settings.go2w_detection_cadence_enabled:\n"
    "                    pool = classify_roles(\n"
    "                        objects, profile_now, source_frame_id=views_source_frame_id\n"
    "                    ).detections\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 缺陷版：默认分支直接拿未过滤的 `objects` 削三件套。
p.write_text(s.replace(needle, "                pool = objects\n"), encoding="utf-8")
PY
run_case L_default_path_skips_freshness \
  test_default_switches_the_stale_box_never_reaches_the_rounds_detection_pool \
  test_default_switches_a_vanished_target_leaves_no_target_box_in_the_pool
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 M：把「没有标记」也当「过期」一律丢（分频关着时整张表被清空） ==="
"$PY" - "$ENTRY" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "                pool = objects\n"
    "                if settings.go2w_detection_cadence_enabled:\n"
    "                    pool = classify_roles(\n"
    "                        objects, profile_now, source_frame_id=views_source_frame_id\n"
    "                    ).detections\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 过度过滤：只留"被组装祝福过"的物体。分频关着时没人写那枚标记 ⇒ 全丢。
bug = "                pool = [o for o in objects if o.get('detection_fresh_slots')]\n"
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case M_overfilters_unmarked_objects \
  test_with_cadence_off_the_default_path_still_keeps_every_object
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 N：消费端不按词槽来源过滤（旧帧的上下文词又赢回来） ==="
"$PY" - "$ROLES" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "            own = {str(slot) for slot in (entry.get(\"detection_slots\") or ())}\n"
    "            if own and not (own & allowed):\n"
    "                continue\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, ""), encoding="utf-8")
PY
run_case N_ignores_slot_provenance \
  test_a_stale_context_word_never_beats_the_new_target_box \
  test_a_stale_context_word_never_survives_a_round_with_no_target \
  test_the_same_context_word_is_dropped_once_the_slot_record_is_present
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 O：把「来源未知」也当过期一律丢（过度过滤） ==="
"$PY" - "$ROLES" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "            if own and not (own & allowed):\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 缺陷版：没有来源记录的框也算过期 —— quick 路径与旧缓存的框会被整批清掉。
p.write_text(s.replace(needle, "            if not (own & allowed):\n"), encoding="utf-8")
PY
run_case O_overfilters_unknown_provenance \
  test_the_context_word_is_kept_when_the_box_carries_no_slot_record
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 P：词→槽归属表恒空（检测器白记） ==="
"$PY" - "$SUBP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "    return tuple(\n"
    "        slot\n"
    "        for slot, terms in (vocab_by_slot or {}).items()\n"
    "        if any(_normalize_prompt_term(candidate) == text for candidate in terms)\n"
    "    )\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "    return ()\n"), encoding="utf-8")
PY
run_case P_attribution_always_empty \
  test_a_box_is_attributed_to_the_slots_whose_word_lists_asked_for_it \
  test_the_boxes_come_back_carrying_the_slot_that_asked_for_them
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 Q：归属算出来了却不写进 DetectedObject ==="
"$PY" - "$SUBP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        detection_slots=tuple(str(slot) for slot in detection_slots or ()),\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, ""), encoding="utf-8")
PY
run_case Q_object_drops_the_slot_record \
  test_the_boxes_come_back_carrying_the_slot_that_asked_for_them
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 R：检测主路径不再把归属算给每个框 ==="
"$PY" - "$SUBP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "                detection_slots=_slots_for_term(\n"
    "                    item.get(\"source_prompt_term\") or item.get(\"label\"),\n"
    "                    vocab_by_slot,\n"
    "                ),\n"
)
# 锚点必须只命中检测主路径那一处：`_to_detected_object` 的**形参**里也有
# `detection_slots=`，但那是带缩进 4 格的签名行，与这里 16 格的调用实参不同。
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, ""), encoding="utf-8")
PY
run_case R_call_site_drops_the_attribution \
  test_the_boxes_come_back_carrying_the_slot_that_asked_for_them
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 S：static base 词表无条件记成场景槽（目标词借场景槽的新鲜度） ==="
"$PY" - "$SUBP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        if terms_to_prompt(base_terms) in prompts:\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "        if True:\n"), encoding="utf-8")
PY
run_case S_base_terms_noted_unconditionally \
  test_a_box_is_attributed_to_the_slots_whose_word_lists_asked_for_it
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 T：词槽来源改回子串匹配（tray 冒领 paper tray） ==="
"$PY" - "$SUBP" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = "        if any(_normalize_prompt_term(candidate) == text for candidate in terms)\n"
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(
    s.replace(needle, "        if terms and _matches_any_term(text, list(terms))\n"),
    encoding="utf-8",
)
PY
run_case T_substring_claims_the_wrong_slot \
  test_a_scene_short_word_does_not_claim_a_longer_target_query \
  test_overlapping_scene_word_cannot_keep_an_old_target_context_box
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
if [ "$FAILED_CASES" -eq 0 ]; then
  echo "二十刀都按预期变红，且每次都逐字节还原。判据有分辨力。"
else
  echo "有刀没被抓住 —— 判据对那条缺陷没有分辨力。" >&2
fi
exit "$FAILED_CASES"
