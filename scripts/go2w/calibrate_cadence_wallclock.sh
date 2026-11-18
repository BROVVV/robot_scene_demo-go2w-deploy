#!/usr/bin/env bash
# 注入校准：证明 tests/test_cadence_wallclock_tool.py 抓得住它声称要抓的**测量链**缺陷。
#
# 这个工具产出报告里每一个"实测"数字。它的错不会让测试变红 —— 它会让结论
# 反过来（"分频把场景物体丢了"、"分频省了 40% 前向"），所以九刀都往测量链上砍：
#
#   A 每轮不再换新帧路径      ⇒ 第一轮之后全是缓存命中，测的变成"缓存命中率"
#   B 入口那段 quick 语句的缓存表没还出来 ⇒ 回放得出"分频丢场景物体"的**假**结论
#   C 前向次数不再分前台/后台 ⇒ 审查 P2 问的那笔"后台加回来的一次"看不见
#   D 本地物体映射少一个字段  ⇒ 与生产 `_gsam_to_dict` 悄悄漂移
#   E 后台管理器的 now 换成 monotonic ⇒ 与 epoch 的 capture_timestamp 不同域，
#     TTL 闸门永久失效、后台前向数被**静默低估**（第一版真踩过）
#   F 混帧标记全局取最后一条 ⇒ 分频关着的臂读到上一个臂的事件
#   G 指纹判据「盖没盖别的帧」恒判 False ⇒ 那把尺子对旧框彻底失明
#   H 回放夹具退化成"目标框一次都没给过" ⇒ 阴性结论变**空**（第一版真踩过）
#   I 记账累加器塞回 `try` 里 ⇒ 90 秒的臂跑完才炸 `NameError`，单测一条不红
#     （第一版真踩过：白跑一整臂。只有真走 `main()` 才到那条路径）
#   J `locked_*` 又变回按候选视图取框（无视职责分离的生效值）⇒ 默认组合下的
#     实测读数变成"阶段二目标配置"的读数，"组装层干净了"被误读成"默认锁框干净了"
#     （第三次补充复审必修：夹具的 `locked_*` 必须来自**生效开关**下选中的那个框）
#
# G/H 是 2026-10-14 补充复审「必修 1」加的两刀。H 砍在**夹具**上
# （tests/test_cadence_wallclock_tool.py 因此在备份/校验范围内）：它要证明
# `first_round_target_candidates >= 1` 那条守卫真的抓得住"这一臂压根没见过目标"
# 这种空结论 —— 少了它，"旧框没留下"和"本来就没有旧框"在汇总表上长得一模一样。
# I 砍的是测量工具**自己**的记账路径，是"判据没覆盖到真跑路径"的实例。
#
# J 是 2026-10-14 第三次补充复审「必修」加的一刀，砍在**尺子取哪个池**上。
# 它需要夹具里两个池子给出**不同的赢家**才考得出来 —— 所以
# `tests/test_cadence_wallclock_tool.py::_cross_frame_assembly` 里的场景物体是
# 特意给了高分的（0.95 的椅子赢过新帧目标框的 0.20，而它根本不进候选池）。
# 两池同赢家的话这一刀砍下去什么都看不出来（第一版夹具就是这样，注入后判据照绿）。
#
# **脚本放版本控制、日志放 artifacts**：`artifacts/` 被 .gitignore 忽略。
# 用法：bash calibrate_cadence_wallclock.sh
# 退出码：0 = 十刀都按预期变红且已还原；非 0 = 校准本身不成立。

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(git -C "$HERE" rev-parse --show-toplevel)
cd "$ROOT" || exit 2

PY=$ROOT/.venv/bin/python
TOOL=scripts/go2w/perception_cadence_wallclock.py
JUDGE=tests/test_cadence_wallclock_tool.py

ART=${GO2W_PHASE2_ARTIFACTS:-$ROOT/artifacts/perception_optimization/20261013_phase2}
LOGDIR=$ART/injection_calibration
mkdir -p "$LOGDIR"
CAL=$LOGDIR/restore_p2wallclock
mkdir -p "$CAL"
cp -f "$TOOL" "$CAL/$(basename "$TOOL").orig"
cp -f "$JUDGE" "$CAL/$(basename "$JUDGE").orig"
( cd "$ROOT" && sha256sum "$TOOL" "$JUDGE" ) >"$CAL/SHA256SUMS"

restore() {
  cp -f "$CAL/perception_cadence_wallclock.py.orig" "$TOOL"
  cp -f "$CAL/test_cadence_wallclock_tool.py.orig" "$JUDGE"
}

trap 'restore' EXIT

FAILED_CASES=0

verify_restored() {
  # 中文 locale 下 `sha256sum -c` 打印的是"成功"而不是 OK —— 判据取 `LC_ALL=C`
  # 之外的**退出码**，不看那行字（踩过）。
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
  local log="$LOGDIR/injection_p2wallclock_${name}.log"
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
echo "=== 刀 A：每轮不再换新帧路径（去重键恒定，测的变成缓存命中率） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = '        target = self._cache_dir / f"bundle_{frame_id}.jpg"\n'
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, '        target = self._cache_dir / "bundle.jpg"\n'),
             encoding="utf-8")
PY
run_case A_fixed_cache_path \
  test_a_fresh_frame_path_per_round_keeps_the_cache_out_of_the_way
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 B：入口那段 quick 语句的缓存表没还出来（回放得出假结论） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = '        + "    return objects, assembly, cadence_entries\\n"\n'
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, '        + "    return objects, assembly, []\\n"\n'),
             encoding="utf-8")
PY
run_case B_entry_forgets_the_slot_cache \
  test_the_on_arm_refreshes_the_scene_less_often_but_keeps_it_in_the_observation
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 C：前向次数不再分前台/后台（审查 P2 那笔加回来的量看不见） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    '        self.counters["forwards_foreground" if lane == "foreground"\n'
    '                      else "forwards_background"] += forwards\n'
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(
    s.replace(needle, '        self.counters["forwards_foreground"] += forwards\n'),
    encoding="utf-8",
)
PY
run_case C_no_lane_split \
  test_the_background_lane_is_counted_apart_from_the_foreground
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 D：本地物体映射与生产漂移（少一个字段） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    '        "mask_size": list(obj.mask_size) if obj.mask_size else None,\n'
    '        "mask_area_ratio": obj.mask_area_ratio,\n'
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(
    s.replace(needle, '        "mask_size": list(obj.mask_size) if obj.mask_size else None,\n'),
    encoding="utf-8",
)
PY
run_case D_mapping_drifts \
  test_the_local_object_mapping_matches_the_production_one
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 E：管理器与 capture_timestamp 不同时钟域（后台前向被静默低估） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    '        # 见 docstring：这里**不能**用 time.monotonic。\n'
    '        now=time.time,\n'
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, '        now=time.monotonic,\n'), encoding="utf-8")
PY
run_case E_manager_clock_domain \
  test_the_background_manager_shares_the_clock_of_the_capture_timestamps
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 F：混帧标记跨臂串味（分频关着的臂读到上一个臂的事件） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    '            e for e in node.events[events_before:]\n'
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, '            e for e in node.events\n'), encoding="utf-8")
PY
run_case F_mixed_frames_cross_arm \
  test_the_off_arm_reports_no_mixed_frame_rounds
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 G：指纹判据「盖没盖别的帧」恒判 False（尺子对旧框失明） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    '    stamped = str(item.get("entity_source_frame_id") or "")\n'
    '    return bool(stamped) and stamped != str(frame_id)\n'
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
p.write_text(s.replace(needle, "    return False\n"), encoding="utf-8")
PY
run_case G_fingerprint_goes_blind \
  test_the_stale_target_fingerprint_can_actually_see_the_defect
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 H：回放夹具退化成「目标框一次都没给过」（阴性结论变空） ==="
"$PY" - "$JUDGE" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = '        is_full_round = slots is None or "scene" in slots\n'
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 缺陷版：拿"槽集里有没有 target"当"是不是全槽轮" —— 全槽轮本来就含 target，
# 于是第一轮的目标框也被删掉，"目标走了"这件事在回放里从未发生。
bug = '        is_full_round = slots is None or "target" not in slots\n'
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case H_fixture_never_sees_the_target \
  test_the_stale_target_fingerprint_is_zero_when_the_target_is_gone \
  test_the_stale_target_fingerprint_can_actually_see_the_defect
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 I：记账累加器又被塞回 try 里（90 秒的臂跑完才炸 NameError） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    '    results: list[dict] = []\n'
    '    try:\n'
    '        if args.warmup_seconds and args.warmup_seconds > 0:\n'
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 缺陷版就是修复前那一版：初始化落在 try 里面，`run_arm` 抛错时那条
# 赋值根本没执行过，后面的 `results.append` 才炸 —— 而且只有真跑 `main()`
# 才走到，单测直接调 `run_arm`，一条都不会红。
bug = (
    '    try:\n'
    '        if args.warmup_seconds and args.warmup_seconds > 0:\n'
)
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case I_bookkeeping_back_inside_try \
  test_main_bookkeeping_runs_from_warmup_to_disk
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
echo "=== 刀 J：locked_* 又变回按候选视图取框（无视职责分离的生效值） ==="
"$PY" - "$TOOL" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
needle = (
    "    best, used_candidates = select_candidate(\n"
    "        partition.target_candidates, partition.detections,\n"
    "        scope=scope,\n"
    "        candidates_authoritative=bool(\n"
    '            getattr(settings, "go2w_result_role_separation", False)\n'
    "        ),\n"
    "    )\n"
)
assert s.count(needle) == 1, f"锚点命中 {s.count(needle)} 次（应为 1）"
# 缺陷版（修复前那一版）：拿框的那一路**写死** `candidates_authoritative=True`，
# 也就是永远按"阶段二目标配置"取框；生效开关只管 `selection_fell_back` / `locked_pool`
# 这两列怎么报。于是默认组合下锁到的其实是全量里的椅子，报出来却像锁在目标候选上。
bug = (
    "    best, _ = select_candidate(\n"
    "        partition.target_candidates, partition.detections,\n"
    "        scope=scope,\n"
    "        candidates_authoritative=True,\n"
    "    )\n"
    "    _, used_candidates = select_candidate(\n"
    "        partition.target_candidates, partition.detections,\n"
    "        scope=scope,\n"
    "        candidates_authoritative=bool(\n"
    '            getattr(settings, "go2w_result_role_separation", False)\n'
    "        ),\n"
    "    )\n"
)
p.write_text(s.replace(needle, bug), encoding="utf-8")
PY
run_case J_locked_box_read_from_the_wrong_pool \
  test_the_locked_box_is_read_under_the_effective_switches_not_the_target_config
restore; verify_restored

# --------------------------------------------------------------------------- #
echo
if [ "$FAILED_CASES" -eq 0 ]; then
  echo "十刀都按预期变红，且每次都逐字节还原。测量判据有分辨力。"
else
  echo "有刀没被抓住 —— 判据对那条缺陷没有分辨力。" >&2
fi
exit "$FAILED_CASES"
