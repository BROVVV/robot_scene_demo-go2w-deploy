#!/usr/bin/env python3
"""Fix #3（原地打转闸门）的注错校准：每一条注入都必须让对应的断言变红。

**没有分辨力的判据不算数**（见 memory: calibrate-before-trusting-a-negative）。
这个脚本把 18 处代码逐条改回"缺陷版"，跑它声称要抓的那几条断言，要求变红，
然后立即还原并用 sha256 复核。任何一条 MISSED 都说明那条断言其实抓不住它。

纪律（见 memory: injection-restore-must-be-persisted-before-writing）：
  - 还原副本**先落盘**再改被测文件（SIGKILL 会绕过 finally）；
  - 每次运行换 PYTHONPYCACHEPREFIX（陈旧 .pyc 会让还原后的副本仍跑注入版）；
  - 起跑线与每一步之后都比 sha256 —— 并发写入（别的 agent、上一个被杀的
    注错运行）会让整轮结论作废，2026-10-14 真遇到过。

**脚本放版本控制、日志放 artifacts**：`artifacts/` 被 .gitignore 忽略。
用法：`env -u PYTHONNOUSERSITE .venv/bin/python3 scripts/go2w/calibrate_explorer_injections.py`
退出码：0 = 18 刀全按预期变红且已还原；1 = 有 MISSED；2/3 = 校准本身不成立。
"""
from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

REPO = subprocess.run(
    ["git", "rev-parse", "--show-toplevel"],
    cwd=os.path.dirname(os.path.abspath(__file__)),
    capture_output=True, text=True, check=True,
).stdout.strip()

TARGET = os.path.join(REPO, "app/live_robot/autonomous_explorer.py")
ART = os.environ.get(
    "GO2W_SPIN_GATE_ARTIFACTS",
    os.path.join(REPO, "artifacts/exploration_stuck_gate/20261014"),
)
BACKUP_DIR = os.path.join(ART, "injection_backup")
BACKUP = os.path.join(BACKUP_DIR, "autonomous_explorer.py.orig")

T_SPIN = "tests/test_in_place_turn_streak.py"
T_EXH = "tests/test_search_exhaustion_vs_stuck.py"
T_E2E = "tests/test_go2w_clearance_viewpoint_e2e.py"

# --------------------------------------------------------------------------
# 注入清单。每条 = (名字, [(old, new), ...], [要跑的 node id], 说明)
#
# 分三组：
#   第一组 I1–I9   判据对**错误实现**的分辨力（含默认路径 nav2 是地图系后端）
#   第二组 I10–I13 位姿取样**时机**与脱困动作入窗（Fix #3 的生死线）
#   第三组 I14–I17 第二个消费者闸门的四个部件：闸门、出口白名单、迟滞位、干跑豁免
# --------------------------------------------------------------------------
INJECTIONS = [
    (
        "I1 _did_translate 退回 bool(result.succeeded)",
        [(
            """        if not result.succeeded:
            return False
        displacement = _observed_translation_m(result.observed_motion)
        if displacement is not None:
            return displacement > _MIN_OBSERVED_TRANSLATION_M
        if self.dry_run or _goal_commands_translation(goal, pose):
            return True
        return False""",
            """        return bool(result.succeeded)""",
        )],
        [
            f"{T_SPIN}::test_did_translate_decision_table",
            f"{T_SPIN}::test_did_translate_discriminates_on_the_same_command",
            f"{T_SPIN}::test_metric_navigation_counts_as_moving",
        ],
        "「后端报成功」被当成「动了」⇒ 判据表首行与『遥测说没动』那行都该红",
    ),
    (
        "I2 repeated_heading_count 退回扇区访问数",
        [(
            "            repeated_heading_count=int(self._in_place_turn_streak),",
            """            repeated_heading_count=self.graph.sector_visited_count(
                observation.heading_sector
            ),""",
        )],
        [f"{T_SPIN}::test_spin_counter_is_not_the_sector_visit_count"],
        "上界为 1 的量拿去比阈值 3 ⇒ 钉这个字段的断言该红",
    ),
    (
        "I3 打转计数不再要求『没换到新信息』",
        [(
            "            if self._last_command_was_in_place_turn() and not learned_something:",
            "            if self._last_command_was_in_place_turn():",
        )],
        [
            f"{T_SPIN}::test_learning_while_rotating_is_not_a_stuck_symptom",
            f"{T_SPIN}::test_new_information_breaks_a_running_spin_streak",
        ],
        "合法的原地观察会被误报成打转 ⇒ 配对负控与时序臂都该红",
    ),
    (
        "I4 『原地转向』不再允许位姿残差（退回精确 == 0.0）",
        [(
            """        return (
            abs(float(last.get("dyaw_deg") or 0.0)) > 0.0
            and abs(float(last.get("distance_m") or 0.0))
            <= _MIN_COMMANDED_TRANSLATION_M
        )""",
            """        return (
            abs(float(last.get("dyaw_deg") or 0.0)) > 0.0
            and abs(float(last.get("distance_m") or 0.0)) == 0.0
        )""",
        )],
        [
            f"{T_SPIN}::test_last_command_was_in_place_turn_reads_the_command_window",
            f"{T_SPIN}::test_translating_commands_never_count_as_in_place_turns",
            f"{T_SPIN}::test_metric_in_place_turn_survives_a_pose_residual",
        ],
        "地图系后端的原地转向会剩下 1e-7 残差 ⇒ 默认后端上判据恒 False（Fix #3 变空操作）",
    ),
    (
        # 注入 I5 在"每轮都有新信息"那条臂上是**空操作**（那个 if 恒为假、计数
        # 从头到尾是 0），所以必须换成"先涨起来再打断"的时序臂来抓。
        "I5 打转计数永不归零",
        [(
            """            else:
                self._in_place_turn_streak = 0
            if learned_something:""",
            """            if learned_something:""",
        )],
        [f"{T_SPIN}::test_new_information_breaks_a_running_spin_streak"],
        "一旦转过就永远算打转 ⇒ 时序臂该红（旧的那条臂抓不住，见注释）",
    ),
    (
        "I6 『原地转向』只认 ROTATE_VIEW 一种 goal_type",
        [(
            """        return (
            abs(float(last.get("dyaw_deg") or 0.0)) > 0.0""",
            """        return (
            str(last.get("goal_type") or "") == "ROTATE_VIEW"
            and abs(float(last.get("dyaw_deg") or 0.0)) > 0.0""",
        )],
        [f"{T_SPIN}::test_last_command_was_in_place_turn_reads_the_command_window"],
        "LOCAL_SCAN 转成的 NAVIGATE_POSE 也是原地转向，按 goal_type 白名单过滤会漏掉",
    ),
    (
        "I7 _commanded_translation_m 退回只认 relative_*（默认路径失灵）",
        [(
            """    dx = abs(float(getattr(goal, "relative_dx", 0.0) or 0.0))
    dy = abs(float(getattr(goal, "relative_dy", 0.0) or 0.0))
    if dx + dy > 0.0:
        return dx + dy
    target = getattr(goal, "position", None)
    if target is None or pose is None:
        return 0.0
    return math.hypot(
        float(target[0]) - float(pose.x), float(target[1]) - float(pose.y)
    )""",
            """    dx = abs(float(getattr(goal, "relative_dx", 0.0) or 0.0))
    dy = abs(float(getattr(goal, "relative_dy", 0.0) or 0.0))
    return dx + dy""",
        )],
        [f"{T_SPIN}::test_metric_navigation_counts_as_moving"],
        "nav2 成功走完的 3 米会被判成『没动』⇒ 真在探索时 no_information_cycles 照样涨到阈值",
    ),
    (
        "I8 _commanded_turn_deg 去掉度/弧换算与回绕",
        [(
            """    delta = float(yaw) - math.degrees(float(pose.yaw))
    return (delta + 180.0) % 360.0 - 180.0""",
            """    return float(yaw) - float(pose.yaw)""",
        )],
        [f"{T_SPIN}::test_commanded_turn_deg_reads_both_representations_and_wraps"],
        "goal.yaw 是度、RobotPose.yaw 是弧度 ⇒ 直接相减会得到 349.83° 而不是 -20°",
    ),
    (
        "I9 open_frontiers 的 .get 退回 getattr（对 dict 恒空）",
        [(
            """            if str(frontier.get("status") or frontier.get("state") or "OPEN").upper()
            == "OPEN\"""",
            """            if str(getattr(frontier, "status", "")).upper() == "OPEN\"""",
        )],
        [
            f"{T_EXH}::test_open_frontier_replans",
            f"{T_EXH}::test_visited_frontier_is_not_an_open_frontier",
            f"{T_EXH}::test_replan_budget_falls_back_to_exhaustion",
        ],
        "拓扑前沿是普通 dict ⇒ open_frontiers 恒空 ⇒ REPLAN 那一支不可达，直接判穷尽",
    ),
    (
        # 位姿取样**时机**：这是 Fix #3 在默认路径上的生死线。地图系后端的
        # ``_commanded_translation_m`` / ``_commanded_turn_deg`` 都是「目标点 −
        # 当前位姿」，取执行之后的位姿 = "还差多远" ⇒ 走到位了恒 0 ⇒ 判据空转。
        "I10 位姿退回『执行之后』取样（默认路径上两条判据一起空转）",
        [
            (
                """            self._last_execution_moved = self._did_translate(
                goal, result, command_pose)""",
                """            self._last_execution_moved = self._did_translate(
                goal, result, self._backend.get_pose())""",
            ),
            (
                "            self._record_execution_history(goal, command_pose, result)",
                """            self._record_execution_history(
                goal, self._backend.get_pose(), result)""",
            ),
            (
                """            command_pose = self._backend.get_pose()
            handle = self._backend.execute_goal(goal)""",
                "            handle = self._backend.execute_goal(goal)",
            ),
        ],
        [f"{T_E2E}::test_metric_command_window_records_the_command_not_the_arrival"],
        "地图系后端走完 3 m 会被记成 distance=0 ⇒ 命令窗口里一条'要走距离'都没有",
    ),
    (
        "I11 接近闭环的第二处取样点退回『执行之后』",
        [
            (
                "            self._record_execution_history(goal, approach_command_pose, result)",
                """            self._record_execution_history(
                goal, self._backend.get_pose(), result)""",
            ),
            (
                """            approach_command_pose = self._backend.get_pose()
            handle = self._backend.execute_goal(goal)""",
                "            handle = self._backend.execute_goal(goal)",
            ),
        ],
        [f"{T_E2E}::test_metric_approach_records_commanded_distance"],
        "接近闭环里地图系目标同样会被记成落点（E2E-4/E2E-5 用相对后端，测不出这处）",
    ),
    (
        # 脱困动作不进命令窗口：下一个周期 ``_last_command_was_in_place_turn``
        # 读到的是**脱困之前**那条陈旧命令，而脱困转向本身就是原地转向原语
        # ⇒ 连跳计数漏数、``detect_stuck`` 拿到脱困盲的窗口。
        "I12 脱困动作（倒车/脱困转向）不写命令窗口",
        [
            (
                """            recovery_pose = self._backend.get_pose()
            handle = self._backend.execute_goal(goal)
            backward_result = self._wait_result(handle, timeout_sec=20.0)
            self._record_execution_history(goal, recovery_pose, backward_result)
""",
                """            handle = self._backend.execute_goal(goal)
            backward_result = self._wait_result(handle, timeout_sec=20.0)
""",
            ),
            (
                """        recovery_pose = self._backend.get_pose()  # 同倒车：位姿取在下发之前
        handle = self._backend.execute_goal(turn)
        turn_result = self._wait_result(handle, timeout_sec=20.0)
        self._record_execution_history(turn, recovery_pose, turn_result)
""",
                """        handle = self._backend.execute_goal(turn)
        turn_result = self._wait_result(handle, timeout_sec=20.0)
""",
            ),
        ],
        [f"{T_EXH}::test_recovery_motion_enters_the_command_window"],
        "命令窗口里一条脱困动作都没有 ⇒ 原地转向连跳会把脱困转向漏数",
    ),
    (
        # 两条脱困支路各走各的见证臂。**第一轮这两条合在一条 I13 里，结果 MISSED**：
        # 见证臂当时用的是 ``outcome_sequence=[FAILED]``，而 ``MockMetricBackend``
        # 把最后一个状态当"剩下的都这样" ⇒ 脱困动作永远失败 ⇒ 位姿从头到尾没动过
        # ⇒ 取样点在这条臂上不可观测（等价变异体）。拆成两条 + 两条各自成功执行
        # 的见证臂之后，两条都被抓住。夹具故障 ≠ 断言漏，但结论一样：证据不成立。
        "I13a 倒车脱困的位姿退回『执行之后』取样",
        [(
            """            recovery_pose = self._backend.get_pose()
            handle = self._backend.execute_goal(goal)
            backward_result = self._wait_result(handle, timeout_sec=20.0)
            self._record_execution_history(goal, recovery_pose, backward_result)""",
            """            handle = self._backend.execute_goal(goal)
            backward_result = self._wait_result(handle, timeout_sec=20.0)
            self._record_execution_history(
                goal, self._backend.get_pose(), backward_result)""",
        )],
        [f"{T_EXH}::test_metric_backward_recovery_records_the_commanded_backoff"],
        "倒车走到面包屑起点后位姿已搬过去 ⇒ distance 记成 0，脱困倒车在窗口里消失",
    ),
    (
        "I13b 脱困转向的位姿退回『执行之后』取样",
        [(
            """        recovery_pose = self._backend.get_pose()  # 同倒车：位姿取在下发之前
        handle = self._backend.execute_goal(turn)
        turn_result = self._wait_result(handle, timeout_sec=20.0)
        self._record_execution_history(turn, recovery_pose, turn_result)""",
            """        handle = self._backend.execute_goal(turn)
        turn_result = self._wait_result(handle, timeout_sec=20.0)
        self._record_execution_history(turn, self._backend.get_pose(), turn_result)""",
        )],
        [f"{T_EXH}::test_recovery_motion_enters_the_command_window"],
        "脱困转向走到位后位姿已搬到目标朝向 ⇒ dyaw 记成 0，连跳计数看不见脱困转向",
    ),
    (
        # 第二个消费者闸门（2026-10-14）。四条注入分别拆掉它的四个部件：
        # 闸门本身、脱困/等地图之外的白名单、迟滞位、干跑豁免。
        "I14 第二个闸门没接上（退回只看『连续 N 轮无信息』）",
        [(
            "            spin_gate = spin_condition and not self._spin_gate_latched",
            "            spin_gate = False",
        )],
        [
            f"{T_SPIN}::test_spin_gate_asks_the_stuck_classifier_before_the_no_information_budget",
            f"{T_SPIN}::test_a_spin_only_consultation_cannot_end_the_search",
            f"{T_SPIN}::test_a_spin_only_consultation_cannot_declare_exhaustion",
        ],
        "连跳计数又变成一个没有消费者的量：三条臂的前提断言（分类被问到了）都该红",
    ),
    (
        "I15 连跳推开的那次咨询直接采信（去掉出口白名单）",
        [(
            "                if not spin_only:",
            "                if True:",
        )],
        [
            f"{T_SPIN}::test_a_spin_only_consultation_cannot_end_the_search",
            f"{T_SPIN}::test_a_spin_only_consultation_cannot_declare_exhaustion",
        ],
        "『原地打转』被读成『提前收工』：REPLAN 会吃掉重规划预算、穷尽那一支会直接 break",
    ),
    (
        "I16 迟滞位失效（闸门每轮都能开）",
        [(
            "            spin_gate = spin_condition and not self._spin_gate_latched",
            "            spin_gate = spin_condition",
        )],
        [f"{T_SPIN}::test_the_spin_gate_asks_at_most_once_per_stretch"],
        "脱困支路是 continue（不涨 planning_cycles）⇒ 循环只剩 max_search_seconds 一个边界",
    ),
    (
        "I17 干跑不豁免连跳闸门",
        [(
            """            spin_condition = (
                not self.dry_run and self._in_place_turn_streak >= spin_limit)""",
            """            spin_condition = self._in_place_turn_streak >= spin_limit""",
        )],
        [f"{T_SPIN}::test_dry_run_never_self_reports_as_stuck"],
        "干跑的合成流被当成机器人状态：自检里冒出 stuck_detected / 脱困",
    ),
]


def sha(path: str) -> str:
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def run_tests(node_ids: list[str]) -> tuple[bool, str]:
    env = dict(os.environ)
    env.pop("PYTHONNOUSERSITE", None)
    run_id = hashlib.sha1(
        ("|".join(node_ids)).encode() + os.urandom(6)
    ).hexdigest()[:10]
    env["PYTHONPYCACHEPREFIX"] = "/tmp/pyc_inj3_" + run_id
    os.makedirs(ART, exist_ok=True)
    report = os.path.join(ART, f"pytest_{run_id}.xml")
    cmd = [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider"]
    cmd += [f"--junitxml={report}"]
    cmd += node_ids
    proc = subprocess.run(cmd, cwd=REPO, env=env, capture_output=True, text=True)
    with open(os.path.join(ART, f"pytest_{run_id}.log"), "w", encoding="utf-8") as fh:
        fh.write(proc.stdout or "")
        fh.write(proc.stderr or "")
    # 收集错误、进程崩溃或环境错误不算抓住注入；必须有实际执行用例的 failure。
    if proc.returncode not in (0, 1) or not os.path.isfile(report):
        raise RuntimeError(f"pytest 未正常完成，exit={proc.returncode}; log={run_id}")
    suites = list(ET.parse(report).getroot().iter("testsuite"))
    errors = sum(int(suite.get("errors", "0")) for suite in suites)
    failures = sum(int(suite.get("failures", "0")) for suite in suites)
    if errors or (proc.returncode == 1 and not failures):
        raise RuntimeError(f"pytest 环境/收集错误不算 CAUGHT; log={run_id}")
    tail = (proc.stdout or "").strip().splitlines()
    summary = tail[-1] if tail else "(no output)"
    return failures > 0, summary


def main() -> int:
    if not sys.executable.startswith(os.path.join(REPO, ".venv")):
        print(f"!! 建议用 {REPO}/.venv/bin/python3 跑（当前 {sys.executable}）")
    os.makedirs(BACKUP_DIR, exist_ok=True)
    original_sha = sha(TARGET)

    # 0) 起跑线守卫：基线跑两分钟，期间**任何**别的进程（并发的复核 agent、
    # 上一个被 SIGKILL 的注错运行）动了这个文件，整轮校准都作废——2026-10-14
    # 就真遇到过：文件里残留着一条被杀的注错（记录点被改回"执行之后"取样），
    # 基线拿它当"原件"，还原副本与原件对不上。
    mtime0 = os.stat(TARGET).st_mtime_ns
    print(f"起跑线 sha256={original_sha[:12]} mtime={mtime0}")

    # 1) 基线：不注入时必须全绿
    print("=== 基线（未注入）===")
    all_tests = sorted({t for _, _, tests, _ in INJECTIONS for t in tests})
    red, summary = run_tests(all_tests)
    print(f"  {summary}")
    if red:
        print("!! 基线就是红的，注错校准无意义。先修测试。")
        return 2
    if os.stat(TARGET).st_mtime_ns != mtime0 or sha(TARGET) != original_sha:
        print("!! 基线期间有别的进程改写了被测文件，本轮作废（并发写入）。")
        return 3

    # 2) 落盘还原副本（先落盘，再动文件）
    shutil.copy2(TARGET, BACKUP)
    assert sha(BACKUP) == original_sha, "还原副本与原件不一致"

    results = []
    try:
        for name, edits, tests, why in INJECTIONS:
            src = open(TARGET, encoding="utf-8").read()
            for old, new in edits:
                count = src.count(old)
                if count != 1:
                    raise RuntimeError(f"{name}: 锚点命中 {count} 次，应为 1 次")
                src = src.replace(old, new)
            with open(TARGET, "w", encoding="utf-8") as fh:
                fh.write(src)
            assert sha(TARGET) != original_sha, f"{name}: 注入没有真正改变文件"

            red, summary = run_tests(tests)
            results.append((name, red, summary, why))
            print(f"[{'CAUGHT' if red else 'MISSED'}] {name}\n    {summary}\n    {why}")

            shutil.copy2(BACKUP, TARGET)
            assert sha(TARGET) == original_sha, f"{name}: 还原失败"
    finally:
        if os.path.exists(BACKUP):
            shutil.copy2(BACKUP, TARGET)

    print("\n=== 汇总 ===")
    missed = [r for r in results if not r[1]]
    for name, red, summary, _ in results:
        print(f"  {'CAUGHT' if red else 'MISSED'}  {name}")
    print(f"\n{len(results) - len(missed)}/{len(results)} 被抓住")
    print(f"文件 sha256 已还原: {sha(TARGET) == original_sha}")
    print(f"还原副本: {BACKUP}")
    return 1 if missed else 0


if __name__ == "__main__":
    raise SystemExit(main())
