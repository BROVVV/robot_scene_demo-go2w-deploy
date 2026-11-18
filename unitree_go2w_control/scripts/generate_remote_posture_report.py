#!/usr/bin/env python3
"""Generate the evidence-based remote posture capture report."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any


def read_json(path: Path, default: Any) -> Any:
    if not path.exists():
        return default
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def value_from_env(path: Path, name: str) -> str:
    if not path.exists():
        return "unknown"
    prefix = name + "="
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith(prefix):
            return line[len(prefix) :]
    return "unknown"


def state_changed(summary: dict[str, Any]) -> bool:
    before = summary.get("body_height_before")
    minimum = summary.get("body_height_min")
    maximum = summary.get("body_height_max")
    height_changed = (
        before is not None
        and minimum is not None
        and maximum is not None
        and max(abs(float(minimum) - float(before)), abs(float(maximum) - float(before)))
        >= 0.02
    )
    return bool(
        height_changed
        or summary.get("mode_before") != summary.get("mode_after")
        or summary.get("leg_joint_change_observed")
    )


def request_signature(request: dict[str, Any]) -> tuple[int, str]:
    return int(request["api_id"]), str(request["parameter"])


def observed_button_map(capture: Path) -> dict[str, Any]:
    round_name = "02_key_calibration"
    timeline = read_jsonl(capture / "text" / f"{round_name}_combined_timeline.jsonl")
    markers = read_jsonl(capture / "text" / f"{round_name}_operator_markers.jsonl")
    wireless_topics = ("/wirelesscontroller", "/wirelesscontroller_unprocessed")
    topic = next(
        (
            candidate
            for candidate in wireless_topics
            if any(row.get("topic") == candidate for row in timeline)
        ),
        None,
    )
    mapping: dict[str, Any] = {}
    if topic is None:
        return mapping
    samples = [row for row in timeline if row.get("topic") == topic]
    for marker in markers:
        if marker.get("event") != "calibration_key_imminent":
            continue
        button = str(marker.get("button"))
        start = int(marker["receive_monotonic_ns"])
        end_marker = next(
            (
                item
                for item in markers
                if item.get("event") == "calibration_key_complete"
                and item.get("button") == button
                and int(item["receive_monotonic_ns"]) >= start
            ),
            None,
        )
        if end_marker is None:
            continue
        end = int(end_marker["receive_monotonic_ns"])
        before = [row for row in samples if int(row["receive_monotonic_ns"]) <= start]
        baseline = int(before[-1]["message"]["keys"]) if before else 0
        candidates = [
            int(row["message"]["keys"])
            for row in samples
            if start <= int(row["receive_monotonic_ns"]) <= end
            and int(row["message"]["keys"]) != baseline
        ]
        if not candidates:
            continue
        keys = max(candidates, key=lambda value: (value.bit_count(), value))
        mapping[button] = {
            "keys_decimal": keys,
            "keys_hex": f"0x{keys:04x}",
            "set_bits": [bit for bit in range(16) if keys & (1 << bit)],
            "topic": topic,
        }
    return mapping


def direction_result(
    direction: str,
    summaries: list[dict[str, Any]],
    baseline_signatures: set[tuple[int, str]],
) -> dict[str, Any]:
    rows = [row for row in summaries if row.get("direction") == direction]
    chords = [
        str(row["chord"]["keys_hex"])
        for row in rows
        if isinstance(row.get("chord"), dict)
    ]
    chord_counter = Counter(chords)
    common_chord, common_count = chord_counter.most_common(1)[0] if chord_counter else (None, 0)
    state_successes = sum(state_changed(row) for row in rows)
    candidates: Counter[tuple[int, str]] = Counter()
    candidate_rows: dict[tuple[int, str], list[tuple[dict[str, Any], dict[str, Any]]]] = {}
    for row in rows:
        response_index = {
            (int(response["request_id"]), int(response["api_id"])): response
            for response in row.get("responses", [])
        }
        seen_this_round: set[tuple[int, str]] = set()
        for request in row.get("requests", []):
            signature = request_signature(request)
            relative = float(request["relative_time_s"])
            if signature in baseline_signatures or not -0.5 <= relative <= 5.0:
                continue
            response = response_index.get(
                (int(request["request_id"]), int(request["api_id"]))
            )
            if response is None or int(response["status_code"]) != 0:
                continue
            candidate_rows.setdefault(signature, []).append((request, response))
            seen_this_round.add(signature)
        candidates.update(seen_this_round)
    public_signature, public_count = candidates.most_common(1)[0] if candidates else (None, 0)
    return {
        "direction": direction,
        "round_count": len(rows),
        "state_success_count": state_successes,
        "repeatable": len(rows) >= 3 and state_successes >= 2 and common_count >= 2,
        "common_chord": common_chord,
        "common_chord_count": common_count,
        "public_request_signature": public_signature,
        "public_request_round_count": public_count,
        "public_request_evidence": []
        if public_signature is None
        else candidate_rows.get(public_signature, []),
        "rounds": rows,
    }


def markdown_table_request(result: dict[str, Any]) -> str:
    evidence = result["public_request_evidence"]
    if not evidence:
        return "未观察到满足重复性、响应和时间关联条件的公开姿态请求。\n"
    lines = [
        "| 相对时间 | API ID | request ID | lease ID | parameter | status |",
        "|---:|---:|---:|---:|---|---:|",
    ]
    for request, response in evidence:
        lines.append(
            f"| {request['relative_time_s']:+.6f}s | {request['api_id']} | "
            f"{request['request_id']} | {request['lease_id']} | "
            f"`{request['parameter']}` | {response['status_code']} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-root", required=True, type=Path)
    parser.add_argument("--project-root", type=Path)
    args = parser.parse_args()
    capture = args.capture_root.resolve()
    project = (
        args.project_root.resolve()
        if args.project_root
        else Path(__file__).resolve().parent.parent
    )
    analysis = capture / "analysis"
    reports = project / "reports"
    reports.mkdir(parents=True, exist_ok=True)
    summaries = read_json(analysis / "action_round_summaries.json", [])
    requests = read_jsonl(analysis / "sport_requests.jsonl")
    baseline_signatures = {
        (int(row["api_id"]), str(row.get("parameter_raw", "")))
        for row in requests
        if row["round"] == "01_idle_baseline"
    }
    down = direction_result("down", summaries, baseline_signatures)
    up = direction_result("up", summaries, baseline_signatures)
    public_request = (
        down["public_request_round_count"] >= 2
        and up["public_request_round_count"] >= 2
    )
    wireless_repeatable = down["repeatable"] and up["repeatable"]
    if public_request:
        path_class = "A"
        replay_ready = True
    elif wireless_repeatable:
        path_class = "B"
        replay_ready = False
    elif summaries:
        path_class = "未确定"
        replay_ready = False
    else:
        path_class = "未确定"
        replay_ready = False

    classification = {
        "implementation_path": path_class,
        "safe_replay_evidence_ready": replay_ready,
        "remote_stand_down_repeatable": down["repeatable"],
        "remote_stand_up_repeatable": up["repeatable"],
        "public_sport_request_observed": public_request,
        "down": down,
        "up": up,
    }
    with (analysis / "path_classification.json").open("w", encoding="utf-8") as handle:
        json.dump(classification, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    button_map = observed_button_map(capture)
    observed_map = {
        "scope": "observed_on_this_remote",
        "button_labels_calibrated": bool(button_map),
        "buttons": button_map,
        "stand_down_chord": down["common_chord"],
        "stand_up_chord": up["common_chord"],
        "note": "bit chords are observed; physical button labels require separate safe single-key calibration",
    }
    with (analysis / "remote_key_map_observed.json").open("w", encoding="utf-8") as handle:
        json.dump(observed_map, handle, ensure_ascii=False, indent=2)
        handle.write("\n")

    network = capture / "metadata" / "network.txt"
    ros_env = capture / "metadata" / "ros_environment.txt"
    repositories = capture / "metadata" / "unitree_repositories.txt"
    report = reports / "go2w_remote_posture_capture_report.md"
    final_confirmation_path = capture / "metadata" / "final_operator_safety_confirmation.txt"
    final_confirmation_ok = (
        final_confirmation_path.exists()
        and final_confirmation_path.read_text(encoding="utf-8").strip()
        == "I_CONFIRM_ROBOT_SAFE_STANDING"
    )
    with report.open("w", encoding="utf-8") as handle:
        handle.write("# Go2-W 遥控器姿态被动抓取报告\n\n")
        handle.write("## 结论摘要\n\n")
        handle.write(f"- 遥控器趴下是否可重复：{'YES' if down['repeatable'] else 'NO'}\n")
        handle.write(f"- 遥控器起立是否可重复：{'YES' if up['repeatable'] else 'NO'}\n")
        handle.write(f"- 是否观察到公开 Sport Request：{'YES' if public_request else 'NO'}\n")
        handle.write(f"- 实现路径：{path_class}\n")
        handle.write(f"- 是否具备安全复刻条件：{'YES' if replay_ready else 'NO'}\n\n")
        handle.write("## 环境\n\n")
        handle.write(f"- 抓取目录：`{capture}`\n")
        handle.write(f"- 网卡：{value_from_env(network, 'ROBOT_IFACE')}\n")
        handle.write(f"- 主机 IP：{value_from_env(network, 'HOST_IP')}\n")
        handle.write("- 机器人 IP：192.168.123.18\n")
        handle.write("- 实际 DDS 数据端点：192.168.123.161（最终无过滤 PCAP 基线观察）\n")
        handle.write(f"- ROS 2：{value_from_env(ros_env, 'ROS_DISTRO')}\n")
        handle.write(f"- RMW：{value_from_env(ros_env, 'RMW_IMPLEMENTATION')}\n")
        handle.write(f"- ROS_DOMAIN_ID：{value_from_env(ros_env, 'ROS_DOMAIN_ID')}\n")
        handle.write("- MotionSwitcher：`robot_form=1`、`motion_name=ai-w`（本任务前已确认）\n")
        handle.write("- 官方仓库版本：\n\n```text\n")
        handle.write(repositories.read_text(encoding="utf-8", errors="replace") if repositories.exists() else "unknown\n")
        handle.write("```\n\n")
        handle.write("## 遥控器组合与状态证据\n\n")
        handle.write(
            f"- 本机遥控器安全单键标定：{json.dumps(button_map, ensure_ascii=False) if button_map else '未完成或未观察到消息'}\n\n"
        )
        for result, title in ((down, "趴下"), (up, "起立")):
            handle.write(f"### {title}\n\n")
            handle.write(
                f"- 轮次：{result['round_count']}；检测到状态变化：{result['state_success_count']}；"
                f"重复组合：`{result['common_chord']}`（{result['common_chord_count']} 轮）。\n"
            )
            for row in result["rounds"]:
                chord = row.get("chord") or {}
                handle.write(
                    f"- {row['round']}: topic=`{row.get('wireless_topic')}` "
                    f"keys=`{chord.get('keys_hex')}` bits={chord.get('set_bits')} "
                    f"hold={chord.get('duration_s')}s mode={row.get('mode_before')}→{row.get('mode_after')} "
                    f"height={row.get('body_height_before')} / [{row.get('body_height_min')}, "
                    f"{row.get('body_height_max')}] joints_changed={row.get('leg_joint_change_observed')}\n"
                )
            handle.write("\n")
        handle.write("## 公开请求证据\n\n")
        handle.write("### 趴下\n\n" + markdown_table_request(down) + "\n")
        handle.write("### 起立\n\n" + markdown_table_request(up) + "\n")
        handle.write(
            "动作窗口及基线共解析到的 Sport 请求均为周期性 `API 1034`、"
            "参数 `[\"economicGait\"]`，未出现动作相关的新 API 或参数。\n\n"
        )
        handle.write("## Go2-W 状态机细节\n\n")
        handle.write(
            "- D1 从 `mode=1` 开始：第一次 `0x0120` 后约 82 ms 进入 `mode=6`，"
            "第二次相同组合后进入 `mode=5` 趴下。\n"
            "- U1/U2/U3 均由一次 `0x0120` 从 `mode=5` 起立到 `mode=6`。"
            "操作者确认此时物理站立且可正常运动。\n"
            "- D2/D3 从起立后的 `mode=6` 开始，一次 `0x0120` 即进入 `mode=5`。"
            "因此“趴下要按两次”只适用于初始 `mode=1`；从 `mode=6` 只需一次。\n"
            "- `body_height` 在全部样本中保持 `0.0`，不适合作为该固件的姿态判据；"
            "mode 与 12 个腿关节的持续变化提供了动作证据。\n\n"
        )
        handle.write("## PCAP 范围说明\n\n")
        handle.write(
            "动作轮 PCAP 使用初始过滤器 `host 192.168.123.18`，实际只记录到该地址的 ARP，"
            "没有记录 DDS UDP。最终 6 秒无过滤静止探针记录到 1056 个 UDP 包，"
            "确认 DDS 数据端点主要为 `192.168.123.161`。因此路径判断只使用 rosbag、"
            "统一时间线、ROS 图和状态证据，不把动作 PCAP 作为 DDS 强证据；"
            "后续抓取过滤器已修正为整个 `192.168.123.0/24` 的 UDP/ARP/ICMP。\n\n"
        )
        handle.write("## 控制仲裁现场观察\n\n")
        handle.write(
            "操作者报告：官方遥控器保持连接时，命令行不能控制机器人。"
            "本轮是严格被动抓取，没有为验证该现象而发送电脑端控制请求；"
            "因此将其标记为现场观察，而不是已独立验证的固件规则。\n\n"
        )
        handle.write("## 路径判断与复刻建议\n\n")
        if path_class == "A":
            handle.write(
                "至少两轮趴下和起立均出现一致、非基线、响应成功且紧随遥控器组合的公开请求。"
                "可以据此进入单请求重构审计，但本次分析不会自动执行任何请求。\n"
            )
        elif path_class == "B":
            handle.write(
                "至少两轮遥控器组合和物理状态变化一致，但未观察到符合条件的新公开 Sport Request。"
                "证据支持 wheeled_sport 内部消费遥控器状态；不生成伪造 wirelesscontroller 发布程序。\n"
            )
        else:
            handle.write("证据尚不足以在 A/B/C 中作确定判断，不生成控制复刻程序。\n")
        handle.write("\n## 安全审计\n\n")
        handle.write(
            "- 抓取工具通过只读源码审计：不申请 Sport lease，不发布 `/api/sport/request` 或 `/lowcmd`。\n"
            "- 未回放 rosbag；离线分析使用只读 rosbag2 API。\n"
            f"- 最终操作者安全站立确认：{'PASS' if final_confirmation_ok else 'MISSING'}。\n"
        )

    question = reports / "unitree_go2w_remote_posture_api_question.md"
    if path_class != "A":
        with question.open("w", encoding="utf-8") as handle:
            handle.write("# 给 Unitree：Go2-W 遥控器姿态公开 API 询问材料\n\n")
            handle.write(
                "- 机型：Go2-W\n- MotionSwitcher：`robot_form=1`、`motion_name=ai-w`\n"
                "- Sport API version：`1.0.0.1`\n- 带有效 lease 的 `StandDown 1005` 返回 `-1`\n"
                "- `Move 1008` 与 `StopMove 1003` 正常\n"
                f"- 被动抓取路径判断：{path_class}\n"
                f"- 趴下组合：`{down['common_chord']}`；起立组合：`{up['common_chord']}`\n\n"
                "- 状态序列：`mode 1 -> 6 -> 5`（首次趴下）；`mode 5 -> 6`（起立）；"
                "`mode 6 -> 5`（后续趴下）\n"
                "- 操作者观察：遥控器连接时命令行控制不可用（未在被动轮中主动复验）\n"
                "- PCAP 观察到的 DDS 数据端点：`192.168.123.161`\n\n"
                "请确认：\n\n1. Go2-W 遥控器姿态动作由哪个公开 API 实现；\n"
                "2. 是否由 wheeled_sport 内部直接消费 `/wirelesscontroller`；\n"
                "3. 是否存在 Go2-W 专用 StandDown/StandUp API；\n"
                "4. 所需前置状态、参数和固件版本；\n"
                "5. `StandDown 1005=-1` 的准确含义。\n"
            )
    print(
        f"REMOTE_POSTURE_REPORT=PASS path={path_class} replay_ready={replay_ready} "
        f"report={report}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
