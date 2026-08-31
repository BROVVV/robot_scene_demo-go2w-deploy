"""Go2-W live-mode controls that remain disabled while capability gates are closed."""

from __future__ import annotations

import contextlib
import io

import streamlit as st

from app.live_robot.ui_status import blockers_for_mode, load_live_ui_status


MODE_LABELS = {
    "只观察": "observe_only",
    "启动短步搜索": "step_search",
    "Nav2 只规划": "nav2_plan_only",
    "允许 Nav2 执行": "nav2_execute",
}


def render_go2w_live_sidebar() -> dict:
    st.markdown("### Go2-W 实时目标搜索")
    status = load_live_ui_status()
    mode_label = st.selectbox("实时搜索模式", list(MODE_LABELS))
    mode = MODE_LABELS[mode_label]
    detector = st.selectbox(
        "实时检测器",
        ["grounded_sam", "llm", "mock"],
        format_func=lambda value: {"grounded_sam": "GroundingDINO+SAM2", "llm": "视觉大模型", "mock": "Mock"}[value],
    )
    use_llm = st.checkbox(
        "启用 LLM 目标画像与裁剪复核（需已配置外部 API Key）",
        value=False,
        help="默认关闭，避免在未授权时触发外部付费/网络模型调用。",
    )
    blockers = blockers_for_mode(mode, status)
    if blockers:
        st.error("当前模式已阻断：" + "、".join(blockers))
    else:
        st.success("当前模式门禁通过")

    confirmation = ""
    if mode == "nav2_execute":
        confirmation = st.text_input(
            "二次确认短语",
            placeholder="I_CONFIRM_GO2W_NAVIGATION_EXECUTION",
        )
    confirmed = confirmation == "I_CONFIRM_GO2W_NAVIGATION_EXECUTION"
    start = st.button(
        mode_label,
        type="primary" if mode == "observe_only" else "secondary",
        disabled=bool(blockers) or (mode == "nav2_execute" and not confirmed),
        use_container_width=True,
    )
    columns = st.columns(3)
    columns[0].button("暂停", disabled=True, use_container_width=True)
    columns[1].button("取消", disabled=True, use_container_width=True)
    columns[2].button("紧急停止", disabled=True, use_container_width=True)
    st.caption("控制执行当前全局禁用；暂停、取消和急停需接入已验收的 leased Action 链后才开放。")
    return {
        "mode": mode,
        "detector": detector,
        "start": start,
        "blockers": blockers,
        "status": status,
        "confirmed": confirmed,
        "use_llm": use_llm,
    }


def render_go2w_live_workspace(config: dict, *, target: str) -> None:
    status = config["status"]
    st.header("Go2-W 实时状态")
    names = (
        "camera",
        "lidar",
        "lio",
        "tf",
        "camera_info_calibrated",
        "rgb_lidar_overlay",
        "rgb_lidar_extrinsics",
        "rgb_lidar_fusion",
        "pandar",
        "dual_lidar",
    )
    cols = st.columns(len(names))
    for column, name in zip(cols, names):
        column.metric(name, "PASS" if status.sensor_health.get(name) else "BLOCKED")
    with st.expander("Pandar / 双雷达诊断状态", expanded=False):
        st.json(
            {
                "pandar": status.pandar_status or {"raw_fresh": False},
                "dual_lidar": status.dual_lidar_status or {
                    "diagnostic_ready": False,
                    "rotation_observability_valid": False,
                    "rotation_clearance_valid": False,
                },
            }
        )
        st.caption(
            "Pandar raw cloud 通过只读验收不等于正式 TF/安全融合 PASS；"
            "rotation_clearance_valid 仍由实时门禁决定。"
        )
    st.json(
        {
            "latest_session_id": status.latest_session_id,
            "latest_frame_id": status.latest_frame_id,
            "control": status.control,
            "search": status.search,
            "plan_only_gate": status.plan_gate,
            "execute_gate": status.execute_gate,
        }
    )
    if not config.get("start"):
        return
    # Keep detector/model imports out of ordinary Streamlit rendering.
    from run_live_robot_demo import main as run_live_main

    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        code = run_live_main(
            [
                "--target",
                target,
                "--detector",
                config["detector"],
                "--search-mode",
                config["mode"],
                "--max-frames",
                "5",
            ]
            + ([] if config["use_llm"] else ["--disable-crop-verify", "--disable-llm-profile"])
        )
    if code == 0:
        st.success(output.getvalue())
    else:
        st.error(output.getvalue())
