"""Navigation2 sidebar controls and result view."""
from __future__ import annotations
import json
from pathlib import Path
import streamlit as st
from app.navigation.nav2_config import Nav2Settings
from app.navigation.nav2_gateway import Nav2Gateway
from app.navigation.nav2_models import Nav2Mode, Nav2Pose
from app.navigation.nav2_request_builder import make_request
from app.ui.nav2_visualization import render_nav2_path_figure

def render_nav2_sidebar() -> dict:
    st.markdown("### Navigation2 导航")
    enabled=st.toggle("启用 Navigation2",value=False)
    labels={"关闭":"disabled","视觉规划预览":"visual_preview","离线路径预览":"offline_preview","Nav2 只规划不执行":"plan_only","Nav2 规划并执行":"execute"}
    label=st.selectbox("导航模式",list(labels),disabled=not enabled)
    mode=labels[label] if enabled else "disabled"
    visual_only=mode=="visual_preview"
    x=st.number_input("目标 X",value=1.0,disabled=not enabled or visual_only); y=st.number_input("目标 Y",value=0.0,disabled=not enabled or visual_only)
    yaw=st.number_input("目标 Yaw（度）",value=0.0,disabled=not enabled or visual_only)
    current=st.checkbox("使用当前机器人位姿作为起点",value=True,disabled=not enabled)
    confirmations={}
    if mode=="execute":
        st.warning("危险操作：环境变量和四项人工确认必须同时通过。")
        confirmations["safe"]=st.checkbox("仿真或机器人处于安全测试区")
        confirmations["estop"]=st.checkbox("急停可用")
        confirmations["footprint"]=st.checkbox("footprint 已实测配置")
        confirmations["motion"]=st.checkbox("允许本次发布运动速度")
    action=st.button({"visual_preview":"视觉规划由视频分析自动生成","offline_preview":"生成离线路径预览","plan_only":"请求 Nav2 规划","execute":"启动 Nav2 导航"}.get(mode,"Navigation2 已关闭"),
                     disabled=not enabled or mode in {"disabled","visual_preview"} or (mode=="execute" and not all(confirmations.values())),
                     use_container_width=True)
    return {"enabled":enabled,"mode":mode,"x":x,"y":y,"yaw":yaw,"current":current,"confirmations":confirmations,"action":action}

def process_and_render_nav2(config: dict) -> None:
    if config.get("mode") == "visual_preview":
        st.info("Visual Preview 只展示视频视觉规划，不会请求 ROS2/Nav2，也不会发布 /cmd_vel。")
        return
    if config.get("action"):
        try:
            settings=Nav2Settings.from_env()
            mode=Nav2Mode(config["mode"])
            pose=Nav2Pose(frame_id=settings.map_frame,x=config["x"],y=config["y"],
                yaw_rad=config["yaw"]*3.141592653589793/180,source="manual_webui",
                provenance={"type":"user_input","details":"Web UI map goal form"})
            confirm=config["confirmations"]
            request=make_request(mode=mode,goal=pose,settings=settings,use_current_start=config["current"],
                allow_execute=confirm.get("motion",False),operator_confirmed=confirm.get("safe",False),
                footprint_confirmed=confirm.get("footprint",False),estop_confirmed=confirm.get("estop",False),
                source="manual_webui")
            handle=Nav2Gateway(settings).execute(request) if mode==Nav2Mode.EXECUTE else Nav2Gateway(settings).plan(request)
            st.session_state.nav2_request_id=handle.request_id
        except Exception as exc:
            st.error(f"Navigation2 请求失败：{exc}")
    request_id=st.session_state.get("nav2_request_id")
    if not config.get("enabled") and not request_id:
        return
    if request_id:
        try:
            Nav2Gateway().get_status(request_id)
        except Exception as exc:
            st.error(f"读取 Nav2 Worker 状态失败：{exc}")
    latest=Path("outputs/nav2_webui_payload.json")
    if not request_id and not latest.exists(): return
    st.header("Navigation2 导航")
    if not latest.exists():
        st.info("Worker 已启动，点击“刷新状态”查看结果。"); st.button("刷新状态"); return
    payload=json.loads(latest.read_text(encoding="utf-8")); status=payload.get("status",{}); path=payload.get("path",{})
    if not status.get("is_real_nav2_path"): st.warning("OFFLINE PREVIEW / 非 Nav2 真实路径 / 不可执行")
    tabs=st.tabs(["导航总览","全局路径","路径步骤","实时反馈","速度指令","原始 JSON","诊断日志"])
    with tabs[0]:
        cols=st.columns(4)
        values=[("模式",payload.get("request",{}).get("mode","-")),("状态",status.get("state","-")),
                ("路径长度",f"{path.get('path_length_m',0):.2f} m"),("恢复次数",status.get("number_of_recoveries",0))]
        for col,(name,value) in zip(cols,values): col.metric(name,value)
        st.write(status.get("message_zh",""))
        if status.get("error_code"): st.error(f"{status['error_code']}: {status.get('error_message','')}")
    with tabs[1]:
        st.pyplot(render_nav2_path_figure(path,status.get("current_pose")),use_container_width=True)
        st.dataframe(path.get("poses",[]),use_container_width=True)
    with tabs[2]:
        st.warning(payload.get("instruction_preview",{}).get("warning","这些步骤不是速度控制指令。"))
        st.dataframe(payload.get("instruction_preview",{}).get("steps",[]),use_container_width=True)
    with tabs[3]:
        st.write(f"剩余距离：{status.get('distance_remaining_m','-')} m；ETA：{status.get('estimated_time_remaining_sec','-')} s")
        st.dataframe(payload.get("feedback_tail",[]),use_container_width=True)
    with tabs[4]:
        mode=payload.get("request",{}).get("mode")
        if mode=="plan_only": st.info("只规划模式不会产生 /cmd_vel。")
        elif mode=="offline_preview": st.warning("模拟轨迹仅用于测试图表。")
        st.dataframe(payload.get("cmd_vel_tail",[]),use_container_width=True)
    with tabs[5]: st.json(payload)
    with tabs[6]: st.json(payload.get("diagnostics",{}))
    if st.button("取消当前导航") and request_id: Nav2Gateway().cancel(request_id)
    st.button("刷新状态")
