#!/usr/bin/env python3
"""Generate a dense four-slide GO2W project architecture presentation.

The deck intentionally uses text, tables and vector diagrams instead of
large photographs so it can serve as a compact technical project briefing.
"""

from __future__ import annotations

from pathlib import Path

from pptx import Presentation
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches

from create_results_ppt import (
    AMBER,
    BG,
    BLUE,
    CYAN,
    GREEN,
    INK,
    LINE,
    MUTED,
    NAVY,
    PALE_AMBER,
    PALE_BLUE,
    PALE_GREEN,
    PALE_PURPLE,
    PALE_RED,
    PALE_TEAL,
    PURPLE,
    RED,
    SLIDE_H,
    SLIDE_W,
    TEAL,
    WHITE,
    add_badge,
    add_box,
    add_box_text,
    add_chevron,
    add_header,
    add_rich_text,
    add_text,
    rgb,
    set_bg,
    set_notes,
)


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "ppt" / "GO2W_自主语义搜索_项目架构与功能_高信息密度4页.pptx"


def add_footer(slide, text: str, page: int) -> None:
    add_text(slide, text, 0.58, 7.16, 11.48, 0.22, size=7.6, color="7C8998")
    add_text(
        slide,
        f"{page}/4",
        12.05,
        7.13,
        0.68,
        0.25,
        size=8.5,
        color=MUTED,
        align=PP_ALIGN.RIGHT,
    )


def add_number_circle(slide, number: str, x: float, y: float, color: str, size: float = 0.36) -> None:
    circle = slide.shapes.add_shape(
        MSO_SHAPE.OVAL, Inches(x), Inches(y), Inches(size), Inches(size)
    )
    circle.fill.solid()
    circle.fill.fore_color.rgb = rgb(color)
    circle.line.fill.background()
    add_text(
        slide,
        number,
        x + 0.01,
        y,
        size - 0.02,
        size - 0.02,
        size=9.2,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )


def add_mini_row(slide, title: str, body: str, x: float, y: float, w: float, accent: str) -> None:
    add_box(slide, x, y, w, 0.45, fill=WHITE, line=LINE)
    stripe = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE,
        Inches(x),
        Inches(y),
        Inches(0.07),
        Inches(0.45),
    )
    stripe.fill.solid()
    stripe.fill.fore_color.rgb = rgb(accent)
    stripe.line.fill.background()
    add_text(slide, title, x + 0.15, y + 0.04, 1.16, 0.24, size=8.8, color=accent, bold=True)
    add_text(
        slide,
        body,
        x + 1.28,
        y + 0.04,
        w - 1.39,
        0.28,
        size=7.8,
        color=INK,
        align=PP_ALIGN.RIGHT,
    )


def slide_one(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(
        slide,
        "01",
        "Go2-W 机器狗自主语义搜索与空间场景理解系统",
        "自然语言驱动 · 主动搜索 · 安全导航",
    )

    add_box(slide, 0.58, 1.18, 12.17, 0.86, fill=NAVY, line=NAVY)
    add_text(
        slide,
        "项目定位",
        0.82,
        1.32,
        1.10,
        0.34,
        size=14.2,
        color="82D8F1",
        bold=True,
        align=PP_ALIGN.CENTER,
    )
    add_text(
        slide,
        "面向真实室内环境，将自然语言目标理解、开放词汇视觉感知、空间记忆、自主探索与 Go2-W 真机运动组织为持续闭环。",
        2.05,
        1.25,
        10.30,
        0.52,
        size=15.6,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )

    add_text(slide, "典型任务闭环", 0.62, 2.24, 1.88, 0.34, size=15.5, color=NAVY, bold=True)
    stages = [
        ("自然语言任务", "类别 / 颜色 / 关系", BLUE, PALE_BLUE),
        ("目标画像", "TargetProfile + GoalGraph", CYAN, "E9F7FC"),
        ("视觉与空间感知", "检测 / 分割 / 深度 / SLAM", TEAL, PALE_TEAL),
        ("世界模型更新", "Object / Place / Frontier", PURPLE, PALE_PURPLE),
        ("Agent 选择视点", "语义 + 信息增益 + 路径", AMBER, PALE_AMBER),
        ("Nav2 安全执行", "规划 / 门控 / 反馈", RED, PALE_RED),
    ]
    for i, (title, body, accent, fill) in enumerate(stages):
        x = 0.58 + i * 2.04
        add_box(slide, x, 2.68, 1.78, 0.96, fill=fill, line=fill)
        add_text(slide, title, x + 0.08, 2.78, 1.62, 0.27, size=10.7, color=accent, bold=True, align=PP_ALIGN.CENTER)
        add_text(slide, body, x + 0.09, 3.08, 1.60, 0.37, size=7.9, color=INK, align=PP_ALIGN.CENTER)
        if i < len(stages) - 1:
            add_chevron(slide, x + 1.84, 2.98, "9DB0C5")
    add_text(slide, "运动后重新获取图像、位姿和地图，回到下一轮观察", 7.52, 3.73, 5.03, 0.27, size=8.5, color=MUTED, bold=True, align=PP_ALIGN.RIGHT)

    add_text(slide, "输入—处理—输出", 0.62, 4.16, 2.10, 0.34, size=15.5, color=NAVY, bold=True)
    column_specs = [
        (
            "输入",
            "自然语言任务\nRGB-D 图像与深度\n激光雷达、IMU、里程计\n历史观测与地图状态",
            BLUE,
            PALE_BLUE,
        ),
        (
            "系统处理",
            "任务、属性、关系解析\n开放词汇检测与视觉复核\n三维定位、SLAM 与对象关联\n候选生成、评分和路径预检",
            PURPLE,
            PALE_PURPLE,
        ),
        (
            "输出",
            "结构化目标画像与目标状态\n二维/三维地图与语义拓扑\n下一观察位置及选择依据\n导航结果、事件证据与回放",
            TEAL,
            PALE_TEAL,
        ),
    ]
    for i, (title, body, accent, fill) in enumerate(column_specs):
        x = 0.58 + i * 4.08
        add_box_text(
            slide,
            title,
            body,
            x,
            4.58,
            3.82,
            1.42,
            fill=fill,
            line=fill,
            accent=accent,
            title_size=13.2,
            body_size=9.4,
        )

    add_text(slide, "核心功能", 0.62, 6.20, 1.26, 0.31, size=14.4, color=NAVY, bold=True)
    features = [
        ("自然语言搜索", BLUE, PALE_BLUE),
        ("开放词汇感知", TEAL, PALE_TEAL),
        ("RGB-D 三维定位", PURPLE, PALE_PURPLE),
        ("语义拓扑记忆", PURPLE, "EEE9FA"),
        ("Frontier 主动探索", AMBER, PALE_AMBER),
        ("Nav2 安全导航", RED, PALE_RED),
        ("WebUI 与回放", GREEN, PALE_GREEN),
    ]
    for i, (label, accent, fill) in enumerate(features):
        add_badge(slide, label, 1.86 + i * 1.52, 6.18, 1.37, accent, fill)

    add_box(slide, 0.58, 6.68, 12.17, 0.31, fill=NAVY, line=NAVY)
    add_text(
        slide,
        "核心价值：机器人在当前视野看不到目标时，仍能利用语义、空间和历史信息主动寻找下一观察位置。",
        0.82,
        6.69,
        11.70,
        0.25,
        size=10.3,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )
    add_footer(slide, "示例任务：寻找办公椅；寻找饮水机旁边的蓝色垃圾桶。", 1)
    set_notes(
        slide,
        "第一页介绍项目定位与功能闭环。输入不仅是图像，还包括自然语言、深度、激光地图、机器人位姿和历史观测。系统输出也不只是检测框，而是目标状态、空间位置、下一观察目标、运动结果和完整证据。项目核心是目标不可见时的主动搜索能力。",
    )


def slide_two(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "02", "系统总体架构：从多模态感知到真机运动", "五层闭环架构与职责边界")

    layers = [
        (
            "01 人机交互与任务层",
            ["自然语言任务 / WebUI", "搜索状态与任务控制", "地图、拓扑与历史回放"],
            BLUE,
            PALE_BLUE,
        ),
        (
            "02 搜索编排与认知层",
            ["SearchSessionService", "Autonomous Search Worker", "TargetProfile / GoalGraph / Agent"],
            CYAN,
            "E9F7FC",
        ),
        (
            "03 视觉感知 + 几何建图层",
            ["RAM++ / DINO / SAM2 / VLM", "D435 RGB-D 三维定位", "Pandar / IMU / Odom / plain_slam"],
            TEAL,
            PALE_TEAL,
        ),
        (
            "04 世界模型与规划层",
            ["SemanticObjectMap / PlaceGraph", "Frontier / SemanticNavigationGraph", "候选生成 / 视点评分 / 拓扑路由"],
            PURPLE,
            PALE_PURPLE,
        ),
        (
            "05 导航与安全执行层",
            ["Nav2SearchBackend", "路径 / 净空 / Collision Monitor", "仲裁 / cmd_vel Bridge / Motion Action"],
            AMBER,
            PALE_AMBER,
        ),
    ]
    for i, (heading, cards, accent, fill) in enumerate(layers):
        y = 1.18 + i * 1.06
        add_box(slide, 0.58, y, 9.23, 0.88, fill=fill, line=fill)
        add_text(slide, heading, 0.82, y + 0.14, 2.46, 0.36, size=12.8, color=accent, bold=True)
        for j, card in enumerate(cards):
            x = 3.22 + j * 2.11
            add_box(slide, x, y + 0.12, 1.91, 0.61, fill=WHITE, line=WHITE)
            add_text(slide, card, x + 0.08, y + 0.18, 1.75, 0.44, size=8.5, color=INK, bold=True, align=PP_ALIGN.CENTER)
        if i < len(layers) - 1:
            add_text(slide, "↓", 4.92, y + 0.83, 0.45, 0.27, size=13, color="8FA4B9", bold=True, align=PP_ALIGN.CENTER)

    add_box(slide, 10.05, 1.18, 2.70, 5.18, fill=WHITE, line=LINE)
    add_text(slide, "架构设计原则", 10.30, 1.39, 2.20, 0.37, size=15.2, color=NAVY, bold=True, align=PP_ALIGN.CENTER)
    principles = [
        ("语义与控制解耦", "Agent 决定去哪里；安全链决定能否执行。", BLUE, PALE_BLUE),
        ("地图职责分离", "三维点云表达空间，二维栅格供 Nav2，语义图供任务推理。", TEAL, PALE_TEAL),
        ("快慢感知分离", "目标快通路保障节奏，完整场景语义异步更新。", PURPLE, PALE_PURPLE),
        ("版本严格绑定", "frame_id、map_epoch、位姿和时间戳阻止旧数据误用。", AMBER, PALE_AMBER),
        ("全过程可审计", "观测、候选、评分、拒绝原因和执行结果全部留档。", RED, PALE_RED),
    ]
    for i, (title, body, accent, fill) in enumerate(principles):
        add_box_text(
            slide,
            title,
            body,
            10.26,
            1.90 + i * 0.86,
            2.28,
            0.70,
            fill=fill,
            line=fill,
            accent=accent,
            title_size=10.2,
            body_size=7.5,
        )

    add_box(slide, 0.58, 6.58, 12.17, 0.40, fill=NAVY, line=NAVY)
    add_text(
        slide,
        "数据流向下完成任务执行；机器人位姿、图像、地图、目标证据和运动结果再向上反馈，驱动下一轮观察与重规划。",
        0.84,
        6.62,
        11.65,
        0.28,
        size=10.6,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )
    add_footer(slide, "主入口：SearchSessionService → autonomous_search_worker → run_semantic_exploration → AutonomousExplorer。", 2)
    set_notes(
        slide,
        "总体架构分为五层。最重要的边界有三个：第一，任务 Agent 不直接发速度；第二，三维点云、二维导航地图和对象语义图各有职责；第三，任何高层目标都必须通过 Nav2 路径规划和实时安全检查。所有结果形成反馈，再进入下一轮感知和决策。",
    )


def slide_three(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "03", "关键模块：语义感知与持续空间世界模型", "目标画像 · 双通路感知 · 三维定位 · 语义导航图")

    # Column 1: task and perception.
    add_box(slide, 0.58, 1.18, 4.02, 5.80, fill=WHITE, line=LINE)
    add_text(slide, "A｜目标理解与双通路感知", 0.84, 1.38, 3.48, 0.36, size=15.0, color=BLUE, bold=True)
    add_box(slide, 0.84, 1.88, 3.50, 0.82, fill=PALE_BLUE, line=PALE_BLUE)
    add_text(slide, "示例：寻找饮水机旁边的蓝色垃圾桶", 0.98, 1.98, 3.22, 0.29, size=10.4, color=INK, bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, "目标=垃圾桶｜属性=蓝色｜关系=NEAR｜锚点=饮水机", 0.98, 2.29, 3.22, 0.24, size=7.8, color=BLUE, bold=True, align=PP_ALIGN.CENTER)

    add_text(slide, "目标快速通路", 0.89, 2.89, 1.30, 0.30, size=12.1, color=TEAL, bold=True)
    fast = ["TargetProfile 任务词", "GroundingDINO / Quick VLM", "PRESENT / POSSIBLE / ABSENT / UNKNOWN"]
    for i, item in enumerate(fast):
        y = 3.27 + i * 0.50
        add_box(slide, 0.92, y, 3.22, 0.36, fill=PALE_TEAL, line=PALE_TEAL)
        add_text(slide, item, 1.02, y + 0.03, 3.02, 0.28, size=8.6, color=INK, bold=i == 2, align=PP_ALIGN.CENTER)
        if i < 2:
            add_text(slide, "↓", 2.35, y + 0.33, 0.36, 0.20, size=9, color="9AAABB", bold=True, align=PP_ALIGN.CENTER)

    add_text(slide, "完整场景通路", 0.89, 4.86, 1.30, 0.30, size=12.1, color=PURPLE, bold=True)
    full = [
        ("RAM++", "扩充普通场景词"),
        ("DINO + SAM2", "开放词汇框与实例 Mask"),
        ("D435 + Full VLM", "三维位置、属性和复杂关系"),
    ]
    for i, (title, body) in enumerate(full):
        add_mini_row(slide, title, body, 0.90, 5.24 + i * 0.50, 3.25, PURPLE if i else CYAN)
    add_box(slide, 0.90, 6.73, 3.25, 0.17, fill=PALE_RED, line=PALE_RED)
    add_text(slide, "UNKNOWN ≠ ABSENT：超时或失败不能写成目标不存在", 0.94, 6.61, 3.17, 0.32, size=7.5, color=RED, bold=True, align=PP_ALIGN.CENTER)

    # Column 2: localization and object memory.
    add_box(slide, 4.82, 1.18, 3.74, 5.80, fill=WHITE, line=LINE)
    add_text(slide, "B｜空间化与跨帧融合", 5.08, 1.38, 3.20, 0.36, size=15.0, color=TEAL, bold=True)
    add_text(slide, "RGB-D 三维反投影", 5.08, 1.89, 2.20, 0.29, size=11.8, color=INK, bold=True)
    add_box(slide, 5.08, 2.24, 3.22, 0.68, fill=NAVY, line=NAVY)
    add_rich_text(
        slide,
        [
            ("Z=d    ", {"size": 13.2, "bold": True, "color": "8EDAF2"}),
            ("X=(u−cx)Z/fx    Y=(v−cy)Z/fy", {"size": 10.3, "bold": True, "color": WHITE}),
        ],
        5.24,
        2.39,
        2.90,
        0.35,
        align=PP_ALIGN.CENTER,
    )
    add_box(slide, 5.08, 3.06, 3.22, 0.54, fill=PALE_TEAL, line=PALE_TEAL)
    add_text(slide, "camera → base_link → map", 5.22, 3.16, 2.94, 0.29, size=11.1, color=TEAL, bold=True, align=PP_ALIGN.CENTER)

    add_text(slide, "证据身份绑定", 5.08, 3.84, 1.70, 0.30, size=11.8, color=INK, bold=True)
    evidence = ["frame_id / capture_timestamp", "robot_pose / transform_source", "semantic_age_ms / spatial_quality", "source：GSAM / Quick VLM / Full VLM"]
    for i, item in enumerate(evidence):
        add_box(slide, 5.08, 4.22 + i * 0.40, 3.22, 0.29, fill="F1F5F9", line="F1F5F9")
        add_text(slide, item, 5.20, 4.24 + i * 0.40, 2.98, 0.23, size=8.1, color=INK, bold=i == 3, align=PP_ALIGN.CENTER)

    add_text(slide, "SemanticObjectMap 持久对象", 5.08, 5.94, 2.78, 0.30, size=11.8, color=PURPLE, bold=True)
    add_text(
        slide,
        "依据类别、属性、三维距离、位姿变化与来源检测词进行跨帧关联；保存持久 ID、位置、出现次数、最后观测时间和目标状态，并过滤动态物体与低质量观测。",
        5.08,
        6.27,
        3.22,
        0.54,
        size=8.4,
        color=MUTED,
        valign=MSO_ANCHOR.TOP,
    )

    # Column 3: graph model.
    add_box(slide, 8.78, 1.18, 3.97, 5.80, fill=WHITE, line=LINE)
    add_text(slide, "C｜Place–Object–Frontier 图", 9.04, 1.38, 3.46, 0.36, size=15.0, color=PURPLE, bold=True)
    add_text(slide, "语义导航图的核心节点", 9.05, 1.89, 2.46, 0.29, size=11.8, color=INK, bold=True)
    nodes = [
        ("PLACE", "稳定地点、位姿、已观察方向与访问历史", BLUE, PALE_BLUE),
        ("OBJECT", "持久物体、属性、三维位置与目标状态", TEAL, PALE_TEAL),
        ("FRONTIER", "Free–Unknown 边界、信息增益与可达性", AMBER, PALE_AMBER),
    ]
    for i, (title, body, accent, fill) in enumerate(nodes):
        add_box_text(slide, title, body, 9.04, 2.27 + i * 0.76, 3.44, 0.61, fill=fill, line=fill, accent=accent, title_size=9.7, body_size=7.4)

    add_text(slide, "关系与决策含义", 9.05, 4.67, 2.20, 0.29, size=11.8, color=INK, bold=True)
    relations = [
        ("CONNECTED_TO", "Place 间可移动连接"),
        ("OBSERVED_FROM", "物体曾从哪个 Place 看到"),
        ("FRONTIER_TO", "从当前地点可继续探索"),
        ("空间语义关系", "NEAR / LEFT_OF / INSIDE"),
    ]
    for i, (title, body) in enumerate(relations):
        add_mini_row(slide, title, body, 9.04, 5.05 + i * 0.43, 3.44, PURPLE)

    add_box(slide, 9.04, 6.75, 3.44, 0.15, fill=PALE_PURPLE, line=PALE_PURPLE)
    add_text(slide, "frame_id + map_epoch + revision：拒绝旧地图、旧对象和错坐标融合", 9.10, 6.61, 3.30, 0.31, size=7.4, color=PURPLE, bold=True, align=PP_ALIGN.CENTER)

    add_footer(slide, "输出从“单帧检测框”升级为“对象—位置—关系—来源—时效—访问历史”的持续世界模型。", 3)
    set_notes(
        slide,
        "第三页分三块。左侧是任务画像和快慢视觉通路；中间将二维检测通过深度和 TF 变成地图坐标，并绑定来源帧、位姿和时间；右侧把结果写入 Place、Object 和 Frontier 组成的语义导航图。系统尤其区分 PRESENT、POSSIBLE、ABSENT 和 UNKNOWN，避免模型超时被误认为目标不存在。",
    )


def slide_four(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "04", "自主搜索 Agent、安全执行与系统展示", "状态机 · 候选评分 · Nav2 门控 · WebUI 与验证边界")

    # Column 1: state machine.
    add_box(slide, 0.58, 1.18, 3.50, 5.80, fill=WHITE, line=LINE)
    add_text(slide, "A｜AutonomousExplorer 状态机", 0.83, 1.38, 2.99, 0.36, size=14.4, color=BLUE, bold=True)
    states = [
        ("BOOTSTRAP", "检查后端、地图、定位与传感器", BLUE),
        ("OBSERVE", "采集最新 RGB-D、位姿和地图", TEAL),
        ("MATCH / VERIFY", "目标图匹配与候选视觉复核", PURPLE),
        ("UPDATE MEMORY", "更新对象、Place、关系与负证据", AMBER),
        ("PLAN", "生成候选并选择下一观察位置", BLUE),
        ("EXECUTE", "Nav2 执行、等待结果并回写", RED),
        ("REOBSERVE", "停稳后重新感知和重规划", GREEN),
    ]
    for i, (title, body, accent) in enumerate(states):
        y = 1.94 + i * 0.63
        add_box(slide, 0.88, y, 2.91, 0.45, fill="F5F8FB", line="E8EDF3")
        add_number_circle(slide, str(i + 1), 0.99, y + 0.05, accent, 0.33)
        add_text(slide, title, 1.43, y + 0.04, 1.08, 0.24, size=8.8, color=accent, bold=True)
        add_text(slide, body, 2.26, y + 0.04, 1.39, 0.29, size=7.2, color=INK, align=PP_ALIGN.RIGHT)
        if i < len(states) - 1:
            add_text(slide, "↓", 2.15, y + 0.41, 0.34, 0.21, size=8.5, color="9AAABB", bold=True, align=PP_ALIGN.CENTER)
    add_box(slide, 0.88, 6.51, 2.91, 0.28, fill=NAVY, line=NAVY)
    add_text(slide, "FOUND / EXHAUSTED / FAILED / STOP", 1.00, 6.53, 2.67, 0.22, size=7.6, color=WHITE, bold=True, align=PP_ALIGN.CENTER)

    # Column 2: planning and safety.
    add_box(slide, 4.30, 1.18, 4.18, 5.80, fill=WHITE, line=LINE)
    add_text(slide, "B｜候选选择与 fail-closed 执行", 4.56, 1.38, 3.65, 0.36, size=14.4, color=PURPLE, bold=True)
    add_text(slide, "四类长期意图", 4.58, 1.89, 1.50, 0.29, size=11.7, color=INK, bold=True)
    intents = [
        ("EXPLORE_FRONTIER", "探索未知区域", BLUE, PALE_BLUE),
        ("INSPECT_ANCHOR", "检查关系锚点", PURPLE, PALE_PURPLE),
        ("VERIFY_TARGET", "换视角复核候选", AMBER, PALE_AMBER),
        ("APPROACH_TARGET", "到安全观察距离", TEAL, PALE_TEAL),
    ]
    for i, (title, body, accent, fill) in enumerate(intents):
        x = 4.58 + (i % 2) * 1.76
        y = 2.26 + (i // 2) * 0.66
        add_box(slide, x, y, 1.58, 0.52, fill=fill, line=fill)
        add_text(slide, title, x + 0.06, y + 0.05, 1.46, 0.21, size=7.5, color=accent, bold=True, align=PP_ALIGN.CENTER)
        add_text(slide, body, x + 0.07, y + 0.27, 1.44, 0.18, size=6.9, color=INK, align=PP_ALIGN.CENTER)

    add_text(slide, "候选联合评分", 4.58, 3.68, 1.50, 0.29, size=11.7, color=INK, bold=True)
    add_box(slide, 4.58, 4.02, 3.62, 0.76, fill=NAVY, line=NAVY)
    add_text(slide, "语义相关性 + 信息增益 + 新颖性 + 历史先验", 4.74, 4.12, 3.30, 0.25, size=9.0, color="9DE3F2", bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, "− 路径代价 − 重复访问 − 导航失败 − 有效负证据", 4.74, 4.39, 3.30, 0.25, size=8.4, color="F6C779", bold=True, align=PP_ALIGN.CENTER)

    add_text(slide, "执行前安全链", 4.58, 5.03, 1.50, 0.29, size=11.7, color=INK, bold=True)
    gates = [
        ("路径预检", "ComputePathToPose / Spin 分流", BLUE),
        ("几何安全", "终点占用、走廊净空、转向扫掠", PURPLE),
        ("实时健康", "地图 / TF / 定位 / LiDAR 新鲜", TEAL),
        ("控制安全", "授权、租约、仲裁、急停、看门狗", RED),
    ]
    for i, (title, body, accent) in enumerate(gates):
        add_mini_row(slide, title, body, 4.58, 5.39 + i * 0.38, 3.62, accent)
    add_text(slide, "默认只规划；所有门通过后才允许物理执行", 4.69, 6.78, 3.40, 0.17, size=7.5, color=RED, bold=True, align=PP_ALIGN.CENTER)

    # Column 3: UI and completion status.
    add_box(slide, 8.70, 1.18, 4.05, 5.80, fill=WHITE, line=LINE)
    add_text(slide, "C｜WebUI、回放与当前完成度", 8.96, 1.38, 3.53, 0.36, size=14.4, color=AMBER, bold=True)
    add_text(slide, "WebUI 集成展示", 8.97, 1.89, 1.70, 0.29, size=11.7, color=INK, bold=True)
    ui_items = [
        "自然语言任务与搜索阶段",
        "RGB / 深度画面与目标证据",
        "三维点云、机器人位姿与语义对象",
        "物体拓扑、Frontier、候选和选择原因",
        "地图、定位、雷达与运动授权状态",
        "暂停 / 恢复 / 停止 / 急停 / 历史回放",
    ]
    for i, item in enumerate(ui_items):
        add_box(slide, 8.98, 2.25 + i * 0.37, 3.49, 0.27, fill="F2F6FA", line="F2F6FA")
        add_text(slide, "• " + item, 9.09, 2.26 + i * 0.37, 3.28, 0.22, size=7.8, color=INK, bold=i in {2, 3})

    add_text(slide, "项目完成情况", 8.97, 4.63, 1.70, 0.29, size=11.7, color=INK, bold=True)
    add_box_text(
        slide,
        "已实现",
        "任务画像、开放词汇感知、RGB-D 定位、二维/三维地图、语义对象图、Frontier 决策、Nav2 接口、安全门控、WebUI 与回放。",
        8.98,
        4.98,
        3.49,
        0.84,
        fill=PALE_GREEN,
        line=PALE_GREEN,
        accent=GREEN,
        title_size=10.0,
        body_size=7.3,
    )
    add_box_text(
        slide,
        "已验证 / 待继续",
        "已有离线回归与部分短距真机 Nav2 证据；连续多个 Frontier、真实目标接近与复杂关系任务仍需完整现场验收。",
        8.98,
        5.95,
        3.49,
        0.82,
        fill=PALE_AMBER,
        line=PALE_AMBER,
        accent=AMBER,
        title_size=10.0,
        body_size=7.2,
    )

    add_footer(slide, "系统强调：每个运动目标都有语义依据、地图依据、安全依据和可回放证据。", 4)
    set_notes(
        slide,
        "最后一页说明搜索如何真正运行。状态机持续执行观察、匹配、复核、更新记忆、规划和导航；候选并不是简单选择最近点，而是联合考虑语义相关性、信息增益、路径、访问历史和负证据。任何目标都必须通过 Nav2 与实时安全链。右侧展示 WebUI 能力，并明确当前已有短距真机和离线验证，但连续多前沿和完整目标接近仍属于后续现场验收内容。",
    )


def validate_bounds(prs: Presentation) -> None:
    width = int(prs.slide_width)
    height = int(prs.slide_height)
    errors: list[str] = []
    for slide_index, slide in enumerate(prs.slides, start=1):
        for shape_index, shape in enumerate(slide.shapes, start=1):
            if shape.left < 0 or shape.top < 0:
                errors.append(f"slide {slide_index} shape {shape_index}: negative origin")
            if shape.left + shape.width > width + 2 or shape.top + shape.height > height + 2:
                errors.append(f"slide {slide_index} shape {shape_index}: outside slide bounds")
    if errors:
        raise RuntimeError("\n".join(errors))


def validate_deck(prs: Presentation) -> None:
    if len(prs.slides) != 4:
        raise RuntimeError(f"expected 4 slides, got {len(prs.slides)}")
    validate_bounds(prs)
    for index, slide in enumerate(prs.slides, start=1):
        text = "\n".join(
            shape.text for shape in slide.shapes if getattr(shape, "has_text_frame", False)
        )
        if len(text.strip()) < 250:
            raise RuntimeError(f"slide {index} is unexpectedly sparse")


def build() -> Path:
    prs = Presentation()
    prs.slide_width = Inches(SLIDE_W)
    prs.slide_height = Inches(SLIDE_H)
    prs.core_properties.title = "GO2W 自主语义搜索项目架构与功能"
    prs.core_properties.subject = "四页高信息密度项目汇报"
    prs.core_properties.author = "robotscene project"
    prs.core_properties.comments = "依据当前仓库源码、配置和验收记录生成。"
    slide_one(prs)
    slide_two(prs)
    slide_three(prs)
    slide_four(prs)
    validate_deck(prs)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    prs.save(OUT)
    return OUT


if __name__ == "__main__":
    print(build())
