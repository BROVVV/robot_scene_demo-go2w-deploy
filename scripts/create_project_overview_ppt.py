#!/usr/bin/env python3
"""Generate a five-slide GO2W architecture and capabilities presentation."""

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
    add_picture_crop,
    add_rich_text,
    add_text,
    rgb,
    set_bg,
    set_notes,
)


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "ppt" / "GO2W_自主语义搜索_项目架构与功能_5页.pptx"

PHOTO_ROBOT = ROOT / "reports" / "image.png"
PHOTO_CAMERA = ROOT / "artifacts" / "perception_profile_20261013" / "frames" / "color_5465.jpg"
TOPOLOGY = ROOT / "outputs" / "topology_graph.png"
WEBUI = (
    ROOT
    / "outputs"
    / "live_runs"
    / "search_20261023_160331_54e4f466"
    / "replay"
    / "artifacts"
    / "cover.jpg"
)


def add_footer(slide, text: str, page: int) -> None:
    add_text(slide, text, 0.58, 7.17, 11.45, 0.22, size=7.6, color="7C8998")
    add_text(
        slide,
        f"{page}/5",
        12.05,
        7.14,
        0.68,
        0.25,
        size=8.5,
        color=MUTED,
        align=PP_ALIGN.RIGHT,
    )


def add_number_circle(slide, number: str, x: float, y: float, color: str, size: float = 0.42) -> None:
    circ = slide.shapes.add_shape(
        MSO_SHAPE.OVAL, Inches(x), Inches(y), Inches(size), Inches(size)
    )
    circ.fill.solid()
    circ.fill.fore_color.rgb = rgb(color)
    circ.line.fill.background()
    add_text(
        slide,
        number,
        x + 0.01,
        y + 0.01,
        size - 0.02,
        size - 0.04,
        size=10,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )


def slide_one(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "01", "面向 Go2-W 的自主语义搜索系统", "项目背景、目标与核心能力")

    add_picture_crop(slide, PHOTO_ROBOT, 0.55, 1.18, 5.05, 5.78)
    overlay = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE,
        Inches(0.55),
        Inches(5.46),
        Inches(5.05),
        Inches(1.50),
    )
    overlay.fill.solid()
    overlay.fill.fore_color.rgb = rgb(NAVY)
    overlay.fill.transparency = 5
    overlay.line.fill.background()
    add_text(slide, "具身智能实验平台", 0.84, 5.60, 2.0, 0.30, size=11.0, color="8EDAF2", bold=True)
    add_text(
        slide,
        "Unitree Go2-W + D435 RGB-D\n+ 原装 LiDAR + PandarXT-16",
        0.84,
        5.92,
        4.35,
        0.76,
        size=17.0,
        color=WHITE,
        bold=True,
        valign=MSO_ANCHOR.TOP,
    )

    add_text(slide, "让机器狗从“看见物体”升级为", 6.00, 1.23, 6.05, 0.44, size=18.5, color=MUTED)
    add_text(
        slide,
        "理解任务、建立世界模型、主动寻找并安全行动",
        6.00,
        1.62,
        6.35,
        0.66,
        size=25.0,
        color=NAVY,
        bold=True,
        valign=MSO_ANCHOR.TOP,
    )

    challenges = [
        ("开放世界目标", "目标不在当前视野；类别、颜色和关系由用户自然语言给出。", BLUE, PALE_BLUE),
        ("空间理解", "系统需要知道物体在哪里、从哪里观察、路径是否可达。", PURPLE, PALE_PURPLE),
        ("真实安全执行", "每个动作必须经过定位、净空、碰撞监控与授权门控。", AMBER, PALE_AMBER),
    ]
    for i, (title, body, accent, fill) in enumerate(challenges):
        add_box_text(
            slide,
            title,
            body,
            6.02 + i * 2.14,
            2.55,
            1.93,
            1.48,
            fill=fill,
            line=fill,
            accent=accent,
            title_size=13.0,
            body_size=9.2,
        )

    add_text(slide, "系统闭环", 6.02, 4.28, 1.25, 0.34, size=14.5, color=INK, bold=True)
    stages = [
        ("任务理解", BLUE, PALE_BLUE),
        ("视觉感知", TEAL, PALE_TEAL),
        ("地图与记忆", PURPLE, PALE_PURPLE),
        ("Agent 决策", AMBER, PALE_AMBER),
        ("导航执行", RED, PALE_RED),
    ]
    for i, (name, accent, fill) in enumerate(stages):
        x = 6.02 + i * 1.27
        add_box(slide, x, 4.75, 1.07, 0.73, fill=fill, line=fill)
        add_text(slide, name, x + 0.06, 4.88, 0.95, 0.40, size=10.4, color=accent, bold=True, align=PP_ALIGN.CENTER)
        if i < len(stages) - 1:
            add_chevron(slide, x + 1.10, 4.94, "A3B3C4")

    add_box(slide, 6.02, 5.78, 6.35, 0.89, fill=NAVY, line=NAVY)
    add_text(
        slide,
        "项目定位：自然语言驱动的开放词汇目标搜索与语义导航系统",
        6.28,
        5.93,
        5.82,
        0.42,
        size=15.0,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )
    add_footer(slide, "核心任务示例：寻找办公椅；寻找饮水机旁边的蓝色垃圾桶。", 1)
    set_notes(
        slide,
        "本项目不是把目标检测模型简单部署到机器狗上，而是建立从自然语言任务、视觉感知、空间世界模型、主动决策到真实运动的闭环。机器人即使当前看不到目标，也会根据地图、语义关系和历史观测选择下一观察位置。",
    )


def slide_two(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "02", "分层式系统总体架构", "任务与交互 · 传感器与地图 · 世界模型与决策 · 导航执行")

    layer_specs = [
        (
            "01 任务与交互层",
            "WebUI / FastAPI",
            "自然语言任务解析",
            "TargetProfile + Capability Gate",
            BLUE,
            PALE_BLUE,
        ),
        (
            "02 传感器与地图层",
            "D435 RGB-D",
            "二维 SLAM / AMCL / TF",
            "PandarXT-16 + plain_slam 三维点云",
            TEAL,
            PALE_TEAL,
        ),
        (
            "03 世界模型与决策层",
            "SemanticObjectMap / PlaceGraph",
            "SemanticNavigationGraph / Memory",
            "AutonomousExplorer + Task Agent",
            PURPLE,
            PALE_PURPLE,
        ),
        (
            "04 导航、安全与执行层",
            "Nav2SearchBackend",
            "路径 / 走廊 / 转向扫掠检查",
            "Collision Monitor / 仲裁 / Motion Action",
            AMBER,
            PALE_AMBER,
        ),
    ]
    for i, (heading, c1, c2, c3, accent, fill) in enumerate(layer_specs):
        y = 1.20 + i * 1.36
        add_box(slide, 0.58, y, 9.00, 1.12, fill=fill, line=fill)
        add_text(slide, heading, 0.83, y + 0.13, 2.35, 0.35, size=14.0, color=accent, bold=True)
        cards = [c1, c2, c3]
        for j, text_value in enumerate(cards):
            x = 3.15 + j * 2.02
            add_box(slide, x, y + 0.15, 1.82, 0.78, fill=WHITE, line=WHITE)
            add_text(
                slide,
                text_value,
                x + 0.09,
                y + 0.23,
                1.64,
                0.56,
                size=9.8,
                color=INK,
                bold=True,
                align=PP_ALIGN.CENTER,
            )
        if i < len(layer_specs) - 1:
            add_text(slide, "↓", 4.83, y + 1.08, 0.48, 0.32, size=17, color="97A9BB", bold=True, align=PP_ALIGN.CENTER)

    add_box(slide, 9.84, 1.20, 2.91, 5.20, fill=WHITE, line=LINE)
    add_text(slide, "推荐两机部署", 10.10, 1.40, 2.36, 0.40, size=16.0, color=NAVY, bold=True, align=PP_ALIGN.CENTER)
    add_box_text(
        slide,
        "Ubuntu 控制主机",
        "视觉模型\n二维 / 三维建图\nNav2 与任务 Agent\nFastAPI WebUI",
        10.14,
        1.98,
        2.31,
        1.63,
        fill=PALE_BLUE,
        line=PALE_BLUE,
        accent=BLUE,
        title_size=13.0,
        body_size=10.0,
        align=PP_ALIGN.CENTER,
    )
    add_text(slide, "ROS 2 DDS / 局域网", 10.23, 3.76, 2.11, 0.32, size=10.2, color=MUTED, bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, "↕", 11.02, 4.00, 0.50, 0.35, size=18, color=CYAN, bold=True, align=PP_ALIGN.CENTER)
    add_box_text(
        slide,
        "Go2-W 机器狗",
        "相机与机器人状态\n底层运动执行\n遥控与实体急停",
        10.14,
        4.40,
        2.31,
        1.47,
        fill=PALE_TEAL,
        line=PALE_TEAL,
        accent=TEAL,
        title_size=13.0,
        body_size=10.0,
        align=PP_ALIGN.CENTER,
    )

    add_box(slide, 0.58, 6.66, 12.17, 0.34, fill=NAVY, line=NAVY)
    add_text(
        slide,
        "二维地图负责 Nav2 导航｜三维点云负责空间展示｜语义拓扑负责任务推理",
        0.90,
        6.67,
        11.52,
        0.28,
        size=11.4,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )
    add_footer(slide, "三条地图链职责分离，并通过 TF、机器人位姿、采集时刻和关键帧建立关联。", 2)
    set_notes(
        slide,
        "总体架构分为四层。任务与交互层把自然语言变成结构化搜索目标；传感器与地图层分别维护二维导航地图、三维点云和 RGB-D 观测；世界模型层组织地点、物体、关系和前沿；导航层只在路径、安全和授权检查通过后执行动作。特别要说明，plain_slam 三维点云不直接替代二维 Nav2 地图。",
    )


def slide_three(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "03", "从图像识别到可搜索的三维语义世界", "目标画像 × 开放词汇感知 × RGB-D 定位 × 语义拓扑")

    pipeline = [
        ("TargetProfile", "类别 / 颜色 / 属性 / 关系", BLUE, PALE_BLUE),
        ("RAM++", "补充普通场景类别", CYAN, "E9F7FC"),
        ("GroundingDINO", "开放词汇目标定位", TEAL, PALE_TEAL),
        ("SAM2", "实例 Mask", PURPLE, PALE_PURPLE),
        ("D435 + VLM", "三维定位与复杂复核", AMBER, PALE_AMBER),
    ]
    for i, (title, body, accent, fill) in enumerate(pipeline):
        x = 0.58 + i * 2.50
        add_box(slide, x, 1.18, 2.14, 0.86, fill=fill, line=fill)
        add_text(slide, title, x + 0.08, 1.25, 1.98, 0.28, size=11.8, color=accent, bold=True, align=PP_ALIGN.CENTER)
        add_text(slide, body, x + 0.08, 1.55, 1.98, 0.29, size=9.2, color=INK, bold=True, align=PP_ALIGN.CENTER)
        if i < len(pipeline) - 1:
            add_chevron(slide, x + 2.18, 1.44, "9DB0C5")

    add_box(slide, 0.58, 2.28, 3.67, 3.96, fill=WHITE, line=LINE)
    add_picture_crop(slide, PHOTO_CAMERA, 0.76, 2.47, 3.31, 2.20)
    add_badge(slide, "真实 RGB-D 画面", 0.94, 2.65, 1.42, CYAN, "E9F7FC")
    add_text(slide, "二维识别结果", 0.86, 4.86, 1.35, 0.32, size=13.2, color=INK, bold=True)
    add_text(slide, "检测框 / Mask / 类别 / 置信度", 0.86, 5.18, 2.92, 0.30, size=10.0, color=MUTED)
    add_box(slide, 0.85, 5.61, 3.11, 0.42, fill=PALE_TEAL, line=PALE_TEAL)
    add_text(slide, "用户目标不会被 RAM++ 漏词卡住", 0.95, 5.66, 2.91, 0.31, size=9.7, color=TEAL, bold=True, align=PP_ALIGN.CENTER)

    add_box(slide, 4.48, 2.28, 3.78, 3.96, fill=WHITE, line=LINE)
    add_text(slide, "三维定位与证据绑定", 4.75, 2.48, 2.72, 0.36, size=15.0, color=PURPLE, bold=True)
    transform_steps = [
        ("1", "深度 + 相机内参", "得到物体相机坐标", BLUE),
        ("2", "相机外参", "转换到机器人坐标", TEAL),
        ("3", "采集时刻 TF", "转换到地图坐标", PURPLE),
        ("4", "质量与时间戳", "拒绝陈旧或错误坐标", AMBER),
    ]
    for i, (num, title, body, accent) in enumerate(transform_steps):
        y = 3.02 + i * 0.70
        add_number_circle(slide, num, 4.79, y, accent, 0.38)
        add_text(slide, title, 5.30, y - 0.01, 1.28, 0.28, size=10.8, color=INK, bold=True)
        add_text(slide, body, 6.62, y - 0.01, 1.25, 0.28, size=8.7, color=MUTED, align=PP_ALIGN.RIGHT)
    add_box(slide, 4.77, 5.85, 3.21, 0.22, fill=PALE_PURPLE, line=PALE_PURPLE)
    add_text(slide, "camera → base_link → map", 5.05, 5.76, 2.67, 0.38, size=10.8, color=PURPLE, bold=True, align=PP_ALIGN.CENTER)

    add_box(slide, 8.49, 2.28, 4.26, 3.96, fill=WHITE, line=LINE)
    add_picture_crop(slide, TOPOLOGY, 8.70, 2.47, 3.84, 2.48)
    add_text(slide, "SemanticNavigationGraph", 8.76, 5.06, 3.73, 0.34, size=14.3, color=PURPLE, bold=True, align=PP_ALIGN.CENTER)
    legends = [
        ("PLACE", BLUE, PALE_BLUE),
        ("OBJECT", TEAL, PALE_TEAL),
        ("FRONTIER", AMBER, PALE_AMBER),
    ]
    for i, (label, accent, fill) in enumerate(legends):
        add_badge(slide, label, 8.76 + i * 1.22, 5.49, 1.05, accent, fill)

    feature_cards = [
        ("跨帧身份", "同一物体持续关联", BLUE, PALE_BLUE),
        ("语义关系", "NEAR / ON / UNDER", PURPLE, PALE_PURPLE),
        ("搜索记忆", "访问、失败与负证据", AMBER, PALE_AMBER),
        ("动态过滤", "移动对象不污染长期拓扑", TEAL, PALE_TEAL),
    ]
    for i, (title, body, accent, fill) in enumerate(feature_cards):
        x = 0.58 + i * 3.05
        add_box_text(slide, title, body, x, 6.44, 2.82, 0.55, fill=fill, line=fill, accent=accent, title_size=10.2, body_size=7.6)

    add_footer(slide, "输出不再只是检测框，而是“物体—位置—关系—来源—时效”的持续世界模型。", 3)
    set_notes(
        slide,
        "视觉链路先由 TargetProfile 保留目标类别、颜色、属性和关系，再用 RAM++ 扩充普通场景词表、GroundingDINO 定位、SAM2 分割、D435 深度定位，最后由 VLM 处理复杂属性和关系复核。物体坐标绑定采集时刻的 TF，并跨帧汇入 SemanticNavigationGraph。",
    )


def slide_four(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "04", "自主搜索 Agent 与安全导航闭环", "观察—匹配—复核—更新记忆—规划—执行—再观察")

    add_box(slide, 0.58, 1.20, 4.05, 5.82, fill=WHITE, line=LINE)
    add_text(slide, "AutonomousExplorer 状态机", 0.87, 1.40, 3.46, 0.39, size=16.0, color=BLUE, bold=True)
    states = [
        ("OBSERVE", "获取图像、深度、地图与位姿", BLUE, PALE_BLUE),
        ("MATCH", "目标、属性与关系图匹配", TEAL, PALE_TEAL),
        ("VERIFY", "候选裁剪或 VLM 复核", PURPLE, PALE_PURPLE),
        ("UPDATE MEMORY", "写入物体、地点、前沿与负证据", AMBER, PALE_AMBER),
        ("PLAN", "选择下一观察位置", BLUE, "EDF3FF"),
        ("EXECUTE / RECOVER", "执行、等待结果、失败重规划", RED, PALE_RED),
    ]
    for i, (title, body, accent, fill) in enumerate(states):
        y = 1.96 + i * 0.73
        add_box(slide, 0.90, y, 3.40, 0.53, fill=fill, line=fill)
        add_text(slide, title, 1.06, y + 0.05, 1.22, 0.25, size=9.6, color=accent, bold=True)
        add_text(slide, body, 2.17, y + 0.05, 1.95, 0.28, size=8.3, color=INK, align=PP_ALIGN.RIGHT)
        if i < len(states) - 1:
            add_text(slide, "↓", 2.43, y + 0.49, 0.35, 0.25, size=11, color="9AABBC", bold=True, align=PP_ALIGN.CENTER)
    add_box(slide, 0.90, 6.49, 3.40, 0.30, fill=NAVY, line=NAVY)
    add_text(slide, "TARGET_FOUND / STOP / EXHAUSTED", 1.08, 6.51, 3.03, 0.23, size=8.7, color=WHITE, bold=True, align=PP_ALIGN.CENTER)

    add_box(slide, 4.87, 1.20, 3.52, 5.82, fill=WHITE, line=LINE)
    add_text(slide, "下一观察点如何选择？", 5.15, 1.40, 2.96, 0.39, size=16.0, color=PURPLE, bold=True)
    sources = [
        "地图 Frontier",
        "未观察方向扇区",
        "目标相关语义锚点",
        "高价值 Place / Object",
        "目标最后出现位置",
        "失败后的替代候选",
    ]
    for i, item in enumerate(sources):
        x = 5.16 + (i % 2) * 1.47
        y = 1.99 + (i // 2) * 0.62
        add_box(slide, x, y, 1.30, 0.45, fill=PALE_PURPLE, line=PALE_PURPLE)
        add_text(slide, item, x + 0.06, y + 0.04, 1.18, 0.32, size=8.2, color=INK, bold=True, align=PP_ALIGN.CENTER)

    add_text(slide, "候选联合评分", 5.15, 3.96, 2.10, 0.34, size=13.5, color=INK, bold=True)
    add_box(slide, 5.15, 4.38, 2.96, 1.28, fill=NAVY, line=NAVY)
    add_text(slide, "语义相关性 + 信息增益\n+ 新颖性 + 历史先验", 5.35, 4.52, 2.56, 0.52, size=11.2, color="A7E1F2", bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, "− 路径代价 − 重复访问\n− 导航失败 − 负证据", 5.35, 5.05, 2.56, 0.45, size=10.4, color="F4C57C", bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, "不是只去最近处，而是去最值得观察处", 5.24, 5.89, 2.78, 0.48, size=10.3, color=PURPLE, bold=True, align=PP_ALIGN.CENTER)

    add_box(slide, 8.63, 1.20, 4.12, 5.82, fill=WHITE, line=LINE)
    add_text(slide, "Nav2 与 fail-closed 安全链", 8.91, 1.40, 3.55, 0.39, size=16.0, color=AMBER, bold=True)
    execution = [
        ("地图目标", "Agent 输出 map 坐标", BLUE),
        ("ComputePathToPose", "真实路径存在才继续", TEAL),
        ("几何检查", "终点占用 / 走廊净空 / 转向扫掠", PURPLE),
        ("NavigateToPose / Spin", "平移与原地转向分流", AMBER),
        ("速度安全链", "平滑 / 碰撞监控 / 仲裁 / Bridge", RED),
        ("Go2-W 执行与反馈", "到达、取消、失败或急停", GREEN),
    ]
    for i, (title, body, accent) in enumerate(execution):
        y = 1.96 + i * 0.70
        add_number_circle(slide, str(i + 1), 8.96, y + 0.03, accent, 0.36)
        add_text(slide, title, 9.45, y, 1.62, 0.27, size=10.1, color=INK, bold=True)
        add_text(slide, body, 11.15, y, 1.25, 0.32, size=7.7, color=MUTED, align=PP_ALIGN.RIGHT)
    add_box(slide, 8.95, 6.43, 3.48, 0.36, fill=PALE_RED, line=PALE_RED)
    add_text(slide, "默认只规划；显式授权后才允许物理执行", 9.07, 6.47, 3.24, 0.26, size=8.8, color=RED, bold=True, align=PP_ALIGN.CENTER)

    add_footer(slide, "Agent 只选择下一观察目标；任何速度都必须经过 Nav2 与实时安全门控。", 4)
    set_notes(
        slide,
        "Agent 不是直接生成电机速度，而是持续维护搜索状态，选择一个具有语义和空间依据的下一观察目标。目标还必须经过 Nav2 真实路径、终点占用、走廊净空、转向扫掠、碰撞监控、控制仲裁、Arm、租约和急停等保护才能进入机器狗执行链。",
    )


def slide_five(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "05", "WebUI 功能与典型任务演示", "把感知、地图、决策、控制与历史回放集中到同一界面")

    add_box(slide, 0.58, 1.18, 7.72, 4.14, fill=WHITE, line=LINE)
    add_picture_crop(slide, WEBUI, 0.75, 1.36, 7.38, 3.77)
    add_badge(slide, "搜索状态与地图回放", 0.95, 1.55, 1.72, CYAN, "E9F7FC")

    add_box(slide, 8.55, 1.18, 4.20, 4.14, fill=WHITE, line=LINE)
    add_text(slide, "用户可见功能", 8.83, 1.39, 2.50, 0.37, size=16.0, color=NAVY, bold=True)
    capabilities = [
        ("自然语言任务", "输入目标、属性与关系", BLUE, PALE_BLUE),
        ("实时感知", "相机、物体列表与目标状态", TEAL, PALE_TEAL),
        ("多图层展示", "二维地图、三维点云、语义拓扑", PURPLE, PALE_PURPLE),
        ("Agent 解释", "候选、选择理由与执行状态", AMBER, PALE_AMBER),
        ("运行控制", "暂停、恢复、停止与急停", RED, PALE_RED),
        ("历史回放", "轨迹、事件、地图和最终结果", GREEN, PALE_GREEN),
    ]
    for i, (title, body, accent, fill) in enumerate(capabilities):
        x = 8.84 + (i % 2) * 1.80
        y = 1.91 + (i // 2) * 0.99
        add_box(slide, x, y, 1.62, 0.78, fill=fill, line=fill)
        add_text(slide, title, x + 0.09, y + 0.08, 1.44, 0.27, size=9.7, color=accent, bold=True, align=PP_ALIGN.CENTER)
        add_text(slide, body, x + 0.10, y + 0.37, 1.42, 0.30, size=7.7, color=INK, align=PP_ALIGN.CENTER)

    add_text(slide, "典型任务：寻找饮水机旁边的蓝色垃圾桶", 0.62, 5.54, 5.70, 0.38, size=16.3, color=NAVY, bold=True)
    demo_steps = [
        ("1", "输入任务", "解析垃圾桶 / 蓝色 / NEAR 饮水机", BLUE, PALE_BLUE),
        ("2", "观察场景", "识别普通物体与目标候选", TEAL, PALE_TEAL),
        ("3", "更新世界", "物体定位并写入语义拓扑", PURPLE, PALE_PURPLE),
        ("4", "选择视点", "结合饮水机锚点与 Frontier", AMBER, PALE_AMBER),
        ("5", "安全导航", "规划、净空检查与受控执行", RED, PALE_RED),
        ("6", "复核目标", "验证蓝色与邻近关系并输出证据", GREEN, PALE_GREEN),
    ]
    for i, (num, title, body, accent, fill) in enumerate(demo_steps):
        x = 0.58 + i * 2.05
        add_box(slide, x, 6.04, 1.77, 0.91, fill=fill, line=fill)
        add_number_circle(slide, num, x + 0.10, 6.14, accent, 0.34)
        add_text(slide, title, x + 0.51, 6.10, 1.12, 0.25, size=9.7, color=accent, bold=True)
        add_text(slide, body, x + 0.12, 6.43, 1.52, 0.36, size=7.5, color=INK, align=PP_ALIGN.CENTER, valign=MSO_ANCHOR.TOP)
        if i < len(demo_steps) - 1:
            add_chevron(slide, x + 1.81, 6.30, "9DB0C5")

    add_footer(slide, "演示建议：相机画面、地图轨迹、语义图和 Agent 决策状态同步展示。", 5)
    set_notes(
        slide,
        "最后一页只展示项目功能和典型演示，不放阶段性成果或边界。演示时可以输入‘寻找饮水机旁边的蓝色垃圾桶’，同步展示相机、物体列表、地图轨迹、语义拓扑和 Agent 当前选择。这样老师能直观看到每个模块如何共同完成一项具身搜索任务。",
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


def build() -> Path:
    for asset in (PHOTO_ROBOT, PHOTO_CAMERA, TOPOLOGY, WEBUI):
        if not asset.is_file():
            raise FileNotFoundError(asset)
    prs = Presentation()
    prs.slide_width = Inches(SLIDE_W)
    prs.slide_height = Inches(SLIDE_H)
    prs.core_properties.title = "GO2W 自主语义搜索项目架构与功能"
    prs.core_properties.subject = "五页项目汇报"
    prs.core_properties.author = "robotscene project"
    prs.core_properties.comments = "依据当前仓库源码与项目素材生成。"
    slide_one(prs)
    slide_two(prs)
    slide_three(prs)
    slide_four(prs)
    slide_five(prs)
    validate_bounds(prs)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    prs.save(OUT)
    return OUT


if __name__ == "__main__":
    print(build())
