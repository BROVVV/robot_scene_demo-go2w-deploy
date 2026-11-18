#!/usr/bin/env python3
"""Generate the four-slide GO2W project results presentation."""

from __future__ import annotations

from pathlib import Path

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches, Pt


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "ppt" / "GO2W_具身语义搜索_阶段成果汇报_4页.pptx"

PHOTO_ROBOT = ROOT / "reports" / "image.png"
PHOTO_CAMERA = ROOT / "artifacts" / "perception_profile_20261013" / "frames" / "color_5465.jpg"
TOPOLOGY = ROOT / "outputs" / "topology_graph.png"
REPLAY = (
    ROOT
    / "outputs"
    / "live_runs"
    / "search_20261023_160331_54e4f466"
    / "replay"
    / "artifacts"
    / "cover.jpg"
)

SLIDE_W = 13.333
SLIDE_H = 7.5
FONT = "Noto Sans CJK SC"

NAVY = "122033"
INK = "17263A"
MUTED = "5F6F82"
BG = "F4F7FB"
WHITE = "FFFFFF"
LINE = "D8E1EC"
CYAN = "20A4D8"
BLUE = "3E74E6"
TEAL = "16A085"
PURPLE = "7557C5"
AMBER = "F0A83B"
GREEN = "2FAA66"
RED = "D65A5A"
PALE_BLUE = "EAF4FF"
PALE_TEAL = "E8F7F3"
PALE_PURPLE = "F1EDFB"
PALE_AMBER = "FFF4E2"
PALE_GREEN = "EAF7EF"
PALE_RED = "FCEDED"


def rgb(hex_color: str) -> RGBColor:
    return RGBColor.from_string(hex_color)


def set_bg(slide, color: str = BG) -> None:
    fill = slide.background.fill
    fill.solid()
    fill.fore_color.rgb = rgb(color)


def add_text(
    slide,
    text: str,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    size: float = 18,
    color: str = INK,
    bold: bool = False,
    align=PP_ALIGN.LEFT,
    valign=MSO_ANCHOR.MIDDLE,
    margin: float = 0.03,
    font: str = FONT,
    line_spacing: float | None = None,
):
    shape = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = shape.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(margin)
    tf.margin_top = tf.margin_bottom = Inches(margin)
    tf.vertical_anchor = valign
    p = tf.paragraphs[0]
    p.alignment = align
    if line_spacing is not None:
        p.line_spacing = line_spacing
    run = p.add_run()
    run.text = text
    run.font.name = font
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = rgb(color)
    return shape


def add_rich_text(
    slide,
    runs: list[tuple[str, dict]],
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    align=PP_ALIGN.LEFT,
    valign=MSO_ANCHOR.MIDDLE,
    margin: float = 0.03,
):
    shape = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = shape.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(margin)
    tf.margin_top = tf.margin_bottom = Inches(margin)
    tf.vertical_anchor = valign
    p = tf.paragraphs[0]
    p.alignment = align
    for content, style in runs:
        run = p.add_run()
        run.text = content
        run.font.name = style.get("font", FONT)
        run.font.size = Pt(style.get("size", 18))
        run.font.bold = style.get("bold", False)
        run.font.color.rgb = rgb(style.get("color", INK))
    return shape


def add_box(
    slide,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    fill: str = WHITE,
    line: str = LINE,
    radius: bool = True,
    line_width: float = 1.0,
):
    kind = MSO_SHAPE.ROUNDED_RECTANGLE if radius else MSO_SHAPE.RECTANGLE
    shape = slide.shapes.add_shape(kind, Inches(x), Inches(y), Inches(w), Inches(h))
    shape.fill.solid()
    shape.fill.fore_color.rgb = rgb(fill)
    shape.line.color.rgb = rgb(line)
    shape.line.width = Pt(line_width)
    return shape


def add_box_text(
    slide,
    title: str,
    body: str,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    fill: str = WHITE,
    line: str = LINE,
    accent: str | None = None,
    title_size: float = 15,
    body_size: float = 11,
    align=PP_ALIGN.LEFT,
):
    box = add_box(slide, x, y, w, h, fill=fill, line=line)
    if accent:
        stripe = slide.shapes.add_shape(
            MSO_SHAPE.ROUNDED_RECTANGLE,
            Inches(x + 0.08),
            Inches(y + 0.12),
            Inches(0.07),
            Inches(max(0.15, h - 0.24)),
        )
        stripe.fill.solid()
        stripe.fill.fore_color.rgb = rgb(accent)
        stripe.line.fill.background()
    tx = x + (0.25 if accent else 0.16)
    add_text(
        slide,
        title,
        tx,
        y + 0.09,
        w - (tx - x) - 0.12,
        0.28,
        size=title_size,
        color=INK,
        bold=True,
        align=align,
    )
    if body:
        add_text(
            slide,
            body,
            tx,
            y + 0.36,
            w - (tx - x) - 0.12,
            h - 0.43,
            size=body_size,
            color=MUTED,
            align=align,
            valign=MSO_ANCHOR.TOP,
            line_spacing=1.0,
        )
    return box


def add_picture_crop(slide, path: Path, x: float, y: float, w: float, h: float):
    with Image.open(path) as img:
        iw, ih = img.size
    image_ratio = iw / ih
    frame_ratio = w / h
    pic = slide.shapes.add_picture(str(path), Inches(x), Inches(y), Inches(w), Inches(h))
    if image_ratio > frame_ratio:
        shown = frame_ratio / image_ratio
        crop = (1.0 - shown) / 2.0
        pic.crop_left = crop
        pic.crop_right = crop
    elif image_ratio < frame_ratio:
        shown = image_ratio / frame_ratio
        crop = (1.0 - shown) / 2.0
        pic.crop_top = crop
        pic.crop_bottom = crop
    return pic


def add_header(slide, number: str, title: str, kicker: str) -> None:
    add_text(slide, number, 0.55, 0.33, 0.55, 0.42, size=13, color=CYAN, bold=True)
    add_text(slide, title, 1.05, 0.24, 8.9, 0.6, size=27, color=NAVY, bold=True)
    add_text(slide, kicker, 9.15, 0.31, 3.6, 0.42, size=10.5, color=MUTED, align=PP_ALIGN.RIGHT)
    line = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.55), Inches(0.94), Inches(12.2), Inches(0.025))
    line.fill.solid()
    line.fill.fore_color.rgb = rgb(LINE)
    line.line.fill.background()


def add_footer(slide, text: str, page: int) -> None:
    add_text(slide, text, 0.58, 7.18, 11.55, 0.2, size=7.7, color="7C8998", valign=MSO_ANCHOR.MIDDLE)
    add_text(slide, f"{page}/4", 12.05, 7.15, 0.68, 0.25, size=8.5, color=MUTED, align=PP_ALIGN.RIGHT)


def add_badge(slide, text: str, x: float, y: float, w: float, color: str, fill: str) -> None:
    add_box(slide, x, y, w, 0.34, fill=fill, line=fill)
    add_text(slide, text, x + 0.06, y + 0.01, w - 0.12, 0.29, size=9.2, color=color, bold=True, align=PP_ALIGN.CENTER)


def add_chevron(slide, x: float, y: float, color: str = "9DB0C5"):
    shape = slide.shapes.add_shape(MSO_SHAPE.CHEVRON, Inches(x), Inches(y), Inches(0.24), Inches(0.34))
    shape.fill.solid()
    shape.fill.fore_color.rgb = rgb(color)
    shape.line.fill.background()
    return shape


def set_notes(slide, text: str) -> None:
    tf = slide.notes_slide.notes_text_frame
    tf.text = text
    for p in tf.paragraphs:
        for run in p.runs:
            run.font.name = FONT


def slide_one(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "01", "面向 GO2W 的自主语义搜索系统", "自然语言 → 感知 → 地图 → 决策 → 受控运动")

    add_picture_crop(slide, PHOTO_ROBOT, 0.55, 1.22, 4.05, 4.88)
    overlay = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.55), Inches(4.95), Inches(4.05), Inches(1.15))
    overlay.fill.solid()
    overlay.fill.fore_color.rgb = rgb(NAVY)
    overlay.fill.transparency = 8
    overlay.line.fill.background()
    add_text(slide, "真实硬件平台", 0.78, 5.06, 1.4, 0.31, size=11, color="89D9F3", bold=True)
    add_text(slide, "Unitree GO2W + D435 RGB-D\n+ 原装 LiDAR + PandarXT-16", 0.78, 5.35, 3.47, 0.58, size=15.5, color=WHITE, bold=True, valign=MSO_ANCHOR.TOP)

    stages = [
        ("1", "任务理解", "目标 / 属性 / 关系 / 安全约束", PALE_BLUE, BLUE),
        ("2", "开放词汇感知", "RAM++ · DINO · SAM2 · VLM", PALE_TEAL, TEAL),
        ("3", "空间世界模型", "物体 / Place / Frontier / 关系", PALE_PURPLE, PURPLE),
        ("4", "任务 Agent", "观察 · 匹配 · 复核 · 重规划", "EEF3FF", BLUE),
        ("5", "Nav2 与安全链", "路径 · 净空 · 仲裁 · 急停", PALE_AMBER, AMBER),
        ("6", "WebUI 与证据", "状态 · 决策理由 · 回放", "EDF4F8", CYAN),
    ]
    x0, y0, bw, bh, gapx, gapy = 4.98, 1.31, 2.32, 1.55, 0.35, 0.35
    # Physical order forms a snake: 1 → 2 → 3 ↓ 4 → 5 → 6.
    physical = [stages[0], stages[1], stages[2], stages[5], stages[4], stages[3]]
    for i, (num, title, body, fill, accent) in enumerate(physical):
        row, col = divmod(i, 3)
        x = x0 + col * (bw + gapx)
        y = y0 + row * (bh + gapy)
        add_box(slide, x, y, bw, bh, fill=fill, line=fill)
        badge = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x + 0.16), Inches(y + 0.16), Inches(0.42), Inches(0.42))
        badge.fill.solid()
        badge.fill.fore_color.rgb = rgb(accent)
        badge.line.fill.background()
        add_text(slide, num, x + 0.17, y + 0.17, 0.40, 0.38, size=10.5, color=WHITE, bold=True, align=PP_ALIGN.CENTER)
        add_text(slide, title, x + 0.68, y + 0.14, bw - 0.82, 0.43, size=14.2, color=INK, bold=True)
        add_text(slide, body, x + 0.17, y + 0.70, bw - 0.34, 0.62, size=10.5, color=MUTED, valign=MSO_ANCHOR.TOP)
        if col < 2:
            chev = add_chevron(slide, x + bw + 0.055, y + 0.59)
            if row == 1:
                chev.rotation = 180
    add_text(slide, "↓", 11.30, 2.83, 0.35, 0.35, size=20, color="9DB0C5", bold=True, align=PP_ALIGN.CENTER)

    map_cards = [
        ("二维导航地图", "Nav2、定位、前沿和路径检查", BLUE, PALE_BLUE),
        ("三维几何地图", "plain_slam 点云展示与建图辅助", TEAL, PALE_TEAL),
        ("语义拓扑图", "物体、地点、关系和搜索记忆", PURPLE, PALE_PURPLE),
    ]
    for i, (title, body, accent, fill) in enumerate(map_cards):
        x = 4.98 + i * 2.67
        add_box_text(slide, title, body, x, 5.15, 2.32, 0.95, fill=fill, line=fill, accent=accent, title_size=12.2, body_size=8.9)

    add_box(slide, 0.55, 6.32, 12.2, 0.63, fill=NAVY, line=NAVY)
    add_text(slide, "核心成果：将离线视觉能力推进为可在真实机器狗上运行、可解释、可回放、受安全门控的具身搜索原型。", 0.82, 6.39, 11.65, 0.43, size=14.2, color=WHITE, bold=True, align=PP_ALIGN.CENTER)
    add_footer(slide, "定位说明：二维地图承担正常导航；三维点云主要用于几何展示与建图辅助。", 1)
    set_notes(
        slide,
        "这一页先说明项目定位。项目不是单独的目标检测演示，而是把自然语言任务、RGB-D 感知、二维导航地图、语义世界模型、任务决策、Nav2 和安全控制串成一条链路。需要特别说明，当前正常导航主链依赖二维 SLAM Toolbox 地图；PandarXT-16 与 plain_slam 的三维点云主要用于建图辅助和 Web 展示，不能表述成已经使用三维点云直接完成 Nav2 导航。",
    )


def slide_two(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "02", "从“看见物体”到“知道物体在哪里”", "开放词汇视觉感知 × RGB-D 空间定位 × 持久语义图")

    pipeline = [
        ("任务目标", "TargetProfile", BLUE, PALE_BLUE),
        ("场景标签", "RAM++", CYAN, "E9F7FC"),
        ("开放词汇定位", "GroundingDINO", TEAL, PALE_TEAL),
        ("实例分割", "SAM2", PURPLE, PALE_PURPLE),
        ("三维定位 / 复核", "D435 Depth + VLM", AMBER, PALE_AMBER),
    ]
    for i, (title, body, accent, fill) in enumerate(pipeline):
        x = 0.62 + i * 2.48
        add_box(slide, x, 1.20, 2.12, 0.78, fill=fill, line=fill)
        add_text(slide, title, x + 0.08, 1.27, 1.96, 0.28, size=11.5, color=accent, bold=True, align=PP_ALIGN.CENTER)
        add_text(slide, body, x + 0.08, 1.54, 1.96, 0.27, size=9.4, color=INK, bold=True, align=PP_ALIGN.CENTER)
        if i < len(pipeline) - 1:
            add_chevron(slide, x + 2.18, 1.41, "9AB2C6")

    add_box(slide, 0.58, 2.20, 3.74, 4.67, fill=WHITE, line=LINE)
    add_picture_crop(slide, PHOTO_CAMERA, 0.73, 2.39, 3.44, 2.28)
    add_badge(slide, "真实 D435 采集帧", 0.89, 2.55, 1.42, CYAN, "E9F7FC")
    add_text(slide, "目标词不会被 RAM++ 的场景标签“卡住”", 0.82, 4.84, 3.28, 0.38, size=13.2, color=INK, bold=True)
    add_text(slide, "用户目标独立进入 GroundingDINO prompt；VLM 只承担复杂属性、关系与疑似目标复核。", 0.82, 5.28, 3.20, 0.72, size=11.2, color=MUTED, valign=MSO_ANCHOR.TOP)
    add_box(slide, 0.82, 6.08, 3.18, 0.54, fill=PALE_BLUE, line=PALE_BLUE)
    add_text(slide, "二维框 / Mask → 深度 → 地图坐标", 0.92, 6.14, 2.98, 0.35, size=11.7, color=BLUE, bold=True, align=PP_ALIGN.CENTER)

    add_box(slide, 4.54, 2.20, 4.11, 4.67, fill=WHITE, line=LINE)
    add_text(slide, "空间化与持久化", 4.79, 2.37, 2.4, 0.36, size=15.2, color=PURPLE, bold=True)
    add_box(slide, 4.80, 2.84, 3.58, 0.63, fill=PALE_PURPLE, line=PALE_PURPLE)
    add_text(slide, "相机坐标  →  机身坐标  →  地图坐标", 4.95, 2.96, 3.28, 0.31, size=10.9, color=PURPLE, bold=True, align=PP_ALIGN.CENTER)
    add_picture_crop(slide, TOPOLOGY, 4.88, 3.68, 3.43, 2.35)
    add_text(slide, "Object ID · Place · Frontier · Relation · Provenance", 4.86, 6.17, 3.48, 0.35, size=10.6, color=INK, bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, "定位或 TF 不可靠时，不虚构地图坐标。", 4.90, 6.50, 3.40, 0.24, size=8.7, color=MUTED, align=PP_ALIGN.CENTER)

    add_box(slide, 8.88, 2.20, 3.87, 4.67, fill=NAVY, line=NAVY)
    add_text(slide, "实测性能与边界", 9.17, 2.37, 2.96, 0.40, size=16.5, color=WHITE, bold=True)
    add_text(slide, "4.47 s", 9.15, 2.98, 1.12, 0.55, size=25, color="B9C6D5", bold=True)
    add_text(slide, "→", 10.29, 3.03, 0.45, 0.40, size=20, color="8AA1B8", bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, "3.22 s", 10.75, 2.98, 1.45, 0.55, size=25, color="68D6B5", bold=True)
    add_text(slide, "本地视觉函数链约 −28.1%", 9.15, 3.50, 3.0, 0.32, size=11.2, color="D3DDE8")
    add_box(slide, 9.15, 4.04, 3.28, 0.75, fill="1C3148", line="1C3148")
    add_rich_text(slide, [("≈70 ms", {"size": 20, "bold": True, "color": "68D6B5"}), ("  语义更新", {"size": 10.5, "color": "D3DDE8"})], 9.32, 4.14, 2.95, 0.46)
    add_box(slide, 9.15, 4.95, 3.28, 0.75, fill="1C3148", line="1C3148")
    add_rich_text(slide, [("14–24 s", {"size": 19, "bold": True, "color": "F7BD65"}), ("  云端 VLM", {"size": 10.5, "color": "D3DDE8"})], 9.32, 5.05, 2.95, 0.46)
    add_text(slide, "样本主要来自同一办公室；尚不能代表跨场景准确率或稳定 p95。", 9.16, 5.91, 3.21, 0.69, size=9.7, color="D3DDE8", valign=MSO_ANCHOR.TOP)

    add_footer(slide, "数据来源：2026-10-13 感知性能报告；结果保持一致仅针对报告中的有限样本。", 2)
    set_notes(
        slide,
        "本页重点是从二维识别到空间世界模型的转换。RAM++ 用于补充普通场景类别，但用户目标独立进入 GroundingDINO，避免场景标签漏词导致任务目标不被搜索。SAM2 提供实例 mask，D435 深度与采集时刻 TF 将物体定位到地图坐标，并跨帧关联成持久 Object。性能数据来自少量静止办公室样本：本地视觉函数链从约 4.47 秒降至 3.22 秒，主要收益来自复用 DINO backbone 和向量化 RLE；不能把它扩展成跨场景准确率结论。",
    )


def slide_three(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "03", "任务 Agent：基于地图、语义与记忆持续重规划", "Agent 不直接发速度；执行前必须通过 Nav2 与安全控制链")

    add_box(slide, 0.58, 1.20, 6.56, 5.82, fill=WHITE, line=LINE)
    add_text(slide, "自主搜索闭环", 0.88, 1.40, 2.1, 0.38, size=16.8, color=BLUE, bold=True)
    steps = [
        ("1", "任务结构化", "目标、属性、关系与约束", BLUE, PALE_BLUE),
        ("2", "获取观察与地图", "RGB-D、机器人位姿、二维栅格", TEAL, PALE_TEAL),
        ("3", "图匹配与世界更新", "Object / Place / Frontier / Memory", PURPLE, PALE_PURPLE),
        ("4", "候选位置排序", "语义相关度、信息增益、路程与失败历史", AMBER, PALE_AMBER),
        ("5", "Nav2 规划与执行", "局部目标、路径、净空与速度门控", RED, PALE_RED),
        ("6", "反馈、复核、重规划", "到达、失败或新证据驱动下一轮", GREEN, PALE_GREEN),
    ]
    sy = 1.92
    for i, (num, title, body, accent, fill) in enumerate(steps):
        y = sy + i * 0.78
        add_box(slide, 1.02, y, 5.64, 0.60, fill=fill, line=fill)
        circ = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(1.18), Inches(y + 0.10), Inches(0.40), Inches(0.40))
        circ.fill.solid()
        circ.fill.fore_color.rgb = rgb(accent)
        circ.line.fill.background()
        add_text(slide, num, 1.19, y + 0.10, 0.38, 0.36, size=9.6, color=WHITE, bold=True, align=PP_ALIGN.CENTER)
        add_text(slide, title, 1.75, y + 0.06, 1.62, 0.30, size=12.2, color=INK, bold=True)
        add_text(slide, body, 3.28, y + 0.07, 3.03, 0.31, size=9.7, color=MUTED, align=PP_ALIGN.RIGHT)
        if i < len(steps) - 1:
            add_text(slide, "↓", 3.70, y + 0.56, 0.35, 0.24, size=11, color="98AABD", bold=True, align=PP_ALIGN.CENTER)
    add_box(slide, 1.02, 6.62, 5.64, 0.22, fill="E9EEF5", line="E9EEF5")
    add_text(slide, "新观察 / 执行结果回流", 2.85, 6.56, 1.96, 0.32, size=8.7, color=MUTED, bold=True, align=PP_ALIGN.CENTER)

    add_box(slide, 7.39, 1.20, 5.36, 5.82, fill=WHITE, line=LINE)
    add_picture_crop(slide, REPLAY, 7.58, 1.40, 4.98, 2.60)
    add_badge(slide, "真实 Web 搜索回放", 7.78, 1.60, 1.55, CYAN, "E9F7FC")
    add_text(slide, "0.17 m", 7.78, 4.18, 1.28, 0.48, size=23, color=GREEN, bold=True)
    add_text(slide, "实测累计行程", 7.79, 4.60, 1.47, 0.27, size=9.2, color=MUTED)
    add_text(slide, "OPERATOR_STOP", 9.40, 4.23, 1.96, 0.37, size=15.2, color=AMBER, bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, "人工停止，不等于搜索失败", 9.45, 4.60, 1.90, 0.27, size=8.9, color=MUTED, align=PP_ALIGN.CENTER)
    add_text(slide, "3 × 0.12 m", 11.34, 4.19, 1.02, 0.44, size=17.2, color=BLUE, bold=True, align=PP_ALIGN.RIGHT)
    add_text(slide, "短目标首次尝试成功", 11.15, 4.60, 1.23, 0.27, size=8.7, color=MUTED, align=PP_ALIGN.RIGHT)

    add_text(slide, "执行前的 fail-closed 安全链", 7.78, 5.03, 3.20, 0.34, size=14.5, color=INK, bold=True)
    gates = ["路径长度 / 终点误差", "机身扫掠 / 硬净空 0.40 m", "地图 / TF / 激光新鲜度", "速度守卫 / 碰撞监控", "租约 / Arm / 急停 / 超时"]
    for i, gate in enumerate(gates):
        x = 7.78 + (i % 2) * 2.35
        y = 5.48 + (i // 2) * 0.47
        w = 2.18 if i < 4 else 4.53
        add_box(slide, x, y, w, 0.35, fill="EEF3F8", line="EEF3F8")
        add_text(slide, "✓ " + gate, x + 0.08, y + 0.02, w - 0.16, 0.28, size=8.9, color=INK, bold=True, align=PP_ALIGN.CENTER)

    add_footer(slide, "证据边界：短程受控闭环已接通；该回放终态为 OPERATOR_STOP，不能表述为“大场地自主搜索完成”。", 3)
    set_notes(
        slide,
        "任务 Agent 的作用是持续维护任务状态、汇集视觉、地图、拓扑和历史记忆，选择下一观察位置，再把目标交给 Nav2。它不是聊天模型直接控制机器狗。右侧是 2026-10-23 的真实 Web 搜索回放，累计行程约 0.17 米，并由操作员主动停止。另有连续三个 0.12 米短目标的受控测试全部在首次尝试完成。这些结果证明链路和停止机制有效，但不能宣传成大场地连续自主搜索已经验收通过。",
    )


def slide_four(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "04", "阶段成果、验证证据与当前边界", "用“已完成 / 部分完成 / 待验证”区分代码能力与现场验收")

    metrics = [
        ("2703", "10 月 23 日版本 Python 测试通过", GREEN, PALE_GREEN),
        ("3 × 0.12 m", "连续短 Nav2 目标首次尝试成功", BLUE, PALE_BLUE),
        ("18", "一次只读 Grounded-SAM 运行的持久物体", PURPLE, PALE_PURPLE),
    ]
    for i, (value, caption, accent, fill) in enumerate(metrics):
        x = 0.58 + i * 4.11
        add_box(slide, x, 1.19, 3.86, 1.06, fill=fill, line=fill)
        value_w = 1.88 if i == 1 else 1.52
        value_size = 20.5 if i == 1 else 23.5
        text_x = x + (1.98 if i == 1 else 1.64)
        add_text(slide, value, x + 0.20, 1.30, value_w, 0.50, size=value_size, color=accent, bold=True)
        add_text(slide, caption, text_x, 1.32, x + 3.66 - text_x, 0.52, size=9.5 if i == 1 else 9.9, color=INK, bold=True, valign=MSO_ANCHOR.TOP)
        add_text(slide, "有结构化运行证据", text_x, 1.86, x + 3.59 - text_x, 0.23, size=8.2, color=MUTED)

    columns = [
        (
            "✓ 已完成并有证据",
            [
                "任务理解—感知—世界模型已接通",
                "二维地图与 Frontier 进入 Agent 决策",
                "Nav2、安全仲裁与运动 Action 已接通",
                "世界快照、事件、回放与 WebUI 已实现",
            ],
            GREEN,
            PALE_GREEN,
        ),
        (
            "◐ 已实现 / 部分验证",
            [
                "保存地图后的进程重定位通过",
                "硬件重启 + 粗略初值重定位通过",
                "短距离 Web / Nav2 真实运动已验证",
                "三档物体与危险告警已接线，待独立验收",
            ],
            AMBER,
            PALE_AMBER,
        ),
        (
            "△ 尚未完成或不可扩大",
            [
                "长时间、多场景连续自主搜索未验收",
                "无先验全局重定位仍未通过",
                "跨场景准确率与稳定 p95 尚缺统计",
                "动态 Point-LIO 导航路线曾冻结或发散",
            ],
            RED,
            PALE_RED,
        ),
    ]
    for i, (heading, bullets, accent, fill) in enumerate(columns):
        x = 0.58 + i * 4.11
        add_box(slide, x, 2.53, 3.86, 3.80, fill=WHITE, line=LINE)
        add_box(slide, x, 2.53, 3.86, 0.62, fill=fill, line=fill)
        add_text(slide, heading, x + 0.18, 2.64, 3.50, 0.34, size=14.0, color=accent, bold=True)
        for j, bullet in enumerate(bullets):
            y = 3.36 + j * 0.67
            mark = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x + 0.23), Inches(y + 0.04), Inches(0.20), Inches(0.20))
            mark.fill.solid()
            mark.fill.fore_color.rgb = rgb(accent)
            mark.line.fill.background()
            add_text(slide, bullet, x + 0.54, y - 0.01, 3.05, 0.48, size=10.4, color=INK, bold=j == 0, valign=MSO_ANCHOR.TOP)
        if i == 1:
            add_badge(slide, "代码完成 ≠ 现场验收", x + 0.82, 5.97, 2.20, AMBER, PALE_AMBER)
        elif i == 2:
            add_badge(slide, "保留失败记录", x + 1.08, 5.97, 1.67, RED, PALE_RED)
        else:
            add_badge(slide, "软件链路可复现", x + 0.98, 5.97, 1.86, GREEN, PALE_GREEN)

    add_box(slide, 0.58, 6.54, 12.20, 0.53, fill=NAVY, line=NAVY)
    add_text(slide, "阶段判断：已形成真实机器人上的具身搜索原型；下一阶段重点是跨场景感知评测、无先验重定位和连续现场搜索验收。", 0.84, 6.61, 11.67, 0.34, size=12.4, color=WHITE, bold=True, align=PP_ALIGN.CENTER)
    add_footer(slide, "注：2703 passed / 3 skipped 为 2026-10-23 验收版本；10-26 后工作区仍有未提交改动，未以旧测试数替代新版本全量验收。", 4)
    set_notes(
        slide,
        "最后一页把成果分级。绿色部分可以直接作为已完成成果；黄色部分要说明代码已经实现或局部现场通过，但仍缺完整验收；红色部分主动说明边界。10 月 23 日验收报告记录完整 Python 测试 2703 项通过、3 项跳过。10 月 26 日后工作区还有新的物体三档分类、危险告警和遥控建图接线，但尚未形成独立完整现场证据，因此这里只把它列为待验收能力。",
    )


def build() -> Path:
    for asset in (PHOTO_ROBOT, PHOTO_CAMERA, TOPOLOGY, REPLAY):
        if not asset.is_file():
            raise FileNotFoundError(asset)
    prs = Presentation()
    prs.slide_width = Inches(SLIDE_W)
    prs.slide_height = Inches(SLIDE_H)
    prs.core_properties.title = "GO2W 具身语义搜索阶段成果汇报"
    prs.core_properties.subject = "四页项目成果汇报"
    prs.core_properties.author = "robotscene project"
    prs.core_properties.comments = "依据仓库源码、运行记录与验收报告生成；保留已验证与待验证边界。"
    slide_one(prs)
    slide_two(prs)
    slide_three(prs)
    slide_four(prs)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    prs.save(OUT)
    return OUT


if __name__ == "__main__":
    print(build())
