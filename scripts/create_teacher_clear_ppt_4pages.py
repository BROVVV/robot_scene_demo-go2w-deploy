#!/usr/bin/env python3
"""Create a clear, professional four-slide GO2W project deck.

The deck is designed for a teacher reading it without a live presenter:
technical terms are retained but explained in plain language, and internal
APIs, source paths and implementation trivia are intentionally omitted.
"""

from __future__ import annotations

from pathlib import Path

from pptx import Presentation
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches

from create_results_ppt import (
    AMBER,
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
    add_box,
    add_header,
    add_text,
    rgb,
    set_bg,
    set_notes,
)


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "ppt" / "GO2W_自主语义搜索_老师易懂专业版_4页.pptx"


def footer(slide, text: str, page: int) -> None:
    add_text(slide, text, 0.58, 7.15, 11.45, 0.22, size=7.4, color="748397")
    add_text(slide, f"{page}/4", 12.05, 7.12, 0.67, 0.25, size=8.3, color=MUTED, align=PP_ALIGN.RIGHT)


def title_body_box(
    slide,
    title: str,
    body: str,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    accent: str,
    fill: str,
    title_size: float = 12.0,
    body_size: float = 9.0,
) -> None:
    add_box(slide, x, y, w, h, fill=fill, line=fill)
    add_text(slide, title, x + 0.17, y + 0.11, w - 0.34, 0.32, size=title_size, color=accent, bold=True)
    add_text(
        slide,
        body,
        x + 0.17,
        y + 0.47,
        w - 0.34,
        h - 0.58,
        size=body_size,
        color=INK,
        valign=MSO_ANCHOR.TOP,
        line_spacing=1.0,
    )


def numbered_step(
    slide,
    number: str,
    title: str,
    body: str,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    accent: str,
    fill: str,
) -> None:
    add_box(slide, x, y, w, h, fill=fill, line=fill)
    circle = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x + 0.13), Inches(y + 0.13), Inches(0.40), Inches(0.40))
    circle.fill.solid()
    circle.fill.fore_color.rgb = rgb(accent)
    circle.line.fill.background()
    add_text(slide, number, x + 0.14, y + 0.13, 0.38, 0.36, size=9.8, color=WHITE, bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, title, x + 0.64, y + 0.10, w - 0.77, 0.32, size=11.2, color=accent, bold=True)
    add_text(slide, body, x + 0.17, y + 0.56, w - 0.34, h - 0.66, size=8.5, color=INK, valign=MSO_ANCHOR.TOP)


def compact_row(slide, title: str, body: str, x: float, y: float, w: float, *, accent: str, fill: str) -> None:
    add_box(slide, x, y, w, 0.52, fill=fill, line=fill)
    add_text(slide, title, x + 0.14, y + 0.08, 1.38, 0.27, size=9.3, color=accent, bold=True)
    add_text(slide, body, x + 1.47, y + 0.07, w - 1.62, 0.35, size=8.0, color=INK, align=PP_ALIGN.RIGHT)


def slide_one(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "01", "项目是什么：让机器狗根据自然语言主动寻找目标", "项目目标、任务示例与完整工作流程")

    add_box(slide, 0.58, 1.16, 12.17, 0.76, fill=NAVY, line=NAVY)
    add_text(slide, "一句话介绍", 0.82, 1.31, 1.27, 0.30, size=13.2, color="86DCF2", bold=True, align=PP_ALIGN.CENTER)
    add_text(
        slide,
        "用户说出要找的物体，机器人一边移动、一边观察和更新地图，直到确认目标、合理搜索完毕，或由操作员停止。",
        2.19,
        1.26,
        10.14,
        0.40,
        size=15.0,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )

    title_body_box(
        slide,
        "任务示例：寻找饮水机旁边的蓝色垃圾桶",
        "这句话同时包含三类信息：要找的物体是“垃圾桶”，目标属性是“蓝色”，空间关系是“在饮水机旁边”。系统不仅要识别垃圾桶，还要判断颜色和邻近关系；如果当前看不到垃圾桶，可以先寻找饮水机，把它作为继续搜索的语义锚点，再选择能够观察饮水机周围区域的位置。",
        0.58,
        2.12,
        4.18,
        1.46,
        accent=BLUE,
        fill=PALE_BLUE,
        title_size=12.5,
        body_size=9.2,
    )
    title_body_box(
        slide,
        "搜索过程中持续维护哪些信息？",
        "系统在每轮观察后同步更新五类状态：机器人当前位姿和朝向；画面中可见物体及其三维位置；已经到过的地点和观察过的方向；地图中仍未探索的区域；目标候选、复核结果和历史负证据。这些信息不会随单帧结束而清空，而是组成持续世界模型，使机器人能够减少重复搜索，并根据新证据随时调整下一步计划。",
        0.58,
        3.77,
        4.18,
        1.55,
        accent=PURPLE,
        fill=PALE_PURPLE,
        body_size=9.0,
    )
    title_body_box(
        slide,
        "最终可得到的结果",
        "系统会给出目标是否找到、目标类别/属性/关系是否满足、目标三维位置和最终观察距离；同时保存机器人搜索过的区域、每一步选择的观察位置及理由、规划路径、真实运动结果和安全拒绝原因。若目标不存在或暂时找不到，也会明确区分搜索耗尽、时间到达上限、地图不可用和人工停止，而不是只返回笼统的“失败”。",
        0.58,
        5.51,
        4.18,
        1.44,
        accent=TEAL,
        fill=PALE_TEAL,
        body_size=9.0,
    )

    add_box(slide, 4.99, 2.12, 7.76, 4.83, fill=WHITE, line=LINE)
    add_text(slide, "机器人如何完成一场搜索", 5.25, 2.31, 7.23, 0.37, size=15.3, color=NAVY, bold=True, align=PP_ALIGN.CENTER)
    steps = [
        ("1", "理解任务", "把自然语言拆成目标类别、属性、空间关系和锚点。", BLUE, PALE_BLUE),
        ("2", "观察环境", "获取彩色图、深度、机器人位姿和当前地图。", TEAL, PALE_TEAL),
        ("3", "识别并定位物体", "开放词汇模型识别物体，RGB-D 将二维检测转换为三维位置。", PURPLE, PALE_PURPLE),
        ("4", "更新空间记忆", "把不同帧中的同一物体合并，并记录地点、关系和未探索区域。", AMBER, PALE_AMBER),
        ("5", "选择下一观察点", "结合目标语义、未知区域、路线距离和历史失败进行排序。", BLUE, "EDF3FF"),
        ("6", "安全移动并重新观察", "路径和安全检查通过后执行；移动结束后使用新画面重新判断。", RED, PALE_RED),
    ]
    for i, (num, title, body, accent, fill) in enumerate(steps):
        col = i % 2
        row = i // 2
        numbered_step(slide, num, title, body, 5.28 + col * 3.57, 2.88 + row * 1.12, 3.30, 0.93, accent=accent, fill=fill)

    add_box(slide, 5.28, 6.35, 6.87, 0.37, fill=NAVY, line=NAVY)
    add_text(slide, "核心闭环：观察 → 理解 → 选择位置 → 安全移动 → 获得新视角 → 再次观察", 5.48, 6.39, 6.47, 0.27, size=10.1, color=WHITE, bold=True, align=PP_ALIGN.CENTER)

    footer(slide, "项目重点不是“识别一次”，而是让机器人在目标暂时不可见时仍能主动、连续、可解释地搜索。", 1)
    set_notes(slide, "本页正文可独立说明项目是什么、和普通目标检测有何不同，以及一场搜索的六个阶段。")


def slide_two(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "02", "总体架构：四个核心模块如何形成闭环", "每个模块分别解决“理解、看见、记住、行动”")

    modules = [
        (
            "① 任务理解模块｜明确要找什么",
            "将自然语言转换为“目标画像”：目标类别、中文/英文别名、颜色/材质、其他属性、空间关系和关系锚点。例如把“蓝色垃圾桶在饮水机旁边”表示为目标节点、锚点节点和 NEAR 关系。该结构一方面生成开放词汇视觉模型需要的检测词，另一方面作为后续判断标准：只有类别、属性和关键关系得到视觉支持时，目标才会进入复核或接近阶段。",
            BLUE,
            PALE_BLUE,
        ),
        (
            "② 感知与建图模块｜看见物体并知道自己在哪里",
            "RGB-D 相机提供彩色图和深度；开放词汇视觉模型识别不局限于固定类别的物体；激光雷达、IMU 和里程计提供机器人运动与环境几何。系统对图像、深度、位姿和地图进行时间与坐标对齐，再将二维目标转换到地图三维坐标。与此同时维护二维占据地图、三维全局点云和机器人位姿，并持续检查数据是否新鲜、坐标变换是否有效，避免使用移动前或旧地图中的过期证据。",
            TEAL,
            PALE_TEAL,
        ),
        (
            "③ 世界模型与搜索决策模块｜记住看过什么并决定下一步",
            "系统把瞬时检测合并成跨帧持久物体，为每个对象保存类别、属性、三维位置、出现次数和最近观测时间；同时记录物体所在地点、物体间关系、已经观察的方向、仍未探索的 Frontier 以及路线成功/失败历史。目标不在当前画面时，系统根据目标或锚点的语义相关性、未知区域的信息量、是否能获得新视角、路线距离和负证据，选择最值得前往的下一观察位置，并在运动后用新观测更新排序。",
            PURPLE,
            PALE_PURPLE,
        ),
        (
            "④ 导航与安全执行模块｜把高层目标变成安全动作",
            "高层决策只给出“希望去哪里”和“为什么去”，不会直接控制电机。系统先围绕目标或 Frontier 生成多个可观察、可站立的候选位姿，再由 Nav2 计算真实可行路径。执行前检查终点占用、通道宽度、机身转向扫过区域、定位/地图/雷达新鲜度、运动授权和急停状态；运动过程中由碰撞监控和命令看门狗持续保护。任意条件不满足，机器人保持停止，并根据拒绝原因更换候选、等待地图或请求人工处理。",
            AMBER,
            PALE_AMBER,
        ),
    ]
    for i, (title, body, accent, fill) in enumerate(modules):
        col = i % 2
        row = i // 2
        title_body_box(slide, title, body, 0.58 + col * 6.14, 1.17 + row * 2.18, 5.89, 1.93, accent=accent, fill=fill, title_size=12.7, body_size=9.2)

    add_text(slide, "三类地图各自负责什么？", 0.62, 5.65, 3.10, 0.35, size=15.2, color=NAVY, bold=True)
    map_specs = [
        (
            "二维占据地图",
            "用栅格标记自由、障碍和未知区域，负责定位、Frontier 提取、可达性判断和 Nav2 路径规划。",
            BLUE,
            PALE_BLUE,
        ),
        (
            "三维全局点云",
            "表达固定世界中的三维几何结构，用于空间展示、环境复核和物体位置对照；局部扫描不能冒充全局地图。",
            TEAL,
            PALE_TEAL,
        ),
        (
            "语义对象与拓扑图",
            "保存持久物体、地点、空间关系、观察方向、访问/失败记录和 Frontier，服务于任务推理，但不替代几何地图。",
            PURPLE,
            PALE_PURPLE,
        ),
    ]
    for i, (title, body, accent, fill) in enumerate(map_specs):
        title_body_box(slide, title, body, 0.58 + i * 4.08, 6.06, 3.82, 0.89, accent=accent, fill=fill, title_size=10.8, body_size=7.7)

    footer(slide, "设计原则：语义模块回答“应该去哪里”，几何地图回答“能否到达”，安全控制回答“现在是否允许行动”。", 2)
    set_notes(slide, "本页用四个核心模块解释总体架构，并说明二维地图、三维点云和语义图为什么同时存在。")


def slide_three(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "03", "关键实现：从自然语言和图像构建持续更新的语义地图", "目标理解、开放词汇感知、三维定位与空间记忆")

    title_body_box(
        slide,
        "1｜自然语言如何变成可搜索目标",
        "系统首先建立“目标画像”，把目标类别、别名、颜色、材质、功能属性和空间关系分开保存；再建立目标关系图。例如“饮水机旁边的蓝色垃圾桶”包含垃圾桶目标、蓝色属性、饮水机锚点和 NEAR 关系。这样既能检测垃圾桶，也能在只看到饮水机时判断该区域值得继续观察。",
        0.58,
        1.17,
        4.00,
        1.46,
        accent=BLUE,
        fill=PALE_BLUE,
        body_size=9.0,
    )
    title_body_box(
        slide,
        "2｜视觉模型如何分工",
        "RAM++ 低频给出当前画面可能包含的普通类别，用于扩大场景词表；用户目标不会依赖 RAM++ 是否识别到。GroundingDINO 根据文本定位开放词汇物体，SAM2 生成更精确的实例轮廓，D435 提供深度。VLM 处理复杂颜色、状态、关系和难例复核，并在基础检测连续漏检时低频兜底。不同路线按配置使用，避免所有模型每帧串行造成过高延迟。",
        0.58,
        2.82,
        4.00,
        1.78,
        accent=TEAL,
        fill=PALE_TEAL,
        body_size=8.9,
    )
    title_body_box(
        slide,
        "3｜快速目标判断与完整场景理解",
        "快速通路优先回答“任务目标是否在当前画面”，保证机器人能及时作出搜索决策；完整场景通路在后台补充普通物体、属性和关系，用于更新长期语义地图。慢速结果会绑定来源帧、采集时间和机器人位姿；如果结果过期、任务已经更换或机器人已经移动，就不会被当作当前证据使用。",
        0.58,
        4.79,
        4.00,
        2.16,
        accent=PURPLE,
        fill=PALE_PURPLE,
        body_size=8.9,
    )

    title_body_box(
        slide,
        "4｜二维检测如何转换为真实三维位置",
        "RGB-D 相机为每个像素提供深度。系统从目标框或实例轮廓中选取有效深度，通过相机内参将像素反投影到相机三维坐标，再利用相机外参和机器人位姿转换到地图坐标。每个结果保存来源帧、拍摄时位姿、坐标系和质量等级；深度或坐标变换不可靠时，只保留视觉证据，不虚构精确地图位置。",
        4.79,
        1.17,
        3.78,
        1.78,
        accent=AMBER,
        fill=PALE_AMBER,
        body_size=8.8,
    )
    title_body_box(
        slide,
        "5｜如何把多帧观测变成持久对象",
        "机器人移动后，同一物体会以不同大小、角度和位置再次出现。系统综合类别、属性、三维距离、机器人位姿和观测质量，将相邻帧中的检测合并为具有稳定 ID 的对象，并记录其位置、出现次数、最近观测时间和目标状态。动态物体、低置信度结果以及地面、机身等非目标区域会被过滤，避免污染长期地图。",
        4.79,
        3.14,
        3.78,
        1.78,
        accent=TEAL,
        fill=PALE_TEAL,
        body_size=8.8,
    )
    title_body_box(
        slide,
        "6｜Place–Object–Frontier 语义拓扑",
        "Place 表示机器人实际到过并进行观察的地点；Object 表示跨帧稳定物体；Frontier 表示已知自由空间和未知空间的边界，也就是值得继续探索的入口。地点之间记录连通和失败历史，物体挂接到观察地点，物体之间保存邻近、左右、包含等关系。系统因此能够知道“在哪里看到了什么、还有哪些方向没看、哪些区域走不通”。",
        4.79,
        5.11,
        3.78,
        1.84,
        accent=PURPLE,
        fill=PALE_PURPLE,
        body_size=8.7,
    )

    add_box(slide, 8.78, 1.17, 3.97, 5.78, fill=WHITE, line=LINE)
    add_text(slide, "目标证据为什么分成四种状态？", 9.04, 1.38, 3.45, 0.37, size=14.0, color=NAVY, bold=True, align=PP_ALIGN.CENTER)
    states = [
        ("PRESENT｜明确存在", "检测、属性/关系和复核形成明确正证据，可以进入目标确认或接近阶段。", GREEN, PALE_GREEN),
        ("POSSIBLE｜可能存在", "出现疑似候选，但证据不足；机器人先停稳并使用新帧再次观察。", AMBER, PALE_AMBER),
        ("ABSENT｜本轮明确否定", "复核已经完成并判断不是目标，可在本轮释放该候选并继续搜索。", BLUE, PALE_BLUE),
        ("UNKNOWN｜没有结论", "模型超时、异常或结果仍在途中；既不能说目标不存在，也不能据此授权运动。", RED, PALE_RED),
    ]
    for i, (title, body, accent, fill) in enumerate(states):
        title_body_box(slide, title, body, 9.04, 1.95 + i * 1.02, 3.45, 0.84, accent=accent, fill=fill, title_size=10.4, body_size=7.8)

    add_box(slide, 9.04, 6.18, 3.45, 0.55, fill=NAVY, line=NAVY)
    add_text(slide, "关键原则：没有得到结论，不等于目标不存在；旧帧证据不能控制新位置上的机器人。", 9.18, 6.25, 3.17, 0.37, size=8.2, color=WHITE, bold=True, align=PP_ALIGN.CENTER)

    footer(slide, "最终得到的不是一组临时检测框，而是一张包含物体、位置、关系、来源和时效的持续语义地图。", 3)
    set_notes(slide, "本页以通俗方式解释目标画像、模型分工、三维定位、跨帧关联、语义拓扑和四状态证据。")


def slide_four(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "04", "关键实现：机器人如何决定下一步，并保证真实运动安全", "主动探索、观察视点、Nav2 执行、WebUI 与当前进展")

    title_body_box(
        slide,
        "1｜目标不在画面时，下一步去哪？",
        "系统首先从地图中提取 Frontier——已知自由空间与未知区域的边界；同时考虑尚未观察的方向、关系锚点附近、目标最后出现位置和已知高价值地点。每个候选根据五类因素排序：与目标的语义相关性、能看到多少未知区域、是否提供新视角、路线长度与转向代价、历史访问/失败/负证据。可以概括为：优先去“最可能获得新证据且能够安全到达”的位置，而不是简单去最近点。",
        0.58,
        1.17,
        4.02,
        2.18,
        accent=PURPLE,
        fill=PALE_PURPLE,
        body_size=8.9,
    )
    title_body_box(
        slide,
        "2｜为什么还需要观察视点规划？",
        "物体中心和 Frontier 中心不一定适合作为机器人终点。系统会在不同观察距离、侧向偏移和朝向上生成多个候选，并检查机身宽度、路径净空、未知区域、相机视场角、RGB-D 有效距离和遮挡。目标确认后，机器人只接近到安全观察距离，再获取新鲜深度和最终视觉证据，不会直接朝物体坐标冲过去。",
        0.58,
        3.54,
        4.02,
        1.67,
        accent=TEAL,
        fill=PALE_TEAL,
        body_size=8.9,
    )
    title_body_box(
        slide,
        "3｜搜索状态机",
        "启动检查 → 获取观察 → 目标匹配 → 候选复核 → 更新记忆 → 生成候选 → 规划与执行 → 停稳后重新观察。结束原因明确区分为：找到目标、搜索区域耗尽、时间/步数达到上限、感知或规划失败、地图不可用，以及操作员停止。异常不会被包装成“搜索成功”。",
        0.58,
        5.40,
        4.02,
        1.55,
        accent=BLUE,
        fill=PALE_BLUE,
        body_size=8.7,
    )

    title_body_box(
        slide,
        "4｜从高层目标到 Go2-W 真实运动",
        "高层模块只给出地图坐标中的目标位置。Nav2 先计算是否存在可行路径；接近原地转向的目标使用专门旋转动作，普通目标使用路径跟踪。执行前依次检查终点占用、通道净空、机器人转向扫过区域、地图和定位新鲜度、LiDAR 状态、碰撞监控、控制权、运动授权和急停。系统默认只规划不运动；任一条件失败就保持停止，这种策略称为 fail-closed。",
        4.79,
        1.17,
        3.78,
        2.18,
        accent=RED,
        fill=PALE_RED,
        body_size=8.8,
    )
    title_body_box(
        slide,
        "5｜卡死、失败和恢复怎么处理？",
        "系统使用机器人真实位移判断是否真正移动，而不是只看命令是否返回成功。连续原地转动、重复观察同一视角或多次路径失败会触发重新规划。恢复动作前必须停车并重新检查周围净空；只有安全证据充分时才允许有限倒车或小角度转向，没有可靠雷达证据时不执行恢复。路径预检失败只更换候选，不进行不必要的实体动作。",
        4.79,
        3.54,
        3.78,
        1.85,
        accent=AMBER,
        fill=PALE_AMBER,
        body_size=8.8,
    )
    title_body_box(
        slide,
        "6｜WebUI 能看到什么？",
        "网页集中展示自然语言任务、实时相机、目标状态、场景物体、三维点云、机器人位姿、语义拓扑、Frontier、候选排序、选择理由、规划/执行结果和安全状态；支持暂停、恢复、停止和急停。历史任务可回放地图、轨迹、事件和最终结果，使每一步决策和运动都有证据可查。",
        4.79,
        5.58,
        3.78,
        1.37,
        accent=BLUE,
        fill=PALE_BLUE,
        body_size=8.5,
    )

    add_box(slide, 8.78, 1.17, 3.97, 5.78, fill=WHITE, line=LINE)
    add_text(slide, "当前项目进展", 9.03, 1.38, 3.47, 0.38, size=15.0, color=NAVY, bold=True, align=PP_ALIGN.CENTER)
    title_body_box(
        slide,
        "已经完成",
        "自然语言目标画像；开放词汇视觉感知；RGB-D 三维定位；二维/三维地图；持久对象、Place 和 Frontier；语义候选排序与视点规划；Nav2 接口和多层安全门控；WebUI、事件记录和搜索回放。",
        9.03,
        1.96,
        3.47,
        1.36,
        accent=GREEN,
        fill=PALE_GREEN,
        title_size=11.3,
        body_size=8.2,
    )
    title_body_box(
        slide,
        "已有验证",
        "大量离线回归测试已经覆盖任务解析、语义图、Frontier、导航候选和安全逻辑；真机上已完成部分短距离 Nav2、保存地图重定位、定位健康和保护停车验证，证明主要软件链路和安全链能够工作。",
        9.03,
        3.51,
        3.47,
        1.34,
        accent=BLUE,
        fill=PALE_BLUE,
        title_size=11.3,
        body_size=8.2,
    )
    title_body_box(
        slide,
        "仍需继续验收",
        "连续通过多个 Frontier 的长时间闭环搜索；真实目标的发现—复核—接近—最终确认；关系目标和多同类干扰目标的系统实验；跨视角对象合并精度和跨场景成功率。因此当前应表述为“已形成真实 Go2-W 上可运行、可解释、可回放的自主语义搜索原型”，而不是“已经完成无人值守大场地搜索”。",
        9.03,
        5.04,
        3.47,
        1.67,
        accent=AMBER,
        fill=PALE_AMBER,
        title_size=11.3,
        body_size=7.9,
    )

    footer(slide, "系统的核心原则：高层语义负责提出目标，几何规划负责证明可达，实时安全门负责决定是否允许执行。", 4)
    set_notes(slide, "本页说明下一观察点选择、视点规划、Nav2 安全链、恢复、WebUI 和当前进展。")


def validate(prs: Presentation) -> None:
    if len(prs.slides) != 4:
        raise RuntimeError(f"expected four slides, got {len(prs.slides)}")
    width, height = int(prs.slide_width), int(prs.slide_height)
    for slide_number, slide in enumerate(prs.slides, 1):
        text_chars = 0
        for shape_number, shape in enumerate(slide.shapes, 1):
            if shape.left < 0 or shape.top < 0:
                raise RuntimeError(f"slide {slide_number} shape {shape_number}: negative origin")
            if shape.left + shape.width > width + 2 or shape.top + shape.height > height + 2:
                raise RuntimeError(f"slide {slide_number} shape {shape_number}: outside slide")
            if getattr(shape, "has_text_frame", False):
                text_chars += len(shape.text.strip())
        if text_chars < 650:
            raise RuntimeError(f"slide {slide_number} too sparse: {text_chars}")


def build() -> Path:
    prs = Presentation()
    prs.slide_width = Inches(SLIDE_W)
    prs.slide_height = Inches(SLIDE_H)
    prs.core_properties.title = "GO2W 自主语义搜索——老师易懂专业版"
    prs.core_properties.subject = "四页项目架构与实现说明"
    prs.core_properties.author = "robotscene project"
    prs.core_properties.comments = "面向独立阅读：强调是什么、为什么和如何实现，不包含内部接口与源码路径。"
    slide_one(prs)
    slide_two(prs)
    slide_three(prs)
    slide_four(prs)
    validate(prs)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    prs.save(OUT)
    return OUT


if __name__ == "__main__":
    print(build())
