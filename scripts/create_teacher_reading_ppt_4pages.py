#!/usr/bin/env python3
"""Generate a four-slide, self-contained GO2W technical reading deck.

Unlike a speaker-led presentation, every slide contains enough explanatory
text to be understood independently by a reviewer reading the PPT alone.
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
    add_rich_text,
    add_text,
    rgb,
    set_bg,
    set_notes,
)


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs" / "ppt" / "GO2W_自主语义搜索_老师独立阅读版_超详细4页.pptx"


def footer(slide, text: str, page: int) -> None:
    add_text(slide, text, 0.58, 7.14, 11.52, 0.23, size=7.2, color="738396")
    add_text(slide, f"{page}/4", 12.08, 7.12, 0.64, 0.25, size=8.2, color=MUTED, align=PP_ALIGN.RIGHT)


def section_box(
    slide,
    title: str,
    body: str,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    accent: str,
    fill: str = WHITE,
    title_size: float = 11.5,
    body_size: float = 8.6,
    title_h: float = 0.35,
) -> None:
    add_box(slide, x, y, w, h, fill=fill, line=LINE)
    bar = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE,
        Inches(x),
        Inches(y),
        Inches(0.08),
        Inches(h),
    )
    bar.fill.solid()
    bar.fill.fore_color.rgb = rgb(accent)
    bar.line.fill.background()
    add_text(slide, title, x + 0.18, y + 0.08, w - 0.30, title_h, size=title_size, color=accent, bold=True, valign=MSO_ANCHOR.TOP)
    add_text(
        slide,
        body,
        x + 0.18,
        y + 0.10 + title_h,
        w - 0.31,
        h - title_h - 0.18,
        size=body_size,
        color=INK,
        valign=MSO_ANCHOR.TOP,
        line_spacing=1.0,
    )


def layer_row(
    slide,
    index: str,
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
    number = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x + 0.16), Inches(y + 0.15), Inches(0.40), Inches(0.40))
    number.fill.solid()
    number.fill.fore_color.rgb = rgb(accent)
    number.line.fill.background()
    add_text(slide, index, x + 0.17, y + 0.15, 0.38, 0.36, size=9.3, color=WHITE, bold=True, align=PP_ALIGN.CENTER)
    add_text(slide, title, x + 0.68, y + 0.10, 1.72, 0.33, size=11.4, color=accent, bold=True)
    add_text(slide, body, x + 2.30, y + 0.10, w - 2.48, h - 0.18, size=8.15, color=INK, valign=MSO_ANCHOR.TOP, line_spacing=1.0)


def compact_label(slide, label: str, text: str, x: float, y: float, w: float, *, accent: str, fill: str) -> None:
    add_box(slide, x, y, w, 0.40, fill=fill, line=fill)
    add_text(slide, label, x + 0.10, y + 0.04, 1.05, 0.25, size=8.1, color=accent, bold=True)
    add_text(slide, text, x + 1.08, y + 0.04, w - 1.18, 0.27, size=7.3, color=INK, align=PP_ALIGN.RIGHT)


def slide_one(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "01", "项目概述：自然语言驱动的 Go2-W 自主语义搜索系统", "独立阅读版｜项目目标、功能范围与完整闭环")

    add_box(slide, 0.58, 1.15, 12.17, 0.72, fill=NAVY, line=NAVY)
    add_text(slide, "系统定义", 0.80, 1.28, 1.05, 0.31, size=13.0, color="84DBF1", bold=True, align=PP_ALIGN.CENTER)
    add_text(
        slide,
        "本项目不是单帧目标检测 Demo，而是一套把任务理解、视觉感知、空间地图、语义记忆、主动探索、Nav2 导航、安全控制和运行证据串联起来的真实机器人闭环。",
        1.94,
        1.23,
        10.46,
        0.42,
        size=13.4,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )

    # Left: what the system does and its functional scope.
    section_box(
        slide,
        "1｜任务输入与系统目标",
        "用户输入完整自然语言任务，例如“寻找办公椅”或“寻找饮水机旁边的蓝色垃圾桶”。系统将句子分解为目标类别、颜色/材质等属性、空间关系、关系锚点和任务约束，形成 TargetProfile 与 GoalGraph。最终目标不是在画面上画框，而是在真实环境中持续搜索、验证正确实例、选择安全观察位置，并输出任务结果及可追溯证据。",
        0.58,
        2.05,
        3.87,
        1.48,
        accent=BLUE,
        fill=PALE_BLUE,
        body_size=8.35,
    )
    section_box(
        slide,
        "2｜系统具备的主要能力",
        "• 自然语言开放词汇目标搜索，支持类别、颜色、属性及 NEAR/LEFT_OF/INSIDE 等关系。\n• RGB-D 目标检测、实例分割、深度定位和地图坐标转换。\n• 三维全局点云、二维占据地图、机器人位姿与地图健康管理。\n• 跨帧对象关联、Place 地点图、Frontier 未知边界及物体关系图。\n• 语义相关候选排序、视觉友好视点规划、Nav2 路径规划与安全执行。\n• WebUI 状态监控、暂停/恢复/停止/急停、会话归档和轨迹视频回放。",
        0.58,
        3.66,
        3.87,
        2.02,
        accent=TEAL,
        fill=PALE_TEAL,
        body_size=8.0,
    )
    section_box(
        slide,
        "3｜输入、内部状态与最终输出",
        "输入：自然语言、RGB/深度、点云、IMU、里程计、TF、地图、历史观测和传感器健康。内部状态：目标状态、持久对象、Place、Frontier、负证据、失败历史、地图版本和搜索预算。输出：目标与三维位置、下一观察点及理由、Nav2 路径、安全拒绝、实测运动和完整会话档案。",
        0.58,
        5.84,
        3.87,
        1.14,
        accent=PURPLE,
        fill=PALE_PURPLE,
        title_size=10.4,
        body_size=7.4,
        title_h=0.27,
    )

    # Right: detailed end-to-end loop.
    add_box(slide, 4.68, 2.05, 8.07, 4.93, fill=WHITE, line=LINE)
    add_text(slide, "端到端搜索闭环：每轮都使用新观测重新判断，不沿用旧运动指令", 4.94, 2.22, 7.55, 0.35, size=14.2, color=NAVY, bold=True, align=PP_ALIGN.CENTER)
    stages = [
        (
            "① 任务理解",
            "TargetProfileResolver 解析目标类别、别名、颜色、材质、属性、关系约束和锚点；GoalGraphBuilder 将“目标—属性—锚点—关系”转成结构化目标图，为检测词表、图匹配和后续搜索提供统一语义。",
            BLUE,
            PALE_BLUE,
        ),
        (
            "② 视觉与空间观测",
            "获取当前 RGB-D 帧、采集时间和机器人位姿；快速通路判断任务目标，完整场景通路补充普通物体与关系；同时读取二维占据地图、三维点云、Frontier 和定位健康状态。",
            TEAL,
            PALE_TEAL,
        ),
        (
            "③ 世界模型更新",
            "检测结果通过深度反投影和 TF 转换到地图坐标；SemanticObjectMap 跨帧关联同一物体，PlaceGraph 记录地点与访问方向，SemanticNavigationGraph 保存 Place–Object–Frontier 及物体关系。",
            PURPLE,
            PALE_PURPLE,
        ),
        (
            "④ 目标匹配与主动决策",
            "系统区分 ZERO/PARTIAL/STRONG 图匹配以及 PRESENT/POSSIBLE/ABSENT/UNKNOWN 目标状态；未找到目标时，综合语义相关性、信息增益、路径代价、访问历史和负证据生成下一观察目标。",
            AMBER,
            PALE_AMBER,
        ),
        (
            "⑤ 路径预检与安全执行",
            "候选目标先经 Nav2 ComputePathToPose/Spin、终点占用、走廊净空、转向扫掠、地图/定位/LiDAR 新鲜度、控制授权、租约、急停和看门狗检查，全部通过后才允许进入运动链。",
            RED,
            PALE_RED,
        ),
        (
            "⑥ 反馈、复核与终止",
            "运动完成后停稳并重新观察；目标候选必须经新帧复核，必要时接近到安全观察距离。搜索最终以 TARGET_FOUND、SEARCH_EXHAUSTED、TIMEOUT、FAILED、OPERATOR_STOP 等明确原因结束并归档。",
            GREEN,
            PALE_GREEN,
        ),
    ]
    for i, (title, body, accent, fill) in enumerate(stages):
        y = 2.74 + i * 0.66
        add_box(slide, 4.96, y, 7.50, 0.55, fill=fill, line=fill)
        add_text(slide, title, 5.10, y + 0.08, 1.48, 0.27, size=9.1, color=accent, bold=True)
        add_text(slide, body, 6.48, y + 0.06, 5.83, 0.40, size=7.45, color=INK, valign=MSO_ANCHOR.TOP)
    add_box(slide, 4.96, 6.75, 7.50, 0.12, fill=NAVY, line=NAVY)
    add_text(slide, "闭环核心：看不到目标时不停止，而是利用地图与语义记忆主动获取更有价值的新视角。", 5.08, 6.63, 7.26, 0.32, size=8.3, color=NAVY, bold=True, align=PP_ALIGN.CENTER)

    footer(slide, "本页不依赖硬件照片：重点说明系统做什么、输入什么、维护什么状态，以及最终如何完成一场搜索。", 1)
    set_notes(slide, "本页为老师独立阅读设计，页面正文已经包含项目定位、能力范围、输入输出和六阶段闭环，不需要额外口头说明。")


def slide_two(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "02", "总体架构：任务、感知、世界模型、导航与控制如何连接", "模块职责、主调用链、地图分工与运行数据")

    # Five detailed layers.
    layer_row(
        slide,
        "1",
        "人机交互与会话层",
        "FastAPI/WebSocket WebUI 接收自然语言任务并展示相机、目标状态、三维地图、对象拓扑、Agent 候选与安全状态。SearchSessionService 负责会话 ID、启动/暂停/恢复/停止/急停、子进程生命周期、事件总线和历史归档；对外核心接口包括 /api/search/state、/readiness、/history 与 /api/slam/map3d。",
        0.58,
        1.17,
        8.62,
        0.94,
        accent=BLUE,
        fill=PALE_BLUE,
    )
    layer_row(
        slide,
        "2",
        "任务认知与编排层",
        "autonomous_search_worker 将 Web 参数转换为真机搜索参数，启动 run_semantic_exploration；TargetProfileResolver、GoalGraphBuilder、GraphMatcher 和 SemanticSearchController 处理任务语义；AutonomousExplorer 维护 BOOTSTRAP→OBSERVE→MATCH→VERIFY→UPDATE_MEMORY→PLAN→EXECUTE 状态机和终止原因。",
        0.58,
        2.22,
        8.62,
        0.94,
        accent=CYAN,
        fill="E9F7FC",
    )
    layer_row(
        slide,
        "3",
        "视觉感知与几何建图层",
        "视觉路线可配置：RAM++ 负责普通场景标签，GroundingDINO 负责开放词汇定位，SAM2 负责实例 Mask，D435 负责深度，VLM 负责复杂属性/关系与漏检兜底；并非所有模型每帧串行运行。几何侧由 PandarXT-16、IMU、轮式里程计、时间同步、外参、plain_slam/二维地图和 TF 提供点云、占据栅格与机器人位姿。",
        0.58,
        3.27,
        8.62,
        0.94,
        accent=TEAL,
        fill=PALE_TEAL,
    )
    layer_row(
        slide,
        "4",
        "世界模型与任务规划层",
        "SemanticObjectMap 维护持久对象；PlaceGraph 把真实位移组织为地点节点；FrontierExtractor 从 Free–Unknown 边界提取未知区域；SemanticNavigationGraph 统一表达地点、物体、关系和前沿。WorldStateSnapshot 将地图版本、候选、对象、搜索记忆和安全摘要固定为本轮推理输入，避免不同时间的数据被混用。",
        0.58,
        4.32,
        8.62,
        0.94,
        accent=PURPLE,
        fill=PALE_PURPLE,
    )
    layer_row(
        slide,
        "5",
        "Nav2 与安全执行层",
        "CandidateGoalGenerator、SemanticReasoner、ViewpointPlanner 和 TopologyRouteExecutor 产生地图目标；Nav2SearchBackend 完成真实路径预检、NavigateToPose/Spin 分流、执行状态与实际位移记录。速度还需经过 Collision Monitor、Velocity Smoother、控制仲裁、cmd_vel Bridge、运动授权、租约、急停、定位/LiDAR 新鲜度和 Motion Action Server。",
        0.58,
        5.37,
        8.62,
        0.94,
        accent=AMBER,
        fill=PALE_AMBER,
    )

    # Right: main chain, map responsibilities, persistence.
    add_box(slide, 9.43, 1.17, 3.32, 5.78, fill=WHITE, line=LINE)
    add_text(slide, "主调用链与关键文件", 9.69, 1.36, 2.82, 0.34, size=13.4, color=NAVY, bold=True, align=PP_ALIGN.CENTER)
    chain = [
        ("Web 服务", "app/manual_web_demo/web_server.py\nsearch_session_service.py", BLUE, PALE_BLUE),
        ("Worker", "scripts/go2w/autonomous_search_worker.py", CYAN, "E9F7FC"),
        ("真机装配", "scripts/go2w/run_semantic_exploration.py", TEAL, PALE_TEAL),
        ("搜索状态机", "app/live_robot/autonomous_explorer.py", PURPLE, PALE_PURPLE),
        ("导航后端", "app/navigation/nav2_search_backend.py", AMBER, PALE_AMBER),
    ]
    for i, (title, body, accent, fill) in enumerate(chain):
        y = 1.86 + i * 0.67
        add_box(slide, 9.70, y, 2.79, 0.53, fill=fill, line=fill)
        add_text(slide, title, 9.82, y + 0.06, 0.86, 0.24, size=8.4, color=accent, bold=True)
        add_text(slide, body, 10.63, y + 0.05, 1.72, 0.38, size=7.2, color=INK, align=PP_ALIGN.RIGHT, valign=MSO_ANCHOR.TOP)

    add_text(slide, "三类地图的职责不能混淆", 9.69, 5.30, 2.82, 0.30, size=11.1, color=RED, bold=True, align=PP_ALIGN.CENTER)
    compact_label(slide, "二维占据地图", "Nav2 定位、Frontier、路径与碰撞检查", 9.70, 5.68, 2.79, accent=BLUE, fill=PALE_BLUE)
    compact_label(slide, "三维全局点云", "pslam_map 固定世界展示与几何检查；局部扫描不能冒充全图", 9.70, 6.12, 2.79, accent=TEAL, fill=PALE_TEAL)
    compact_label(slide, "语义对象/拓扑", "目标推理、对象关系、访问记忆；不替代可通行地图", 9.70, 6.56, 2.79, accent=PURPLE, fill=PALE_PURPLE)

    add_box(slide, 0.58, 6.49, 8.62, 0.46, fill=NAVY, line=NAVY)
    add_text(
        slide,
        "运行证据按会话保存：task.json、events.jsonl、decisions.jsonl、summary.json、semantic_map.json、exploration_graph.json、world_manifest，以及 WebUI 状态/图快照与搜索回放素材。",
        0.78,
        6.55,
        8.20,
        0.29,
        size=8.0,
        color=WHITE,
        bold=True,
        align=PP_ALIGN.CENTER,
    )

    footer(slide, "系统采用数据流与控制流分离：语义层提供目标偏好，地图与安全层决定其是否能转化为真实动作。", 2)
    set_notes(slide, "本页正文已经给出五层架构、关键文件、地图分工和归档产物，便于老师不看代码也能理解模块间的真实连接关系。")


def slide_three(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "03", "关键模块：目标语义、视觉证据与空间世界模型", "从自然语言到可验证、可定位、可长期保存的对象证据")

    # Three dense columns.
    section_box(
        slide,
        "A｜目标画像、GoalGraph 与场景图匹配",
        "TargetProfileResolver 将原始任务解析为：canonical target、中文/英文别名、颜色、材质、属性、affordance、关系约束、上下文词和关系锚点。GoalGraphBuilder 创建 Target/Anchor/Context 节点及 NEAR、LEFT_OF、IN_FRONT_OF、INSIDE 等边。\n\nGraphMatcher 不只判断类别名：目标节点约占主要权重，属性支持会影响节点得分，显式关系必须在场景图中得到支持。匹配输出 ZERO、PARTIAL、STRONG：ZERO 表示当前视角无相关证据；PARTIAL 常表示看到了锚点或部分属性，应围绕相关区域重观察；STRONG 仍不能直接结束任务，必须进入现有视觉复核链。\n\n这一设计使关系约束贯穿“检测词表—场景匹配—锚点搜索—下一观察位置”，而不是只在最终界面展示。",
        0.58,
        1.17,
        4.00,
        2.82,
        accent=BLUE,
        fill=PALE_BLUE,
        body_size=8.1,
    )
    section_box(
        slide,
        "B｜快通路、完整语义通路与模型分工",
        "快速目标通路面向本轮搜索，优先回答“当前帧是否有任务目标候选”，输出目标框、得分和 PRESENT/POSSIBLE/ABSENT/UNKNOWN。完整场景通路面向长期世界模型，产生普通物体、属性与关系，并可异步更新。\n\nRAM++ 只提供当前视野可能出现的普通场景类别，用于扩大词表覆盖；用户目标和关系锚点由 TargetProfile 独立进入 GroundingDINO prompt，因此 RAM++ 漏词不能阻止任务目标被搜索。GroundingDINO 负责开放词汇框，SAM2 负责实例 Mask，D435 负责深度；VLM 处理复杂属性、关系、整帧兜底和候选裁剪复核。实际路线由配置选择，并不是所有模型每帧串行调用。",
        0.58,
        4.15,
        4.00,
        2.80,
        accent=TEAL,
        fill=PALE_TEAL,
        body_size=8.0,
    )

    section_box(
        slide,
        "C｜目标状态、异步结果与证据时效",
        "目标状态明确区分：PRESENT=有明确正证据；POSSIBLE=存在候选但需要新帧复核；ABSENT=复核链已得到终结性否定；UNKNOWN=超时、异常、结果在途或没有形成结论。UNKNOWN 不能降级成 ABSENT，也不能授权运动。\n\nAsyncSemanticObservationManager 以有限并发运行完整场景分析，合并过时待处理帧并保留 latest success；AsyncVerifier 使用请求序号、任务代次、目标身份和有界队列，拒绝任务切换前或客户端超时后的迟到结果。每项结果携带 frame_id、capture/completed timestamp、robot pose、semantic age、模型 source 和 spatial quality。\n\n主循环只在证据仍满足 TTL、来源帧身份匹配且采集位姿有效时复用结果；运动后必须等待新帧，避免把移动前的检测框投到移动后的地图位置。",
        4.78,
        1.17,
        3.78,
        2.82,
        accent=PURPLE,
        fill=PALE_PURPLE,
        body_size=7.85,
    )
    section_box(
        slide,
        "D｜RGB-D 三维定位、TF 与跨帧持久对象",
        "目标框或 Mask 内的有效深度 d 通过相机内参反投影：Z=d，X=(u−cx)Z/fx，Y=(v−cy)Z/fy；随后使用采集时刻的 T(map←base)·T(base←camera) 转换到地图坐标。结果保留 frame_id、transform source、深度方法和空间质量；TF 不可用或坐标系不一致时拒绝伪造地图坐标。\n\nSemanticObjectMap 根据类别/别名、目标身份、三维距离、位姿变化和观测质量进行跨帧关联，形成 persistent object ID；每个对象保存位置、协方差/质量、出现次数、最后观测时间、来源帧、状态及关系。动态物体、低置信度、地板/机身等非对象标签通过角色、移动性和持久性门控过滤，防止污染长期地图。",
        4.78,
        4.15,
        3.78,
        2.80,
        accent=AMBER,
        fill=PALE_AMBER,
        body_size=7.8,
    )

    section_box(
        slide,
        "E｜Place–Object–Frontier 语义导航图",
        "PlaceGraph 将连续机器人位姿聚合为稳定地点：原地转动只更新观察方向，发生有效平移才创建或重访 Place；Place 之间使用 CONNECTED_TO，保存距离、成功/失败和阻塞状态。\n\nSemanticObjectMap 中的物体通过 OBSERVED_FROM 挂接到观测 Place；物体之间保存 NEAR、LEFT_OF、ON、INSIDE 等关系。FrontierExtractor 从二维 Free–Unknown 边界产生候选，记录位置、规模、可达性、信息增益、访问次数与失败次数，并通过 FRONTIER_TO 与当前 Place 关联。\n\nSemanticNavigationGraph 因而可以同时回答：当前在哪、看过哪些方向、有哪些稳定物体、目标关系是否满足、哪些区域尚未探索、哪些路线失败过。它是任务推理图，不替代 Nav2 的几何可通行地图。",
        8.77,
        1.17,
        3.98,
        3.32,
        accent=PURPLE,
        fill=WHITE,
        body_size=7.85,
    )
    section_box(
        slide,
        "F｜地图版本、坐标系与 WorldStateSnapshot",
        "三维地图、二维地图和语义对象分别保存，但通过 frame_id、map_epoch、map_revision、graph/grid/semantic revision 和机器人位姿绑定。地图桥重启或 SLAM reset 会产生新 epoch；旧对象若 epoch 或 frame 不一致，只保留为历史证据，不允许叠加到新地图或生成导航目标。\n\n每轮规划前，WorldStateSnapshot 固化本轮地图版本、机器人位姿、Frontier、对象、搜索记忆、传感器健康和候选列表。Agent 与确定性规划器都读取同一快照，避免“候选来自旧地图、位姿来自新时刻、对象来自更早帧”的时序混用。",
        8.77,
        4.65,
        3.98,
        2.30,
        accent=RED,
        fill=PALE_RED,
        body_size=7.75,
    )

    footer(slide, "关键代码集中在 app/reasoning/semantic_navigation、app/perception、app/live_robot、app/spatial 与 run_semantic_exploration.py。", 3)
    set_notes(slide, "本页是感知与世界模型的详细说明，正文包括模型职责、证据四状态、异步时效、深度定位、持久对象、Place/Frontier 和地图版本绑定。")


def slide_four(prs: Presentation) -> None:
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    set_bg(slide)
    add_header(slide, "04", "关键模块：主动搜索决策、安全执行、WebUI 与验证状态", "从“下一步去哪”到“为何允许运动”及“结果如何被证明”")

    section_box(
        slide,
        "A｜AutonomousExplorer 搜索状态机与失败语义",
        "BOOTSTRAP 检查后端和传感器；OBSERVE 采集最新帧；MATCH 执行目标图匹配；VERIFY 对 POSSIBLE/STRONG 候选复核；UPDATE_MEMORY 写入对象、Place、关系、Frontier 和负证据；PLAN 生成并评分候选；EXECUTE/WAIT_RESULT 调用 Nav2 并记录请求/实测运动；失败时进入 RECOVER 或重新规划。\n\n终局原因被明确区分：TARGET_FOUND=目标确认并满足结束条件；SEARCH_EXHAUSTED=地图健康且无可达有价值 Frontier；MAP_UNAVAILABLE=地图/安全证据长期不可用；PERCEPTION_FAILURE/PLANNING_ERROR/BACKEND_FAILURE=对应模块失败；TIMEOUT/MAX_STEPS/BUDGET_EXHAUSTED=预算终止；OPERATOR_STOP=人工停止。系统不会把异常、地图冻结或预算耗尽伪装成“正常搜索完成”。",
        0.58,
        1.17,
        4.02,
        2.42,
        accent=BLUE,
        fill=PALE_BLUE,
        body_size=7.85,
    )
    section_box(
        slide,
        "B｜候选来源、长期意图与联合评分",
        "候选来自：二维地图 Frontier、Place 未观察方向、目标相关锚点、目标最后出现位置、已知 Object/Place 和失败后的替代区域。SemanticSearchController 根据目标图匹配与世界快照产生 EXPLORE_FRONTIER、INSPECT_ANCHOR_REGION、VERIFY_TARGET、APPROACH_TARGET 四类意图。\n\n候选评分同时考虑：目标/锚点语义相关性、未知区域信息增益、新颖性、观察记忆先验、Nav2 路径代价、转向成本、重复访问、历史导航失败与仍有效的负证据。其目的不是“去最近点”，而是选择在可通行前提下最可能获得新任务证据的位置。Frontier 的 NO_PATH、TOO_NARROW、GOAL_OCCUPIED 等几何拒绝会渐进惩罚该候选；授权或基础设施错误不会污染空间记忆。",
        0.58,
        3.76,
        4.02,
        2.42,
        accent=PURPLE,
        fill=PALE_PURPLE,
        body_size=7.8,
    )
    section_box(
        slide,
        "C｜视觉友好视点与目标接近",
        "围绕目标或 Frontier 生成不同距离、侧移和朝向候选，检查 footprint、净空、未知区、FOV、有效深度与遮挡。确认后只接近至 0.70–1.10 m 观察环，并要求新鲜深度、视线和最终复核；不会把物体中心直接作为终点。",
        0.58,
        6.19,
        4.02,
        0.76,
        accent=TEAL,
        fill=PALE_TEAL,
        title_size=9.8,
        body_size=7.2,
        title_h=0.24,
    )

    section_box(
        slide,
        "D｜Nav2 与 fail-closed 运动执行链",
        "Agent 只输出 map 坐标目标和决策来源，不能直接发电机速度。Nav2SearchBackend 先检查定位有效性、地图版本和后端服务，再调用 ComputePathToPose；近零平移目标根据路径长度与角度路由到原生 Spin，普通目标进入 NavigateToPose。执行前后保存目标、路径、请求运动、实测位移、到达误差、状态与拒绝原因。\n\n真实速度链还包括：终点占用、走廊宽度、机器人膨胀 footprint、转向扫掠、未知空间不可通行、LiDAR/定位/TF 新鲜度、Collision Monitor、Velocity Smoother、控制仲裁、cmd_vel Bridge、设备所有者授权、operator supervision、trial/goal 里程预算、控制租约、software arm、急停、机器人错误和命令看门狗。默认只规划；任一门失败即不发送运动。",
        4.80,
        1.17,
        3.78,
        2.82,
        accent=RED,
        fill=PALE_RED,
        body_size=7.75,
    )
    section_box(
        slide,
        "E｜卡死判断、重规划与安全恢复",
        "系统使用真实位移而非“命令返回成功”判断是否移动，并记录连续原地转向、重复视点、路径失败和感知信息增益。无新信息不等于搜索耗尽：首先区分地图不新鲜、机器人卡死、仍有可达 Frontier 或真实无可探索区域。\n\n恢复前必须停车并刷新局部 LiDAR 安全快照；只有后方净空、证据新鲜、回退距离和累计恢复预算满足时，才允许有限倒车或小角度转向。没有安全证据时 fail-closed，不执行恢复。路径预检 REJECTED 只切换候选，不记为实体导航失败，也不触发盲目恢复。",
        4.80,
        4.15,
        3.78,
        2.80,
        accent=AMBER,
        fill=PALE_AMBER,
        body_size=7.8,
    )

    section_box(
        slide,
        "F｜WebUI 展示、接口与会话证据",
        "WebUI 同时展示自然语言任务、当前状态机阶段、RGB/深度帧、目标状态、场景物体、三维全局点云、机器人位姿、对象拓扑、Frontier、候选排名、选择原因、路径预检、地图/定位/雷达健康和运动授权。操作员可以暂停、恢复、停止和急停；恢复后重新观察和规划，不恢复旧指令。\n\n关键接口：/api/search/state 返回小状态和唯一 graph；/api/search/readiness 返回搜索/运动前置条件；/api/slam/map3d 返回全局点云及通过 epoch/frame 校验的语义对象；/api/search/history/{session_id} 与 replay 接口提供历史任务、最终地图、事件、轨迹和视频。每场会话保存原始 events、decisions、summary、semantic_map、exploration_graph、world manifest 和 WebUI 快照。",
        8.78,
        1.17,
        3.97,
        2.82,
        accent=BLUE,
        fill=WHITE,
        body_size=7.55,
    )
    section_box(
        slide,
        "G｜当前完成度与必须保留的验证边界",
        "已实现：自然语言目标画像与 GoalGraph；Grounded-SAM/VLM/RGB-D 感知路线；三维点云、二维导航地图、持久对象、Place/Frontier 与语义图；语义候选评分、视点规划、Nav2 后端、安全门控、WebUI、事件归档和回放。现有报告记录完整 Python 回归 2707 passed、3 skipped、4 subtests passed，并有短距离 Nav2、保存地图重定位和安全保护链的现场证据。\n\n尚不能扩大表述：连续通过 2–3 个 Frontier 的长时间搜索、真实目标“发现—复核—接近—最终确认”、关系目标与多同类干扰任务、跨回环对象漂移和跨场景准确率仍需完整现场实验。因此项目应描述为“真实 Go2-W 上已形成可运行、可解释、可回放的自主语义搜索原型，完成部分真机验证”，不能描述为“无人看守大场地连续搜索已全面验收”。",
        8.78,
        4.15,
        3.97,
        2.80,
        accent=GREEN,
        fill=PALE_GREEN,
        body_size=7.55,
    )

    footer(slide, "安全与证据原则：每个运动目标都必须同时具有任务语义依据、地图可达依据、实时安全依据和可回放执行记录。", 4)
    set_notes(slide, "本页正文已经覆盖状态机、候选评分、视点规划、Nav2 安全链、恢复、WebUI 接口和当前验证边界，可独立阅读。")


def validate(prs: Presentation) -> None:
    if len(prs.slides) != 4:
        raise RuntimeError(f"expected four slides, got {len(prs.slides)}")
    width = int(prs.slide_width)
    height = int(prs.slide_height)
    for slide_no, slide in enumerate(prs.slides, 1):
        chars = 0
        for shape_no, shape in enumerate(slide.shapes, 1):
            if shape.left < 0 or shape.top < 0:
                raise RuntimeError(f"slide {slide_no} shape {shape_no}: negative origin")
            if shape.left + shape.width > width + 2 or shape.top + shape.height > height + 2:
                raise RuntimeError(f"slide {slide_no} shape {shape_no}: outside slide")
            if getattr(shape, "has_text_frame", False):
                chars += len(shape.text.strip())
        if chars < 1100:
            raise RuntimeError(f"slide {slide_no} does not contain enough reading content: {chars}")


def build() -> Path:
    prs = Presentation()
    prs.slide_width = Inches(SLIDE_W)
    prs.slide_height = Inches(SLIDE_H)
    prs.core_properties.title = "GO2W 自主语义搜索系统——老师独立阅读版"
    prs.core_properties.subject = "四页超详细项目架构与功能说明"
    prs.core_properties.author = "robotscene project"
    prs.core_properties.comments = "依据当前仓库源码、配置与验收报告生成；正文可脱离讲解独立阅读。"
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
