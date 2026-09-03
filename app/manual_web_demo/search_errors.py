"""Structured, operator-facing errors for autonomous search.

The robot worker still transports a human readable ``message`` for backward
compatibility.  This module adds a stable code/category and an actionable
Chinese explanation so the WebUI never has to guess why a task failed.
"""

from __future__ import annotations

import re
import time
from typing import Any


def search_error(
    message: Any,
    *,
    code: str | None = None,
    source: str = "search_service",
    stage: str | None = None,
    detail: str | None = None,
    recoverable: bool | None = None,
) -> dict[str, Any]:
    text = str(message or "未知搜索错误").strip()
    lowered = text.lower()
    category = "internal"
    title = "搜索执行异常"
    cause = "搜索链路返回了未分类异常。"
    suggestion = "查看搜索日志与本次会话事件，确认首个 ERROR 事件后重试。"
    inferred_code = "SEARCH_INTERNAL_ERROR"
    inferred_recoverable = True
    log_ref = "outputs/autonomous_search/logs/search_worker.log"

    rules = [
        (r"未找到.*搜索记录|超过最近.*保留|session.*not found", "history", "搜索记录不可用",
         "指定会话不存在，或已超出最近 10 次的滚动保留范围。", "从“最近搜索记录”中选择仍在保留的会话。",
         "SESSION_NOT_FOUND", False),
        (r"required|too long|unsupported|不可执行|任务理解失败", "request", "任务输入或理解失败",
         "自然语言任务未通过输入校验或任务理解。", "修改任务描述，使目标、属性和空间关系更明确后重试。",
         "TASK_INVALID", True),
        (r"already active|control.*owner|manual.*active|占用|emergency_stop|estop", "control", "控制权冲突",
         "当前存在搜索、手动控制或急停锁存，新的自主任务不能取得控制权。",
         "停止占用任务；若为急停，请确认现场安全后先解除急停。", "CONTROL_CONFLICT", True),
        (r"no module named|modulenotfounderror|importerror", "dependency", "Worker 依赖缺失",
         "真机搜索进程缺少 Python/ROS 运行依赖。", "检查 worker 使用的解释器及缺失模块，再重新启动 WebUI。",
         "WORKER_DEPENDENCY_MISSING", False),
        (r"rclpy|ros2|dds|topic|service unavailable|action server", "ros", "ROS2 通信异常",
         "ROS2 节点、Topic、Action 或 DDS 网络未就绪。", "检查 ROS_DOMAIN_ID、网卡、DDS 配置和真机节点状态。",
         "ROS_UNAVAILABLE", True),
        (r"camera|d435|rgb-?d|frame.*stale|相机|深度", "perception", "相机 / RGB-D 异常",
         "搜索没有获得新鲜且可用的视觉或深度帧。", "检查 D435 供电、HTTP/ROS 图像流、帧年龄和标定服务。",
         "CAMERA_UNAVAILABLE", True),
        (r"mapping frozen|地图已冻结|假平移|lio.*drift|建图.*冻结", "spatial", "全局地图已冻结",
         "LIO 位姿与轮式里程计不一致（原地转向被判为米级假平移），永久地图已停止写入。",
         "先停止运动，运行 scripts/go2w/reset_plain_slam_session.sh 重建 mapping session；"
         "确认 mapping_health=HEALTHY 后再开始自主搜索。",
         "MAPPING_FROZEN", True),
        (r"lidar|pandar|rtab|spatial|occupancy|定位|雷达", "spatial", "空间感知或定位异常",
         "激光雷达、RTAB-Map 或空间坐标链路不可用。", "检查点云时间戳、TF、RTAB-Map 状态和定位质量。",
         "SPATIAL_PROVIDER_UNAVAILABLE", True),
        (r"collision|clearance|safety|robot_mode|error_code|motion|导航失败|不可达", "motion", "运动安全门禁拒绝",
         "机器人状态、避障净空或运动执行结果不满足安全条件。", "先停车并检查 robot_mode、error_code、雷达净空和运动 Action。",
         "MOTION_SAFETY_REJECTED", True),
        (r"llm|siliconflow|openai|api key|http 4\d\d|http 5\d\d|timeout.*model", "reasoning", "大模型服务异常",
         "任务理解、视觉推理或语义决策模型调用失败。", "检查 API Key、模型名、网络和服务配额；保留本次错误后重试。",
         "LLM_UNAVAILABLE", True),
        (r"timeout|max_steps|max_planning|budget|超时", "budget", "搜索预算结束",
         "搜索达到时间、规划轮次或运动步数上限。", "查看已探索区域和最后决策；必要时提高搜索预算后重新搜索。",
         "SEARCH_BUDGET_EXHAUSTED", True),
        (r"worker.*exit|worker.*gone|interrupted|restart|进程.*退出|服务.*重启", "worker", "搜索进程意外中断",
         "Web 服务或搜索 worker 在产生正常结束事件前退出。", "确认 worker 日志中的退出原因，然后从新任务重新开始。",
         "WORKER_INTERRUPTED", True),
    ]
    for pattern, cat, ttl, why, fix, rule_code, can_retry in rules:
        if re.search(pattern, lowered, flags=re.IGNORECASE):
            category, title, cause, suggestion = cat, ttl, why, fix
            inferred_code, inferred_recoverable = rule_code, can_retry
            break

    return {
        "schema_version": "search_error_v1",
        "code": str(code or inferred_code),
        "category": category,
        "title": title,
        "message": text,
        "detail": str(detail or text),
        "cause": cause,
        "suggestion": suggestion,
        "source": source,
        "stage": stage or "",
        "recoverable": inferred_recoverable if recoverable is None else bool(recoverable),
        "log_ref": log_ref,
        "timestamp": time.time(),
    }
