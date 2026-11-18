"""三维物体框的两道门：持久地图入口门 与 WebUI 显示门。

两道门刻意分开，判的东西不一样：

* **入口门**（`filter_persistent_observations`）在 `SemanticObjectMap` 之前，
  按单次观测的分数与标签决定"这条观测配不配成为持久实体"。它挡的是污染：
  2026-09-17 那轮 41 个持久 ID 里 30 个的分数低于 0.35，全是 GroundingDINO
  在 box_threshold=0.12 上捞回来的碎片框。**不动检测阈值** —— 目标搜索要的
  就是高召回，低分候选留在逐帧原始事件里供取证。
* **显示门**（`select_display_objects`）在对象生产者里，按**实体聚合后**的分数、
  确认状态与观测次数决定"这个实体配不配画在三维地图上"。它挡的是视觉噪声：
  分数尚可但只见过一次的颜色词、地面、角落。

在线只有一个投影函数 `build_object_snapshot`（2026-10-08 计划书 §2.3）：两个
对象生产者（自主搜索的 `AutonomousExplorer` 与遥控建图的语义侧车）各自把内存
`SemanticObjectMap` 的 `to_dict()` 交给它，显示门因此只有一份实现。**它不读
任何文件**；两个生产者读同一份轻量 `slam_map_meta.json` 取 `map_epoch`，读取端
按 `map_epoch` + `frame_id` 配对，对不上就一个都不画。

下面的 `load_object_records` 是从 `events.jsonl` 还原记录表的**离线**解析器，
只给取证回放（`scripts/go2w/replay_object_display_gate.py`）和归档迁移用。
在线地图桥不再调用它 —— 从 events 里挑"哪份物体属于当前地图"这条路
2026-09-17 已被实测证伪，不要加回来。

本模块**不依赖 rclpy**，也不读文件：入参就是普通 dict / 观测对象，出参就是
列表 + 统计。对象生产者与快照写端都是 ROS 节点，pytest 起不了，所以判定逻辑
必须住在这里。

判据与门限的出处：docs/GO2W_WebUI三维物体框中文统一与低置信过滤_整改实施计划_20260917.md
§5.1（入口门）与 §5.2（显示门）。
"""

from __future__ import annotations

import math
from typing import Any, Iterable

from app.detectors.vocabulary import (
    LABEL_ZH_FALLBACK,
    is_non_object_label,
    resolve_display_label,
)
from app.spatial.semantic_object_map import (
    ENTITY_CONFIRMED,
    ENTITY_STALE,
    ENTITY_TENTATIVE,
    valid_confidence,
)

# ---------------------------------------------------------------- 门限默认值
# 与 SAM2_MIN_BOX_SCORE 对齐：低于它的候选可以用于目标搜索与原始取证，
# 但不具备进入持久三维对象图的最低质量。
DEFAULT_INGEST_MIN_CONFIDENCE = 0.35
# 显示门（计划书 §5.2 的 A/B 两条）。
DEFAULT_CONFIRMED_MIN_CONFIDENCE = 0.45
DEFAULT_TENTATIVE_MIN_CONFIDENCE = 0.65
# 2026-09-18 真机验收后由 2 提到 3。检测器的 confidence 不是标定概率——被观测
# 137 次的垃圾桶只有 0.63 分，同一物体的另一件是 0.45 分，所以抬置信度会先删掉
# 证据最硬的物体；抬观测次数只删证据最薄的。实测 59 → 41 个，无 obs≥3 的物体被删。
DEFAULT_CONFIRM_MIN_OBSERVATIONS = 3
# 合法 AABB 的边长范围（米）。房间里的独立物体没有上 5 m 的；一旦超过，
# 多半是整面墙/整块地面的分割泄漏，画出来就是罩住半个房间的巨框。
MIN_BOX_SIZE_M = 1e-3
MAX_BOX_SIZE_M = 5.0

# 丢弃原因。是稳定的机器可读字符串 —— 统计事件与审计表都按它聚合，
# 改了就等于改了对外契约。
DROP_LOW_CONFIDENCE = "low_confidence"
DROP_NON_OBJECT_LABEL = "non_object_label"
DROP_INVALID_SCORE = "invalid_score"
DROP_INVALID_POSITION = "invalid_position"
DROP_MISSING_LABEL_ZH = "missing_label_zh"
DROP_BELOW_DISPLAY_THRESHOLD = "below_display_threshold"

DECISION_DISPLAY = "DISPLAY"
DECISION_TARGET_VERIFIED = "TARGET_VERIFIED"
DECISION_DROP = "DROP"


def _has_chinese(text: object) -> bool:
    value = str(text or "")
    return any("一" <= char <= "鿿" for char in value)


def _finite_xyz(record: dict) -> tuple[float, float, float] | None:
    """取合法三维坐标；缺一个或有一个不是有限数就返回 None。"""
    try:
        xyz = (float(record["x"]), float(record["y"]), float(record["z"]))
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in xyz):
        return None
    return xyz


def valid_box_size(size: object) -> tuple[float, float, float] | None:
    """AABB 边长必须是三个有限正数且在合理上限内。

    返回 None 表示**不能画框**（按既有逻辑降级成画点），不表示这个物体该丢 ——
    尺寸坏了是几何信息缺失，不是"它不存在"。伪框比没框更坏：一个 0 边长的框
    在 canvas 上就是一条线，一个 40 m 的框会罩住整个房间。
    """
    if not isinstance(size, (list, tuple)) or len(size) != 3:
        return None
    try:
        values = tuple(float(value) for value in size)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    if not all(MIN_BOX_SIZE_M < value <= MAX_BOX_SIZE_M for value in values):
        return None
    return values


# ------------------------------------------------------------------ 入口门
def filter_persistent_observations(
    observations: Iterable[Any],
    *,
    min_confidence: float = DEFAULT_INGEST_MIN_CONFIDENCE,
) -> tuple[list[Any], dict[str, int]]:
    """持久地图入口门：返回 (保留的观测, 各原因丢弃计数)。

    顺序即优先级，丢弃原因只记**第一个**命中的：
    非物体标签 → 分数非法 → 分数过低。先判标签是因为
    `purple` 这种词即使拿了 0.9 也不该进实体图，把它记成 low_confidence
    会掩盖真因。

    入参是 `ObjectSpatialObservation`（有 .label/.label_zh/.confidence），
    返回的仍是**同一批对象**、保持原顺序 —— 调用方拿它当
    `update_with_associations()` 的入参，下标必须在同一张表上连续。
    """
    kept: list[Any] = []
    dropped = {
        DROP_LOW_CONFIDENCE: 0,
        DROP_NON_OBJECT_LABEL: 0,
        DROP_INVALID_SCORE: 0,
    }
    for observation in observations:
        if is_non_object_label(
            getattr(observation, "label", None),
            getattr(observation, "label_zh", None),
        ):
            dropped[DROP_NON_OBJECT_LABEL] += 1
            continue
        score = valid_confidence(getattr(observation, "confidence", None))
        if score is None:
            dropped[DROP_INVALID_SCORE] += 1
            continue
        if score < min_confidence:
            dropped[DROP_LOW_CONFIDENCE] += 1
            continue
        kept.append(observation)
    return kept, dropped


# --------------------------------------------------- 离线取证 / 迁移专用读口
def load_object_records(
    events_path: str,
    *,
    confirm_min_observations: int = DEFAULT_CONFIRM_MIN_OBSERVATIONS,
) -> list[dict]:
    """读一份 events.jsonl，返回可判定的记录表（按 object_id 去重）。

    去重保留**最后一次**出现的记录：实体每次合并都会把聚合置信度、确认状态、
    观测次数重写进新事件，所以"最新一条"就是"当前状态"。早先不去重时同一张
    桌子/主机/垃圾桶会被画两遍（2026-09-10 实际发生过）。

    这里**不按分数单条拦**：单次观测的分数不足以判定实体，判定统一交给
    `select_display_objects`。回放脚本（`replay_object_display_gate.py`）与
    在线投影调用同一个判据，离线结论才对在线行为有约束力。
    """
    import json

    localized: list[dict] = []
    frame_labels: dict[tuple[str, str], str] = {}
    with open(events_path, encoding="utf-8") as handle:
        for line in handle:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            name = event.get("event")
            # 逐帧事件与实体事件在同一条流里，先一起收齐：实体的原始英文标签
            # 只写在逐帧事件里，而两者谁先出现不保证，边读边判会漏掉后到的。
            if name == "semantic_frame_objects_localized":
                frame_id = str(event.get("frame_id") or "")
                for item in event.get("objects") or []:
                    if isinstance(item, dict) and item.get("label"):
                        frame_labels[(frame_id,
                                      str(item.get("frame_object_id") or ""))] = \
                            str(item["label"])
                continue
            if name != "semantic_object_localized":
                continue
            obj = event.get("object") or {}
            if not obj.get("map_xyz"):
                continue
            localized.append(obj)

    deduped: dict[str, dict] = {}
    for obj in localized:
        xyz = obj.get("map_xyz")
        key = str(
            obj.get("object_id")
            or f"{obj.get('label')}|{xyz[0]:.2f}|{xyz[1]:.2f}"
        )
        deduped[key] = normalize_snapshot_object(
            obj,
            confirm_min_observations=confirm_min_observations,
            frame_labels=frame_labels,
        )
    return list(deduped.values())


def _raw_label_from_frames(
    record: dict,
    frame_labels: dict[tuple[str, str], str] | None,
) -> str:
    """实体创建那一帧里，这个框的**检测器原话**（英文标签）。

    实体自身的 `label` 是关联键而不是原话：整改后它是中文（`办公椅`），
    所以拿它当 `raw_label` 会让"可反查原始英文标签"（计划书 §6 F.4）落空。

    必须用 `provenance.frame_id`（**创建**那一帧）配对，不能用
    `source_observation_ids` —— 后者会被合并累积成一串，而各帧的
    `gsam_obj_00N` 编号是各自独立的，串用会张冠李戴（实测 obj_002 的
    `source_object_id=gsam_obj_001` 在创建帧 69870 里是 `office chair`，
    在合并进来的 72243 帧里却是 `bin`，而它确实是垃圾桶）。
    """
    if not frame_labels:
        return ""
    provenance = record.get("provenance")
    provenance = provenance if isinstance(provenance, dict) else {}
    frame_id = str(provenance.get("frame_id") or "")
    object_id = str(provenance.get("source_object_id")
                    or record.get("source_object_id") or "")
    if not frame_id or not object_id:
        return ""
    return frame_labels.get((frame_id, object_id), "")


def normalize_snapshot_object(
    record: dict,
    *,
    confirm_min_observations: int = DEFAULT_CONFIRM_MIN_OBSERVATIONS,
    frame_labels: dict[tuple[str, str], str] | None = None,
) -> dict:
    """把一条 `semantic_object_localized` 的 object 字段补成显示门能判的形态。

    历史事件里没有 `label_zh` / `status`：它们写于本次整改之前。缺什么补什么 ——
    缺 `label_zh` 就走统一解析器，缺 `status` 就按 `observation_count` 推导
    （多帧见过 = CONFIRMED，这正是 `status` 当初的定义，只是老事件没把它写下来）。

    两个标签字段分工不同，别互相顶替：`raw_label` 是**检测器原话**（取证 + 关联，
    计划书 §6 F.4 举的例就是英文 `speaker`），`label_zh` 才是**可见文字**；
    已知词优先中文，未知词显示检测器原始英文。`frame_labels` 给了就从逐帧事件里
    取原话，没给（或那一帧没留痕）才退回实体自己的 `label` —— 老归档因此
    行为不变。
    """
    raw_label = _raw_label_from_frames(record, frame_labels) or str(
        record.get("label") or record.get("name") or ""
    )
    label_zh_value = str(record.get("label_zh") or "").strip()
    if not label_zh_value or label_zh_value == LABEL_ZH_FALLBACK:
        label_zh_value = resolve_display_label(raw_label)
    elif not _has_chinese(label_zh_value):
        # 老写端可能把英文直接放在 label_zh；统一以检测器原始标签为准。
        label_zh_value = resolve_display_label(
            raw_label, runtime_zh=label_zh_value
        )

    count = int(record.get("observation_count") or 0)
    status = str(record.get("status") or "").strip()
    if status not in (ENTITY_CONFIRMED, ENTITY_TENTATIVE, ENTITY_STALE):
        status = (
            ENTITY_CONFIRMED
            if count >= int(confirm_min_observations)
            else ENTITY_TENTATIVE
        )

    score = valid_confidence(record.get("confidence"))
    maximum = valid_confidence(record.get("confidence_max"))
    if maximum is None:
        # 老事件只有旧 `confidence`（当时的语义就是历史最大值）。
        maximum = score if score is not None else 0.0

    # 事件里的坐标键是 map_xyz，快照里的键是 x/y/z。两边都认：调用方可能
    # 拿逐帧事件来喂，也可能拿已经拍平的记录来喂。
    xyz = record.get("map_xyz")
    if isinstance(xyz, (list, tuple)) and len(xyz) == 3:
        x, y, z = xyz
    else:
        x, y, z = record.get("x"), record.get("y"), record.get("z")

    return {
        "object_id": str(record.get("object_id") or ""),
        **{key: record.get(key) for key in (
            "mobility_class", "mobility_state", "mobility_reason",
            "mobility_confidence", "topology_eligible", "is_hazard",
            "hazard_rule_id", "hazard_label", "hazard_event_id", "last_seen",
            "hazard_location_mode", "hazard_position_status",
            "last_checked_at", "lifecycle_state", "missing_observation_count",
            "position_epoch", "position_changed_at")},
        "raw_label": str(raw_label),
        "label_zh": label_zh_value,
        "category": str(record.get("category") or ""),
        "x": x,
        "y": y,
        "z": z,
        "size": record.get("size") if record.get("size") is not None
        else record.get("box_size"),
        # 分数保持原值不做夹取：非法值要能被显示门识别成 invalid_score，
        # 夹成 0.0 会伪装成"合法但很低"，把真因藏起来。
        "confidence": score if score is not None else None,
        "confidence_max": maximum,
        "observation_count": count,
        "status": status,
        "target_verified": bool(record.get("target_verified")),
    }


# ------------------------------------------------- 在线对象快照（唯一一份投影）
OBJECT_SNAPSHOT_SCHEMA = "semantic_object_display_v1"


def build_object_snapshot(
    records: Iterable[dict],
    *,
    map_epoch: str,
    frame_id: str,
    producer_session_id: str = "",
    graph_revision: int = 0,
    confirmed_min_confidence: float = DEFAULT_CONFIRMED_MIN_CONFIDENCE,
    tentative_min_confidence: float = DEFAULT_TENTATIVE_MIN_CONFIDENCE,
    confirm_min_observations: int = DEFAULT_CONFIRM_MIN_OBSERVATIONS,
    topology_accepted_only: bool = False,
) -> dict:
    """§2.3：把内存里的持久实体投影成 WebUI 的显示对象快照。

    自主搜索（``AutonomousExplorer.semantic_graph.object_map``）和遥控语义建图
    （``remote_control_semantic_mapping_worker`` 的 ``self._semantic_map``）**只调
    这一个函数**，显示门因此只有一份实现。入参是 ``SemanticObjectEntry.to_dict()``
    那种形状的记录。

    ``map_epoch`` / ``frame_id`` 由调用方从地图桥写的轻量 meta 里读进来：这两个
    标识是这份对象"属于哪张地图"的全部声明。合成端（``/api/slam/map3d``）只认
    两个都相等的一对，否则一个都不画 —— 不靠包围盒、不靠文件修改时间猜。
    """
    normalized = []
    frame_mismatch = 0
    for record in records:
        # meta 只能证明当前地图的 frame，不能替实体的坐标作证。自主搜索的
        # 退化投影可能产出 pslam_odom 坐标；把它直接盖上 pslam_map 标签会
        # 让 Web 配对成功却把框画到错误位置。
        if record.get("map_xyz") is not None:
            record_frame = str((record.get("provenance") or {}).get("map_frame") or "")
            if record_frame != frame_id:
                frame_mismatch += 1
                continue
        normalized.append(normalize_snapshot_object(
            record, confirm_min_observations=confirm_min_observations))
    objects, stats = select_display_objects(
        normalized,
        confirmed_min_confidence=confirmed_min_confidence,
        tentative_min_confidence=tentative_min_confidence,
        confirm_min_observations=confirm_min_observations,
        topology_accepted_only=topology_accepted_only,
    )
    if frame_mismatch:
        stats["input"] += frame_mismatch
        stats["filtered"] += frame_mismatch
        stats["dropped"]["map_frame_mismatch"] = frame_mismatch
    for item in objects:
        for axis in ("x", "y", "z"):
            value = item[axis]
            item[axis] = round(float(value), 3) if value is not None else None
    return {
        "schema_version": OBJECT_SNAPSHOT_SCHEMA,
        "map_epoch": str(map_epoch),
        "frame_id": str(frame_id),
        "producer_session_id": str(producer_session_id),
        "graph_revision": int(graph_revision),
        "objects": objects,
        "object_filter_stats": stats,
    }


# ------------------------------------------------------------------ 显示门
def display_decision(
    record: dict,
    *,
    confirmed_min_confidence: float = DEFAULT_CONFIRMED_MIN_CONFIDENCE,
    tentative_min_confidence: float = DEFAULT_TENTATIVE_MIN_CONFIDENCE,
    confirm_min_observations: int = DEFAULT_CONFIRM_MIN_OBSERVATIONS,
    topology_accepted_only: bool = False,
) -> tuple[bool, str]:
    """返回 (是否显示, 原因)。原因在显示时是 DISPLAY/TARGET_VERIFIED。"""
    if ((topology_accepted_only and record.get("topology_eligible") is not True)
            or (not topology_accepted_only and record.get("topology_eligible") is False)
            or record.get("mobility_class") == "dynamic"
            or str(record.get("lifecycle_state") or "active") == "removed"):
        return False, "not_topology_eligible"
    if _finite_xyz(record) is None:
        # 没有合法坐标就什么都画不了，目标例外也救不回来。
        return False, DROP_INVALID_POSITION
    if record.get("target_verified"):
        # 计划书 §5.2：视觉复核过的目标单独放行，但必须带显式字段。
        # 原始检测候选没有任何路径能自动获得这个字段（它只能由复核环节写入）。
        return True, DECISION_TARGET_VERIFIED
    if is_non_object_label(record.get("raw_label"), record.get("label_zh")):
        return False, DROP_NON_OBJECT_LABEL
    score = valid_confidence(record.get("confidence"))
    if score is None:
        return False, DROP_INVALID_SCORE
    if not str(record.get("label_zh") or "").strip():
        return False, DROP_MISSING_LABEL_ZH
    status = str(record.get("status") or "")
    lifecycle = str(record.get("lifecycle_state") or "active")
    count = int(record.get("observation_count") or 0)
    if topology_accepted_only:
        # 二维遥控建图已经在单次观测入口按同一个置信度门限过滤过。这里只核对
        # 持久实体确实达到确认态并被拓扑接纳，不能再套旧三维框的 0.45/3 门，
        # 否则拓扑图和二维标记会静默少掉已经确认的实体。
        if (
            status == ENTITY_CONFIRMED
            and count >= int(confirm_min_observations)
            and score >= confirmed_min_confidence
        ):
            return True, DECISION_DISPLAY
        return False, DROP_BELOW_DISPLAY_THRESHOLD
    if (lifecycle in {"stale", "missing_candidate"}
            and count >= int(confirm_min_observations)
            and score >= confirmed_min_confidence):
        return True, DECISION_DISPLAY
    if (
        status == ENTITY_CONFIRMED
        and count >= int(confirm_min_observations)
        and score >= confirmed_min_confidence
    ):
        return True, DECISION_DISPLAY
    if status == ENTITY_TENTATIVE and score >= tentative_min_confidence:
        # 只见过一次也可能显示 —— 但门槛要比确认物体高一截，
        # 这就是"只看 observation_count 就无条件显示"的反面。
        return True, DECISION_DISPLAY
    return False, DROP_BELOW_DISPLAY_THRESHOLD


def select_display_objects(
    records: Iterable[dict],
    *,
    confirmed_min_confidence: float = DEFAULT_CONFIRMED_MIN_CONFIDENCE,
    tentative_min_confidence: float = DEFAULT_TENTATIVE_MIN_CONFIDENCE,
    confirm_min_observations: int = DEFAULT_CONFIRM_MIN_OBSERVATIONS,
    topology_accepted_only: bool = False,
) -> tuple[list[dict], dict[str, Any]]:
    """显示门：返回 (该画的记录, 统计)。

    会就地给保留的记录加上 `decision`（DISPLAY / TARGET_VERIFIED）与
    `box_valid`：前端据此换颜色、并在尺寸非法时退回画点。
    """
    visible: list[dict] = []
    # 注意这里**没有** low_confidence：显示门的两档门限不一样（0.45 / 0.65），
    # "分数低"不是一个数，统一记成 below_display_threshold。low_confidence
    # 是入口门的词汇（那边的门限只有一个）。
    dropped = {
        DROP_NON_OBJECT_LABEL: 0,
        DROP_INVALID_SCORE: 0,
        DROP_INVALID_POSITION: 0,
        DROP_MISSING_LABEL_ZH: 0,
        DROP_BELOW_DISPLAY_THRESHOLD: 0,
    }
    total = 0
    for record in records:
        total += 1
        shown, reason = display_decision(
            record,
            confirmed_min_confidence=confirmed_min_confidence,
            tentative_min_confidence=tentative_min_confidence,
            confirm_min_observations=confirm_min_observations,
            topology_accepted_only=topology_accepted_only,
        )
        if not shown:
            dropped[reason] = dropped.get(reason, 0) + 1
            continue
        record["decision"] = reason
        box = valid_box_size(record.get("size"))
        record["box_valid"] = box is not None
        # 非法尺寸直接抹成 null：前端"有 size 才画框、没有就画点"的老逻辑
        # 因此自动降级，不必依赖它记得读 box_valid。
        record["size"] = [float(value) for value in box] if box is not None else None
        visible.append(record)
    stats = {
        "input": total,
        "displayed": len(visible),
        "filtered": total - len(visible),
        "dropped": {key: value for key, value in dropped.items() if value},
    }
    return visible, stats


def audit_rows(
    records: Iterable[dict],
    *,
    confirmed_min_confidence: float = DEFAULT_CONFIRMED_MIN_CONFIDENCE,
    tentative_min_confidence: float = DEFAULT_TENTATIVE_MIN_CONFIDENCE,
    confirm_min_observations: int = DEFAULT_CONFIRM_MIN_OBSERVATIONS,
) -> list[dict]:
    """给每次判定留一行审计（计划书 §8.1 的审计表）。

    显示与否都要留痕：只记被丢的，事后无法回答"某个东西为什么没上屏、
    换个门限会不会上屏"。
    """
    rows: list[dict] = []
    for record in records:
        shown, reason = display_decision(
            record,
            confirmed_min_confidence=confirmed_min_confidence,
            tentative_min_confidence=tentative_min_confidence,
            confirm_min_observations=confirm_min_observations,
        )
        rows.append(
            {
                "object_id": record.get("object_id"),
                "raw_label": record.get("raw_label"),
                "label_zh": record.get("label_zh"),
                "confidence": record.get("confidence"),
                "confidence_max": record.get("confidence_max"),
                "observation_count": record.get("observation_count"),
                "status": record.get("status"),
                "decision": reason if shown else DECISION_DROP,
                "drop_reason": None if shown else reason,
                "box_valid": valid_box_size(record.get("size")) is not None,
            }
        )
    return rows
