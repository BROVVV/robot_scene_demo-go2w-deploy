"""把一次检测的结果按**职责**拆成三份视图，并给每个实体一个稳定 ID。

为什么要有这个模块（计划书 §5.1.2）：

* 检测器一次请求会同时问**目标词**和 **RAM 场景词**（`_detect_grounded_sam`
  把 `_ram_scene_terms` 拼进同一次请求）。回来后这一堆框被当成一个平铺的
  `detections` 列表交给下游，于是 `target_present` 退化成
  `bool(detections)` —— **画面里只要有任何一件东西，目标就算"存在"**。
* `relation_anchors` 这个字段此前全仓不存在：关系目标（「垃圾桶**旁边**的椅子」）
  的另一半根本没被单独拿出来。
* 检测项与全场景物体项来自**两次不同的检测调用**，各带各的编号
  （`detections` 连编号都没有，`scene_objects` 用 `gsam_obj_{index:03d}`），
  所以同一个实体在两条链上对不上号。

本模块是**纯函数**、不依赖 rclpy，可以单测。三条对外契约：

1. `assign_entity_ids` 给每个实体一个**与顺序无关**的 ID（量化 bbox + label 的
   哈希）。用序号不行：两次检测调用返回顺序一变，同一个实体的编号就变了，
   "对上号"这件事就无从谈起。
2. `classify_roles` 依据 **`source_prompt_term`**（这个框是哪个词生成的）分类，
   不重跑分类器、不做模糊猜测。同一个实体可以同时是场景物体与关系锚点。
3. `derive_target_state` 给三态，且**如实划出能力边界**（见该函数文档）。
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import Any, Iterable

from app.detectors.grounded_sam_subprocess import _matches_any_term
from app.perception.target_state import (
    TARGET_ABSENT,
    TARGET_POSSIBLE,
    TARGET_PRESENT,
)

ROLE_SCENE = "scene"
ROLE_TARGET = "target"
ROLE_ANCHOR = "anchor"

# 归一化到 4 位小数再哈希：bbox_2d 是归一化到 0~1 的图像坐标，
# 4 位小数 ≈ 千分之一像素级差异。留太细会让同一件东西因为浮点末位抖动
# 拿到两个 ID；留太粗会把相邻的两个物体并成一个。
_BBOX_DECIMALS = 4


def _bbox_key(bbox: Any) -> str:
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return "?"
    try:
        return ",".join(f"{float(v):.{_BBOX_DECIMALS}f}" for v in bbox)
    except (TypeError, ValueError):
        return "?"


def entity_id_for(item: dict[str, Any]) -> str:
    """一个实体的稳定 ID：同标签同框必得同 ID，与它在列表里的位置无关。"""
    label = str(item.get("label") or item.get("label_zh") or "").strip().lower()
    digest = hashlib.sha1(
        f"{label}|{_bbox_key(item.get('bbox_2d') or item.get('bbox'))}".encode("utf-8")
    ).hexdigest()[:8]
    return f"det_{digest}"


def assign_entity_ids(
    objects: Iterable[dict[str, Any]], *, source_frame_id: str | None = None
) -> list[dict[str, Any]]:
    """给每个实体补上 ``entity_id``（已存在就不覆盖），不改动别的键。

    刻意**不动**已有的 ``id`` 字段：`analyze_semantic` 用的是
    ``gsam_obj_{index:03d}``，另有消费方（`_detection_frame_objects` 的
    ``object_id`` 等）走那条线，改它的语义会波及无关链路。新字段是可加的，
    老字段保持原样。

    ``source_frame_id`` 是**这一批框来自哪一帧**。它只作为元数据写上
    （``entity_source_frame_id``），**不**进 ID 的哈希：进了哈希，验评侧的
    `entity_id_for(best)`（手里那个 dict 没有帧号）就算不出同一个 ID，
    "候选身份"立刻断链。跨帧的同一性由 `entities_are_the_same()` 负责 ——
    它要求 ID 与源帧**都**相同，光有哈希不算数。
    """
    # 空串与 None 一样是"不知道"，**不**写成 `entity_source_frame_id: ""` ——
    # 写进去会让"来源帧未知"看起来像"来源帧就是空"。
    frame = str(source_frame_id) if source_frame_id else None
    out: list[dict[str, Any]] = []
    for item in objects or ():
        if not isinstance(item, dict):
            continue
        entry = dict(item)
        if not entry.get("entity_id"):
            entry["entity_id"] = entity_id_for(entry)
        if frame is not None and not entry.get("entity_source_frame_id"):
            entry["entity_source_frame_id"] = frame
        out.append(entry)
    return out


def entities_are_the_same(a: dict[str, Any], b: dict[str, Any]) -> bool:
    """两个条目是不是**同一实体**：``entity_id`` 相同**且**源帧相同。

    ID 是"量化框 + 标签"的哈希，**只在同一帧内**保证唯一。跨帧它只说明
    "这个位置上又出现了一个同标签的东西" —— 静止的办公室里那几乎总是同一个，
    机器人一动、或者桌面被人挪过，就不是了。所以认同必须带源帧。

    源帧未知（空串）时**一律不算**同一实体：不知道就别认，这正是审查 P1-c
    点出的那条 —— 不能仅凭"4 位小数 bbox + label"的哈希假定跨检测同一。
    """
    if not a or not b:
        return False
    if not a.get("entity_id") or a.get("entity_id") != b.get("entity_id"):
        return False
    frame_a = str(a.get("entity_source_frame_id") or "")
    frame_b = str(b.get("entity_source_frame_id") or "")
    return bool(frame_a) and frame_a == frame_b


def _terms_from(value: Any) -> list[str]:
    if value is None:
        return []
    if callable(value):
        try:
            value = value()
        except Exception:  # pragma: no cover - 画像实现异常不该拖垮分类
            return []
    try:
        return [str(item) for item in value if str(item).strip()]
    except TypeError:
        return []


def target_terms(profile: Any) -> list[str]:
    """目标本身的词（含别名）。`detector_terms()` 掺了颜色组合，不能当目标词用。"""
    return _terms_from(getattr(profile, "direct_terms", None))


def anchor_terms(profile: Any) -> list[str]:
    return _terms_from(getattr(profile, "relation_anchor_labels_en", None))


@dataclass
class RolePartition:
    """三份视图，元素是**同一批**带 `entity_id` 与 `roles` 的 dict。

    ``detections`` 是那"同一批"本身（全量、带 ID、带 ``source_prompt_term``）。
    三份视图都是它的**子集视图**（同一个 dict 对象），不是三份各算各的拷贝 ——
    这正是"共用实体 ID"能成立的原因。``source_frame_id`` 记下这批框来自哪一帧。
    """

    scene_objects: list[dict[str, Any]] = field(default_factory=list)
    target_candidates: list[dict[str, Any]] = field(default_factory=list)
    relation_anchors: list[dict[str, Any]] = field(default_factory=list)
    detections: list[dict[str, Any]] = field(default_factory=list)
    source_frame_id: str = ""


def classify_roles(
    objects: Iterable[dict[str, Any]],
    profile: Any,
    *,
    source_frame_id: str | None = None,
) -> RolePartition:
    """按 `source_prompt_term` 把实体分到三份视图里。

    视图之间的边界（与既有的 quick 契约一致：目标**不**混进场景物体）：

    * ``target_candidates``：词命中目标词表的实体。
    * ``relation_anchors``：词命中关系锚点词表的实体。
    * ``scene_objects``：**除目标以外**的实体。锚点**同时**属于场景物体 ——
      「椅子旁边的垃圾桶」里那个垃圾桶是真真实实要看进图的东西，不能因为
      它是锚点就不进图。

    ``source_frame_id`` 会写进 ``partition.source_frame_id`` 与每个条目的
    ``entity_source_frame_id``（见 `assign_entity_ids`）。**只在同一帧内**，
    这三个视图的 ID 才表示"同一个实体"。

    **按槽归属**（2026-10-14 补充复审必修 1）：带 ``detection_fresh_slots``
    的条目（来自 `detection_scheduler.assemble_slot_objects`）只对**它还是最新
    的那几个槽**作数，于是：

    * 命中了目标/锚点词、但那个角色槽的最新一次检测已经不是它 ⇒ **丢掉**。
      这不是丢证据，恰恰是**采信新证据**：那个槽的最新一次用的词表里就含这个
      词，它没报出这个框，就是"此刻不在这儿"的结论。留着它，旧帧的高分框会在
      `select_candidate` 里把新帧的低分真框顶掉，甚至在新帧**完全没有目标**时
      报出 PRESENT。
    * 条目里带了 ``detection_slots``（**这个框由哪些槽问出来**，检测器在拼
      prompt 时记账）⇒ 只要与最新的槽**一个都不交**就**整条丢掉**，不看它有没有
      角色。这是 2026-10-14 第四次补充复审的必修：目标槽的**上下文词**
      （画像的 `context_labels_en`，如「打印机」画像的 `paper tray`）不是目标词也
      不是锚点词，旧帧 0.95 的 `paper tray` 因此能躲过角色过滤，在默认选框里压过
      新帧 0.20 的 `printer`；新帧连打印机都没有时仍报 PRESENT 并锁旧纸盒。
      丢的只是**过期的角色证据**：场景/建图走的不是这条链（`observation.
      scene_objects` 来自后台全量语义那条路），所以不丢地图内容。
    * ``detection_slots`` 是**空元组**（quick 路径、单测手工构造的 dict、旧
      worker 缓存）⇒ "不知道它属于哪个槽"，**不判**，退回按角色过滤的旧行为。
      宁可留着旧框，也不能把未知猜成 scene 而白丢能力。
    * 不带 ``detection_fresh_slots``（即分频没开）⇒ 没有"它还新不新"的信息，
      按旧行为处理，不凭空判人家过期。
    """
    direct = target_terms(profile)
    anchors = anchor_terms(profile)
    partition = RolePartition(source_frame_id=str(source_frame_id or ""))
    for entry in assign_entity_ids(objects, source_frame_id=source_frame_id):
        term = str(entry.get("source_prompt_term") or entry.get("label") or "")
        roles: list[str] = []
        if direct and _matches_any_term(term, direct):
            roles.append(ROLE_TARGET)
        if anchors and _matches_any_term(term, anchors):
            roles.append(ROLE_ANCHOR)

        fresh = entry.get("detection_fresh_slots")
        if fresh is not None:
            allowed = {str(slot) for slot in fresh}
            # 每个框**自己**由哪个槽问出来（`detection_slots`，检测器算的最准，
            # 见 `grounded_sam_subprocess._slots_for_term`）。这条与上面那枚
            # **条目级**标记是两件事：`fresh` 说"这条检测还对哪些槽算最新"，
            # `own` 说"这个框是哪个槽问出来的"。旧的全量条目为了保住 `scene`
            # 被选中时，它里面那个**目标槽**问出来的框会连 `scene` 一起拿到；
            # 画像的上下文词（`context_labels_en`，如「打印机」画像的
            # `paper tray`）不属于 `direct`/`anchors`，只靠下面那条角色过滤挡
            # 不住 —— 旧帧 0.95 的 `paper tray` 会压过新帧 0.20 的 `printer`，
            # 新帧连打印机都没有时还会报 PRESENT。
            #
            # 空的 `own` = 检测器没记（quick 路径、单测手工构造的 dict、旧
            # worker 缓存）⇒ **不知道就不判**，落到下面按角色过滤的旧行为，
            # 绝不猜成 scene。
            own = {str(slot) for slot in (entry.get("detection_slots") or ())}
            if own and not (own & allowed):
                continue
            if roles:
                roles = [role for role in roles if role in allowed]
                if not roles:
                    continue
        if not roles:
            roles.append(ROLE_SCENE)
        # 就地打 `roles`，**不**复制。`assign_entity_ids` 已经拷过一层，改这里
        # 碰不到调用方的 dict。三份视图必须指向**同一个** dict：复制一份出来，
        # `detections` 与候选视图就成了两批对象，`entity_id` 虽然还相等，但下游
        # 任何"按对象认同一"的判断都会当场失效 —— 而"共用实体 ID"是本模块的
        # 全部意义（审查 P1-c）。
        entry["roles"] = roles
        partition.detections.append(entry)
        if ROLE_TARGET in roles:
            partition.target_candidates.append(entry)
        else:
            partition.scene_objects.append(entry)
        if ROLE_ANCHOR in roles:
            partition.relation_anchors.append(entry)
    return partition


def _score(item: dict[str, Any]) -> float:
    try:
        value = float(item.get("score", item.get("confidence", 0.0)))
    except (TypeError, ValueError):
        return 0.0
    return value if value == value else 0.0  # NaN 归零


# --------------------------------------------------------------------------- #
# 复核候选选择（计划书 §5.1.3）
# --------------------------------------------------------------------------- #
VERIFY_SCOPE_TARGET_CANDIDATES = "target_candidates"
VERIFY_SCOPE_ALL_DETECTIONS = "all_detections"


def select_candidate(
    candidates: Iterable[dict[str, Any]] | None,
    detections: Iterable[dict[str, Any]] | None,
    *,
    scope: str = VERIFY_SCOPE_TARGET_CANDIDATES,
    candidates_authoritative: bool = False,
) -> tuple[dict[str, Any] | None, bool]:
    """挑出**供复核/展示**的那一个框。返回 ``(best, 用的是不是权威候选视图)``。

    病根：排序池此前是**全量** ``detections``，而 grounded_sam 那条路的
    ``detections`` 混着 RAM 场景词 —— 于是一把 0.95 的椅子会把 0.32 的目标桶
    挤出复核位：复核（和画面上的锁框）指向的根本不是要找的东西。

    两个入参的分工：

    * ``scope``：运营开关。**只有**恰为 ``all_detections`` 才退回旧行为；
      其它取值（含拼错的）走候选视图 —— 因为"复核目标"才是本意，拼错时
      偏向新语义比偏向旧缺陷安全。
    * ``candidates_authoritative``：候选视图**是否是本次观测的权威结论**
      （即任务 2 的职责分离真的跑了）。这一条不能省：空表有两种含义 ——
      "分类过了，确实没有目标候选"与"没人产过候选视图"。前者必须**拒绝复核**
      （否则又会去复核场景物体 ⇒ 假"找到"），后者只能按旧行为退回
      ``detections``。少一个判据就把这两种情况混成一种。

    分数取 ``_score``（NaN 归零）：`max()` 遇到 NaN 会把它当最大值吐出来，
    于是"分数没算出来"的框反而抢占复核位。
    """
    if candidates_authoritative and scope != VERIFY_SCOPE_ALL_DETECTIONS:
        pool = [item for item in (candidates or ()) if isinstance(item, dict)]
        authoritative = True
    else:
        pool = [item for item in (detections or ()) if isinstance(item, dict)]
        authoritative = False
    if not pool:
        return None, authoritative
    return max(pool, key=_score), authoritative


def derive_target_state(
    objects: Iterable[dict[str, Any]],
    profile: Any,
    target_score_min: float,
) -> str:
    """三态目标判定：**只看目标词的候选**，不看场景里有没有别的东西。

    规则：

    * 有目标词候选且最高分 >= ``target_score_min`` ⇒ ``PRESENT``
    * 有目标词候选但都够不着阈值 ⇒ ``POSSIBLE``
    * 一个目标词候选都没有 ⇒ ``ABSENT``

    **能力边界（必须如实写进报告，不能假装做了）**：颜色/材质/状态这类**属性**
    判定在实时链里算不出来 —— 属性只活在 VLM 复核的 prompt 里
    （`app/detectors/crop_verifier.py` 的 `_verification_prompt`），检测器只按
    类别出框。所以本函数的 ``POSSIBLE`` 只有"类别命中但分数/阈值不够"这一个
    来源，**不是**"属性存疑"。属性层面的判定要等复核层，别把这里的 POSSIBLE
    当成"颜色不对"。
    """
    partition = classify_roles(objects, profile)
    if not partition.target_candidates:
        return TARGET_ABSENT
    best = max((_score(item) for item in partition.target_candidates), default=0.0)
    return TARGET_PRESENT if best >= float(target_score_min) else TARGET_POSSIBLE
