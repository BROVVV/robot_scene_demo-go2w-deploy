"""检测分频（计划书 §5.1.4）：决定这一轮**该跑哪些词槽**。

缺口：`observe()` 每轮都拿整套 prompt 打一次 GroundingDINO，全无节流 ——
只有 RAM++ 的 tag 是 1.8 s 一次。而一次检测的成本≈（一次 backbone 编码 +
按词数线性增长的对齐/解码），所以"每轮都问一遍场景词"是纯浪费：
狗站着没动、朝向没变、地点没变的时候，场景词那半张词表的结果不会变。

本模块是**纯函数**，不依赖 rclpy、不碰检测器，`now` 由调用方注入 ——
所以可以逐条单测"到点/没到点"。

三类槽位（各自独立时间戳与 TTL）：

* ``target``：目标词 + 上下文词。**默认每轮都刷**（TTL 0），因为漏检目标是
  §3.2 明令不许新增的失败。
* ``anchor``：关系锚点词（「垃圾桶旁边的椅子」里的"垃圾桶"）。
* ``scene``：RAM++ 场景词。

除 TTL 之外还有两条**强制刷新**判据，它们优先于 TTL：

* **视图变了**：地点切换 / 朝向扇区变了 / 位移超过阈值 —— 换了地方还沿用
  旧结果就是在看图说话。
* **starvation 上限**：`scene_max_stale_s` 是硬顶。运营可以把 ``ttl_scene_s``
  调到很大（"场景只在换地方时刷"），但绝不能因此让场景词无限期不刷 ——
  到顶就强制刷一次，并把实际等了多久记下来（§5.2 要求可见）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable, Mapping, Sequence

SLOT_TARGET = "target"
SLOT_ANCHOR = "anchor"
SLOT_SCENE = "scene"

ALL_SLOTS = (SLOT_TARGET, SLOT_ANCHOR, SLOT_SCENE)


@dataclass(frozen=True)
class DetectionTrigger:
    """Unified full-frame detection decision for autonomous search.

    The scheduler owns only cadence.  It never calls a detector and never
    authorizes motion.  Translation and rotation are accumulated from the
    odometry samples supplied by the caller, so a sequence of small turns or
    moves cannot evade the refresh thresholds.
    """

    should_detect: bool
    reason: str = "reuse"
    reasons: tuple[str, ...] = field(default=())
    elapsed_s: float | None = None
    translation_m: float = 0.0
    rotation_deg: float = 0.0
    debounced: bool = False


class DetectionScheduler:
    """Stateful event/time/odometry cadence shared by local backends."""

    EVENT_REASONS = {
        "arrival",
        "nav_failure",
        "nav_cancelled",
        "replan_failure",
        "junction_arrival",
        "place_changed",
        "target_candidate",
    }

    def __init__(
        self,
        *,
        interval_s: float = 3.0,
        translation_refresh_m: float = 0.8,
        rotation_refresh_deg: float = 30.0,
        min_interval_s: float = 1.0,
    ) -> None:
        self.interval_s = max(0.0, float(interval_s))
        self.translation_refresh_m = max(0.0, float(translation_refresh_m))
        self.rotation_refresh_deg = max(0.0, float(rotation_refresh_deg))
        self.min_interval_s = max(0.0, float(min_interval_s))
        self._last_detection_s: float | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._translation_m = 0.0
        self._rotation_deg = 0.0
        self._last_place_id: str | None = None

    @staticmethod
    def _pose(value: Sequence[float] | None) -> tuple[float, float, float] | None:
        if value is None or len(value) < 3:
            return None
        try:
            pose = (float(value[0]), float(value[1]), float(value[2]))
        except (TypeError, ValueError):
            return None
        return pose if all(math.isfinite(item) for item in pose) else None

    def evaluate(
        self,
        *,
        now: float,
        pose: Sequence[float] | None,
        event: str | None = None,
        place_id: str | None = None,
    ) -> DetectionTrigger:
        current = self._pose(pose)
        if current is not None and self._last_pose is not None:
            self._translation_m += math.hypot(
                current[0] - self._last_pose[0], current[1] - self._last_pose[1]
            )
            delta_yaw = math.atan2(
                math.sin(current[2] - self._last_pose[2]),
                math.cos(current[2] - self._last_pose[2]),
            )
            self._rotation_deg += abs(math.degrees(delta_yaw))
        if current is not None:
            self._last_pose = current

        elapsed = (
            None
            if self._last_detection_s is None
            else max(0.0, float(now) - self._last_detection_s)
        )
        reasons: list[str] = []
        if self._last_detection_s is None:
            reasons.append("initial")
        else:
            if self.interval_s > 0.0 and elapsed is not None and elapsed >= self.interval_s:
                reasons.append("time")
            if (
                self.translation_refresh_m > 0.0
                and self._translation_m >= self.translation_refresh_m
            ):
                reasons.append("translation")
            if (
                self.rotation_refresh_deg > 0.0
                and self._rotation_deg >= self.rotation_refresh_deg
            ):
                reasons.append("rotation")
            normalized_event = str(event or "").strip().lower()
            if normalized_event in self.EVENT_REASONS:
                reasons.append(normalized_event)
            if (
                place_id is not None
                and self._last_place_id is not None
                and str(place_id) != self._last_place_id
                and "place_changed" not in reasons
            ):
                reasons.append("place_changed")

        debounced = bool(
            reasons
            and self._last_detection_s is not None
            and elapsed is not None
            and elapsed < self.min_interval_s
        )
        return DetectionTrigger(
            should_detect=bool(reasons) and not debounced,
            reason=reasons[0] if reasons and not debounced else "reuse",
            reasons=tuple(reasons),
            elapsed_s=elapsed,
            translation_m=self._translation_m,
            rotation_deg=self._rotation_deg,
            debounced=debounced,
        )

    def mark_detected(
        self,
        *,
        now: float,
        pose: Sequence[float] | None,
        place_id: str | None = None,
    ) -> None:
        current = self._pose(pose)
        if current is not None:
            self._last_pose = current
        self._last_detection_s = float(now)
        self._translation_m = 0.0
        self._rotation_deg = 0.0
        self._last_place_id = None if place_id is None else str(place_id)


@dataclass(frozen=True)
class DetectionCadence:
    """这一轮该刷新哪些槽，以及为什么。"""

    refresh_target: bool
    refresh_anchor: bool
    refresh_scene: bool
    # 场景这一次是被 starvation 上限**逼**出来的（而不是 TTL 到期）。
    forced_scene: bool = False
    # 被逼出来时，实际已经等了多久（秒）。没被逼就是 None。
    scene_wait_s: float | None = None
    reasons: tuple[str, ...] = field(default=())

    @property
    def slots(self) -> frozenset[str]:
        """要跑的槽位集合。空集 = 这一轮一次检测都不用发。"""
        out = set()
        if self.refresh_target:
            out.add(SLOT_TARGET)
        if self.refresh_anchor:
            out.add(SLOT_ANCHOR)
        if self.refresh_scene:
            out.add(SLOT_SCENE)
        return frozenset(out)


def _due(last: float | None, now: float, ttl: float) -> bool:
    if last is None:
        return True  # 从没刷过 ⇒ 冷启动，必须刷
    if ttl <= 0.0:
        return True  # TTL 0 = 每轮都刷
    return (now - last) >= ttl


def decide_detection_refresh(
    *,
    now: float,
    last_target_s: float | None,
    last_anchor_s: float | None,
    last_scene_s: float | None,
    ttl_target_s: float = 0.0,
    ttl_anchor_s: float = 0.0,
    ttl_scene_s: float = 4.0,
    scene_max_stale_s: float = 20.0,
    place_id: str | None = None,
    last_place_id: str | None = None,
    sector: int | None = None,
    last_sector: int | None = None,
    translation_m: float = 0.0,
    translation_refresh_m: float = 0.25,
) -> DetectionCadence:
    """算出这一轮要跑哪些槽。

    判据优先级：**冷启动 > 视图变了 > starvation 上限 > 按类 TTL**。
    前两条一旦成立就全部刷新 —— 视图变了却只刷一半，等于拿旧视角的词表去解释
    新画面。
    """
    reasons: list[str] = []

    place_changed = (
        place_id is not None
        and last_place_id is not None
        and str(place_id) != str(last_place_id)
    )
    sector_changed = (
        sector is not None and last_sector is not None and int(sector) != int(last_sector)
    )
    moved = float(translation_m) >= float(translation_refresh_m) > 0.0

    cold = last_target_s is None or last_scene_s is None
    if cold:
        reasons.append("cold_start")
    if place_changed:
        reasons.append("place_changed")
    if sector_changed:
        reasons.append("sector_changed")
    if moved:
        reasons.append("translation")

    if cold or place_changed or sector_changed or moved:
        # `scene_wait_s` 只在 starvation 那条路上有值：它回答的是"被上限逼出来
        # 时到底等了多久"，视图变了并不是被逼的，填进来就把两个含义混成一个。
        return DetectionCadence(
            refresh_target=True,
            refresh_anchor=True,
            refresh_scene=True,
            reasons=tuple(reasons),
        )

    refresh_target = _due(last_target_s, now, ttl_target_s)
    refresh_anchor = _due(last_anchor_s, now, ttl_anchor_s)
    refresh_scene = _due(last_scene_s, now, ttl_scene_s)
    if refresh_target:
        reasons.append("target_ttl")
    if refresh_anchor:
        reasons.append("anchor_ttl")
    if refresh_scene:
        reasons.append("scene_ttl")

    forced_scene = False
    scene_wait_s: float | None = None
    if last_scene_s is not None:
        waited = max(0.0, now - last_scene_s)
        # starvation 上限是**硬顶**：运营可以把 ttl_scene_s 调到很大
        # （"场景只在换地方时刷"），但不能因此让场景词无限期不刷。
        if not refresh_scene and waited >= float(scene_max_stale_s):
            refresh_scene = True
            forced_scene = True
            scene_wait_s = waited
            reasons.append("scene_max_stale")

    return DetectionCadence(
        refresh_target=refresh_target,
        refresh_anchor=refresh_anchor,
        refresh_scene=refresh_scene,
        forced_scene=forced_scene,
        scene_wait_s=scene_wait_s,
        reasons=tuple(reasons),
    )


# --------------------------------------------------------------------------- #
# 按槽保留结果（审查 P2）
# --------------------------------------------------------------------------- #
# 病根：分频**切得动词槽，却留不住词槽**。`observe` 每一轮拿本次 `_detect` 的
# 返回值**整体覆盖**缓存，于是这一轮只刷 target 槽时，上一轮那些场景物体就
# 从观测里**消失**了 —— 物体地图/拓扑跟着掉一段；下一轮只刷 scene 槽时，
# 目标候选又消失。分频于是不是"少算一点"，而是"每轮只看半张画面"。
#
# 修法：缓存**按产生它的那次检测**存（`slots` + 源帧 + 时刻），组装时给每个
# 槽挑"覆盖它且最新的那一条"。这样：
#
# * 这一轮刷了 target，anchor/scene 的上次结果**还在**，一起组装进观测；
# * 每件物体带上它**真实的来源帧**（``entity_source_frame_id``），旧帧的框
#   不会冒充当前帧的事实（这正是 P1-c 那条认同判据要用的东西）；
# * 组进来的物体来自不止一帧时，``views_source_frame_id`` 如实给空串
#   （"不知道"），而不是把当前帧号按在旧框上。
#
# 为什么按"检测"存而不是按"槽"存：一次 `_detect(slots={"target","anchor"})`
# 返回的框混着两个槽的词，缓存**只能**记住"这条检测覆盖了哪些槽"。按槽把这批
# 框硬拆开是在猜；按检测存是记事实。
#
# 但**框自己**属于哪个槽是另一回事，而且检测器算得出来（词表就在它手里，
# 见 `grounded_sam_subprocess._slots_for_term`）—— 它随框带 `detection_slots`。
# 两者都要：`detection_fresh_slots` 说"这条检测还对哪些槽算最新"，
# `detection_slots` 说"这个框由哪个槽问出来"。少了后者，旧全量条目里的
# **目标槽上下文词**会借 `scene` 的新鲜度留下来（2026-10-14 第四次补充复审）。


@dataclass(frozen=True)
class SlotCacheEntry:
    """一次检测的产物，连同它是为哪些槽跑的、来自哪一帧、什么时候跑的。"""

    slots: frozenset[str]
    objects: list[dict[str, Any]]
    frame_id: str
    at: float


@dataclass(frozen=True)
class SlotAssembly:
    """本轮观测该用的物体，以及它们各自的来路。"""

    objects: list[dict[str, Any]]
    entries: tuple[SlotCacheEntry, ...]
    # 组装进来的物体**全部**来自同一帧时是那一帧；混帧时是空串（"不知道"）。
    frame_id: str

    @property
    def mixed_frames(self) -> bool:
        return not self.frame_id

    def provenance(self, *, now: float, ttl_by_slot: Mapping[str, float]) -> list[dict[str, Any]]:
        """逐条来路，落事件流用：哪个槽、哪一帧、多久前、有没有超出 TTL。"""
        out: list[dict[str, Any]] = []
        for entry in self.entries:
            age = max(0.0, float(now) - float(entry.at))
            slots = sorted(entry.slots)
            ttls = [float(ttl_by_slot[s]) for s in slots if s in ttl_by_slot]
            # 一条检测覆盖多个槽时，按**最宽松**的那个 TTL 判过期：
            # 收紧会让"多槽一起刷"的轮次凭空报警。
            ttl = max(ttls) if ttls else None
            out.append(
                {
                    "slots": slots,
                    "frame_id": str(entry.frame_id or ""),
                    "age_s": round(age, 3),
                    "objects": len(entry.objects),
                    "stale": bool(ttl is not None and age > ttl),
                }
            )
        return out


def _object_key(item: Any) -> tuple[str, str]:
    """去重键：同一个词、同一个框 = 同一件东西。"""
    if not isinstance(item, dict):
        return ("?", "?")
    term = str(item.get("source_prompt_term") or item.get("label") or "").strip().lower()
    bbox = item.get("bbox_2d") or item.get("bbox")
    try:
        box = ",".join(f"{float(v):.4f}" for v in bbox)
    except (TypeError, ValueError):
        box = "?"
    return (term, box)


def put_slot_entry(
    entries: Sequence[SlotCacheEntry],
    entry: SlotCacheEntry,
    *,
    keep: int = 4,
) -> list[SlotCacheEntry]:
    """记下一次检测的产物。**同槽集合**只留最新一条（它已经覆盖旧的）。"""
    kept = [
        item
        for item in entries
        if frozenset(item.slots) != frozenset(entry.slots)
    ]
    kept.append(entry)
    kept.sort(key=lambda item: float(item.at))
    return kept[-max(1, int(keep)) :]


def assemble_slot_objects(
    entries: Sequence[SlotCacheEntry],
    *,
    now: float,
    ttl_by_slot: Mapping[str, float] | None = None,
    order: Iterable[str] = ALL_SLOTS,
) -> SlotAssembly:
    """给每个槽挑"覆盖它且最新的那一条"，再把这些条目的物体并起来。

    这不是"取最新一条"：只刷 target 的那一轮，最新一条里**没有**场景物体，
    取最新就等于把场景丢了 —— 正是要修的那个缺陷。

    但并**不能整条整条地并**（2026-10-14 补充复审必修 1）：一条检测常常覆盖
    多个槽，它被选中的理由可能只是其中**一个**槽。旧的全量条目
    ``target+anchor+scene`` 为了保住 ``scene`` 被选中时，它**里面那个过期的
    target 框**也一起并了进来 —— 于是新帧的目标哪怕已经移动、哪怕已经**消失**，
    都会被旧帧的高分框顶掉，`select_candidate` 锁在旧位置上，`mixed_frames`
    只是如实标记、拦不住它。

    所以每条入选条目都要记下**它对哪些槽还算最新**
    （``detection_fresh_slots``），由下游（`entity_roles.classify_roles`）
    按槽取用；同一个 key 出现在多条条目里时**留最新的那条** —— 旧写法按时间
    升序并、先到先得，留下的反而是旧帧那个框。
    """
    wanted = tuple(order)

    # 每个槽"最新的那一条"是谁。逐条入选条目据此算出自己对哪些槽还作数。
    latest_by_slot: dict[str, SlotCacheEntry] = {}
    for slot in wanted:
        best: SlotCacheEntry | None = None
        for entry in entries:
            if slot not in entry.slots:
                continue
            if best is None or float(entry.at) > float(best.at):
                best = entry
        if best is not None:
            latest_by_slot[slot] = best

    picked: list[SlotCacheEntry] = []
    for slot in wanted:
        best = latest_by_slot.get(slot)
        if best is None:
            continue
        if not any(best is item for item in picked):
            picked.append(best)
    picked.sort(key=lambda item: float(item.at))

    # key -> (entry.at, 本条在 picked 里的序号, 物体)。同一件东西被多条条目
    # 报过时留**最新**的那次（序号只用来打破同一时刻的并列，保证顺序确定）。
    best_by_key: dict[tuple[str, str], tuple[float, int, dict[str, Any]]] = {}
    frames: set[str] = set()
    for index, entry in enumerate(picked):
        frame = str(entry.frame_id or "")
        age = max(0.0, float(now) - float(entry.at))
        slots = "+".join(sorted(entry.slots))
        fresh = [s for s in wanted
                 if s in entry.slots and latest_by_slot.get(s) is entry]
        if frame:
            frames.add(frame)
        for obj in entry.objects or ():
            item = dict(obj) if isinstance(obj, dict) else {"label": str(obj)}
            if frame:
                # 真实来源帧写在物体自己身上。跨帧的同一 bbox 哈希**不**构成
                # 同一实体（见 `entity_roles.entities_are_the_same`）。
                item["entity_source_frame_id"] = frame
            item["detection_slot"] = slots
            item["detection_fresh_slots"] = list(fresh)
            item["detection_age_s"] = round(age, 3)
            key = _object_key(obj)
            prev = best_by_key.get(key)
            if prev is None or (float(entry.at), index) > prev[:2]:
                best_by_key[key] = (float(entry.at), index, item)

    objects = [
        record[2] for record in sorted(best_by_key.values(), key=lambda rec: rec[:2])
    ]

    return SlotAssembly(
        objects=objects,
        entries=tuple(picked),
        frame_id=frames.pop() if len(frames) == 1 else "",
    )
