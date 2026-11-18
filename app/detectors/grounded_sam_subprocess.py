"""Grounding DINO + SAM2 detector via one persistent external Python worker."""

from __future__ import annotations

from dataclasses import asdict
import json
import os
import re
import subprocess
import tempfile
import uuid
from pathlib import Path
from typing import Any, Mapping, Sequence

from app.config import Settings
from app.detectors.base import BaseObjectDetector, DetectedObject
from app.detectors.process_priority import detector_process_command
from app.detectors.vocabulary import (
    TARGET_TERM_RULES,
    build_detection_prompts,
    build_detection_terms,
    category_for_label,
    color_for_label,
    filter_ram_scene_term_pairs,
    filter_ram_scene_terms,
    normalize_scene_tag,
    resolve_label_zh,
    terms_to_prompt,
)
from app.perception.grounding_prompt_planner import (
    GroundingPromptError,
    GroundingPromptPlanner,
)
from app.task_understanding.schemas import GroundingPromptPlan
from app.reasoning.target_profile import TargetProfile


class DetectorRuntimeError(RuntimeError):
    """Raised when the external detector cannot run or returns invalid output."""


def detection_cache_key(
    image_path: str, prompt: str, slots: Any = None, *,
    backend: str = "", model_id: str = "",
) -> tuple[Any, ...] | None:
    """`node._detect` 的去重键。取不到文件属性时返回 None（= 不缓存）。

    **词槽必须进这个键**（计划书 §5.1.4）：同一个 prompt 在"只跑场景槽"与
    "全跑"下是两次**不同的检测**。共用一个键的后果不是"慢一点"，而是先跑的
    那次把结果喂给后跑的那次 —— 表现是目标时有时无，且随调用顺序变，
    查起来像随机故障。

    放在这里（而不是留在 `run_autonomous_loop` 里）是因为那个文件在宿主
    `.venv` 下 import 不进来（模块顶层就 import rclpy），判据就只能靠读源码。
    """
    try:
        stat = os.stat(image_path)
    except OSError:
        return None
    return (
        os.path.abspath(image_path),
        stat.st_mtime_ns,
        stat.st_size,
        prompt,
        # 去重 + 排序：槽集是**集合**语义，`["scene","target"]` 与
        # `["target","scene"]`（乃至带重复的）必须是同一个键。
        None if slots is None else tuple(sorted({str(item) for item in slots})),
        str(backend),
        str(model_id),
    )


def target_profile_diagnostics(profile: Any) -> dict[str, Any]:
    """把检测器**实际拿到的**目标画像折成可观测字段（计划书 §3.1.2「记生效值」）。

    为什么必须有这一条：网页主入口此前从不给检测器设画像
    （`GroundedSAMSubprocessDetector(target_profile=None)`），而 `_build_prompts`
    对 `None` 是**静默降级**——只是少几个词，不报错、日志里也看不出来。
    没有这几个字段，"画像到底接上没有"就只能在事件流里数词数反推。

    这里回读的是 `detector.target_profile`（构造时传进去的那一个），不是
    settings 里的配置项，也不是"打算用的值"。用 `getattr` 是因为该纯函数要能
    对 stub 检测器跑（测试里不构造真 detector）。
    """
    if profile is None:
        return {
            "target_profile_present": False,
            "target_profile_source": None,
            "target_profile_has_anchors": False,
            "relation_anchor_term_count": 0,
            "context_term_count": 0,
            "detector_term_count": 0,
        }
    anchors = list(getattr(profile, "relation_anchor_labels_en", None) or [])
    context = list(getattr(profile, "context_labels_en", None) or [])
    detector_terms = getattr(profile, "detector_terms", None)
    try:
        term_count = len(detector_terms()) if callable(detector_terms) else 0
    except Exception:  # pragma: no cover - 画像实现异常不该拖垮诊断
        term_count = 0
    return {
        "target_profile_present": True,
        "target_profile_source": getattr(profile, "resolver_source", None),
        "target_profile_has_anchors": bool(anchors) or bool(context),
        "relation_anchor_term_count": len(anchors),
        "context_term_count": len(context),
        "detector_term_count": term_count,
    }


def diagnostics_event_payload(detector: Any, *, frame_id: Any = "") -> dict[str, Any] | None:
    """把一次检测的诊断折成 `detector_diagnostics` 事件的负载；没内容时返回 None。

    抽成模块级纯函数，是因为这三条判据必须能被**直接跑**测到（独立审查 R2），
    而主机循环 `scripts/go2w/run_autonomous_loop.py` 依赖 rclpy，在宿主 .venv
    里 import 不进来 —— 判据留在那边就只能靠读源码来"验证"。

    * `frame_id` 是**这次检测**的帧号，由调用点传进来。不读 `node._latest_frame_id`
      之类的全局"最近一帧"：后台 semantic 线程和前台会互相覆盖它，读全局等于
      把诊断贴到别的帧上。
    * `attempts` 是这一帧发出的**全部** worker 请求（高召回重试、prompt 扩展
      重试各算一次）。只留最后一条会把首次请求的词数/前向次数丢掉，
      "这一帧干了多少活"就低估了。
    * 某次响应没带 diagnostics（旧版 worker）时，那一条明确记
      `diagnostics: null`，顶层 `diagnostics` 也随之置空 —— **不拿上一条的
      历史值冒充这一次的生效值**。
    """
    attempts = list(getattr(detector, "detect_attempts", None) or [])
    available = bool(getattr(detector, "last_diagnostics_available", False))
    diagnostics = getattr(detector, "last_diagnostics", None)
    if not attempts and not (available and diagnostics):
        return None
    return {
        "event": "detector_diagnostics",
        "frame_id": str(frame_id),
        "diagnostics": diagnostics if available else None,
        "diagnostics_available": available,
        "attempts": attempts,
        # 画像字段只是**搭车**，不作为"要不要写这条事件"的理由：第四轮复审刚
        # 认证过"完全没诊断的帧整条事件都不写"，而
        # scripts/go2w/check_perception_rollback_session.sh 正是拿"本会话零条
        # detector_diagnostics"当"尚无法验证"的门。为一个静态配置再加一个
        # 发射理由，会让那道门再也触发不了 —— 那就等于偷偷放松了核验。
        **target_profile_diagnostics(getattr(detector, "target_profile", None)),
    }


class GroundedSAMSubprocessDetector(BaseObjectDetector):
    def __init__(
        self,
        settings: Settings,
        target_profile: TargetProfile | None = None,
        parsed_task: Any | None = None,
        navigation_task: Any | None = None,
        llm_client: Any | None = None,
    ) -> None:
        self.settings = settings
        self.target_profile = target_profile
        self.parsed_task = parsed_task
        self.navigation_task = navigation_task
        self.llm_client = llm_client
        self.grounding_prompt_plan: GroundingPromptPlan | None = None
        self.grounding_prompt_retry_plan: GroundingPromptPlan | None = None
        self._worker: subprocess.Popen | None = None
        self._worker_stderr: Any = None
        # 最近一次响应里 worker 回填的 `diagnostics`（生效的等价优化开关、
        # 词数/前向次数/backbone 次数、profiling 时的分项耗时）。**这是"生效值"
        # 的唯一可信来源** —— settings 只是"打算用的值"，中间还隔着请求体、
        # 命令行默认值和 worker 侧的兜底。
        #
        # 注意它只是"最近一次**带** diagnostics 的响应"：worker 是另一个解释器，
        # 旧版本不回这个字段。要判断"这一次请求有没有拿到诊断"，看
        # `last_diagnostics_available` / `detect_attempts`，别把这里的历史值
        # 当成当前请求的生效值。
        self.last_diagnostics: dict[str, Any] = {}
        self.last_diagnostics_available = False
        # 最近一次 `detect_with_dynamic_terms()` 发出的**全部** worker 请求，
        # 一条一个 dict：request_id / operation / frame_id / source_image，
        # 以及 `diagnostics`（没有就 None + 一句原因）。之所以要留全部而不是
        # 最后一条：高召回重试是第二次请求，只留最后一条会把首次请求的词数、
        # 前向次数整个丢掉，"这一帧到底跑了多少活"就低估了。
        self.detect_attempts: list[dict[str, Any]] = []
        # 请求 ID：让日志能分清"这一次检测"和"worker 又跑了一遍"（高召回重试、
        # 同帧缓存未命中都会造成同一帧出现多条记录）。
        self._request_seq = 0
        self._session_id = uuid.uuid4().hex[:8]
        # RAM++ 最近一次 `_tag` 返回的 en -> zh 配对。只活在这个实例里：
        # 对外接口（`discover_scene_terms`）仍然只给英文词表，
        # GroundingDINO 的 prompt 契约不变（计划书 §6 阶段 B.4）。
        self._ram_term_zh: dict[str, str] = {}

    def configure_target(self, target_profile: TargetProfile | None) -> None:
        # The persistent worker owns only model weights. Prompt construction is
        # client-side, so swapping this immutable profile is sufficient and
        # does not restart CUDA/model state. Callers serialize it with detect().
        self.target_profile = target_profile
        self.grounding_prompt_plan = None
        self.grounding_prompt_retry_plan = None

    def detect(self, image_path: str, target_text: str) -> list[DetectedObject]:
        return self.detect_with_dynamic_terms(image_path, target_text, [])

    def detect_with_dynamic_terms(
        self,
        image_path: str,
        target_text: str,
        dynamic_terms: list[str],
        ram_scene_terms: list[str] | None = None,
        *,
        frame_id: str = "",
        slots: "frozenset[str] | None" = None,
    ) -> list[DetectedObject]:
        image = Path(image_path).resolve()
        if not image.is_file():
            raise FileNotFoundError(f"Image file not found: {image_path}")

        # 本次调用（= 本次「一帧」）发出的请求清单从这里重新开始。放在第一个
        # `_request()` 之前 —— RAM++ 的 tag 请求是调用方在实参里先跑的，
        # 不属于这一次检测，不该混进来。
        self.detect_attempts = []
        # 计划书 §5.1.4：空槽集 = 这一轮一个词都不问。**不**发请求、**不**起
        # worker：分频省下来的必须是这一次前向，而不是"发了再丢"。
        if slots is not None and not slots:
            return []

        self._ensure_worker()

        # prompt 与词槽归属**同一次算出来**：归属记的是"这个框的词是哪个槽问的"，
        # 事后从渲染好的字符串里拆是拆不回来的（上下文词与锚点词同组）。
        prompts, vocab_by_slot = self._slot_prompts(
            target_text, dynamic_terms, ram_scene_terms, slots=slots
        )
        prompt_text = " || ".join(prompts)
        if self.settings.grounding_prompt_require_non_empty and not prompt_text.strip():
            raise GroundingPromptError(
                "GroundingDINO prompt is empty. Detection was not executed."
            )
        payload = self._request(
            image=image,
            text_prompt=prompt_text,
            box_threshold=self.settings.grounding_dino_box_threshold,
            text_threshold=self.settings.grounding_dino_text_threshold,
            frame_id=frame_id,
        )
        if not payload.get("objects"):
            payload = self._maybe_retry_prompt_expansion(
                image=image,
                target_text=target_text,
                previous_prompt=prompt_text,
                previous_payload=payload,
                frame_id=frame_id,
            )
        if not payload.get("objects"):
            payload = self._maybe_retry_high_recall_thresholds(
                image=image,
                text_prompt=(
                    self.grounding_prompt_retry_plan.grounding_prompt
                    if self.grounding_prompt_retry_plan is not None
                    else prompt_text
                ),
                previous_payload=payload,
                frame_id=frame_id,
            )

        return [
            _to_detected_object(
                item,
                self.target_profile,
                # 归一化必须和配对表建表时用的是同一套，否则查不到（表是空的
                # 时候表现为"RAM 中文全丢"，不会报错）。
                runtime_zh=self._ram_term_zh.get(
                    normalize_scene_tag(item.get("label") or "")
                ),
                detection_slots=_slots_for_term(
                    item.get("source_prompt_term") or item.get("label"),
                    vocab_by_slot,
                ),
            )
            for item in payload.get("objects", [])
        ]

    def discover_scene_terms(self, image_path: str) -> list[str]:
        """问 RAM++「这一帧大概有哪些类别」，返回过滤后的英文物体词。

        只用于给 GroundingDINO 扩词，**不返回 embedding、不落库**。
        中文配对不从这里出去（prompt 只要英文），而是存在实例上供
        `_to_detected_object` 查 —— RAM++ 已经算好的中文不能再丢（计划书 §2.2）。
        """
        image = Path(image_path).resolve()
        if not image.is_file():
            raise FileNotFoundError(f"Image file not found: {image_path}")
        self._ensure_worker()
        payload = self._request(image=image, operation="tag")
        pairs = filter_ram_scene_term_pairs(
            list(payload.get("tags_en", [])),
            list(payload.get("tags_zh", [])),
            self.settings.ram_scene_max_terms,
        )
        self._ram_term_zh = {term: zh for term, zh in pairs if zh}
        return [term for term, _zh in pairs]

    def close(self) -> None:
        worker = self._worker
        if worker is None:
            return
        self._worker = None
        try:
            if worker.stdin is not None:
                worker.stdin.close()
            worker.terminate()
            worker.wait(timeout=5)
        except Exception:
            try:
                worker.kill()
            except Exception:
                pass
        if self._worker_stderr is not None:
            try:
                self._worker_stderr.close()
            except Exception:
                pass
            self._worker_stderr = None

    def _ensure_worker(self) -> None:
        if self._worker is not None and self._worker.poll() is None:
            return
        self._worker = None
        env = os.environ.copy()
        if self.settings.grounded_sam_pythonpath:
            existing_pythonpath = env.get("PYTHONPATH")
            env["PYTHONPATH"] = (
                self.settings.grounded_sam_pythonpath
                if not existing_pythonpath
                else f"{self.settings.grounded_sam_pythonpath}:{existing_pythonpath}"
            )
        command = [
            self.settings.grounded_sam_python,
            str(Path(__file__).with_name("grounded_sam_worker.py")),
            "--serve",
            "--root",
            self.settings.grounded_sam_root,
            "--grounding-config",
            self.settings.grounding_dino_config,
            "--grounding-checkpoint",
            self.settings.grounding_dino_checkpoint,
            "--sam2-config",
            self.settings.sam2_config,
            "--sam2-checkpoint",
            self.settings.sam2_checkpoint,
            "--device",
            self.settings.detection_device,
        ]
        if not self.settings.enable_sam2:
            command.append("--disable-sam2")
        if self.settings.ram_plus_root:
            command.extend(["--ram-root", self.settings.ram_plus_root])
        if self.settings.ram_plus_checkpoint:
            command.extend(["--ram-checkpoint", self.settings.ram_plus_checkpoint])
        if not self.settings.enable_ram_plus:
            command.append("--disable-ram-plus")
        self._worker_stderr = tempfile.NamedTemporaryFile(
            prefix="grounded_sam_worker_", suffix=".log", delete=False
        )
        self._worker = subprocess.Popen(
            detector_process_command(command),
            cwd=self.settings.grounded_sam_root,
            env=env,
            text=True,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self._worker_stderr,
        )

    def _request(
        self,
        *,
        image: Path,
        text_prompt: str = "",
        box_threshold: float = 0.0,
        text_threshold: float = 0.0,
        operation: str = "detect",
        frame_id: str = "",
    ) -> dict[str, Any]:
        self._ensure_worker()
        self._request_seq += 1
        request = {
            "operation": operation,
            "image": str(image),
            "text_prompt": text_prompt,
            "box_threshold": box_threshold,
            "text_threshold": text_threshold,
            "max_objects": self.settings.max_detected_objects,
            # 源帧号随请求走，再由 worker 回填进 diagnostics —— 这样"这一次
            # 耗时/计数是哪一帧的"由请求自己带，不依赖客户端记住"最近一帧"。
            "frame_id": str(frame_id or ""),
            # 阶段一的等价优化开关：和阈值一样**只走请求**，不走命令行 ——
            # worker 侧是 `request.get(key, args.key)`，请求永远优先，命令行那份
            # 对 --serve 路径是够不着的死路径。worker 会把**生效值**连同
            # 词数/前向次数/backbone 次数一起回填到 diagnostics。
            "dino_frame_feature_cache": self.settings.enable_dino_frame_feature_cache,
            "mask_rle_encoder": self.settings.mask_rle_encoder,
            # §5.1.6：多词分组前向。与上面两个同一条约定 —— **只走请求体**，
            # 所以改它不用重启任何进程（worker 是 `request.get(key, args.key)`，
            # 请求永远优先）。worker 回填的 `prompt_mode` 是生效值，
            # `prompt_mode_requested` 是这里发过去的值。
            "dino_prompt_mode": self.settings.go2w_dino_prompt_mode,
            "profile_detection": self.settings.detection_profile_enabled,
            "request_id": f"{self._session_id}-{self._request_seq}",
            # 轻量化计划 §20。**只走请求、不加命令行参数**：四个阈值里其余三个
            # 两边都传，但 worker 的 `request.get(key, args.key)` 永远优先取请求值
            # —— 命令行那份对 --serve 路径是够不着的死路径（看着像生效、其实从不
            # 生效）。worker 的 argparse 默认值已经等于这里的默认值，手动跑
            # --image 那条路照样有门，所以少传一份不丢能力。
            "sam2_min_box_score": self.settings.sam2_min_box_score,
        }
        assert self._worker is not None and self._worker.stdin is not None
        self._worker.stdin.write(json.dumps(request, ensure_ascii=False) + "\n")
        self._worker.stdin.flush()
        payload = self._read_response()
        attempt: dict[str, Any] = {
            "request_id": request["request_id"],
            "operation": operation,
            "frame_id": request["frame_id"],
            "source_image": str(image),
        }
        diagnostics = payload.get("diagnostics")
        if isinstance(diagnostics, dict):
            # 只在有内容时覆盖：worker 是另一个解释器，旧版本不回这个字段，
            # 用空字典把上一次的记录抹掉会让"生效值"看起来像"没生效"。
            self.last_diagnostics = diagnostics
            self.last_diagnostics_available = True
            attempt["diagnostics"] = diagnostics
        else:
            # **不冒充**：这一次响应没有诊断，就明确记成"没有"，而不是把
            # `last_diagnostics` 里上一条请求的历史值当成这一次的生效值。
            self.last_diagnostics_available = False
            attempt["diagnostics"] = None
            attempt["diagnostics_unavailable"] = (
                "worker response carried no diagnostics (older worker?)"
            )
        self.detect_attempts.append(attempt)
        return payload

    def _read_response(self) -> dict[str, Any]:
        assert self._worker is not None and self._worker.stdout is not None
        while True:
            line = self._worker.stdout.readline()
            if line == "":
                raise DetectorRuntimeError(self._worker_died_message())
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue  # library stdout noise, not a response
            if "error" in payload:
                raise DetectorRuntimeError(
                    "Grounding DINO/SAM2 worker error: " + str(payload["error"])
                )
            return payload

    def _worker_died_message(self) -> str:
        tail = ""
        if self._worker_stderr is not None:
            try:
                self._worker_stderr.seek(0)
                tail = self._worker_stderr.read()[-2000:]
            except Exception:
                tail = ""
            if isinstance(tail, bytes):
                tail = tail.decode("utf-8", errors="replace")
        return "Grounding DINO/SAM2 worker exited unexpectedly. stderr tail:\n" + tail

    def _build_prompts(
        self,
        target_text: str,
        dynamic_terms: list[str],
        ram_scene_terms: list[str] | None = None,
        *,
        slots: "frozenset[str] | None" = None,
    ) -> list[str]:
        """把词表拼成 prompt。

        ``slots`` 是**词槽子集**（计划书 §5.1.4 的检测分频）：``None`` = 全跑
        （与历史行为逐字一致）；给了集合就只放集合里的槽 —— ``target`` 管目标
        词+上下文词、``anchor`` 管关系锚点词、``scene`` 管 RAM 场景词与 static
        base 词表。空集合直接返回空表，让调用方这一轮一次检测都不发。

        为什么要在这一层切：分频如果只改调用点的 prompt 字符串，是切不动的 ——
        画像里的目标词/上下文词/锚点词是**这里**才拼进去的，调用点根本看不到。

        词槽归属一并算在 `_slot_prompts` 里，这里只取 prompt —— **一份实现**，
        两处各写一套必然会漂。
        """
        return self._slot_prompts(
            target_text, dynamic_terms, ram_scene_terms, slots=slots
        )[0]

    def _slot_prompts(
        self,
        target_text: str,
        dynamic_terms: list[str],
        ram_scene_terms: list[str] | None = None,
        *,
        slots: "frozenset[str] | None" = None,
    ) -> "tuple[list[str], dict[str, list[str]]]":
        """``(prompts, vocab_by_slot)``：词表拼成的 prompt，**以及每个词属于哪个槽**。

        为什么要这份归属（2026-10-14 第四次补充复审·必修）：分频按**条目**存
        检测结果，而一条检测常常覆盖多个槽。旧的全量条目为了保住 ``scene`` 被
        组装选中时，它里面那个**目标槽**问出来的框也跟着留下 —— 画像的上下文
        词（`context_labels_en`，如「打印机」画像的 ``paper tray``）就是这一类：
        它不属于 `classify_roles` 的目标词/锚点词，于是旧帧 0.95 的 ``paper
        tray`` 会在默认选框里压过新帧 0.20 的 ``printer``，新帧连打印机都没有时
        还会报 PRESENT。`detection_fresh_slots` 只说得出"这条目对哪些槽还算最
        新"，说不出"这个框是哪个槽问出来的" —— 必须按词记账。

        归属只有这里算得出来（词表就在这一层）。实际结果回来后，
        `_slots_for_term` 按完整查询短语认领槽；角色分类另用宽松匹配。
        """
        ram_scene_terms = ram_scene_terms or []
        # 参数默认是 None（没跑 RAM++ 就是没有场景词），下面两个分支都按 list 用。
        include_target = slots is None or "target" in slots
        include_anchor = slots is None or "anchor" in slots
        include_scene = slots is None or "scene" in slots
        if slots is not None and not slots:
            return [], {}
        vocab: dict[str, list[str]] = {}

        def _note(slot: str, terms: "Any") -> None:
            bucket = vocab.setdefault(slot, [])
            for term in terms or ():
                text = str(term).strip()
                if text and text not in bucket:
                    bucket.append(text)

        if (
            self.settings.grounding_prompt_llm_expansion_enabled
            and self.parsed_task is not None
        ):
            planner = GroundingPromptPlanner(
                llm_client=self.llm_client,
                settings=self.settings,
                max_terms=self.settings.grounding_prompt_max_terms,
                min_terms=self.settings.grounding_prompt_min_terms,
            )
            self.grounding_prompt_plan = planner.build(
                parsed_task=self.parsed_task,
                navigation_task=self.navigation_task,
                target_profile=self.target_profile,
            )
            self._write_prompt_plan(
                self.grounding_prompt_plan,
                self.settings.grounding_prompt_debug_output,
            )
            # 这一支的锚点词由 planner 自己决定，切不出独立的 anchor 槽 ——
            # 如实记在报告里：`grounding_prompt_llm_expansion_enabled=true` 时
            # anchor 槽与 target 槽联动，只有 scene 槽是独立可切的。
            plan_terms = (
                _prompt_to_terms(self.grounding_prompt_plan.grounding_prompt)
                if include_target
                else []
            )
            terms = list(plan_terms)
            if include_target:
                terms.extend(dynamic_terms)
                _note("target", [*plan_terms, *dynamic_terms])
            if include_scene:
                # RAM 词排在 planner 的目标词之后，目标词永远不会被截断。
                scene_terms = filter_ram_scene_terms(
                    ram_scene_terms, self.settings.ram_scene_max_terms
                )
                terms.extend(scene_terms)
                _note("scene", scene_terms)
            prompt = _terms_to_prompt(_dedupe_terms(terms))
            return ([prompt] if prompt else []), vocab

        target_terms = (
            self.target_profile.detector_terms()
            if self.target_profile is not None and include_target
            else []
        )
        context_terms: list[str] = []
        if self.target_profile is not None:
            if include_target:
                context_terms.extend(self.target_profile.context_labels_en)
            if include_anchor:
                # §15：关系锚点是关系目标的一半，必须一起搜。
                context_terms.extend(self.target_profile.relation_anchor_labels_en)
        # 词槽归属：**只有这一处**知道哪张词表属于哪个槽。三份各自记账，是因为
        # 上下文词与锚点词被 `build_detection_prompts` 并进了同一个 prompt 组 ——
        # 渲染出来的字符串再也分不开它们，只能在拼之前记下来。
        if include_target:
            _note("target", [*target_terms, *dynamic_terms])
            if self.target_profile is not None:
                _note("target", self.target_profile.context_labels_en)
        if include_anchor and self.target_profile is not None:
            _note("anchor", self.target_profile.relation_anchor_labels_en)
        # §4.1：目标画像的上下文词（如「挂着书包的椅子」的 backpack）属于
        # mandatory target terms，必须无条件进 prompt —— 它跟 static base 词表
        # 是两回事，static_object_prompts_enabled 只该管后者。
        # 少这一条，「挂着书包的椅子」就只搜 chair，搜不到单独的 backpack（§15）。
        prompts = build_detection_prompts(
            target_text if include_target else "",
            dynamic_terms=(
                [*target_terms, *dynamic_terms] if include_target else []
            ),
            context_terms=(
                context_terms if self.target_profile is not None else None
            ),
            include_base_terms=(
                self.settings.static_object_prompts_enabled
                and not target_terms
                and include_scene
            ),
        )
        # RAM++ 的场景词单独占一个 prompt 槽位，不混进目标词那一槽。
        # 否则 RAM 返回的 tag 一多，目标词就会被该槽位的 16 个名额挤掉。
        ram_terms = (
            filter_ram_scene_terms(
                ram_scene_terms, self.settings.ram_scene_max_terms
            )
            if include_scene
            else []
        )
        if ram_terms:
            ram_prompt = terms_to_prompt(ram_terms)
            if ram_prompt not in prompts:
                prompts.append(ram_prompt)
        if include_scene:
            _note("scene", ram_terms)
        prompts = [p for p in prompts if p.strip()]
        if not prompts:
            if not include_scene:
                # 一个槽都没开、或只开了 target/anchor 而它们没词：**不要**拿
                # static base 词表兜底 —— 那等于偷偷又跑了一次场景检测，分频白做。
                return [], {}
            prompts = build_detection_prompts(
                target_text if include_target else "",
                include_base_terms=self.settings.static_object_prompts_enabled,
            )
        # static base 词表（默认**关着**）：它由场景槽问出去，算 scene。但**只有
        # 这一轮真把 base prompt 发出去了**才能记账 —— 没发还记账，等于给一批
        # 本轮根本没被问过的词安上 scene 归属，目标槽问出来的框就会借场景槽的
        # 新鲜度留下来，正是这次要修的毛病。判据用"渲染出来的字符串在不在这轮
        # prompt 里"，不重算开关条件（重算必然与 `build_detection_prompts` 的
        # 内部判据漂开）。`TARGET_TERM_RULES` 命中 `target_text` 的那几个词也算
        # 在这张表里，但它们是**目标词**，归 target —— 否则直接目标词框会在场景
        # 槽过期时被当过期证据丢掉（第三次补充复审刚修过的那条会复发）。
        base_terms = build_detection_terms(
            target_text if include_target else "",
            dynamic_terms=(
                [*target_terms, *dynamic_terms] if include_target else []
            ),
        )
        base_hits = [
            term
            for key, values in TARGET_TERM_RULES.items()
            if key in (target_text if include_target else "")
            for term in values
        ]
        # base 词表**没发出去**时这几个词也照样会进目标 prompt（`build_detection_
        # prompts` 自己会把它们并进 target_terms），所以归 target 这一条不设门。
        _note("target", base_hits)
        if terms_to_prompt(base_terms) in prompts:
            _note(
                "scene",
                [term for term in base_terms if term not in set(base_hits)],
            )
        return prompts, vocab

    def _maybe_retry_prompt_expansion(
        self,
        *,
        image: Path,
        target_text: str,
        previous_prompt: str,
        previous_payload: dict[str, Any],
        frame_id: str = "",
    ) -> dict[str, Any]:
        if (
            self.grounding_prompt_plan is None
            or not self.settings.grounding_prompt_retry_on_empty
            or self.settings.grounding_prompt_max_retries <= 0
        ):
            return previous_payload
        planner = GroundingPromptPlanner(
            llm_client=self.llm_client,
            settings=self.settings,
            max_terms=self.settings.grounding_prompt_max_terms,
            min_terms=self.settings.grounding_prompt_min_terms,
        )
        summary = {
            "num_raw_candidates": len(previous_payload.get("objects", [])),
            "target": target_text,
        }
        self.grounding_prompt_retry_plan = planner.build_retry(
            parsed_task=self.parsed_task,
            previous_prompt=previous_prompt,
            detection_summary=summary,
            target_profile=self.target_profile,
            previous_plan=self.grounding_prompt_plan,
        )
        self._write_prompt_plan(
            self.grounding_prompt_retry_plan,
            self.settings.grounding_prompt_retry_debug_output,
        )
        return self._request(
            image=image,
            text_prompt=self.grounding_prompt_retry_plan.grounding_prompt,
            box_threshold=self.settings.grounding_dino_box_threshold,
            text_threshold=self.settings.grounding_dino_text_threshold,
            frame_id=frame_id,
        )

    def _maybe_retry_high_recall_thresholds(
        self,
        *,
        image: Path,
        text_prompt: str,
        previous_payload: dict[str, Any],
        frame_id: str = "",
    ) -> dict[str, Any]:
        if not (
            self.settings.enable_gdino_high_recall
            and (
                self.settings.grounding_dino_box_threshold
                > self.settings.grounding_dino_high_recall_box_threshold
                or self.settings.grounding_dino_text_threshold
                > self.settings.grounding_dino_high_recall_text_threshold
            )
        ):
            return previous_payload
        try:
            return self._request(
                image=image,
                text_prompt=text_prompt,
                box_threshold=self.settings.grounding_dino_high_recall_box_threshold,
                text_threshold=self.settings.grounding_dino_high_recall_text_threshold,
                frame_id=frame_id,
            )
        except DetectorRuntimeError:
            return previous_payload

    @staticmethod
    def _write_prompt_plan(plan: GroundingPromptPlan, path_value: str) -> None:
        path = Path(path_value)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(asdict(plan), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


def _slots_for_term(
    term: Any, vocab_by_slot: "Mapping[str, Sequence[str]]"
) -> tuple[str, ...]:
    """这个词是**哪些槽**问出来的（检测分频的词槽归属）。

    默认逐词 worker 把实际查询短语原样写入 `source_prompt_term`，所以来源
    必须按**完整短语**匹配。角色分类的子串匹配不能用于这里：场景词 `tray`
    会误认目标槽查询的 `paper tray`，让旧目标框借场景槽的新鲜度留下。
    匹配不上（重试 prompt 另拼的词、实验分组模式吐出的新短语）返回空元组；
    下游按来源未知的旧行为处理，不凭空判它过期。
    """
    text = _normalize_prompt_term(term)
    if not text:
        return ()
    return tuple(
        slot
        for slot, terms in (vocab_by_slot or {}).items()
        if any(_normalize_prompt_term(candidate) == text for candidate in terms)
    )


def _to_detected_object(
    item: dict,
    target_profile: TargetProfile | None = None,
    runtime_zh: str | None = None,
    detection_slots: "Sequence[str]" = (),
) -> DetectedObject:
    label = str(item.get("label") or "object").lower().strip()
    bbox = item.get("bbox_2d") or [0.0, 0.0, 1.0, 1.0]
    attributes = list(item.get("attributes") or [])
    attributes.append("detected_by_grounding_dino")
    if item.get("mask_area_ratio") is not None:
        attributes.append("segmented_by_sam2")
        attributes.append(f"mask_area_ratio={float(item['mask_area_ratio']):.4f}")

    is_direct_target = (
        target_profile is not None
        and _matches_any_term(label, target_profile.direct_terms())
    )
    # `runtime_zh` 是 RAM++ 同一次 `_tag` 返回的中文配对。它优先于静态词典：
    # RAM 说是「容器」就不该被我们写死成「垃圾桶」（计划书 §6 阶段 A.3）。
    label_zh_value = resolve_label_zh(
        label,
        runtime_zh=runtime_zh,
        target_zh=(
            target_profile.canonical_name_zh
            if is_direct_target and target_profile is not None
            else None
        ),
    )
    return DetectedObject(
        label=label,
        label_zh=label_zh_value,
        category=(
            target_profile.target_type
            if is_direct_target and target_profile is not None
            else category_for_label(label)
        ),
        color=color_for_label(label),
        bbox_2d=(
            _clamp(float(bbox[0])),
            _clamp(float(bbox[1])),
            _clamp(float(bbox[2])),
            _clamp(float(bbox[3])),
        ),
        score=_clamp(float(item.get("score", 0.5))),
        text_score=(
            None
            if item.get("text_score") is None
            else _clamp(float(item["text_score"]))
        ),
        attributes=attributes,
        mask_area_ratio=(
            None
            if item.get("mask_area_ratio") is None
            else _clamp(float(item["mask_area_ratio"]))
        ),
        mask_rle=_mask_rle(item.get("mask_rle")),
        mask_size=_mask_size(item.get("mask_size")),
        source="grounding_dino_sam2",
        source_prompt_term=str(item.get("source_prompt_term") or label),
        detection_slots=tuple(str(slot) for slot in detection_slots or ()),
    )


def _mask_rle(value: Any) -> list[list[int]] | None:
    if not isinstance(value, list) or not value:
        return None
    runs: list[list[int]] = []
    for run in value:
        if isinstance(run, list) and len(run) == 3:
            runs.append([int(run[0]), int(run[1]), int(run[2])])
    return runs or None


def _mask_size(value: Any) -> tuple[int, int] | None:
    if not isinstance(value, list) or len(value) != 2:
        return None
    try:
        return (int(value[0]), int(value[1]))
    except (TypeError, ValueError):
        return None


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


def _matches_any_term(label: str, terms: list[str]) -> bool:
    normalized_label = " ".join(
        re.sub(r"[^a-z0-9一-鿿]+", " ", label.lower()).split()
    )
    for term in terms:
        normalized_term = " ".join(
            re.sub(r"[^a-z0-9一-鿿]+", " ", term.lower()).split()
        )
        if normalized_term and normalized_term in normalized_label:
            return True
    return False


def _normalize_prompt_term(term: Any) -> str:
    """只折叠大小写、分隔符和空白；不把短词扩成较长查询词。"""
    return " ".join(re.sub(r"[^a-z0-9一-鿿]+", " ", str(term or "").lower()).split())


def _prompt_to_terms(prompt: str) -> list[str]:
    return [item.strip() for item in re.split(r"\s*\.\s*", prompt) if item.strip()]


def _terms_to_prompt(terms: list[str]) -> str:
    cleaned = _dedupe_terms(terms)
    return " . ".join(cleaned) + (" ." if cleaned else "")


def _dedupe_terms(terms: list[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for term in terms:
        normalized = " ".join(
            re.sub(r"[^a-z0-9 -]+", " ", str(term).lower()).split()
        ).strip()
        if not normalized or normalized in seen:
            continue
        seen.add(normalized)
        result.append(normalized)
    return result
