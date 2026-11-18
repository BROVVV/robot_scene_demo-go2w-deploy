"""Request / response / session models for the autonomous search WebUI
(plan book §10, §21, §37).  The project uses plain dataclasses, not pydantic.

计划书 §3-C 收口后的边界：请求只表达四件事 —— **任务、预算、检测模式、
干跑/运动授权**。后端、reasoner、空间提供者与 RGB-D/空间探索档位是生产
Web 的固定配置（来自 ``load_search_defaults()``），请求体不再能选它们。
离线 mock 由测试直接注入 ``InProcessMockExecutor``，不从这里伪装。
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any
from uuid import uuid4

MAX_TARGET_LENGTH = 500

#: 生产搜索固定使用的 reasoner：``run_semantic_exploration.py --reasoner`` 的
#: 现有 CLI 名。请求体不再提供选择项，worker 命令固定传它；旧内部值
#: ``semantic`` 只是这个 CLI 名的别名，不再出现在会话归档里。
SEARCH_REASONER = "semantic_navigation"


@dataclass
class SearchStartRequest:
    """Body of ``POST /api/search/start`` (plan book §10)."""

    # ``target`` is retained for one release as an input compatibility alias.
    # The service immediately normalizes it to task_text and then parses it.
    target: str = ""
    task_text: str = ""
    # Normal production path is Nav2. The legacy relative-motion backend is
    # available only in an explicitly started, operator-supervised trial.
    backend: str = "nav2"  # nav2 | go2w_experimental (opt-in) | mocks
    # 物体识别后端，透传给 --detector（取值与那条 argparse 的 choices 一致）。
    detector: str = "llm"  # llm | grounded_sam | yoloe_seg
    finish_on_visual_confirmation: bool = False
    turn_only: bool = False
    enable_autonomous_motion: bool = False
    # Explicitly carried through the WebUI -> worker -> semantic runner
    # boundary.  For a real Go2-W motion session this is the repository's
    # existing operator_supervised_experiment profile, not a new gate.
    operator_supervised_experiment: bool = False
    dry_run_motion: bool = False
    allow_degraded: bool = False
    # 以下四项是生产固定档位：``from_dict`` 只从配置读，请求体不再能选。
    # RGB-D spatial exploration (D435 atomic HTTP source)
    rgbd_source: bool = True
    rgbd_base_url: str = "http://192.168.123.18:8080"
    # Live semantic spatial exploration loop
    spatial_v2: bool = True
    # PandarXT-16 plain_slam supplies the metric map/frontiers.  ``rtabmap``
    # 已从请求模型删除：这台狗的空间提供者是 plain_slam，RTAB-Map 只是
    # CLI 的离线诊断选项。
    spatial_provider: str = "plain_slam"
    # optional budget overrides
    max_seconds: float | None = None
    max_planning_cycles: int | None = None
    max_motion_steps: int | None = None
    llm_model: str | None = None
    verify_min_confidence: float | None = None

    def __post_init__(self) -> None:
        # Keep direct dataclass construction compatible with clients that
        # still send the one-field form; the service only consumes task_text.
        if not self.task_text and self.target:
            self.task_text = self.target.strip()

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "SearchStartRequest":
        defaults = load_search_defaults()
        enable_motion = bool(
            value.get(
                "enable_autonomous_motion",
                defaults["enable_autonomous_motion"],
            )
        )
        operator_supervised = bool(
            value.get(
                "operator_supervised_experiment",
                defaults.get("operator_supervised_experiment", enable_motion),
            )
        )
        backend = str(value.get("backend") or defaults["backend"])
        # The normal WebUI profile is read-only.  Make that contract explicit
        # at the request boundary so a real backend cannot reach the worker
        # without either a motion opt-in or an explicit dry-run flag.
        dry_run_motion = bool(
            value.get(
                "dry_run_motion",
                backend not in {"mock", "mock_metric"}
                and not enable_motion
                and not operator_supervised,
            )
        )
        allow_degraded = bool(value.get("allow_degraded", dry_run_motion))
        return cls(
            task_text=str(value.get("task_text") or value.get("target") or "").strip(),
            target=str(value.get("target") or "").strip(),
            backend=backend,
            detector=str(value.get("detector") or defaults["detector"]),
            finish_on_visual_confirmation=bool(
                value.get(
                    "finish_on_visual_confirmation",
                    defaults["finish_on_visual_confirmation"],
                )
            ),
            turn_only=bool(value.get("turn_only", False)),
            enable_autonomous_motion=enable_motion,
            operator_supervised_experiment=operator_supervised,
            dry_run_motion=dry_run_motion,
            allow_degraded=allow_degraded,
            # RGB-D / 空间探索 / 空间提供者是生产固定档位：只从配置读，
            # 请求体不再能选（旧 ``rtabmap`` 字段已删）。
            rgbd_source=bool(defaults["rgbd_source"]),
            rgbd_base_url=str(defaults["rgbd_base_url"]),
            spatial_v2=bool(defaults["spatial_v2"]),
            spatial_provider=str(defaults["spatial_provider"]),
            max_seconds=_optional_float(value.get("max_seconds")),
            max_planning_cycles=_optional_int(value.get("max_planning_cycles")),
            max_motion_steps=_optional_int(value.get("max_motion_steps")),
            llm_model=value.get("llm_model"),
            verify_min_confidence=_optional_float(value.get("verify_min_confidence")),
        )

    def validate(
        self, *, allow_mock: bool = False,
        allow_legacy_supervised: bool = False,
        legacy_motion_armed: bool = False,
    ) -> str | None:
        """Return an error message or None when the request is acceptable.

        ``allow_mock`` 是「这个 app 实例是测试台」的标记，由调用方按
        ``SearchSessionService._executor_factory_is_mock`` 传进来，默认 False。
        计划书 §6.6-2：生产搜索请求只接受 ``nav2``，测试代码直接注入 mock，
        不经生产请求参数暴露——否则给真机 WebUI 发一个 ``{"backend": "mock"}``
        就会跑出一场编造的搜索，而界面上看不出任何区别。
        """
        task_text = (self.task_text or self.target).strip()
        if not task_text:
            return "task_text/target is required"
        if len(task_text) > MAX_TARGET_LENGTH:
            return f"task_text too long (max {MAX_TARGET_LENGTH} chars)"
        allowed_backends = {"nav2"} | ({"mock", "mock_metric"} if allow_mock else set())
        if allow_legacy_supervised:
            allowed_backends.add("go2w_experimental")
        if self.backend not in allowed_backends:
            return f"unsupported backend: {self.backend}"
        if self.backend == "go2w_experimental" and not self.dry_run_motion:
            if not (legacy_motion_armed and self.enable_autonomous_motion
                    and self.operator_supervised_experiment):
                return "legacy supervised motion requires launcher arm and operator supervision"
        # 请求体已经选不了空间提供者（``from_dict`` 只从配置读），这道门留着
        # 是为了挡住直接构造出的对象。``rtabmap`` 不再是合法取值：这台狗的
        # 提供者是 plain_slam，camera 只是它自带的退化档。
        if self.spatial_provider not in {"camera", "plain_slam", "nav2"}:
            return f"unsupported spatial_provider: {self.spatial_provider}"
        # 取值集合逐字对齐 run_semantic_exploration.py 的 --detector choices；
        # 不在这里拦，非法值要等 worker 起来才炸成 argparse 错误。
        if self.detector not in {"llm", "grounded_sam", "yoloe_seg"}:
            return f"unsupported detector: {self.detector}"
        return None


@dataclass
class SearchSessionInfo:
    """Lightweight session record for history / state endpoints."""

    session_id: str
    target: str
    status: str
    task_text: str = ""
    task_context: dict[str, Any] = field(default_factory=dict)
    result: str = ""
    started_at: float | None = None
    finished_at: float | None = None
    backend: str = ""
    reasoner: str = ""
    detector: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def new_session_id() -> str:
    return f"search_{time.strftime('%Y%m%d_%H%M%S')}_{uuid4().hex[:8]}"


_DEFAULT_SEARCH_CONFIG = "configs/go2w/autonomous_search_web.yaml"
_PROJECT_ROOT = Path(__file__).resolve().parents[2]


def load_search_defaults() -> dict[str, Any]:
    """Defaults for a start request (plan book §117).

    Precedence: request fields > AUTONOMOUS_SEARCH_* env vars >
    ``configs/go2w/autonomous_search_web.yaml`` > built-in defaults.

    配置**读取失败一律显式抛错**：早先这里把所有异常都吞掉，于是一份写坏的
    yaml（或读不到的路径）会让 WebUI 静默退回另一套默认值 —— 后端和检测器
    悄悄换了，界面上看不出任何区别。缺文件不算读取失败（那是"没配"）。
    """
    import os

    defaults: dict[str, Any] = {
        "backend": "nav2",
        # 物体识别走哪条路，直接透传给 run_semantic_exploration.py 的 --detector：
        #   grounded_sam —— 本地 GroundingDINO+SAM2 出真实 mask，因而每个物体都有
        #                   3D box（box_size）；目标的复核仍由 VLM 做（verify）。
        #   llm          —— 云端 VLM 出全场景文本，产不出 mask ⇒ box_size 恒为 None，
        #                   WebUI 上物体只能画成红点。
        "detector": "llm",
        # 计划书 §18.2：默认走完目标接近再收工，不在视觉确认那一刻就 finish。
        "finish_on_visual_confirmation": False,
        "enable_autonomous_motion": False,
        "operator_supervised_experiment": False,
        # 生产固定档位（请求体不再能选）：见 SearchStartRequest 的注释。
        "rgbd_source": True,
        "rgbd_base_url": "http://192.168.123.18:8080",
        "spatial_v2": True,
        "spatial_provider": "plain_slam",
    }
    config_path = _PROJECT_ROOT / _DEFAULT_SEARCH_CONFIG
    if config_path.is_file():
        import yaml

        try:
            data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise RuntimeError(
                f"搜索默认配置 {config_path} 读取失败: {exc}"
            ) from exc
        if data is not None and not isinstance(data, dict):
            raise RuntimeError(f"搜索默认配置顶层必须是映射: {config_path}")
        search = (data or {}).get("search") or {}
        if not isinstance(search, dict):
            raise RuntimeError(f"搜索默认配置的 search 段必须是映射: {config_path}")
        for key in ("backend", "detector"):
            if search.get(key):
                defaults[key] = str(search[key])
        if search.get("finish_on_visual_confirmation") is not None:
            defaults["finish_on_visual_confirmation"] = bool(
                search["finish_on_visual_confirmation"]
            )
        if search.get("rgbd_source") is not None:
            defaults["rgbd_source"] = bool(search["rgbd_source"])
        if search.get("rgbd_base_url"):
            defaults["rgbd_base_url"] = str(search["rgbd_base_url"])
        if search.get("spatial_v2") is not None:
            defaults["spatial_v2"] = bool(search["spatial_v2"])
        if search.get("spatial_provider"):
            defaults["spatial_provider"] = str(search["spatial_provider"])
        if search.get("enable_autonomous_motion") is not None:
            defaults["enable_autonomous_motion"] = bool(
                search["enable_autonomous_motion"]
            )
        if search.get("operator_supervised_experiment") is not None:
            defaults["operator_supervised_experiment"] = bool(
                search["operator_supervised_experiment"]
            )
    # The camera service moves onto the robot with this deployment. Use the
    # same trusted host setting as the ROS bridge; request bodies cannot
    # override the production sensor endpoint.
    camera_url = os.getenv("D435_BASE_URL", "").strip()
    if camera_url:
        defaults["rgbd_base_url"] = camera_url.rstrip("/")
    if os.getenv("AUTONOMOUS_SEARCH_DEFAULT_BACKEND"):
        defaults["backend"] = os.getenv("AUTONOMOUS_SEARCH_DEFAULT_BACKEND")
    if os.getenv("AUTONOMOUS_SEARCH_DEFAULT_DETECTOR"):
        defaults["detector"] = os.getenv(
            "AUTONOMOUS_SEARCH_DEFAULT_DETECTOR", "grounded_sam"
        ).strip()
    motion_env = os.getenv("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION")
    if motion_env is not None:
        defaults["enable_autonomous_motion"] = motion_env.strip().lower() in {
            "1", "true", "yes", "on",
        }
    if os.getenv("AUTONOMOUS_SEARCH_OPERATOR_SUPERVISED") is not None:
        defaults["operator_supervised_experiment"] = os.getenv(
            "AUTONOMOUS_SEARCH_OPERATOR_SUPERVISED", ""
        ).strip().lower() in {"1", "true", "yes", "on"}
    elif defaults["enable_autonomous_motion"]:
        # Backward-compatible launcher contract: requesting real autonomous
        # motion means using the existing supervised experiment profile.
        defaults["operator_supervised_experiment"] = True
    if os.getenv("AUTONOMOUS_SEARCH_SPATIAL_PROVIDER"):
        # 非 plain_slam/camera 的取值会在这里读进去、然后被 validate() 显式拒掉
        # （而不是悄悄跑成另一条链路）。
        defaults["spatial_provider"] = os.getenv(
            "AUTONOMOUS_SEARCH_SPATIAL_PROVIDER", "plain_slam"
        ).strip()
    return defaults


def _optional_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _optional_int(value: Any) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None
