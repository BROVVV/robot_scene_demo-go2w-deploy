"""Request / response / session models for the autonomous search WebUI
(plan book §10, §21, §37).  The project uses plain dataclasses, not pydantic.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any
from uuid import uuid4

MAX_TARGET_LENGTH = 500


@dataclass
class SearchStartRequest:
    """Body of ``POST /api/search/start`` (plan book §10)."""

    # ``target`` is retained for one release as an input compatibility alias.
    # The service immediately normalizes it to task_text and then parses it.
    target: str = ""
    task_text: str = ""
    reasoner: str = "semantic"
    backend: str = "go2w_experimental"  # go2w_experimental | mock | mock_metric
    finish_on_visual_confirmation: bool = True
    turn_only: bool = False
    enable_autonomous_motion: bool = False
    # Explicitly carried through the WebUI -> worker -> semantic runner
    # boundary.  For a real Go2-W motion session this is the repository's
    # existing operator_supervised_experiment profile, not a new gate.
    operator_supervised_experiment: bool = False
    dry_run_motion: bool = False
    allow_degraded: bool = False
    # RGB-D spatial exploration (D435 atomic HTTP source)
    rgbd_source: bool = False
    rgbd_base_url: str = "http://192.168.123.18:8080"
    # Live semantic spatial exploration loop
    spatial_v2: bool = False
    # Explicit provider selection.  This keeps metric LiDAR mapping distinct
    # from the legacy RTAB-Map compatibility switch.
    spatial_provider: str = "camera"
    # Use RTAB-Map ROS2 topics as the SpatialProvider
    rtabmap: bool = False
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
        requested_reasoner = str(value.get("reasoner") or "semantic")
        # Migrate historical goal-selector values without persisting their
        # old branding into the new session or event stream.
        if requested_reasoner.lower() in {"semantic_navigation"} or requested_reasoner.lower().endswith("goal"):
            requested_reasoner = "semantic"
        backend = str(value.get("backend") or defaults["backend"])
        operator_supervised = bool(
            value.get(
                "operator_supervised_experiment",
                defaults.get("operator_supervised_experiment", enable_motion),
            )
        )
        # The normal WebUI profile is read-only.  Make that contract explicit
        # at the request boundary so a real backend cannot reach the worker
        # without either a motion opt-in or an explicit dry-run flag.
        dry_run_motion = bool(
            value.get(
                "dry_run_motion",
                backend == "go2w_experimental"
                and not enable_motion
                and not operator_supervised,
            )
        )
        allow_degraded = bool(value.get("allow_degraded", dry_run_motion))
        return cls(
            task_text=str(value.get("task_text") or value.get("target") or "").strip(),
            target=str(value.get("target") or "").strip(),
            reasoner=requested_reasoner,
            backend=backend,
            finish_on_visual_confirmation=bool(
                value.get("finish_on_visual_confirmation", True)
            ),
            turn_only=bool(value.get("turn_only", False)),
            enable_autonomous_motion=enable_motion,
            operator_supervised_experiment=operator_supervised,
            dry_run_motion=dry_run_motion,
            allow_degraded=allow_degraded,
            rgbd_source=bool(value.get("rgbd_source", defaults["rgbd_source"])),
            rgbd_base_url=str(
                value.get("rgbd_base_url") or defaults["rgbd_base_url"]
            ),
            spatial_v2=bool(value.get("spatial_v2", defaults.get("spatial_v2", False))),
            spatial_provider=str(
                value.get("spatial_provider")
                or defaults.get("spatial_provider", "camera")
            ),
            rtabmap=bool(value.get("rtabmap", defaults.get("rtabmap", False))),
            max_seconds=_optional_float(value.get("max_seconds")),
            max_planning_cycles=_optional_int(value.get("max_planning_cycles")),
            max_motion_steps=_optional_int(value.get("max_motion_steps")),
            llm_model=value.get("llm_model"),
            verify_min_confidence=_optional_float(value.get("verify_min_confidence")),
        )

    def validate(self) -> str | None:
        """Return an error message or None when the request is acceptable."""
        task_text = (self.task_text or self.target).strip()
        if not task_text:
            return "task_text/target is required"
        if len(task_text) > MAX_TARGET_LENGTH:
            return f"task_text too long (max {MAX_TARGET_LENGTH} chars)"
        if self.reasoner not in {"legacy", "semantic", "hybrid"}:
            return f"unsupported reasoner: {self.reasoner}"
        if self.backend not in {"go2w_experimental", "mock", "mock_metric"}:
            return f"unsupported backend: {self.backend}"
        if self.spatial_provider not in {"camera", "rtabmap", "plain_slam"}:
            return f"unsupported spatial_provider: {self.spatial_provider}"
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
    """
    import os

    defaults: dict[str, Any] = {
        "backend": "go2w_experimental",
        "enable_autonomous_motion": False,
        "operator_supervised_experiment": False,
        "rgbd_source": False,
        "rgbd_base_url": "http://192.168.123.18:8080",
        "spatial_v2": False,
        "spatial_provider": "camera",
        "rtabmap": False,
        "max_search_seconds": None,
        "max_planning_cycles": None,
        "max_motion_steps": None,
    }
    config_path = _PROJECT_ROOT / _DEFAULT_SEARCH_CONFIG
    if config_path.is_file():
        try:
            import yaml

            data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
            search = data.get("search") or {}
            for key in ("backend",):
                if search.get(key):
                    defaults[key] = str(search[key])
            if search.get("rgbd_source") is not None:
                defaults["rgbd_source"] = bool(search["rgbd_source"])
            if search.get("rgbd_base_url"):
                defaults["rgbd_base_url"] = str(search["rgbd_base_url"])
            if search.get("spatial_v2") is not None:
                defaults["spatial_v2"] = bool(search["spatial_v2"])
            if search.get("spatial_provider"):
                defaults["spatial_provider"] = str(search["spatial_provider"])
            if search.get("rtabmap") is not None:
                defaults["rtabmap"] = bool(search["rtabmap"])
            for key in ("max_search_seconds", "max_planning_cycles",
                        "max_motion_steps"):
                if search.get(key) is not None:
                    defaults[key] = search[key]
            if search.get("enable_autonomous_motion") is not None:
                defaults["enable_autonomous_motion"] = bool(
                    search["enable_autonomous_motion"]
                )
            if search.get("operator_supervised_experiment") is not None:
                defaults["operator_supervised_experiment"] = bool(
                    search["operator_supervised_experiment"]
                )
        except Exception:  # noqa: BLE001 - config must never break startup
            pass
    if os.getenv("AUTONOMOUS_SEARCH_DEFAULT_BACKEND"):
        defaults["backend"] = os.getenv("AUTONOMOUS_SEARCH_DEFAULT_BACKEND")
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
        defaults["spatial_provider"] = os.getenv(
            "AUTONOMOUS_SEARCH_SPATIAL_PROVIDER", "camera"
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
