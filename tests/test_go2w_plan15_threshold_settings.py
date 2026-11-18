"""§15 配置项统一化：12 个 GO2W_* 阈值旋钮的契约测试。

计划书 §15 的四条硬要求（原文的表头是"建议/例如"，那 12 个具体数值只是举例）：

1. 有默认值
2. 有范围校验
3. 有 README/.env.example 注释
4. 启动日志打印关键生效值

前两条在这里断言；第三条断言 .env.example 里确实有这 12 个键名（否则文档会
悄悄烂掉）；第四条断言 worker 里那行 `[GO2W-CONFIG]` 还在。

**默认值取的是代码原本硬编码的值，不是计划书例表里的值。** 两者有 4 处不同
（重观测间隔 0.2/0.3、三个关系阈值），这里按现值断言 —— 接线这件事本身不该
改行为。计划书的数是否要采纳，是另一个决定，不在这条测试的射程内。
"""

from __future__ import annotations

import subprocess
import sys
from dataclasses import fields
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.config import (  # noqa: E402
    Settings,
    get_settings,
)

# env 名 -> (Settings 字段名, 默认值, 允许范围)
KNOBS: tuple[tuple[str, str, float, tuple[float, float]], ...] = (
    ("GO2W_LIO_ROTATION_WHEEL_MAX_TRANSLATION_M",
     "go2w_lio_rotation_wheel_max_translation_m", 0.08, (0.0, 1.0)),
    ("GO2W_LIO_ROTATION_PSLAM_MAX_TRANSLATION_M",
     "go2w_lio_rotation_pslam_max_translation_m", 0.20, (0.0, 2.0)),
    ("GO2W_LIO_MAX_POSE_SPEED_MPS",
     "go2w_lio_max_pose_speed_mps", 2.0, (0.1, 10.0)),
    ("GO2W_LIO_RECOVERY_VALID_SAMPLES",
     "go2w_lio_recovery_valid_samples", 4, (1, 100)),
    ("GO2W_PLACE_MERGE_DISTANCE_M",
     "go2w_place_merge_distance_m", 0.35, (0.01, 5.0)),
    ("GO2W_PLACE_RELOCATION_MIN_M",
     "go2w_place_relocation_min_m", 0.50, (0.0, 5.0)),
    ("GO2W_PERCEPTION_SETTLE_SECONDS",
     "go2w_perception_settle_seconds", 0.8, (0.0, 30.0)),
    ("GO2W_TARGET_POSSIBLE_VERIFY_ATTEMPTS",
     "go2w_target_possible_verify_attempts", 2, (1, 20)),
    ("GO2W_TARGET_POSSIBLE_REOBSERVE_DELAY_SECONDS",
     "go2w_target_possible_reobserve_delay_seconds", 0.2, (0.0, 10.0)),
    ("GO2W_RELATION_LEFT_RIGHT_MIN_NORM",
     "go2w_relation_left_right_min_norm", 0.10, (0.0, 2.0)),
    ("GO2W_RELATION_DEPTH_ORDER_MIN_M",
     "go2w_relation_depth_order_min_m", 0.25, (0.0, 5.0)),
    ("GO2W_RELATION_NEAR_MAX_M",
     "go2w_relation_near_max_m", 0.90, (0.0, 10.0)),
)


@pytest.fixture(autouse=True)
def _fresh_settings():
    """每个用例前后都清缓存，免得上一个用例设的环境变量漏进来。"""
    get_settings.cache_clear()
    yield
    get_settings.cache_clear()


def test_all_twelve_knobs_have_defaults_on_settings() -> None:
    """要求 1：12 个键都在 Settings 上，且默认值等于代码原本硬编码的值。"""
    names = {f.name for f in fields(Settings)}
    settings = get_settings()
    for env_name, attr, default, _ in KNOBS:
        assert attr in names, f"{env_name} 没进 Settings（字段 {attr} 不存在）"
        assert getattr(settings, attr) == default, (
            f"{env_name} 默认值漂了：期望 {default}，实得 {getattr(settings, attr)}"
        )


@pytest.mark.parametrize("env_name,attr,default,bounds", KNOBS)
def test_knob_default_when_unset(monkeypatch, env_name, attr, default, bounds) -> None:
    monkeypatch.delenv(env_name, raising=False)
    get_settings.cache_clear()
    assert getattr(get_settings(), attr) == default


@pytest.mark.parametrize("env_name,attr,default,bounds", KNOBS)
def test_knob_rejects_out_of_range(monkeypatch, env_name, attr, default, bounds) -> None:
    """要求 2：越界必须回落到默认值，而不是被夹到最近的边界值。

    夹边界会悄悄改掉操作员设的数 —— 那正是"默认值 ≠ 生效值"那类假绿的来源。
    """
    low, high = bounds
    for bad in (low - 1.0, high + 1.0):
        if isinstance(default, int):
            bad = int(bad)
            if bad == default:
                continue
        monkeypatch.setenv(env_name, str(bad))
        get_settings.cache_clear()
        assert getattr(get_settings(), attr) == default, (
            f"{env_name}={bad} 越界后没有回落默认值 {default}"
        )


@pytest.mark.parametrize("env_name,attr,default,bounds", KNOBS)
def test_knob_accepts_in_range(monkeypatch, env_name, attr, default, bounds) -> None:
    """范围校验不能宽到把合法值也拒掉。"""
    low, high = bounds
    mid = (low + high) / 2.0
    probe = int(mid) if isinstance(default, int) else round(mid, 3)
    if probe == default:
        pytest.skip("区间中点恰好等于默认值，测不出透传")
    monkeypatch.setenv(env_name, str(probe))
    get_settings.cache_clear()
    assert getattr(get_settings(), attr) == pytest.approx(probe)


def test_env_example_documents_all_twelve() -> None:
    """要求 3：.env.example 必须列出全部 12 个键，否则文档会悄悄烂掉。"""
    text = (ROOT / ".env.example").read_text(encoding="utf-8")
    missing = [env for env, *_ in KNOBS if env not in text]
    assert not missing, f".env.example 缺这些键的说明：{missing}"


def test_worker_prints_effective_values() -> None:
    """要求 4：worker 启动时打一行 [GO2W-CONFIG]，列出 12 个生效值。

    只断言这个发射点还在、且 12 个键名都出现在那一行里 —— 它是范围校验的
    唯一出口（越界回落只在 stderr 出声，不对着这一行看就发现不了）。
    """
    src = (
        ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"
    ).read_text(encoding="utf-8")
    assert "[GO2W-CONFIG]" in src, "worker 不再打印生效值了"
    marker = src.index("[GO2W-CONFIG]")
    block = src[marker : marker + 2000]
    missing = [env for env, *_ in KNOBS if f'("{env[5:]}",' not in block]
    assert not missing, f"[GO2W-CONFIG] 那一行里漏了：{missing}"


def test_knobs_actually_reach_consumers(monkeypatch) -> None:
    """接线自检：旋钮必须真的到达消费者，不能只是躺在 Settings 里。

    这是本条最重要的断言。§15 之前的病根不是"没写进 settings"，而是阈值
    **硬编码在调用点**、模块默认值是死的 —— 一个只存在于 settings 里的旋钮
    和那种情况一样没用，却更难发现。
    """
    monkeypatch.setenv("GO2W_PERCEPTION_SETTLE_SECONDS", "1.25")
    get_settings.cache_clear()

    from app.live_robot.search_state_machine import SearchStateMachine
    from app.live_robot.step_search_runner import StepSearchConfig

    settle_field = {f.name: f for f in fields(StepSearchConfig)}["stop_settle_seconds"]
    assert settle_field.default_factory() == pytest.approx(1.25), (
        "StepSearchConfig.stop_settle_seconds 没有跟随 GO2W_PERCEPTION_SETTLE_SECONDS"
    )
    assert SearchStateMachine().stop_settle_seconds == pytest.approx(1.25), (
        "SearchStateMachine 的默认没跟随 settings"
    )
    # 显式传值必须压过 settings —— 否则调用方原有的显式行为被悄悄改掉了。
    assert SearchStateMachine(stop_settle_seconds=0.0).stop_settle_seconds == 0.0


def test_worker_passes_settings_into_pose_validator_and_place_graph() -> None:
    """worker 里 9 个旋钮的落点是同一处构造，逐个查它读的是 settings 不是字面量。"""
    src = (
        ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"
    ).read_text(encoding="utf-8")
    for attr in (
        "go2w_place_merge_distance_m",
        "go2w_place_relocation_min_m",
        "go2w_lio_rotation_wheel_max_translation_m",
        "go2w_lio_rotation_pslam_max_translation_m",
        "go2w_lio_max_pose_speed_mps",
        "go2w_lio_recovery_valid_samples",
        "go2w_relation_left_right_min_norm",
        "go2w_relation_depth_order_min_m",
        "go2w_relation_near_max_m",
    ):
        assert f"settings.{attr}" in src, f"worker 没把 {attr} 接进去"


def test_relation_knobs_reach_the_extractor(monkeypatch) -> None:
    """关系阈值必须真的到达 extract_geometric_relations（端到端，不止 grep）。

    提取器比的是 bbox_2d 的中心差，而 bbox_2d 是**归一化到 0~1 的图像坐标**
    （app/prompts.py:47,103 明文规定），所以这三个阈值都是归一化量、不是米。
    这里的用例也用归一化坐标。
    """
    from app.spatial.geometric_relation_extractor import extract_geometric_relations

    # 两个框的中心横坐标差恰好 0.5（归一化单位）。
    objects = [
        {"id": "a", "label": "chair", "bbox_2d": [-0.5, 0.0, 0.5, 1.0]},
        {"id": "b", "label": "table", "bbox_2d": [0.0, 0.0, 1.0, 1.0]},
    ]
    # 默认下限 0.10：0.5 的分离应当判出左右关系。
    rels = extract_geometric_relations(objects)
    assert any(r["relation"] == "left_of" for r in rels), (
        f"默认阈值下本该有 left_of，实得 {rels}"
    )

    # 把下限抬到 0.9：同一对物体就不该再有左右关系了 —— 证明这个参数
    # 真的在起作用，而不是被忽略。
    assert not any(
        r["relation"] == "left_of"
        for r in extract_geometric_relations(objects, min_horizontal_separation=0.9)
    ), "阈值抬到 0.9 后仍产 left_of，说明这个参数没接进去"


def test_config_module_has_no_ros_dependency() -> None:
    """app.config 被 live_robot 全系 import，不能沾 rclpy（离线测试环境没有）。"""
    proc = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; sys.path.insert(0, %r); "
            "import app.config; assert 'rclpy' not in sys.modules" % str(ROOT),
        ],
        capture_output=True,
        text=True,
    )
    assert proc.returncode == 0, proc.stderr
