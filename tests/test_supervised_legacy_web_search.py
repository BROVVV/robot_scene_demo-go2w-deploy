"""The historical WebUI motion path is explicit and bounded at both gates."""

import pytest

from app.manual_web_demo.search_models import SearchStartRequest
from app.navigation.exploration_config import load_go2w_experiment_profile
from scripts.go2w.autonomous_search_worker import (
    _LEGACY_MOTION_PROFILES,
    build_argv,
    validate_legacy_worker_params,
)
from scripts.go2w.run_semantic_exploration import _forward_duration_scale_for_run


def _argument(argv: list[str], flag: str) -> str:
    return argv[argv.index(flag) + 1]


def test_default_production_request_still_rejects_legacy_backend() -> None:
    request = SearchStartRequest(task_text="找垃圾桶", backend="go2w_experimental")
    assert "unsupported backend" in (request.validate() or "")


def test_explicit_legacy_dry_run_requires_named_launcher_mode() -> None:
    request = SearchStartRequest(
        task_text="找垃圾桶", backend="go2w_experimental", dry_run_motion=True,
    )
    assert request.validate(allow_legacy_supervised=True) is None
    assert "unsupported backend" in (request.validate() or "")


def test_legacy_motion_requires_launcher_and_request_arm() -> None:
    request = SearchStartRequest(
        task_text="找垃圾桶", backend="go2w_experimental",
        enable_autonomous_motion=True, operator_supervised_experiment=True,
    )
    assert "launcher arm" in (
        request.validate(allow_legacy_supervised=True) or ""
    )
    assert request.validate(
        allow_legacy_supervised=True, legacy_motion_armed=True,
    ) is None
    request.operator_supervised_experiment = False
    assert "operator supervision" in (
        request.validate(allow_legacy_supervised=True, legacy_motion_armed=True) or ""
    )


def test_worker_rechecks_authorization(monkeypatch: pytest.MonkeyPatch) -> None:
    params = {
        "backend": "go2w_experimental", "enable_autonomous_motion": True,
        "operator_supervised_experiment": True, "dry_run_motion": False,
    }
    monkeypatch.delenv("GO2W_ALLOW_SUPERVISED_LEGACY_SEARCH", raising=False)
    with pytest.raises(ValueError, match="not enabled"):
        validate_legacy_worker_params(params)
    monkeypatch.setenv("GO2W_ALLOW_SUPERVISED_LEGACY_SEARCH", "1")
    with pytest.raises(ValueError, match="launcher arm"):
        validate_legacy_worker_params(params)
    monkeypatch.setenv("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION", "1")
    validate_legacy_worker_params(params)
    params["operator_supervised_experiment"] = False
    with pytest.raises(ValueError, match="operator supervision"):
        validate_legacy_worker_params(params)
    params["dry_run_motion"] = True
    validate_legacy_worker_params(params)


def test_worker_hard_bounds_legacy_search_without_changing_nav2() -> None:
    argv = build_argv({
        "target": "找垃圾桶", "backend": "go2w_experimental",
        "max_seconds": 999, "max_planning_cycles": 999,
        "max_motion_steps": 999, "max_turn_deg": 90,
        "forward_step_m": 2.0,
    })
    assert _argument(argv, "--max-seconds") == "240.0"
    assert _argument(argv, "--max-planning-cycles") == "3"
    assert _argument(argv, "--max-motion-steps") == "3"
    assert _argument(argv, "--max-turn-deg") == "15.0"
    assert _argument(argv, "--forward-step-m") == "0.2"
    assert _argument(argv, "--max-radius") == "1.0"
    assert _argument(argv, "--max-observed-forward-m") == "0.2"
    assert _argument(argv, "--max-navigation-goals") == "3"
    assert _argument(argv, "--max-motion-commands") == "3"
    # 路径由 _project_path 解析成绝对路径（和其它 --*-config 一样，不依赖 cwd）。
    assert _argument(argv, "--profile-config").endswith(
        "configs/go2w/high_level_experiment.yaml",
    )
    nav2 = build_argv({"target": "找垃圾桶", "backend": "nav2"})
    assert "--max-radius" not in nav2
    assert "--profile-config" not in nav2


def test_short_range_trial_does_not_apply_historical_forward_overdrive() -> None:
    profile = {"forward_command_duration_scale": 1.5}
    assert _forward_duration_scale_for_run(profile, 3) == 1.0
    assert _forward_duration_scale_for_run(profile, 0) == 1.5
    assert _forward_duration_scale_for_run(
        {"forward_command_duration_scale": 0.8}, 3,
    ) == 0.8


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1])
def test_worker_rejects_invalid_legacy_budget(value: float) -> None:
    with pytest.raises(ValueError, match="invalid legacy search budget"):
        build_argv({
            "target": "找垃圾桶", "backend": "go2w_experimental",
            "max_seconds": value,
        })


@pytest.mark.parametrize(
    "key",
    # ``max_radius=0`` 在 run_semantic_exploration 里的字面含义是"不限半径"；
    # 它必须是短程档拒绝的，否则请求体可以用一个 0 把 1 m 边界整个抹掉。
    ["max_seconds", "max_planning_cycles", "max_motion_steps",
     "max_navigation_goals", "max_motion_commands", "max_radius"],
)
def test_worker_zero_cannot_disable_trial_budget(key: str) -> None:
    with pytest.raises(ValueError, match="invalid legacy search budget"):
        build_argv({
            "target": "找垃圾桶", "backend": "go2w_experimental",
            key: 0,
        })


@pytest.mark.parametrize("bogus", ["field_enormous", "", "   "])
def test_unknown_legacy_profile_fails_closed(
    monkeypatch: pytest.MonkeyPatch, bogus: str,
) -> None:
    # 空名字必须是错误，不是"回落到默认档"：启动器横幅会按默认档宣讲，
    # 操作员却以为自己选的是另一个档。
    monkeypatch.setenv("GO2W_LEGACY_SEARCH_MOTION_PROFILE", bogus)
    with pytest.raises(ValueError, match="legacy search motion profile"):
        build_argv({"target": "找垃圾桶", "backend": "go2w_experimental"})
    # 名字错时也不能让 Nav2 路径跟着炸：它根本不读档位。
    assert "--forward-step-m" not in build_argv(
        {"target": "找垃圾桶", "backend": "nav2"},
    )


def test_field_large_profile_is_opt_in_and_unbounded_in_count(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("GO2W_LEGACY_SEARCH_MOTION_PROFILE", "field_large")
    base = {"target": "找垃圾桶", "backend": "go2w_experimental"}
    # 请求体什么都不说时，计数类闸门落到档位默认值 0 = 不限次。
    default = build_argv(dict(base))
    assert _argument(default, "--max-navigation-goals") == "0"
    assert _argument(default, "--max-motion-steps") == "0"
    assert _argument(default, "--max-planning-cycles") == "0"
    assert _argument(default, "--max-motion-commands") == "0"
    assert _argument(default, "--max-seconds") == "0.0"
    assert _argument(default, "--max-radius") == "5.0"
    # 大场地默认不限总时长；显式有限预算仍可收窄。会话半径、单目标距离和
    # 转向仍有硬顶。
    argv = build_argv({
        **base, "max_seconds": 99999, "max_planning_cycles": 999,
        "max_motion_steps": 999, "max_navigation_goals": 999,
        "max_turn_deg": 90, "forward_step_m": 50.0,
        "max_observed_forward_m": 50.0,
    })
    assert _argument(argv, "--forward-step-m") == "3.0"
    assert _argument(argv, "--max-observed-forward-m") == "3.0"
    assert _argument(argv, "--max-radius") == "5.0"
    assert _argument(argv, "--max-seconds") == "99999.0"
    assert _argument(argv, "--max-turn-deg") == "30.0"
    assert _argument(argv, "--profile-config").endswith(
        "configs/go2w/high_level_experiment_large_field.yaml",
    )
    # 计数类键没有"顶"可压：一个正数请求只会比"不限"更紧，所以原样放行。
    # （档位默认 0 早在上一条断言里钉住了；能放大包线的只有 forward_step_m /
    # max_observed_forward_m / max_seconds / max_turn_deg 这几个。）
    assert _argument(argv, "--max-navigation-goals") == "999"
    assert _argument(argv, "--max-motion-steps") == "999"


@pytest.mark.parametrize("name", sorted(_LEGACY_MOTION_PROFILES))
def test_profile_yaml_agrees_with_the_worker_envelope(name: str) -> None:
    """档位 YAML 与 worker 包线表必须是同一件事。

    操作员在 ``I_CONFIRM`` 横幅上看到的是 worker 表，而 ``run_go2w`` 从 YAML 读
    运动原语和时长补偿。两者一旦分叉，横幅就是在描述一个不会执行的包线 ——
    这正是"挂错开关"这类缺陷的温床，所以在这里钉住。
    """
    profile = _LEGACY_MOTION_PROFILES[name]
    bounds = profile["bounds"]
    yaml = load_go2w_experiment_profile(profile["profile_config"])
    limits = yaml.get("limits") or {}
    primitives = yaml.get("motion_primitives") or {}
    execution = yaml.get("execution") or {}

    # YAML 的 limits 是**天花板**，worker 表是请求值：run_go2w 取两者较小者。
    # 所以断言"请求 ≤ 天花板"，不是相等 —— 短程档就是 15° 请求压在 30° 天花板下。
    assert float(limits["max_turn_deg"]) >= float(bounds["max_turn_deg"])
    assert float(limits["max_forward_step_m"]) >= float(bounds["forward_step_m"])
    yaml_radius = float(limits["max_radius_m"])
    assert yaml_radius <= 0.0 or yaml_radius >= float(bounds["max_radius"])
    # 自动倒车恢复走 ``b<metres>`` 原语，必须和档位声明的 REVERSE 同源；
    # run_go2w 里那个开关现在读的就是 motion_primitives["REVERSE"]。
    assert primitives["REVERSE"] is False
    # 实际生效的时长补偿（含 run_semantic_exploration 对有限目标次数的 clamp）
    # 必须 ≤1.0，否则实测位移会越过 max_observed_forward_m。
    effective = _forward_duration_scale_for_run(
        execution, int(bounds["max_navigation_goals"]),
    )
    assert 0.0 < effective <= 1.0
    assert execution["stop_after_each_step"] is True


def test_field_large_is_not_reachable_from_a_request_body(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """档位只能由启动器选；请求体里塞任何名字都改不动包线。"""
    monkeypatch.delenv("GO2W_LEGACY_SEARCH_MOTION_PROFILE", raising=False)
    argv = build_argv({
        "target": "找垃圾桶", "backend": "go2w_experimental",
        # 这些键在请求体里出现过，但 worker 一个都不读：
        # ``max_forward_step_m``/``profile`` 不在包线键表里，
        # 而 ``legacy_profile`` 从来不是 worker 的参数名（真名是环境变量）。
        "max_forward_step_m": 3.0, "profile": "operator_supervised_experiment_large_field",
        "legacy_profile": "field_large", "max_observed_forward_m": 3.0,
        "max_navigation_goals": 999, "max_motion_steps": 999,
    })
    assert _argument(argv, "--forward-step-m") == "0.2"
    assert _argument(argv, "--max-observed-forward-m") == "0.2"
    assert _argument(argv, "--max-navigation-goals") == "3"
    assert _argument(argv, "--max-motion-steps") == "3"


def test_nav2_large_field_profile_uses_2d_provider_and_continuous_budget(monkeypatch):
    monkeypatch.setenv('GO2W_NAV2_SEARCH_PROFILE', 'field_large')
    argv = build_argv({'backend':'nav2','target':'找垃圾桶'})
    assert _argument(argv, '--spatial-provider') == 'nav2'
    assert _argument(argv, '--max-seconds') == '0.0'
    assert _argument(argv, '--max-motion-steps') == '0'
    assert _argument(argv, '--max-planning-cycles') == '0'
    assert _argument(argv, '--forward-step-m') == '3.0'
    assert _argument(argv, '--max-radius') == '5.0'
