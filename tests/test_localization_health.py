from copy import deepcopy
import json
import math

import pytest

from app.navigation.localization_health import (
    StructuralScanFreshnessLatch,
    authoritative_map_authorization,
    audit_expectations,
    canonical_node_name,
    lifecycle_manager_owners,
    evaluate_localization,
    map_statistics,
    sanitize_non_finite,
    stationary_since,
    observe_motion_since,
    tf_fresh,
)


def test_structural_scan_latch_debounces_one_late_mapping_frame_only():
    latch = StructuralScanFreshnessLatch(.25)
    assert latch.update(structural_fresh=True, safety_scan_fresh=True, now=1.0) == (True, False)
    assert latch.update(structural_fresh=False, safety_scan_fresh=True, now=1.1) == (True, True)
    assert latch.update(structural_fresh=False, safety_scan_fresh=True, now=1.34) == (True, True)
    assert latch.update(structural_fresh=False, safety_scan_fresh=True, now=1.35) == (False, False)
    assert latch.update(structural_fresh=True, safety_scan_fresh=True, now=1.36) == (True, False)


def test_structural_scan_latch_never_debounces_raw_safety_scan_loss():
    latch = StructuralScanFreshnessLatch(.25)
    assert latch.update(structural_fresh=False, safety_scan_fresh=False, now=2.0) == (False, False)
    assert latch.update(structural_fresh=False, safety_scan_fresh=True, now=2.01) == (True, True)


@pytest.mark.parametrize('age,received,expected', [
    (0., 0., True), (.3, .3, True), (.4, .3, True), (-.3, 0., True),
    (.400001, 0., False), (-.300001, 0., False), (0., .300001, False),
    (0., -.001, False), (float('nan'), 0., False), (float('inf'), 0., False),
    (None, 0., False), (0., float('nan'), False), (0., None, False),
])
def test_dynamic_tf_buffer_and_health_share_time_policy(age, received, expected):
    assert tf_fresh({'age_s': age, 'receive_age_s': received}) is expected


def test_ros_graph_node_names_are_canonical_for_root_and_nested_namespaces():
    assert canonical_node_name("map_server", "/") == "/map_server"
    assert canonical_node_name("manager", "/nav") == "/nav/manager"


def test_foxy_manager_companion_does_not_hide_a_second_real_manager():
    names=['/lifecycle_manager_plan_only','/lifecycle_manager_plan_only_service_client']
    assert lifecycle_manager_owners(names)==['/lifecycle_manager_plan_only']
    assert len(lifecycle_manager_owners(names+['/lifecycle_manager_navigation']))==2
    assert lifecycle_manager_owners([names[1]])==[names[1]]


def test_map_authorization_requires_the_unique_live_health_owner_and_matching_evidence():
    status = {
        "mode": "localization",
        "valid": True,
        "checks": {"map_authorized": True},
        "evidence": {"map_authorized": True},
    }
    assert authoritative_map_authorization(
        status, ["/go2w_localization_health"], mode="localization"
    )
    assert not authoritative_map_authorization(status, ["/impostor"], mode="localization")
    assert not authoritative_map_authorization(status, ["/go2w_localization_health"], mode="mapping")
    for field in ("valid",):
        rejected = dict(status, **{field: False})
        assert not authoritative_map_authorization(
            rejected, ["/go2w_localization_health"], mode="localization"
        )
    for section in ("checks", "evidence"):
        rejected = dict(status)
        rejected[section] = {"map_authorized": False}
        assert not authoritative_map_authorization(
            rejected, ["/go2w_localization_health"], mode="localization"
        )


def healthy():
    stamp = {"age_s": 0.03, "receive_age_s": 0.03, "publishers": 1}
    transform = {**stamp, "authorities": 1, "finite": True}
    return {
        "map": {**stamp, **map_statistics(2, 2, .05, [0, 0, 100, -1]), "frame": "map"},
        "scan": {**stamp, "valid_ranges": 100},
        "mapping_scan": {**stamp, "valid_ranges": 200},
        "odom": {**stamp, "frame": "odom_fused", "child": "base_link", "finite": True},
        "tf": {edge: dict(transform) for edge in ("map->odom_fused", "odom_fused->base_link")},
        "localizer_active": True, "pose_jump": False,
    }


def test_healthy_mapping_is_independent_of_three_dimensional_lio():
    assert evaluate_localization(healthy())["valid"]


def test_future_tf_cache_recovery_is_required_even_with_other_fresh_evidence():
    snapshot = healthy()
    snapshot['tf_future_recovery_pending'] = ['map->odom_fused']
    result = evaluate_localization(snapshot)
    assert not result['valid']
    assert result['blocking'] == ['tf_future_cache_recovered']
    snapshot['tf_future_recovery_pending'] = []
    assert evaluate_localization(snapshot)['valid']


@pytest.mark.parametrize("topic,key,value", [
    ("scan", "age_s", 0.401), ("scan", "receive_age_s", 0.301),
    ("scan", "age_s", -.11), ("scan", "stamp_regressions", 1),
    ("odom", "age_s", .31), ("odom", "publishers", 2),
    ("map", "publishers", 2), ("map", "age_s", 12 * 3600),
    ("map", "valid", False), ("odom", "finite", False),
])
def test_stale_missing_or_duplicate_evidence_fails_closed(topic, key, value):
    snapshot = healthy()
    snapshot[topic][key] = value
    assert not evaluate_localization(snapshot)["valid"]


def test_scan_acquisition_latency_does_not_look_like_a_topic_dropout():
    snapshot = healthy()
    snapshot["scan"].update(age_s=.23, receive_age_s=.06)
    snapshot["mapping_scan"].update(age_s=.23, receive_age_s=.06)
    assert evaluate_localization(snapshot)["valid"]

    # Arrival freshness remains strict: continuously replaying an old packet
    # or losing the topic must still close the localization gate quickly.
    # The release18 physical trace contained one 0.254 s arrival gap with a
    # 0.361 s acquisition stamp age.  Both remain inside this localization
    # gate, while Collision Monitor retains its independent 0.30 s receive
    # timeout.
    snapshot["scan"].update(age_s=.361, receive_age_s=.254)
    snapshot["mapping_scan"].update(age_s=.361, receive_age_s=.254)
    assert evaluate_localization(snapshot)["valid"]

    snapshot["scan"]["receive_age_s"] = .301
    result = evaluate_localization(snapshot)
    assert not result["valid"] and "scan_fresh" in result["blocking"]

    snapshot = healthy()
    snapshot["scan"]["age_s"] = .401
    result = evaluate_localization(snapshot)
    assert not result["valid"] and "scan_fresh" in result["blocking"]


def test_fused_odom_acquisition_latency_does_not_look_like_topic_loss():
    snapshot = healthy()
    snapshot["odom"].update(age_s=.227, receive_age_s=.033)
    snapshot["tf"]["odom_fused->base_link"].update(
        age_s=.346, receive_age_s=.246
    )
    assert evaluate_localization(snapshot)["valid"]

    # Transport freshness remains unchanged: replaying the same old source
    # packet cannot keep localization alive past the receive deadlines.
    snapshot["odom"]["receive_age_s"] = .201
    result = evaluate_localization(snapshot)
    assert not result["valid"] and "odom_fresh" in result["blocking"]


def test_tf_duplicates_and_jumps_fail_closed():
    snapshot = healthy()
    snapshot["tf"]["odom_fused->base_link"]["authorities"] = 2
    assert not evaluate_localization(snapshot)["valid"]
    snapshot = healthy()
    snapshot["pose_jump"] = True
    assert not evaluate_localization(snapshot)["valid"]


def test_zero_data_and_unknown_map_are_invalid():
    assert not map_statistics(2, 2, .05, [-1] * 4)["valid"]
    assert not map_statistics(2, 2, .05, [0] * 4)["valid"]
    assert not map_statistics(2, 2, .05, [0, 100])["valid"]


def test_localization_requires_map_validation_and_bounded_covariance():
    snapshot = healthy()
    snapshot["map"]["age_s"] = 3600  # A static map need not be republished.
    snapshot["amcl"] = {"age_s": .1, "receive_age_s": .1, "covariance": [.01, .01, .01]}
    assert not evaluate_localization(snapshot, mode="localization")["valid"]
    snapshot["map_authorized"] = True
    assert evaluate_localization(snapshot, mode="localization")["valid"]
    snapshot["amcl"]["covariance"][0] = 100
    assert not evaluate_localization(snapshot, mode="localization")["valid"]


# ---------------------------------------------------------------------------
# §P4-5：静止时 /amcl_pose 本来就不重发，拿 1 秒新鲜度卡它等于给定位门判死
#
# nav2_amcl 1.1.20 laserReceived() 只在
# resampled || force_publication || !first_pose_sent_ 时 publishAmclPose()；
# 车停住后 update_min_d/a（本仓 0.1）不再触发更新 ⇒ /amcl_pose 停发，
# 而同一函数的 else-if 分支仍在重发 map->odom_fused。
# ---------------------------------------------------------------------------


def localizing(amcl, odom):
    snapshot = healthy()
    snapshot["map"]["age_s"] = 3600  # 静态图不必重发。
    snapshot["odom"].update(odom)
    snapshot["amcl"] = amcl
    snapshot["map_authorized"] = True
    return evaluate_localization(snapshot, mode="localization")


def test_stationary_stale_amcl_is_accepted_while_moving_stale_is_not():
    stale = {"age_s": 9.0, "receive_age_s": 9.0, "covariance": [.01, .01, .01],
             "odom_at_arrival": [1.0, 2.0, .5], 'motion_observation_complete': True,
             'max_position_excursion_m': 0., 'max_yaw_excursion_rad': 0.}
    still = localizing(stale, {"x": 1.0, "y": 2.0, "yaw": .5})
    assert still["valid"] and still["amcl_fresh_basis"] == "stationary_exempt"
    # 同一条陈旧样本，但此后车挪了 30 cm：不能再当"还有效"。
    moved = localizing(stale, {"x": 1.3, "y": 2.0, "yaw": .5})
    assert not moved["valid"] and "amcl_fresh" in moved["blocking"]
    assert moved["amcl_fresh_basis"] == "stale"
    turned = localizing(stale, {"x": 1.0, "y": 2.0, "yaw": .55})
    assert not turned["valid"]


def test_stationary_exemption_needs_the_odom_sample_not_just_silence():
    """没有到达时刻的 odom 就不认豁免 —— 否则"从没收到 /amcl_pose"也会变绿。"""
    assert not stationary_since({"age_s": 9.0}, {"x": 0.0, "y": 0.0, "yaw": 0.0})
    assert not stationary_since({"odom_at_arrival": [None, 0, 0]}, {"x": 0, "y": 0, "yaw": 0})
    assert not stationary_since({"odom_at_arrival": [0, 0]}, {"x": 0, "y": 0, "yaw": 0})
    assert not stationary_since({"odom_at_arrival": [0, 0, 0]}, {"x": 0, "y": 0})
    snapshot = healthy()
    snapshot["map"]["age_s"] = 3600
    snapshot["odom"].update({"x": 0.0, "y": 0.0, "yaw": 0.0})
    snapshot["amcl"] = {"age_s": 9.0, "receive_age_s": 9.0, "covariance": [.01, .01, .01]}
    snapshot["map_authorized"] = True
    result = evaluate_localization(snapshot, mode="localization")
    assert not result["valid"] and "amcl_fresh" in result["blocking"]


def test_fresh_amcl_reports_fresh_basis_and_mapping_mode_reports_none():
    fresh_amcl = {"age_s": .1, "receive_age_s": .1, "covariance": [.01, .01, .01],
                  "odom_at_arrival": [1.0, 2.0, .5]}
    assert localizing(fresh_amcl, {"x": 9.0, "y": 9.0, "yaw": 1.0})["amcl_fresh_basis"] == "fresh"
    assert "amcl_fresh_basis" not in evaluate_localization(healthy())


# ---------------------------------------------------------------------------
# §P4-5：审计不能只有 mapping 一种口径
#
# audit_navigation_runtime.py 原来把 mode 写死（monitor 的 mode 参数默认 mapping）、
# 生命周期只查 planner/bt、lifecycle_manager 数量硬要求 == 1。于是拿它出的报告
# 去声称"localization 模式验收过"是口径错的。口径本身抽到这里，好在没有 ROS 的
# 机器上也能测。
# ---------------------------------------------------------------------------


def test_audit_expectations_differ_between_modes():
    mapping = audit_expectations("mapping")
    assert mapping["lifecycle_nodes"] == ["planner_server", "bt_navigator"]
    assert mapping["max_lifecycle_managers"] == 1 and mapping["required_managers"] == []
    localizing_plan = audit_expectations("localization")
    # 定位模式查的是地图与 AMCL，不是规划链。
    assert localizing_plan["lifecycle_nodes"] == ["map_server", "amcl"]
    # 正常就有两个 manager（localization_2d 自己的 + 导航栈 plan_only），
    # 所以"必须恰好 1 个"这条规则在定位模式下是错的。
    assert localizing_plan["max_lifecycle_managers"] == 2
    assert localizing_plan["required_managers"] == ["/lifecycle_manager_localization_2d"]


def test_audit_expectations_keep_the_planning_chain_when_execution_is_required():
    both = audit_expectations("localization", require_execution=True)
    assert set(both["lifecycle_nodes"]) == (
        set(audit_expectations("mapping", require_execution=True)["lifecycle_nodes"])
        | {"map_server", "amcl"})
    assert len(both["lifecycle_nodes"]) == len(set(both["lifecycle_nodes"]))


def test_unknown_mode_raises_instead_of_falling_back_to_mapping():
    with pytest.raises(ValueError):
        audit_expectations("localisation")  # 英式拼写：静默退回 mapping 就是口径错配


# ---------------------------------------------------------------------------
# §P3-6：诊断数值里的非有限值不能把健康监视器打死
#
# tick() 是定时器回调，而它用 json.dumps(..., allow_nan=False) 发
# /go2w/nav/localization_status。快照里带的是**原始**诊断数值（地图分辨率、
# odom 的 x/y、amcl 协方差、map->base_link 的 pose），任何一个 NaN/inf 都会让
# 那次 dumps 抛 ValueError。抛出去等于把监视器打死，而它正是
# /go2w/nav/localization_valid 的发布者 —— 现场看到的是"没人发布"，归因指向
# 定位失效，真相是"监视器自己崩了"。
# ---------------------------------------------------------------------------


def test_non_finite_values_really_break_strict_json() -> None:
    """钉住前提：allow_nan=False 对 NaN 抛，而 allow_nan=True 写的不是合法 JSON。"""
    with pytest.raises(ValueError):
        json.dumps({"x": float("nan")}, allow_nan=False)
    # Python 自己读得回来，但 JS 的 JSON.parse 读不了 —— 而消费者正是 JS。
    assert "NaN" in json.dumps({"x": float("nan")}, allow_nan=True)
    assert not math.isfinite(json.loads('{"x": NaN}')["x"])


def test_sanitize_replaces_non_finite_and_reports_where() -> None:
    cleaned, offenders = sanitize_non_finite(
        {"map": {"resolution": float("nan"), "width": 4},
         "pose": [float("inf"), 0.5, -float("inf")]}
    )
    assert cleaned == {"map": {"resolution": None, "width": 4},
                       "pose": [None, 0.5, None]}
    assert offenders == ["$['map']['resolution']", "$['pose'][0]", "$['pose'][2]"]
    # 换完之后严格 JSON 编得出来，且不丢键。
    assert json.loads(json.dumps(cleaned, allow_nan=False)) == cleaned


def test_sanitize_leaves_finite_payload_untouched() -> None:
    payload = healthy()
    cleaned, offenders = sanitize_non_finite(payload)
    assert offenders == []
    assert cleaned == payload


@pytest.mark.parametrize('excursion', [{'x': .3, 'y': 0, 'yaw': 0}, {'x': 0, 'y': 0, 'yaw': .3}])
def test_amcl_stationary_exemption_cannot_recover_after_out_and_back(excursion):
    sample = {'odom_at_arrival': [0, 0, 0], 'motion_observation_complete': True,
              'max_position_excursion_m': 0., 'max_yaw_excursion_rad': 0.}
    origin = {'x': 0, 'y': 0, 'yaw': 0}
    assert stationary_since(sample, origin)
    sample = observe_motion_since(sample, excursion, continuous=True)
    sample = observe_motion_since(sample, origin, continuous=True)
    assert not stationary_since(sample, origin)


def test_missing_odom_history_revokes_exemption_until_another_amcl_sample():
    sample = {'odom_at_arrival': [0, 0, 0], 'motion_observation_complete': True,
              'max_position_excursion_m': 0., 'max_yaw_excursion_rad': 0.}
    origin = {'x': 0, 'y': 0, 'yaw': 0}
    sample = observe_motion_since(sample, origin, continuous=False)
    sample = observe_motion_since(sample, origin, continuous=True)
    assert not stationary_since(sample, origin)


@pytest.mark.parametrize('field,value', [('age_s', -.11), ('age_s', float('nan')), ('receive_age_s', -.01)])
def test_stationary_exemption_never_overrides_future_or_invalid_timestamps(field, value):
    sample = {'odom_at_arrival': [0, 0, 0], 'motion_observation_complete': True,
              'max_position_excursion_m': 0., 'max_yaw_excursion_rad': 0., field: value}
    assert not stationary_since(sample, {'x': 0, 'y': 0, 'yaw': 0})
