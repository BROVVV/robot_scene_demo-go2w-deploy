import json
from pathlib import Path

import pytest

from app.navigation.foxy_supervised_trial import validate_supervised_trial
from app.navigation.trial_budget import NavigationTrialBudget


@pytest.fixture
def ledger(tmp_path):
    path=tmp_path/'ledger.json'
    NavigationTrialBudget.initialize(path,window_id='operator_20cm',
        reviewed_robot_boot_id='this-boot',maximum_speed_mps=.12,maximum_steps=1)
    return path


def test_one_unused_commissioning_window_is_accepted(ledger):
    original=ledger.read_bytes()
    assert validate_supervised_trial(ledger,boot_id='this-boot')['maximum_translation_m']==.2
    assert ledger.read_bytes()==original


def test_one_unused_collision_diagnostic_window_is_accepted(tmp_path):
    path=tmp_path/'collision.json'
    NavigationTrialBudget.initialize_collision_diagnostic(
        path,window_id='rear-contact-shell',reviewed_robot_boot_id='this-boot')
    original=path.read_bytes()
    result=validate_supervised_trial(path,boot_id='this-boot')
    assert result['version']==7
    assert result['maximum_translation_m']==.25
    assert result['maximum_duration_s']==15.
    assert path.read_bytes()==original


@pytest.mark.parametrize('key,value', [
    ('maximum_translation_m',.26),('maximum_duration_s',16.),
    ('maximum_lateral_m',.11),('forward_only',False),
    ('reviewed_robot_boot_id','old-boot'),('steps',[{'consumed':True}]),
])
def test_collision_diagnostic_policy_expansion_stays_closed(tmp_path,key,value):
    path=tmp_path/'collision.json'
    NavigationTrialBudget.initialize_collision_diagnostic(
        path,window_id='rear-contact-shell',reviewed_robot_boot_id='this-boot')
    data=json.loads(path.read_text());data[key]=value;path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        validate_supervised_trial(path,boot_id='this-boot')


@pytest.mark.parametrize('key,value', [
    ('maximum_steps',3),('maximum_steps',True),('maximum_translation_m',3.),
    ('maximum_speed_mps',.3),('stop_reserve_m',0.),('reviewed_robot_boot_id','old-boot'),
    ('steps',[{'consumed':True}]),('active_token','already-reserved'),
    ('version',4),('purpose','forward_observed_recheck'),('created_wall',None)])
def test_expanded_consumed_or_unreviewed_windows_stay_closed(ledger,key,value):
    data=json.loads(ledger.read_text());data[key]=value;ledger.write_text(json.dumps(data))
    with pytest.raises(ValueError):validate_supervised_trial(ledger,boot_id='this-boot')


def test_expired_and_future_dated_window_stay_closed(ledger):
    created=json.loads(ledger.read_text())['created_wall']
    for now in [created-1,created+301]:
        with pytest.raises(ValueError):validate_supervised_trial(ledger,boot_id='this-boot',now=now)


def test_alias_cannot_substitute_a_ledger(ledger,tmp_path):
    alias=tmp_path/'alias.json';alias.symlink_to(ledger)
    with pytest.raises(ValueError):validate_supervised_trial(alias,boot_id='this-boot')


def test_trial_keeps_native_plugin_abi_and_desktop_configuration():
    import yaml
    from scripts.go2w.prepare_nav2_trial import parameter_source,trial_parameters
    foxy=yaml.safe_load(parameter_source('foxy').read_text())
    result=trial_parameters(foxy,.12)
    controller=result['controller_server']['ros__parameters']
    assert controller['FollowPath']['plugin']=='go2w_foxy_rpp::RegulatedPurePursuitController'
    assert controller['goal_checker_plugin']=='general_goal_checker'
    assert result['planner_server']['ros__parameters']['GridBased']['plugin']=='nav2_navfn_planner/NavfnPlanner'
    assert result['behavior_server']['ros__parameters']['spin']['plugin']=='go2w_foxy_spin::Spin'
    assert controller['FollowPath']['desired_linear_vel']==.12
    assert parameter_source('humble').name=='nav2_params_humble.yaml'
    assert foxy['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel']==.3


def test_robot_selects_foxy_before_parent_shell_sources_ros(monkeypatch):
    import platform
    from scripts.go2w.prepare_nav2_trial import parameter_source
    monkeypatch.delenv('ROS_DISTRO',raising=False)
    monkeypatch.setattr(platform,'machine',lambda:'aarch64')
    original=Path.is_file
    monkeypatch.setattr(Path,'is_file',lambda p:True if str(p)=='/etc/nv_tegra_release' else original(p))
    assert parameter_source().name=='nav2_params_foxy_execution.yaml'
    assert parameter_source('humble').name=='nav2_params_humble.yaml'
