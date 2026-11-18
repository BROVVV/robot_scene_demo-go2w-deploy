import copy
import pytest
from scripts.go2w.prepare_nav2_trial import validate_boot_review


def audit(path):
    return dict(wall_time=100.,healthy_for_plan=True,sensor_geometry=dict(valid=True,profile_sha256='a'*64),
        physical_execution=dict(verdict='disabled'),trial_budget_profile=dict(configuration_valid=True,
            parameters={'/go2w_cmd_vel_bridge':dict(trial_ledger=str(path.resolve()),execution_enabled=False)}))


@pytest.mark.parametrize('bad',['stale','geometry','ledger','executing','unhealthy'])
def test_unreviewed_reboot_cannot_be_rebound_with_wrong_or_stale_evidence(tmp_path,bad):
    path=tmp_path/'trial.json';data=audit(path)
    if bad=='stale':data['wall_time']=0.
    elif bad=='geometry':data['sensor_geometry']['profile_sha256']='b'*64
    elif bad=='ledger':data['trial_budget_profile']['parameters']['/go2w_cmd_vel_bridge']['trial_ledger']='other.json'
    elif bad=='executing':data['physical_execution']['verdict']='enabled'
    else:data['healthy_for_plan']=False
    with pytest.raises(ValueError,match='fresh healthy read-only audit'):
        validate_boot_review(data,profile_sha256='a'*64,ledger_path=path,now=101.)


def test_fresh_same_geometry_and_fixed_ledger_review_is_accepted(tmp_path):
    path=tmp_path/'trial.json'
    validate_boot_review(audit(path),profile_sha256='a'*64,ledger_path=path,now=101.)
