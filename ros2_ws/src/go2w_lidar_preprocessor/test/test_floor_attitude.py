import math
from pathlib import Path
import numpy as np
import pytest
from go2w_lidar_preprocessor.floor_attitude import estimate_floor,joint_angles_within_limits,rpy_matrix,standing_height


def profile():
    # Raw sensor normal points down, as on the upside-down mounted unit.
    return {'frames':{'base_to_cloud':{'rotation_rpy_rad':[math.pi,0.,0.],'translation_m':[0.,0.,0.]}},
            'posture':{'body_roll_pitch_rad':[0.,0.],'maximum_attitude_deviation_rad':.015},
            'raw_floor_plane':{'normal':[0.,0.,1.],'distance_m':.43,'tolerance_m':.02}}


def cloud(rp=(0.,0.),height_delta=0.):
    x,y=np.meshgrid(np.linspace(-2,2,35),np.linspace(-2,2,35));n=rpy_matrix(math.pi,0,0).T@rpy_matrix(*rp).T@np.array([0.,0.,-1.]);d=.43+height_delta
    return np.c_[x.ravel(),y.ravel(),(d-n[0]*x.ravel()-n[1]*y.ravel())/n[2]]


def test_live_attitude_and_kinematics_are_checked_by_independent_floor():
    rp=(.012,-.025);delta=.01
    result=estimate_floor(cloud(rp,delta),profile(),rp,height_delta_m=delta)
    assert result.angle_residual_rad<1e-6
    assert abs(result.height_residual_m)<1e-6
    assert result.points>=80
    with pytest.raises(ValueError):estimate_floor(cloud(rp,delta),profile(),(0.,0.))


@pytest.mark.parametrize('rp,dh', [((.06,0.),0.),((0.,0.),.05)])
def test_wrong_imu_or_wrong_height_does_not_validate_a_plane(rp,dh):
    with pytest.raises(ValueError):estimate_floor(cloud(rp,dh),profile(),(0.,0.))


def test_obstacle_above_floor_does_not_replace_missing_floor():
    with pytest.raises(ValueError):estimate_floor(cloud(height_delta=-.20),profile(),(0.,0.))


def test_sparse_line_and_nonfinite_posture_fail_closed():
    x=np.linspace(-2,2,200);line=np.c_[x,np.zeros(200),np.full(200,.43)]
    for points,rp in [(cloud()[:20],(0.,0.)),(line,(0.,0.)),(cloud(),(math.nan,0.)),(cloud(),(.3,0.))]:
        with pytest.raises(ValueError):estimate_floor(points,profile(),rp)


def test_fk_matches_pinned_reference_and_rejects_changed_model():
    root=Path(__file__).resolve().parents[4]
    q=[-.023835837841,.66428130865,-1.27899634838,.0330401957,.69349908829,-1.31202793121,-.03247003257,.69687879085,-1.3152346611,.01887542009,.69893759489,-1.3195158243]
    rp=[-.00406458462,-.00353347533]
    urdf=(root/'configs/go2w/calibration_20261019/official_go2w.urdf').read_bytes()
    height=standing_height(urdf,q,rp)
    assert height==pytest.approx(.43464796980225795)
    with pytest.raises(ValueError):standing_height(urdf+b' ',q,rp)
    with pytest.raises(ValueError):standing_height(urdf,[math.nan]*12,(0.,0.))


def test_fk_rejects_joint_outside_pinned_urdf_limit():
    root=Path(__file__).resolve().parents[4]
    urdf=(root/'configs/go2w/calibration_20261019/official_go2w.urdf').read_bytes()
    import yaml
    profile=yaml.safe_load((root/'configs/go2w/navigation_sensor_profile.yaml').read_text())
    q=profile['posture']['joint_q'].copy()
    assert joint_angles_within_limits(urdf,q)
    q[0]=100.
    assert not joint_angles_within_limits(urdf,q)
    with pytest.raises(ValueError,match='invalid standing kinematics'):
        standing_height(urdf,q,profile['posture']['body_roll_pitch_rad'])
