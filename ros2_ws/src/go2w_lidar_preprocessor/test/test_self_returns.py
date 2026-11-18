import numpy as np
import pytest

from go2w_lidar_preprocessor.self_returns import RawSelfReturnFilter, measured_low_self_band


def test_only_measured_surface_neighborhood_is_removed_not_the_surrounding_region():
    model=RawSelfReturnFilter([[.5,0.,-.3]],.025)
    points=np.array([[.5,0.,-.3],[.51,0.,-.3],[.53,0.,-.3],[.5,0.,0.],
        [1.,0.,0.],[np.nan,0.,0.]])
    assert model.keep(points).tolist()==[False,False,True,True,True,True]


def test_bounded_search_preserves_the_exact_reviewed_surface_boundary():
    radius=.03
    model=RawSelfReturnFilter([[0.,0.,0.]],radius)
    x=np.array([np.nextafter(radius,0.),radius,np.nextafter(radius,np.inf),1.])
    points=np.column_stack((x,np.zeros((len(x),2))))
    assert model.keep(points).tolist()==[False,False,True,True]


def test_surface_membership_matches_all_points_distance_with_near_and_far_returns():
    rng=np.random.default_rng(42)
    surface=rng.uniform(-.5,.5,(80,3))
    queries=np.vstack((surface+rng.normal(0,.02,surface.shape),
                       rng.uniform(-6.,6.,(500,3))))
    nearest=np.sqrt(((queries[:,None,:]-surface[None,:,:])**2).sum(axis=2)).min(axis=1)
    assert np.array_equal(RawSelfReturnFilter(surface,.03).keep(queries),nearest>.03)


def test_moving_extension_is_limited_to_measured_low_leg_band():
    model=RawSelfReturnFilter([[.5,0.,-.3]],.03)
    raw=np.array([[.54,0.,-.3],[.54,0.,-.3],[.54,0.,-.3],
                  [.54,0.,-.3],[.54,0.,-.3],[.6,0.,-.3]])
    base=np.array([[0.,.30,-.28],[0.,.46,-.28],[.46,.30,-.28],
                   [0.,.30,-.10],[0.,.30,-.50],[0.,.30,-.28]])
    region=dict(x_abs_max=.4,y_abs_min=.255,y_abs_max=.4,z_min=-.38,z_max=-.18)
    assert model.keep_moving(raw,base,radius=.05,region=region).tolist()==[
        False,True,True,True,True,True]


def test_measured_low_band_is_tight_and_symmetric():
    region=dict(x_abs_max=.4,y_abs_min=.255,y_abs_max=.4,z_min=-.38,z_max=-.18)
    points=np.array([[0.,.30,-.28],[0.,-.30,-.28],[0.,.46,-.28],
                     [.46,.30,-.28],[0.,.30,-.10],[.8,0.,0.]])
    assert measured_low_self_band(points,region).tolist()==[
        True,True,False,False,False,False]


@pytest.mark.parametrize('radius,region', [
    (.051,dict(x_abs_max=.4,y_abs_min=.255,y_abs_max=.4,z_min=-.38,z_max=-.18)),
    (.05,{}),
    (.05,dict(x_abs_max=.8,y_abs_min=.255,y_abs_max=.4,z_min=-.38,z_max=-.18)),
])
def test_unsafe_moving_extension_is_rejected(radius,region):
    model=RawSelfReturnFilter([[.5,0.,-.3]],.03)
    with pytest.raises(ValueError):
        model.keep_moving(np.array([[.5,0.,-.3]]),np.array([[0.,.3,-.3]]),
                          radius=radius,region=region)


@pytest.mark.parametrize('points,radius', [([], .025),([[float('nan'),0.,0.]],.025),
    ([[0.,0.,0.]],.1)])
def test_wide_or_nonfinite_models_are_rejected(points,radius):
    with pytest.raises(ValueError):RawSelfReturnFilter(points,radius)
