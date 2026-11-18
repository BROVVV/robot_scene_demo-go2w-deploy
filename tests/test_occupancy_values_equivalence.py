"""Compare signed-byte fast counts to the prior scalar validation policy."""
from array import array
import math

import numpy as np
import pytest

from app.navigation.localization_health import map_statistics
from app.navigation.occupancy_values import int8_grid_view


def scalar_statistics(width,height,resolution,data):
    counts=dict(free=0,occupied=0,unknown=0,invalid=0)
    for v in data:
        if v==-1:counts['unknown']+=1
        elif v==0:counts['free']+=1
        elif 0<v<=100:counts['occupied']+=1
        else:counts['invalid']+=1
    return dict(width=width,height=height,resolution=resolution,cells=len(data),**counts,
        valid=(width>0 and height>0 and math.isfinite(resolution) and resolution>0
               and len(data)==width*height and not counts['invalid']
               and counts['free']>0 and counts['occupied']>0))


@pytest.mark.parametrize('data',[
    array('b',range(-128,128)),memoryview(array('b',range(-128,128))),
    memoryview(array('b',range(-128,128)))[::-1],
    np.arange(-128,128,dtype=np.int8),np.arange(-128,128,dtype=np.int8)[::2],
    array('h',[-257,-1,0,1,50,100,127,255,256]),
    bytes([0,1,50,100,255]),bytearray([0,1,50,100,255]),
    np.array([-1,0,.5,49.5,50,100,100.5,np.nan,np.inf]),
    np.array([False,True],dtype=bool),[-1,0,1,50,100],[],array('b'),
])
def test_counts_match_scalar_without_reinterpreting_other_types(data):
    before=list(data)
    expected=scalar_statistics(len(data),1,.05,data)
    assert map_statistics(len(data),1,.05,data)==expected
    for a,b in zip(before,list(data)):
        assert a==b or (isinstance(a,(float,np.floating)) and math.isnan(a) and math.isnan(b))


@pytest.mark.parametrize('width,height,resolution',[
    (2,2,.05),(1,4,.05),(0,4,.05),(-1,4,.05),(3,2,.05),
    (2,2,0.),(2,2,float('inf')),
])
def test_dimension_and_resolution_gates_are_unchanged(width,height,resolution):
    data=array('b',[-1,0,49,100])
    assert map_statistics(width,height,resolution,data)==scalar_statistics(width,height,resolution,data)


def test_view_is_zero_copy_and_signed_only():
    data=array('b',[-1,0,100]);view=int8_grid_view(data)
    assert np.shares_memory(view,np.frombuffer(data,dtype=np.int8))
    assert view.tolist()==[-1,0,100]
    assert int8_grid_view(bytes([255,0,100])) is None
    assert int8_grid_view(array('h',[-1,0,100])) is None
    assert int8_grid_view(np.array([-.5,0,100])) is None


def test_readonly_signed_view_keeps_unknown_and_all_probability_values():
    data=memoryview(bytes([255,0,1,49,50,65,100])).cast('b')
    assert data.readonly
    assert map_statistics(7,1,.05,data)==dict(width=7,height=1,resolution=.05,
        cells=7,free=1,occupied=5,unknown=1,invalid=0,valid=True)
