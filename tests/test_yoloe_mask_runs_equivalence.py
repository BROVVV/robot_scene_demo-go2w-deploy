"""The optimized encoder must preserve every pixel, hole and run ordering."""
import numpy as np
import pytest
from app.detectors.yoloe_seg_worker import encode_row_runs


def reference(mask):
    data=np.asarray(mask,dtype=bool)
    if data.ndim!=2 or not data.any():return []
    runs=[]
    for y,row in enumerate(data):
        changes=np.flatnonzero(np.diff(np.pad(row.astype(np.int8),(1,1))))
        for start,end in changes.reshape(-1,2):runs.append([int(y),int(start),int(end-start)])
    return runs


def check(mask):
    runs=encode_row_runs(mask)
    assert runs==reference(mask)
    data=np.asarray(mask,dtype=bool)
    if data.ndim==2:
        decoded=np.zeros(data.shape,dtype=bool)
        for y,x,length in runs:decoded[y,x:x+length]=True
        np.testing.assert_array_equal(decoded,data)


@pytest.mark.parametrize('shape',[(0,0),(3,0),(0,5),(1,1),(4,7),(720,1280)])
def test_full_and_empty_masks_include_all_edge_pixels(shape):
    check(np.ones(shape,bool));check(np.zeros(shape,bool))


def test_holes_rows_and_noncontiguous_views_preserve_canonical_order():
    rng=np.random.default_rng(301)
    for _ in range(80):
        data=rng.random((int(rng.integers(1,30)),int(rng.integers(1,40))))>.5
        for view in (data,data[::-1,::-1],data.T,np.asfortranarray(data),data[:,::2]):check(view)


def test_existing_boolean_coercion_and_invalid_dimensions_remain_unchanged():
    check([[0,-1,float('nan')],[float('inf'),0,2]])
    for value in (True,[],[1,0],np.zeros((2,3,4))):check(value)


def test_zero_margins_preserve_offsets_holes_and_disconnected_components():
    mask = np.zeros((720, 1280), dtype=bool)
    mask[150:190, 590:650] = True
    mask[160:180, 600:640] = False
    mask[500, 23:25] = True
    mask[2, 1200] = True
    for view in (mask, mask[::-1, ::-1], mask.T, mask[:, ::2]):
        check(view)
