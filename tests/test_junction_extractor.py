from app.spatial.junction_extractor import extract_junction_candidates
from app.spatial.models import SpatialMapSnapshot, SpatialPose


def _cross_map():
    width = height = 100
    free = []
    for row in range(height):
        for column in range(width):
            x = (column - 50) * .05
            y = (row - 50) * .05
            if abs(x) <= .5 or abs(y) <= .5:
                free.append((column, row))
    return SpatialMapSnapshot(
        revision=1,
        resolution_m=.05,
        origin=(-2.5, -2.5),
        width=width,
        height=height,
        free=free,
        provenance={"frame_id": "map"},
    )


def test_junction_candidates_are_stable_known_free_clearance_points():
    snapshot = _cross_map()
    pose = SpatialPose(0., 0., frame_id="map")
    candidates = extract_junction_candidates(snapshot, pose)
    assert len(candidates) >= 2
    assert len({item.frontier_id for item in candidates}) == len(candidates)
    for item in candidates:
        assert item.provenance["source"] == "junction"
        assert item.provenance["known_free_endpoint"] is True
        assert item.frame == "map"


def test_single_corridor_is_not_a_junction():
    snapshot = _cross_map()
    snapshot.free = [
        cell for cell in snapshot.free
        if abs((cell[1] - 50) * .05) <= .5
    ]
    assert extract_junction_candidates(
        snapshot, SpatialPose(0., 0., frame_id="map")
    ) == []
