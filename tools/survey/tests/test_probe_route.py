"""probe_route: stop detection and stop-place clustering on synthetic series."""
import numpy as np

import probe_route as pr


def test_stop_needs_min_duration():
    t = np.arange(0.0, 60.0, 0.1)
    v = np.full_like(t, 5.0)
    v[(t >= 10) & (t < 25)] = 0.0      # 15 s stop
    v[(t >= 40) & (t < 45)] = 0.0      # 5 s pause at a light
    segs = pr.stop_segments(t, v, v_max=0.2, min_s=10.0)
    assert len(segs) == 1
    i0, i1 = segs[0]
    assert abs(t[i0] - 10.0) < 1e-9 and abs(t[i1 - 1] - 24.9) < 1e-9


def test_stop_at_the_end_of_the_series_counts():
    t = np.arange(0.0, 30.0, 0.1)
    v = np.where(t < 15, 5.0, 0.0)
    assert pr.stop_segments(t, v, min_s=10.0) == [(150, 300)]


def test_moving_series_has_no_stops():
    t = np.arange(0.0, 30.0, 0.1)
    assert pr.stop_segments(t, np.full_like(t, 3.0)) == []


def test_cluster_merges_close_points_and_keeps_far_apart():
    pts = np.array([[0.0, 0.0], [3.0, 0.0], [0.0, 4.0], [100.0, 0.0], [102.0, 1.0]])
    groups = sorted(sorted(m) for _, m in pr.cluster(pts, radius=20.0))
    assert groups == [[0, 1, 2], [3, 4]]


def test_cluster_centre_is_member_mean():
    pts = np.array([[0.0, 0.0], [4.0, 0.0]])
    (centre, members), = pr.cluster(pts, radius=20.0)
    assert np.allclose(centre, [2.0, 0.0]) and sorted(members) == [0, 1]


def test_to_xy_one_degree_north_is_111_km():
    xy = pr.to_xy([55.0, 56.0], [37.0, 37.0], 55.0, 37.0)
    assert abs(xy[1, 1] - 111319.5) < 1.0 and abs(xy[1, 0]) < 1e-6
