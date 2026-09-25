"""Tests for tools/pathgraph/build_route.py on synthetic tracks (no bags needed)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_route as br  # noqa: E402

REPO = Path(__file__).resolve().parents[3]


def curve(s):
    """Truth track: 200 m straight east, then a left 90 deg arc of radius 50 m, then north."""
    s = np.asarray(s, float)
    arc = np.pi / 2 * 50.0
    x = np.where(s < 200, s, np.where(s < 200 + arc, 200 + 50 * np.sin((s - 200) / 50), 250.0))
    y = np.where(s < 200, 0.0, np.where(s < 200 + arc, 50 - 50 * np.cos((s - 200) / 50),
                                         50 + (s - 200 - arc)))
    return np.column_stack([x, y])


def test_enu_origin_and_north_scale():
    e, n, u = br.lla_to_enu(np.array([55.8104, 55.8114]), np.array([37.4623, 37.4623]),
                            np.array([150.0, 150.0]), 55.8104, 37.4623, 150.0)
    assert abs(e[0]) < 1e-6 and abs(n[0]) < 1e-6
    # WGS84 meridian arc at 55.81 deg: M * 0.001 deg = 111.338 m
    assert n[1] == pytest.approx(111.338, abs=0.01)
    assert abs(e[1]) < 1e-3


def test_enu_east_scale():
    e, n, _ = br.lla_to_enu(np.array([55.8104]), np.array([37.4723]), np.array([0.0]),
                            55.8104, 37.4623, 0.0)
    # 0.01 deg of longitude at 55.81 deg: N * cos(lat) * 0.01 deg = 626.98 m
    assert e[0] == pytest.approx(626.98, abs=0.02)
    assert abs(n[0]) < 0.1  # curvature term only


def test_reject_outliers_drops_spikes_keeps_track():
    xy = curve(np.arange(0, 300, 1.5))
    xy[40] += [1500.0, -900.0]
    xy[41] += [1500.0, -900.0]
    xy[120] += [0.0, 25.0]
    keep = br.reject_outliers(xy, win=11, max_dev=3.0)
    assert not keep[40] and not keep[41] and not keep[120]
    assert keep.sum() == len(xy) - 3


def test_resample_step_and_monotonic_s():
    xy = curve(np.linspace(0, 350, 40))
    xy = np.vstack([xy[:10], np.repeat(xy[10:11], 5, axis=0), xy[10:]])  # standing still
    s, out = br.resample(xy, step=1.0)
    assert np.all(np.diff(s) > 0)
    assert np.max(np.hypot(*np.diff(out, axis=0).T)) <= 1.0 + 1e-9
    assert np.allclose(out[0], xy[0]) and np.allclose(out[-1], xy[-1], atol=1.0)


def test_project_straight_line_signed_offset():
    s, poly = br.resample(np.array([[0.0, 0.0], [100.0, 0.0]]), step=1.0)
    ps, pe = br.project(s, poly, np.array([[30.3, 2.0], [70.0, -1.5]]))
    assert ps == pytest.approx([30.3, 70.0], abs=1e-6)
    assert pe == pytest.approx([2.0, -1.5], abs=1e-6)  # left of travel direction is positive


def test_refine_branch_recovers_truth_from_shifted_reference():
    rng = np.random.default_rng(0)
    total = 200 + np.pi / 2 * 50 + 150
    s_true, truth = br.resample(curve(np.linspace(0, total, 2000)), step=1.0)
    normal = np.column_stack([-np.gradient(truth[:, 1]), np.gradient(truth[:, 0])])
    normal /= np.hypot(*normal.T)[:, None]
    passes = []
    for bias in (-0.4, 0.0, 0.3, 0.5, -0.2):
        idx = np.sort(rng.integers(0, len(truth), 3000))
        off = bias + rng.normal(0, 0.3, len(idx))
        passes.append(truth[idx] + normal[idx] * off[:, None])
    _, ref = br.resample(truth + normal * 2.0, step=1.0)   # initial reference 2 m off
    s, out = br.refine_branch(ref, passes, step=1.0, gates=(6.0, 6.0, 6.0), min_passes=3,
                              smooth_m=15.0)
    _, err = br.project(s_true, truth, out)
    inner = (s > 20) & (s < s[-1] - 20)
    # truth + median of the pass biases (0.0)
    assert np.max(np.abs(err[inner])) < 0.25
    assert np.max(np.hypot(*np.diff(out, axis=0).T)) <= 1.0 + 1e-9


def test_route_csv_roundtrip_matches_contract(tmp_path):
    s0, xy0 = br.resample(curve(np.linspace(0, 300, 50)), step=1.0)
    s1, xy1 = br.resample(curve(np.linspace(300, 0, 50)), step=1.0)
    path = tmp_path / 'route.csv'
    br.write_route(path, [(s0, xy0), (s1, xy1)], 'frame: ENU, origin_lat=55.8104, origin_lon=37.4623')
    lines = path.read_text().splitlines()
    assert lines[0].startswith('# frame: ENU, origin_lat=55.8104, origin_lon=37.4623')
    assert 'branch,s_m,x_m,y_m' in lines
    route = br.read_route(path)
    assert sorted(route) == [0, 1]
    for b, (s, xy) in route.items():
        assert s[0] == 0.0 and np.all(np.diff(s) > 0) and np.max(np.diff(s)) <= 2.0
    assert np.allclose(route[1][1], xy1, atol=1e-3)


def test_split_bag_names_stay_strings():
    # PyYAML's safe_load turns 30618_68847170 into an int (YAML 1.1 allows '_' in ints)
    train = br.split_bags(REPO / 'tools' / 'eval' / 'splits.yaml', 'train')
    assert '30618_68847170' in train
    assert all(isinstance(n, str) for n in train)


def test_reference_ok_accepts_dropout_and_small_step_rejects_big_step_and_long_gap():
    t = np.arange(0, 100, 0.1)
    xy = np.column_stack([t * 10.0, np.zeros_like(t)])   # 10 m/s east
    assert br.reference_ok(t, xy)
    drop = np.r_[0:300, 330:len(t)]                       # 3 s dropout = 30 m at 10 m/s
    assert br.reference_ok(t[drop], xy[drop])
    small = xy.copy()
    small[500:600, 1] += 5.0                              # 5 m step: first wide gate fixes it
    assert br.reference_ok(t, small)
    stepped = xy.copy()
    stepped[500:600, 1] += 12.0                           # 12 m position step within 0.1 s
    assert not br.reference_ok(t, stepped)
    long_gap = np.r_[0:300, 360:len(t)]                   # 6 s = 60 m dropout
    assert not br.reference_ok(t[long_gap], xy[long_gap])


def _line_pass(x, y, t0=0.0, speed=10.0):
    return t0 + (x - x[0]) / speed, np.column_stack([x, y])   # time by x: a y-jump takes 0.1 s


def test_refine_branch_ignores_bins_where_passes_split_between_tracks():
    x = np.arange(0.0, 400.0, 1.0)
    passes = [np.column_stack([x, np.zeros_like(x)]) for _ in range(3)]
    passes += [np.column_stack([x, np.where(x < 200, 0.0, 8.0)]) for _ in range(3)]  # other track
    s, out = br.refine_branch(np.column_stack([x, np.zeros_like(x)]), passes, step=1.0,
                              gates=(15.0, 6.0), min_passes=3, smooth_m=15.0)
    assert np.max(np.abs(out[:, 1])) < 0.5


def test_coverage_ignores_standing_jitter():
    rng = np.random.default_rng(1)
    standing = rng.normal(0, 0.5, (1000, 2))
    line = np.column_stack([np.arange(0.0, 100.0, 1.0), np.zeros(100)])
    assert br.coverage(standing) <= 4
    assert br.coverage(line) >= 20


def test_extra_branch_for_side_track_but_not_for_gnss_step():
    x = np.arange(0.0, 400.0, 1.0)
    main = br.resample(np.column_stack([x, np.zeros_like(x)]), 1.0)
    tracks = [_line_pass(x, np.zeros_like(x)) for _ in range(3)]
    xs = np.arange(0.0, 450.0, 1.0)
    side_y = np.clip((xs - 300.0) / 20.0, 0.0, 1.0) * 10.0      # smooth turnout onto a track 10 m away
    tracks += [_line_pass(xs, side_y) for _ in range(2)]
    glitch = np.zeros_like(x)
    glitch[100:150] = 12.0                                       # GNSS step, out and back in 0.1 s
    tracks.append(_line_pass(x, glitch))
    branches = br.add_extra_branches([main], tracks)
    assert len(branches) == 2
    s, poly = branches[1]
    _, d = br.project(s, poly, np.array([[400.0, 10.0], [440.0, 10.0]]))
    assert np.max(np.abs(d)) < 1.0
