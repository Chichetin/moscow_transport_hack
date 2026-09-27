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
    z0, z1 = 0.02 * s0, -0.01 * s1
    path = tmp_path / 'route.csv'
    br.write_route(path, [(s0, xy0, z0), (s1, xy1, z1)],
                   'frame: ENU, origin_lat=55.8104, origin_lon=37.4623')
    lines = path.read_text(encoding='utf-8').splitlines()
    assert lines[0].startswith('# frame: ENU, origin_lat=55.8104, origin_lon=37.4623')
    assert 'branch,s_m,x_m,y_m,z_m' in lines
    route = br.read_route(path)
    assert sorted(route) == [0, 1]
    for b, (s, xy, z) in route.items():
        assert s[0] == 0.0 and np.all(np.diff(s) > 0) and np.max(np.diff(s)) <= 2.0
    assert np.allclose(route[1][1], xy1, atol=1e-3)
    assert np.allclose(route[0][2], z0, atol=1e-3) and np.allclose(route[1][2], z1, atol=1e-3)


def test_clean_track_returns_enu_up():
    lat0, lon0, alt0 = br.ORIGIN
    fixes = np.array([[float(i), lat0, lon0, alt0 + 2.0, 2.0] for i in range(20)])
    t, xy, z = br.clean_track(fixes, (2,))
    assert len(t) == 20 and np.allclose(xy, 0.0, atol=1e-6)
    assert np.allclose(z, 2.0, atol=1e-6)


def test_branch_height_is_median_of_passes_along_branch():
    rng = np.random.default_rng(2)
    x = np.arange(0.0, 400.0, 1.0)
    s, poly = br.resample(np.column_stack([x, np.zeros_like(x)]), 1.0)
    truth = 0.03 * s + 2.0 * np.sin(s / 60.0)
    passes = []
    for bias in (-0.3, 0.0, 0.1, 0.2, -0.1):
        px = np.sort(rng.uniform(0, 399, 3000))
        xy = np.column_stack([px, rng.normal(0, 0.3, len(px))])
        passes.append((xy, 0.03 * px + 2.0 * np.sin(px / 60.0) + bias + rng.normal(0, 0.2, len(px))))
    passes.append((np.column_stack([x, np.full_like(x, 8.0)]), np.full_like(x, 50.0)))  # other track
    glitch = 0.03 * x + 2.0 * np.sin(x / 60.0) + np.where((x > 100) & (x < 200), 20.0, 0.0)
    passes.append((np.column_stack([x, np.zeros_like(x)]), glitch))   # same track, GNSS height jump
    z = br.branch_height(s, poly, passes, gate=6.0, min_passes=3, smooth_m=15.0)
    inner = (s > 10) & (s < s[-1] - 10)
    assert np.max(np.abs(z - truth)[inner]) < 0.15


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


def test_refine_branch_follows_recent_passes_where_the_track_was_relaid():
    x = np.arange(0.0, 400.0, 1.0)
    relaid = np.clip(np.minimum(x - 150.0, 250.0 - x) / 20.0, 0.0, 1.0) * 4.0  # 4 m off at 170..230
    passes = [np.column_stack([x, np.zeros_like(x)]) for _ in range(5)]        # older majority
    rng = np.random.default_rng(2)
    for _ in range(3):                      # current layout, fixes ~1 m apart: 1 m bins with holes
        xs = np.sort(rng.uniform(0.0, 400.0, 400))
        passes.append(np.column_stack([xs, np.interp(xs, x, relaid)]))
    recent = np.array([False] * 5 + [True] * 3)
    ref = np.column_stack([x, np.zeros_like(x)])
    for mask, y_mid in ((None, 0.0), (recent, 4.0)):
        s, out = br.refine_branch(ref, passes, step=1.0, gates=(15.0, 6.0), min_passes=3,
                                  smooth_m=15.0, recent=mask)
        mid = (out[:, 0] > 180) & (out[:, 0] < 220)
        away = (out[:, 0] < 120) | (out[:, 0] > 280)
        assert np.max(np.abs(out[mid, 1] - y_mid)) < 0.3
        assert np.max(np.abs(out[away, 1])) < 0.05     # agreeing bins stay where all passes are
        assert np.max(np.abs(np.diff(out[:, 1]))) < 0.25   # no zig-zag from the holes


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


def _line(x0, x1, y, n):
    return np.column_stack([np.linspace(x0, x1, n), np.full(n, float(y))])


def test_a_pass_against_the_only_nearby_branch_is_uncovered():
    """A single-ended tram never drives a branch backwards: a westbound pass 3 m off an
    eastbound branch is a track of its own (the depot entry at the west terminal, #138)."""
    branches = [br.resample(_line(0.0, 500.0, 0.0, 501), br.STEP_M)]
    west = _line(400.0, 200.0, 3.0, 1001)                  # 0.2 m per sample: 10 Hz at 2 m/s
    t = np.arange(len(west)) * 0.1
    assert len(br.uncovered_pieces(t, west, branches)) == 1
    east = west[::-1]
    assert br.uncovered_pieces(t, east, branches) == []


def test_a_standing_tram_has_no_direction_and_stays_covered():
    xy = np.column_stack([np.full(200, 100.0), np.full(200, 3.0)]) + 0.05 * np.sin(np.arange(200))[:, None]
    assert not br._travel_direction(xy).any()


def test_a_piece_alongside_a_branch_of_its_direction_is_a_gnss_shift_not_a_track():
    branches = [br.resample(_line(500.0, 0.0, 0.0, 501), br.STEP_M)]   # westbound branch
    shifted = _line(400.0, 280.0, -6.0, 1201)               # westbound, 6 m off all along
    diverging = np.column_stack([np.linspace(400.0, 280.0, 1201), np.linspace(-6.0, -60.0, 1201)])
    assert br._shifted_branch(shifted, branches)
    assert not br._shifted_branch(diverging, branches)
