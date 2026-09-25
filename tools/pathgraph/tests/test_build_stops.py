"""Tests for tools/pathgraph/build_stops.py on synthetic data (no bags needed)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_stops as bs  # noqa: E402


def test_stretch_needs_min_duration():
    t = np.arange(0.0, 30.0, 1.0)
    v = np.full(30, 5.0)
    v[3:6] = 0.0            # 2 s between first and last slow sample: too short
    v[10:18] = 0.1          # 7 s: a stop
    v[25:] = 0.0            # open at the end of the bag, 4 s: too short
    assert bs.stop_stretches(t, v) == [(10.0, 17.0)]


def test_stretch_ignores_nan_speed():
    t = np.arange(0.0, 20.0, 1.0)
    v = np.full(20, np.nan)
    assert bs.stop_stretches(t, v) == []


def test_places_need_min_bags_and_cluster_by_gap():
    ev = [(f'b{i}', 0, 100.0 + i) for i in range(bs.MIN_BAGS)]          # recurring, one place
    ev += [('b0', 0, 400.0), ('b1', 0, 401.0)]                           # one-off: too few bags
    ev += [('b0', 1, 50.0), ('b0', 1, 51.0)] + [(f'c{i}', 1, 50.0) for i in range(bs.MIN_BAGS)]
    places = bs.stop_places(ev)
    assert [(b, round(s), n) for b, s, n in places] == [(0, 102, bs.MIN_BAGS), (1, 50, bs.MIN_BAGS + 1)]


def test_places_split_at_gap():
    ev = [(f'b{i}', 0, 100.0) for i in range(bs.MIN_BAGS)]
    ev += [(f'b{i}', 0, 100.0 + 2 * bs.GAP_M) for i in range(bs.MIN_BAGS)]
    assert [round(s) for _, s, _ in bs.stop_places(ev)] == [100, 100 + 2 * int(bs.GAP_M)]


def test_write_stops(tmp_path):
    p = tmp_path / 'stops.csv'
    bs.write_stops(p, [(0, 146.04, 17)], 'h')
    assert p.read_text(encoding='utf-8').splitlines() == ['# h', 'branch,s_m,n_bags', '0,146.0,17']


def test_edge_projection_is_not_a_place():
    s = np.arange(0.0, 100.0, 1.0)
    assert bs.edge_free(s, 50.0)
    assert not bs.edge_free(s, 0.0) and not bs.edge_free(s, 1.5)
    assert not bs.edge_free(s, 99.0) and not bs.edge_free(s, 97.5)
