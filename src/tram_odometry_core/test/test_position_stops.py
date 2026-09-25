"""Snap to stop places and the online wheel scale of PathTracker (D-034)."""
import dataclasses
import math

import pytest

from test_position import (ORIGIN, PARAMS, ROOT, _enu_bag, _fix_msg, _lla, _route, _wheels)
from tram_odometry_core.pipeline import Odometry
from tram_odometry_core.position import PathTracker
from tram_odometry_core.types import Route, load_route

P = PARAMS.position
START_S = 1000.0                                  # the anchor sits 1 km along branch 0


def _tracker(stops):
    r = _route()
    tr = PathTracker(PARAMS, Route(origin=r.origin, branches=r.branches, stops=tuple(stops)))
    tr.on_fix(*_lla(-START_S, 0.0, 170.0), 2, distance=0.0)
    return tr


def _arc(tr, distance, origin=None):
    """Map arc from `origin` (default: the anchor of the tracker); branch 0 runs due west."""
    x, y, _, _, _ = tr.advance(distance)
    return math.hypot(x - origin[0], y - origin[1])


def _anchor_xy(tr):
    return tr.advance(0.0)[:2]


def test_stop_near_a_place_snaps_s_and_cuts_the_variance():
    tr = _tracker([(0, 1500.0)])
    origin, before = _anchor_xy(tr), tr.advance(495.0)[4][0]
    assert tr.on_stop(495.0)
    assert _arc(tr, 495.0, origin) == pytest.approx(500.0, abs=0.7)   # was 495; place is 500 m on
    assert tr.advance(495.0)[4][0] < before


def test_stop_far_from_every_place_keeps_s():
    tr = _tracker([(0, 1500.0)])
    origin = _anchor_xy(tr)
    at = _arc(tr, 470.0, origin)
    assert not tr.on_stop(470.0)                                # 30 m short of the place
    assert _arc(tr, 470.0, origin) == pytest.approx(at)


def test_stop_at_a_nan_path_does_nothing():
    tr = _tracker([(0, 1500.0)])
    assert not tr.on_stop(float('nan'))
    assert tr.advance(495.0) is not None


def test_stop_without_places_or_alignment_does_nothing():
    assert not _tracker([]).on_stop(100.0)
    assert not PathTracker(PARAMS, _route()).on_stop(100.0)


def test_scale_follows_two_snaps_and_stays_in_bounds():
    tr = _tracker([(0, 1500.0), (0, 2000.0)])
    tr.on_stop(500.0)
    tr.on_stop(1005.0)                                          # wheels 505 m, map 500 m: -1 %
    assert tr._scale == pytest.approx(1.0 + P.scale_alpha * (500.0 / 505.0 - 1.0))


def test_scale_is_clamped_against_an_outlier_between_stops():
    tr = _tracker([(0, 1500.0), (0, 2000.0), (0, 2500.0), (0, 3000.0)])
    tr.on_stop(500.0)
    d = 500.0
    for _ in range(3):
        d += 5000.0                                             # wheel path 10x the map arc
        tr._last_snap = (0, tr._last_snap[1] - 500.0, tr._last_snap[2])   # keep a valid pair
        tr._anchor = (0, 1500.0 + 500.0 - 500.0, d)             # tram back at the place
        tr.on_stop(d)
    assert 1.0 - P.scale_max_dev <= tr._scale <= 1.0 + P.scale_max_dev


def test_scale_needs_a_long_enough_arc_between_snaps():
    tr = _tracker([(0, 1500.0), (0, 1600.0)])
    tr.on_stop(500.0)
    tr.on_stop(605.0)                                           # map 100 m < scale_min_arc_m, wheels 105
    assert tr._scale == 1.0


def _stream(with_late_gnss):
    """10 m/s for 60 s, 8 s standing 10 m short of a stop place, then 10 s on: the estimates
    at the end of the standstill and at the end."""
    r = _route()
    odo = Odometry(PARAMS, route=Route(origin=r.origin, branches=r.branches, stops=((0, 1600.0),)))
    odo.step(_fix_msg(0.0, _lla(-START_S, 0.0, 170.0)))
    _wheels(odo, 0.0, 59.0, 36.0)                     # 590 m: s = 1590
    standing = _wheels(odo, 59.1, 68.0, 0.0)
    if with_late_gnss:
        odo.step(_fix_msg(PARAMS.gnss.init_window_s + 30.0, _lla(-1900.0, 50.0, 190.0)))
    return standing, _wheels(odo, 68.1, 78.0, 36.0)


def test_pipeline_snaps_at_a_standstill():
    standing, _ = _stream(False)
    start = _lla(-START_S, 0.0, 170.0)
    fx, fy, _ = _enu_bag(*_lla(-1600.0, 0.0, 170.0), origin=start)
    assert math.hypot(standing.x - fx, standing.y - fy) < 2.0        # snapped from s = 1590


def test_late_gnss_changes_nothing_bit_for_bit():
    assert [dataclasses.astuple(e) for e in _stream(False)] == \
           [dataclasses.astuple(e) for e in _stream(True)]


def test_load_route_reads_the_stops_next_to_it(tmp_path):
    (tmp_path / 'route.csv').write_text(
        '# frame: ENU, origin_lat=55.8104, origin_lon=37.4623, origin_alt=168.0\n'
        'branch,s_m,x_m,y_m,z_m\n0,0.0,0.0,0.0,0.0\n0,1.0,1.0,0.0,0.0\n', encoding='utf-8')
    assert load_route(tmp_path / 'route.csv').stops == ()
    (tmp_path / 'stops.csv').write_text('# h\nbranch,s_m,n_bags\n0,0.5,7\n', encoding='utf-8')
    assert load_route(tmp_path / 'route.csv').stops == ((0, 0.5),)
    for bad in ('3,0.5,7', '0,5.0,7', '0,nan,7'):        # no such branch, past its end, NaN
        (tmp_path / 'stops.csv').write_text(f'# h\nbranch,s_m,n_bags\n{bad}\n', encoding='utf-8')
        with pytest.raises(ValueError):
            load_route(tmp_path / 'route.csv')


def test_stops_of_the_repository_map_are_valid():
    r = load_route(ROOT / 'src' / 'tram_odometry' / 'maps' / 'route.csv')
    assert len(r.stops) >= 20
    for b, s in r.stops:
        assert 0.0 < s < r.branches[b].s[-1]                # none clamped onto a branch end
