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
WHEEL = 1.015                                     # wheels read 1.5 % long: another tram (trap 17)


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


def test_stop_with_two_places_inside_gate_does_not_snap_to_neighbour():
    tr = _tracker([(0, 1500.0), (0, 1517.8)])
    origin = _anchor_xy(tr)
    at = _arc(tr, 510.0, origin)                              # 10 m after the first place

    assert not tr.on_stop(510.0)
    assert _arc(tr, 510.0, origin) == pytest.approx(at)
    assert tr._last_snap is None
    assert tr._scale == 1.0


@pytest.mark.parametrize("distance", [518.0, 519.0])
def test_stop_with_indistinguishable_second_place_just_outside_gate_does_not_snap(distance):
    tr = _tracker([(0, 1500.0), (0, 1540.0)])
    origin = _anchor_xy(tr)
    at = _arc(tr, distance, origin)

    assert not tr.on_stop(distance)
    assert _arc(tr, distance, origin) == pytest.approx(at)
    assert tr._last_snap is None
    assert tr._scale == 1.0


def test_stop_clear_of_neighbour_still_snaps():
    tr = _tracker([(0, 1500.0), (0, 1517.8)])
    assert tr.on_stop(500.0)
    assert tr._last_snap == (0, 1500.0, 500.0)


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


def _gain(var, r):
    return var / (var + r)


def test_scale_is_a_kalman_update_weighted_by_the_arc():
    """#154: the scale is a Kalman state with the prior along_drift_frac; a snap measures the
    ratio of map arc to wheel path from the reference with the variance of both ends / path^2."""
    tr = _tracker([(0, 1500.0), (0, 2000.0)])
    anchor, v = tr._anchor[1], P.along_drift_frac ** 2         # the fix lands ~1000 m on
    assert tr.on_stop(500.0)                                    # from the anchor: ratio ~1
    g = _gain(v, (P.anchor_std_m ** 2 + P.stop_std_m ** 2) / 500.0 ** 2)
    scale, v = 1.0 + g * ((1500.0 - anchor) / 500.0 - 1.0), v * (1.0 - g)
    assert tr._scale == pytest.approx(scale, abs=1e-12)
    assert tr.on_stop(1005.0)                                   # wheels 505 m, map 500 m: -1 %
    g = _gain(v, 2.0 * P.stop_std_m ** 2 / 505.0 ** 2)
    assert tr._scale == pytest.approx(scale + g * (500.0 / 505.0 - scale), abs=1e-12)


def test_a_long_arc_weighs_more_than_a_short_one():
    short, long_ = _tracker([(0, 1400.0)]), _tracker([(0, 2500.0)])
    assert short.on_stop(404.0) and long_.on_stop(1515.0)      # both -1 % from the anchor
    assert 1.0 - long_._scale > 1.0 - short._scale > 0.0


def test_scale_is_clamped_against_an_outlier():
    r = _route()
    params = dataclasses.replace(PARAMS, position=dataclasses.replace(P, along_drift_frac=1.0))
    tr = PathTracker(params, Route(origin=r.origin, branches=r.branches, stops=((0, 1500.0),)))
    tr.on_fix(*_lla(-START_S, 0.0, 170.0), 2, distance=0.0)
    assert tr.on_stop(519.0)                                    # map 500 m over 519 m: -3.7 %
    assert tr._scale == pytest.approx(1.0 - P.scale_max_dev)


def test_short_arc_keeps_the_reference_of_the_scale():
    """A snap 100 m after the anchor (< scale_min_arc_m) does not measure the scale; the next
    one 500 m on still measures it from the anchor, not from the short snap."""
    tr = _tracker([(0, 1100.0), (0, 1500.0)])
    anchor = tr._anchor[1]
    assert tr.on_stop(100.0 * WHEEL)
    assert tr._scale == 1.0
    assert tr.on_stop(500.0 * WHEEL)
    wheel = 500.0 * WHEEL
    g = _gain(P.along_drift_frac ** 2, (P.anchor_std_m ** 2 + P.stop_std_m ** 2) / wheel ** 2)
    assert tr._scale == pytest.approx(1.0 + g * ((1500.0 - anchor) / wheel - 1.0), abs=1e-12)


def test_scale_is_not_measured_across_a_branch_join():
    """The join of two branches is ~10 m off along in the real map (terminal loop), so an arc
    through it is not the map's: the reference on the other branch is dropped."""
    r = _route()
    tr = PathTracker(PARAMS, Route(origin=r.origin, branches=r.branches, stops=((1, 250.0),)))
    tr.on_fix(*_lla(-4900.0, 0.0, 178.0), 2, distance=0.0)     # 100 m before the join
    assert tr.on_stop(350.0 * WHEEL)                           # 250 m down branch 1
    assert tr._scale == 1.0


def test_first_stop_after_the_anchor_already_corrects_the_wheel_scale():
    """#154: the GNSS anchor is the first reference of the scale. Wheels 1.5 % long must be
    corrected at the first stop after the window, not from the second snap on (main kept the
    scale at 1.0 there and missed the next place by 8.3 m)."""
    tr = _tracker([(0, 1500.0), (0, 2000.0)])
    origin = _anchor_xy(tr)
    assert tr.on_stop(500.0 * WHEEL)                           # at the first place
    ahead = _arc(tr, 1000.0 * WHEEL, origin)                   # at the second place, not snapped
    assert tr._scale < 1.0
    assert abs(ahead - 1000.0) < 5.0


def test_lost_lock_is_recovered_by_consistent_misses():
    """#154: no place for 1.4 km, wheels 1.5 % long: the misses grow 21 -> 24 -> 27 m, all
    past stop_snap_max_m. Two misses on one side growing with the path are a wheel scale error,
    not a signal: the tracker must lock onto the places again (main never snapped again)."""
    places = [2400.0, 2600.0, 2800.0, 3000.0, 3200.0]
    tr = _tracker([(0, s) for s in places])
    origin = _anchor_xy(tr)
    snapped = [tr.on_stop((s - START_S) * WHEEL) for s in places]
    assert snapped[-2:] == [True, True]
    ahead = _arc(tr, (3400.0 - START_S) * WHEEL, origin)
    assert abs(ahead - (3400.0 - START_S)) < 3.0


def test_relock_learns_the_scale_as_unknown_again():
    """Four exact snaps make the scale confident; then the wheels read 1.5 % long (another
    tram in the stress test) and the lock is lost. The relock must measure the scale with the
    prior variance again, not with the confidence the lost lock has disproved."""
    tr = _tracker([(0, s) for s in (1500.0, 2000.0, 2500.0, 3000.0, 4400.0, 4600.0, 4800.0)])
    assert all(tr.on_stop(d) for d in (500.0, 1000.0, 1500.0, 2000.0))
    assert tr._scale_var < 0.1 * P.along_drift_frac ** 2
    assert [tr.on_stop(2000.0 + (s - 3000.0) * WHEEL) for s in (4400.0, 4600.0)] == [False, True]
    assert tr._scale == pytest.approx(1.0 / WHEEL, abs=1e-3)


def test_relock_pair_stays_out_of_the_speed_chain():
    """The pair of snaps across a lost lock does not enter the speed chain of #153 (D-082): the
    lost stretch may hold a wheel gap or the relock may be false (3 of 16 relocks in the train
    stress gap_both_30). The chain goes on from the relock place."""
    tr = _tracker([(0, s) for s in (1500.0, 2000.0, 2500.0, 3000.0, 4400.0, 4600.0, 4800.0)])
    assert all(tr.on_stop(d) for d in (500.0, 1000.0, 1500.0, 2000.0))   # chain: 1500 m / 1500 m
    at = [2000.0 + (s - 3000.0) * WHEEL for s in (4400.0, 4600.0, 4800.0)]
    assert [tr.on_stop(d) for d in at] == [False, True, True]            # relock at 4600
    prior = P.speed_scale_prior_m
    assert tr.speed_scale == pytest.approx((1700.0 + prior) / (1500.0 + at[2] - at[1] + prior))


def test_stops_off_the_places_do_not_fake_a_lock():
    """Two misses on one side that do not grow with the path as a scale error does (the second
    is 8.8 m off the line of the first from the anchor, 2.7 sigma) are signals: s is kept."""
    tr = _tracker([(0, 2400.0), (0, 2700.0)])
    origin = _anchor_xy(tr)
    at = [_arc(tr, d, origin) for d in (1370.0, 1655.0)]
    assert not tr.on_stop(1370.0)                              # 30 m short of 2400
    assert not tr.on_stop(1655.0)                              # 45 m short of 2700
    assert [_arc(tr, d, origin) for d in (1370.0, 1655.0)] == pytest.approx(at)


def test_a_miss_no_wheel_scale_explains_is_a_signal():
    """25 m and 50 m short 100 and 200 m after the anchor lie on one line, but would need a
    scale 25 % off: past scale_max_dev, so they are signals."""
    tr = _tracker([(0, 1125.0), (0, 1250.0)])
    assert not tr.on_stop(100.0)
    assert not tr.on_stop(200.0)


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


def test_speed_scale_is_one_before_any_chain():
    tr = _tracker([(0, 1500.0)])
    assert tr.speed_scale == 1.0
    tr.on_stop(495.0)                                  # a single snap is not a pair
    assert tr.speed_scale == 1.0


def test_speed_scale_telescopes_consecutive_snaps_including_short_pairs():
    # #153: map 100 m + 500 m over wheels 103 m + 507 m; the 100 m pair is below
    # scale_min_arc_m for the path scale but still counts in the chain
    tr = _tracker([(0, 1500.0), (0, 1600.0), (0, 2100.0)])
    assert tr.on_stop(500.0) and tr.on_stop(603.0) and tr.on_stop(1110.0)
    prior = P.speed_scale_prior_m
    assert tr._chain_arc == pytest.approx(600.0)
    assert tr._chain_wheel == pytest.approx(610.0)
    assert tr.speed_scale == pytest.approx((600.0 + prior) / (610.0 + prior))


def test_speed_scale_does_not_pair_snaps_on_different_branches():
    tr = _tracker([(0, 1500.0), (0, 2100.0)])
    assert tr.on_stop(500.0)
    tr._last_snap = (1, tr._last_snap[1], tr._last_snap[2])   # as if the last snap was on branch 1
    assert tr.on_stop(1100.0)
    assert tr._chain_arc == 0.0 and tr._chain_wheel == 0.0 and tr.speed_scale == 1.0


def test_speed_scale_is_clamped():
    tr = _tracker([(0, 1500.0)])
    tr._chain_arc, tr._chain_wheel = 1.0e6, 2.0e6                 # a ratio of 0.5
    assert tr.speed_scale == pytest.approx(1.0 - P.scale_max_dev)


def test_pipeline_publishes_speed_times_the_chain_scale():
    r = _route()
    route = Route(origin=r.origin, branches=r.branches, stops=((0, 1600.0),))
    plain, scaled = Odometry(PARAMS, route=route), Odometry(PARAMS, route=route)
    for odo in (plain, scaled):
        odo.step(_fix_msg(0.0, _lla(-START_S, 0.0, 170.0)))
        _wheels(odo, 0.0, 9.0, 36.0)
    scaled._tracker._chain_arc, scaled._tracker._chain_wheel = 1010.0, 1000.0
    k = scaled._tracker.speed_scale
    assert k > 1.0
    a, b = _wheels(plain, 9.1, 10.0, 36.0), _wheels(scaled, 9.1, 10.0, 36.0)
    assert b.speed == pytest.approx(a.speed * k)
    assert (b.x, b.y, b.distance) == (a.x, a.y, a.distance)       # the path keeps its own scale


def test_speed_scale_skips_a_pair_that_is_not_one_stretch_of_track():
    # review #153: two snaps on branch 0 a whole loop apart (no snap on the way) -- map 600 m,
    # wheels 10 km; pairing them would pin the speed scale at the clamp for the rest of the run
    tr = _tracker([(0, 1500.0), (0, 2100.0)])
    assert tr.on_stop(500.0)
    tr._anchor = (0, 2100.0, 10500.0)
    assert tr.on_stop(10500.0)
    assert tr._chain_arc == 0.0 and tr._chain_wheel == 0.0 and tr.speed_scale == 1.0


def test_speed_scale_skips_a_place_behind_the_previous_one():
    # 30 m back over 5 m of wheels passes the stretch guard; only the order check stops it
    tr = _tracker([(0, 1500.0), (0, 1530.0)])
    assert tr.on_stop(530.0)                           # s = 1530
    tr._anchor = (0, 1500.0, 535.0)
    assert tr.on_stop(535.0)                           # place 1500 behind 1530
    assert tr._chain_arc == 0.0 and tr._chain_wheel == 0.0


def test_speed_scale_ignores_a_stop_at_a_shorter_path():
    tr = _tracker([(0, 1500.0), (0, 1600.0)])
    assert tr.on_stop(500.0) and tr.on_stop(603.0)
    sums = (tr._chain_arc, tr._chain_wheel)
    assert tr.on_stop(600.0)                           # the same place 3 m of path earlier
    assert (tr._chain_arc, tr._chain_wheel) == sums
