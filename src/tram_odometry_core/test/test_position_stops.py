"""Snap to stop places and the online wheel scale of PathTracker (D-034)."""
import dataclasses
import math

import pytest

from test_position import (ORIGIN, PARAMS, ROOT, _depot_route, _enu_bag, _fix_msg, _grid, _lla,
                           _route, _wheels)
from tram_odometry_core.pipeline import Odometry
from tram_odometry_core.position import PathTracker
from tram_odometry_core.types import Route, load_route

P = PARAMS.position
START_S = 1000.0                                  # the anchor sits 1 km along branch 0
WHEEL = 1.015                                     # wheels read 1.5 % long: another tram (trap 17)


class _Main(PathTracker):
    """PathTracker of main before #154: a stop past the snap gate is always a signal."""

    def _relock(self, innovation, distance, ambiguous):
        return False


def _tracker(stops, params=PARAMS, cls=PathTracker):
    r = _route()
    tr = cls(params, Route(origin=r.origin, branches=r.branches, stops=tuple(stops)))
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
    fx, fy, _ = _grid(*_lla(-1600.0, 0.0, 170.0))
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


# --- relock of a lost lock (#154, H3) ---------------------------------------------------------

def _position(**changes):
    return dataclasses.replace(PARAMS, position=dataclasses.replace(P, **changes))


def test_lost_lock_is_recovered_by_two_misses_on_one_line():
    """#154: no place for 1.4 km and wheels 1.5 % long: the misses grow 21 -> 24 m, past
    stop_snap_max_m, and on main the lock was lost for good (the error only grows). Two misses
    on one side on the line of a scale error from the anchor relock onto the place."""
    places = [2400.0, 2600.0, 2800.0, 3000.0, 3200.0]
    tr = _tracker([(0, s) for s in places])
    origin = _anchor_xy(tr)
    assert [tr.on_stop((s - START_S) * WHEEL) for s in places] == [False, True, True, True, True]
    assert tr._last_snap[1] == 3200.0
    assert abs(_arc(tr, 2200.0 * WHEEL, origin) - 2200.0) < 3.0


def test_relock_is_a_snap_for_the_path_scale_but_not_for_the_speed_chain():
    """The relock is an ordinary snap for the EMA of the path scale (pair: the last snap -> the
    relock place). The speed chain of #153 (D-082) skips that pair: the lost stretch may hold
    a wheel gap or the relock may be false. The chain goes on from the relock place."""
    tr = _tracker([(0, s) for s in (1500.0, 2000.0, 2500.0, 3000.0, 4400.0, 4600.0, 4800.0)])
    assert all(tr.on_stop(d) for d in (500.0, 1000.0, 1500.0, 2000.0))   # exact: 1500 m / 1500 m
    assert tr._scale == 1.0
    at = [2000.0 + (s - 3000.0) * WHEEL for s in (4400.0, 4600.0, 4800.0)]
    assert [tr.on_stop(d) for d in at] == [False, True, True]            # relock at 4600
    assert tr._scale == pytest.approx(1.0 + P.scale_alpha * (1600.0 / (at[1] - 2000.0) - 1.0))
    prior = P.speed_scale_prior_m
    assert tr.speed_scale == pytest.approx((1700.0 + prior) / (1500.0 + at[2] - at[1] + prior))


def test_stops_off_the_line_of_a_scale_error_are_signals():
    """Two misses on one side that do not grow with the path as a scale error does (the second
    is 8.8 m off the line of the first from the anchor, 2.7 sigma) are signals: s is kept."""
    tr = _tracker([(0, 2400.0), (0, 2700.0)])
    origin = _anchor_xy(tr)
    at = [_arc(tr, d, origin) for d in (1370.0, 1655.0)]
    assert not tr.on_stop(1370.0)                              # 30 m short of 2400
    assert not tr.on_stop(1655.0)                              # 45 m short of 2700
    assert [_arc(tr, d, origin) for d in (1370.0, 1655.0)] == pytest.approx(at)
    assert tr._last_snap is None and tr._scale == 1.0


def test_relock_sigma_is_the_width_of_the_line():
    """The same two stops relock with relock_sigma = 3: the gate is sigmas of the residual."""
    tr = _tracker([(0, 2400.0), (0, 2700.0)], _position(relock_sigma=3.0))
    assert not tr.on_stop(1370.0)
    assert tr.on_stop(1655.0)
    assert tr._last_snap == (0, 2700.0, 1655.0)


def test_a_miss_no_wheel_scale_explains_is_a_signal():
    """25 m and 50 m short 100 and 200 m after the anchor lie on one line, but would need a
    scale 25 % off: past scale_max_dev, so they are signals."""
    tr = _tracker([(0, 1125.0), (0, 1250.0)])
    assert not tr.on_stop(100.0)
    assert not tr.on_stop(200.0)


@pytest.mark.parametrize('signal_between, relocked', [(False, True), (True, False)])
def test_a_signal_between_two_misses_drops_the_first(signal_between, relocked):
    """A stop 180 m from every place (a signal no wheel scale explains) between two misses on
    one line: the misses are not consecutive, so no relock."""
    tr = _tracker([(0, 2400.0), (0, 2800.0)])
    stops = (1421.0, 1620.0, 1827.0) if signal_between else (1421.0, 1827.0)
    assert [tr.on_stop(d) for d in stops] == [False] * (len(stops) - 1) + [relocked]


@pytest.mark.parametrize('place, relocked', [(3800.0, True), (3770.0, True), (3867.0, False)])
def test_line_of_two_misses_from_a_loose_anchor(place, relocked):
    """With a loose anchor (100 m) the line of the first miss (21 m short) is wide: the
    anchor error a enters both misses, (1 - q) a is left of it. A second miss on the line (42 m
    short) or 30 m off it relocks; one 25 m past the place fits the line too, but a scale
    error puts both misses on one side: no relock."""
    tr = _tracker([(0, 2400.0), (0, place)], _position(anchor_std_m=100.0))
    assert not tr.on_stop(1421.0)                              # 21 m short of 2400
    assert tr.on_stop(2842.0) is relocked                      # q = 2


@pytest.mark.parametrize('sign, relocked', [(-1.0, False), (1.0, True)])
def test_a_miss_is_explained_from_the_scale_already_learnt(sign, relocked):
    """At a scale of 0.98 misses of -2 % of the path need a scale of 0.96, past scale_max_dev
    (signals); misses of +2 % need 1.00 (a relock)."""
    stops = (1500.0, 1800.0)
    tr = _tracker([(0, START_S + (0.98 + sign * 0.02) * d) for d in stops])
    tr._scale = 0.98
    assert [tr.on_stop(d) for d in stops] == [False, relocked]


def test_a_miss_at_the_anchor_itself_is_a_signal():
    tr = _tracker([(0, 1100.0)])
    assert not tr.on_stop(0.0)                                 # 100 m off, no path to scale


def test_a_snap_drops_the_pending_miss():
    """A miss, a snap, a miss: the first miss belongs to the old anchor. With places this
    loose (stop_std_m 50 m) the line of it would take the last miss."""
    tr = _tracker([(0, 2400.0), (0, 2600.0), (0, 3990.0)], _position(stop_std_m=50.0))
    assert [tr.on_stop(d) for d in (1421.0, 1615.0, 3015.0)] == [False, True, False]


def test_a_second_stop_at_the_same_path_is_not_a_second_miss():
    tr = _tracker([(0, 2400.0)])
    assert [tr.on_stop(1421.0), tr.on_stop(1421.0)] == [False, False]


@pytest.mark.parametrize('from_anchor', [1000.0, 1400.0, 2000.0, 3000.0])
@pytest.mark.parametrize('queue', [25.0, 30.0, 40.0])
@pytest.mark.parametrize('creep', [0.3, 0.5, 2.0, 5.0])
def test_a_creep_in_a_queue_is_one_standstill(from_anchor, queue, creep):
    """Reviewer of #160: exact wheels, a queue `queue` m short of a place and a creep of
    `creep` m in it (the pipeline sees two standstills). Both misses are one place - s apart by
    the creep, q = 1 + creep / path: the residual y - q y1 is about the creep, inside 2 sigma
    (5.7 m) of the line, and 6853330 relocked onto the place in the queue (on_stop
    [F, T, F, F, F], +28.9 m at 1.4 km). A second stop closer than stop_snap_max_m of path to the
    pending miss is the same standstill: the stops, the anchor, the path scale and the speed
    scale are those of main; with both stops past the gate the place and the next ones snap
    exactly."""
    places = [START_S + from_anchor + k * 500.0 for k in range(3)]
    tr = _tracker([(0, s) for s in places])
    main = _tracker([(0, s) for s in places], cls=_Main)
    stops = [from_anchor - queue, from_anchor - queue + creep] + [s - START_S for s in places]
    got = [tr.on_stop(d) for d in stops]
    assert got == [main.on_stop(d) for d in stops]
    assert (tr._anchor, tr._scale, tr.speed_scale) == (main._anchor, main._scale, main.speed_scale)
    if queue - creep > P.stop_snap_max_m:
        assert got == [False, False, True, True, True]
        assert tr._state(stops[-1])[1] == pytest.approx(places[-1], abs=0.01)


def test_a_creep_in_a_queue_keeps_the_first_miss():
    """Wheels 1.5 % long: 21 m past the place of 2400, a creep of 10 m (31 m past it, off the
    line of the first miss), then 24 m past 2600: on the line of the first miss, not of the
    creep. The creep is the same standstill and keeps the first miss: relock at 2600."""
    tr = _tracker([(0, 2400.0), (0, 2600.0)])
    assert [tr.on_stop(d) for d in (1421.0, 1431.0, 1624.0)] == [False, False, True]
    assert tr._last_snap == (0, 2600.0, 1624.0)


def test_a_creep_past_the_scale_bound_keeps_the_first_miss():
    """Wheels 2.9 % long: 29 m past the place of 2000 (2.8 % of the path), a creep of 5 m in
    the same standstill to 34 m past it (3.3 %: no wheel scale explains it) and 34.8 m past
    2200 on the line of the first miss. The creep is not a signal: the first miss stays."""
    tr = _tracker([(0, 2000.0), (0, 2200.0)])
    assert [tr.on_stop(d) for d in (1029.0, 1034.0, 1234.8)] == [False, False, True]


@pytest.mark.parametrize('creep, relocked', [(19.9, False), (20.0, True)])
def test_the_same_standstill_is_closer_than_the_snap_gate(creep, relocked):
    """The bound is stop_snap_max_m of path. With loose places (stop_std_m 10 m, 2 sigma about
    28 m) a creep of 20 m from 45 m to 25 m short of the place fits the line of the first miss:
    at 20 m it is another stop and relocks, closer it is the same standstill."""
    tr = _tracker([(0, 3000.0)], _position(stop_std_m=10.0))
    assert [tr.on_stop(1955.0), tr.on_stop(1955.0 + creep)] == [False, relocked]


def test_a_true_scale_error_still_relocks_past_a_queue():
    """Wheels 1.5 % long and a queue with a creep of 0.5 m before the first missed place: the
    creep changes nothing, the next place on the line relocks and the rest snap."""
    places = [2400.0, 2600.0, 2800.0, 3000.0]
    tr = _tracker([(0, s) for s in places])
    origin = _anchor_xy(tr)
    stops = [1421.0, 1421.5] + [(s - START_S) * WHEEL for s in places[1:]]
    assert [tr.on_stop(d) for d in stops] == [False, False, True, True, True]
    assert abs(_arc(tr, 2000.0 * WHEEL, origin) - 2000.0) < 3.0


@pytest.mark.parametrize('wheel', [0.975, 0.985, 1.015, 1.025])
@pytest.mark.parametrize('creep', [0.3, 0.5, 2.0, 5.0])
def test_a_true_scale_error_is_recovered_whatever_the_creep_at_the_first_miss(wheel, creep):
    """Wheels 1.5-2.5 % off another tram's (trap 17), no place for 1.4 km: the tram stands at
    the place of 2400 (a miss of 21-35 m), creeps `creep` m and stands again, then at 2600,
    2800, 3000. The branch gets the lock back, and the creep, one standstill with the miss, does
    not stop it. main does not (30-50 m off at 3000), unless the creep brings short wheels back
    into the snap gate at the second standstill."""
    places = [2400.0, 2600.0, 2800.0, 3000.0]
    true = [places[0] - START_S, places[0] - START_S + creep] + [s - START_S for s in places[1:]]
    errors = []
    for cls in (PathTracker, _Main):
        tr = _tracker([(0, s) for s in places], cls=cls)
        origin = _anchor_xy(tr)
        for d in true:
            tr.on_stop(d * wheel)
        errors.append(abs(_arc(tr, true[-1] * wheel, origin) - true[-1]))
    assert errors[0] < 3.0
    if abs(true[0] - wheel * true[1]) > P.stop_snap_max_m:
        assert errors[1] > 29.0


@pytest.mark.parametrize('ambiguous_at', [1627.04, 1640.0])
def test_an_ambiguous_place_is_skipped_and_keeps_the_first_miss(ambiguous_at):
    """D-047: a miss at two places 3 m apart (on the line of the first miss or off it) neither
    relocks nor replaces the first miss; the next miss on its line relocks."""
    tr = _tracker([(0, 2400.0), (0, 2600.0), (0, 2603.0), (0, 2800.0)])
    assert [tr.on_stop(d) for d in (1421.0, ambiguous_at, 1827.0)] == [False, False, True]
    assert tr._last_snap == (0, 2800.0, 1827.0)


def _depot(stops):
    """The depot route of test_position with stop places; a miss is anything past 1 m, so a
    scale error of 1-3 % gives one on these short branches."""
    tr = PathTracker(_position(stop_snap_max_m=1.0),
                     Route(origin=ORIGIN, branches=_depot_route().branches, stops=tuple(stops)))
    tr.on_fix(*_lla(0.0, 0.0), 2, distance=0.0)
    return tr


def test_a_side_switch_drops_the_pending_miss():
    """A miss 1.2 m short at 45 m, then the switch onto the side track at 70 m, then a miss 2 m
    short 100 m down the side track: the first miss belongs to the old anchor, no relock."""
    tr = _depot([(0, 46.2), (3, 132.0)])
    assert not tr.on_stop(45.0)
    tr.advance(70.0, 7.0)                                      # onto the side track
    assert tr._anchor[0] == 3
    assert not tr.on_stop(170.0)


def test_undoing_a_side_switch_drops_the_pending_miss():
    """A miss on the side track, then the switch is undone (the tram ran past its dead end):
    the miss belongs to the undone anchor. A miss on the line of it from the restored anchor
    (branch 2 behind the loop) must not relock."""
    tr = _depot([(3, 132.0), (2, 12.94)])
    tr.advance(70.0, 7.0)                                      # onto the side track
    assert not tr.on_stop(170.0)                               # 2 m short of 132
    side_len = tr._s[3][-1]
    tr.advance(40.0 + side_len + 31.0, 3.0)                    # past its dead end: undone
    assert tr._anchor[0] == 0
    assert not tr.on_stop(250.0)                               # 2.94 m short of 12.94 on branch 2


def test_a_new_anchor_drops_the_pending_miss():
    tr = _tracker([(0, 2400.0), (0, 3800.0)])
    assert not tr.on_stop(1421.0)                              # 21 m short of 2400
    tr.on_fix(*_lla(-2400.0, 0.0, 170.0), 2, distance=1421.0)  # a fix of the window re-anchors
    assert not tr.on_stop(2842.0)                              # 21 m short of 3800 from there
