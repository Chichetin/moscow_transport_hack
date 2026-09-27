"""GNSS reference: ENU, status and outlier filter, origin in the window, arc without jitter."""
import numpy as np
import pytest

from tram_eval.reference import (A_WGS84, E2_WGS84, arc_polyline, build_reference,
                                 geodetic_to_enu)

LAT0, LON0, ALT0 = 55.75, 37.60, 150.0
M_PER_DEG_N = np.radians(1) * A_WGS84 * (1 - E2_WGS84) / (1 - E2_WGS84 * np.sin(np.radians(LAT0)) ** 2) ** 1.5
M_PER_DEG_E = np.radians(1) * A_WGS84 * np.cos(np.radians(LAT0)) / np.sqrt(1 - E2_WGS84 * np.sin(np.radians(LAT0)) ** 2)


def fixes_east(n, step_m=1.0, t0=0.0, status=0):
    """n fixes 10 Hz moving east by step_m per fix."""
    t = t0 + 0.1 * np.arange(n)
    lon = LON0 + np.arange(n) * step_m / M_PER_DEG_E
    llas = np.column_stack([np.full(n, LAT0), lon, np.full(n, ALT0), np.full(n, status)])
    return t, llas


def test_enu_axes_and_scale():
    enu = geodetic_to_enu([LAT0, LAT0 + 1e-4, LAT0], [LON0, LON0, LON0 + 1e-4], [ALT0, ALT0, ALT0 + 2.0],
                          (LAT0, LON0, ALT0))
    assert enu[0] == pytest.approx([0, 0, 0], abs=1e-6)
    assert enu[1] == pytest.approx([0, 1e-4 * M_PER_DEG_N, 0], abs=2e-3)    # north
    assert enu[2] == pytest.approx([1e-4 * M_PER_DEG_E, 0, 2.0], abs=2e-3)  # east, up


def test_reference_track_speed_and_origin():
    t, llas = fixes_east(100)
    ref = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), window_end=5.0)
    assert ref.origin == pytest.approx((LAT0, LON0, ALT0))
    assert ref.pos[-1] == pytest.approx([99.0, 0, 0], abs=0.01)
    assert ref.pos_s[-1] == pytest.approx(99.0, abs=0.01)
    assert ref.speed == pytest.approx(np.full(100, 10.0))


def test_outlier_km_jump_and_no_fix_are_dropped_first_point_too():
    t, llas = fixes_east(100)
    llas[0, 0] += 0.03          # first fix 3 km north: must not become the origin
    llas[50, 1] += 0.05         # a km jump in the middle
    llas[70, 3] = -1            # NO_FIX
    ref = build_reference(t, llas, [], np.zeros((0, 2)), window_end=5.0)
    assert len(ref.pos_t) == 97
    assert ref.origin == pytest.approx((LAT0, llas[1, 1], ALT0))
    assert np.abs(ref.pos[:, 1]).max() < 0.01
    assert ref.pos_s[-1] == pytest.approx(98.0, abs=0.01)


def test_origin_is_first_fix_inside_the_window():
    t, llas = fixes_east(100, t0=10.0)
    ref = build_reference(t, llas, [], np.zeros((0, 2)), window_end=10.35)
    assert ref.pos[0] == pytest.approx([0, 0, 0], abs=1e-6)


def test_origin_is_first_window_fix_when_status_2_comes_only_later():
    # #70: window has only status 0, status 2 starts 30 s (300 m) later. The tracker's frame `map`
    # starts at the first valid fix of the window (D-030); the reference must start there too,
    # while its track stays status 2 only
    t, llas = fixes_east(600)
    llas[300:, 3] = 2
    ref = build_reference(t, llas, [], np.zeros((0, 2)), window_end=5.0)
    assert ref.origin == pytest.approx((LAT0, LON0, ALT0))
    assert len(ref.pos_t) == 300 and ref.pos_t[0] == pytest.approx(30.0)
    assert ref.pos[0] == pytest.approx([300.0, 0, 0], abs=0.05)     # not 0: same frame as the tracker (1 cm: parallel vs ENU plane)


def test_status_2_in_the_window_still_wins_the_origin():
    t, llas = fixes_east(600)
    llas[20:, 3] = 2                                  # status 2 from 2 s, inside the 5 s window
    ref = build_reference(t, llas, [], np.zeros((0, 2)), window_end=5.0)
    assert ref.origin == pytest.approx((LAT0, llas[20, 1], ALT0))
    assert ref.pos[0] == pytest.approx([0, 0, 0], abs=1e-6)


def test_km_outlier_in_the_window_does_not_become_the_origin_before_late_status_2():
    t, llas = fixes_east(600)
    llas[300:, 3] = 2
    llas[0, 0] += 0.03                                # first fix 3 km north (trap 10)
    ref = build_reference(t, llas, [], np.zeros((0, 2)), window_end=5.0)
    assert ref.origin == pytest.approx((LAT0, llas[1, 1], ALT0))


def test_no_gnss_gives_empty_reference():
    ref = build_reference([], np.zeros((0, 4)), [], np.zeros((0, 2)), window_end=5.0)
    assert ref.origin is None and len(ref.pos) == 0 and len(ref.vel_t) == 0


def test_standing_jitter_adds_no_path():
    rng = np.random.default_rng(0)
    xy = rng.normal(0.0, 0.05, size=(600, 2))           # one minute standing, 5 cm noise
    _, poly_s, point_s = arc_polyline(xy)
    assert poly_s[-1] < 1.0
    assert point_s.max() < 1.5


def test_glitchy_gnss_speed_is_dropped_and_stamps_sorted():
    ref = build_reference([], np.zeros((0, 4)), [0.2, 0.1, 0.1, 0.3],
                          np.array([[1.0, 0], [2.0, 0], [2.5, 0], [80.0, 0]]), window_end=5.0)
    assert list(ref.vel_t) == [0.1, 0.2]
    assert list(ref.speed) == [2.0, 1.0]


def test_gbas_fixes_win_over_offset_status_0():
    t, llas = fixes_east(100, status=2)
    llas[::3, 3] = 0
    llas[::3, 0] += 5.0 / M_PER_DEG_N          # status 0 solution sits 5 m north
    ref = build_reference(t, llas, [], np.zeros((0, 2)), window_end=5.0)
    assert len(ref.pos_t) == 66
    assert np.abs(ref.pos[:, 1]).max() < 0.01


def test_status_0_only_bag_keeps_its_fixes():
    t, llas = fixes_east(50, status=0)
    assert len(build_reference(t, llas, [], np.zeros((0, 2)), window_end=5.0).pos_t) == 50


def test_arc_follows_doppler_speed_not_fix_wander():
    rng = np.random.default_rng(1)
    t, llas = fixes_east(300, step_m=0.0)       # 30 s standing...
    llas[:, 0] += rng.normal(0, 0.7, 300) / M_PER_DEG_N   # ...while the fixes wander 0.7 m
    ref = build_reference(t, llas, t, np.zeros((300, 2)), window_end=5.0)
    assert ref.pos_s.max() == pytest.approx(0.0)
    t, llas = fixes_east(100, step_m=1.0)
    ref = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), window_end=5.0)
    assert ref.pos_s[-1] == pytest.approx(99.0, abs=0.01)


# base_link by the organizers' tf (D-077): master 9.873 m behind, rover 2.563 m ahead, both 3.0 m up

def _rover_of(t, llas, dt=0.0, base_m=12.436):
    """Rover fixes base_m east of every master fix (the tram runs east), stamps shifted by dt."""
    rover = llas.copy()
    rover[:, 1] += base_m / M_PER_DEG_E
    return t + dt, rover


def test_base_link_is_on_the_line_master_rover_and_at_rail_level():
    t, llas = fixes_east(100)
    rt, rl = _rover_of(t, llas, dt=0.02)
    ref = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), window_end=5.0,
                          point='base_link', rover_t=rt, rover_llas=rl)
    master = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), window_end=5.0)
    assert ref.origin == master.origin                        # frame origin: the master fix
    assert ref.pos - master.pos == pytest.approx(np.tile([9.873, 0.0, -3.0], (100, 1)), abs=0.01)
    assert ref.poly == pytest.approx(ref.pos[:, :2])
    assert ref.pos_s == pytest.approx(master.pos_s)


def test_base_link_without_rover_is_ahead_on_the_track_and_extended_past_its_end():
    t, llas = fixes_east(100)
    ref = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), window_end=5.0,
                          point='base_link')
    assert ref.pos[0] == pytest.approx([9.873, 0.0, -3.0], abs=0.01)
    assert ref.pos[-1] == pytest.approx([99.0 + 9.873, 0.0, -3.0], abs=0.01)   # past the end


def test_rover_pair_of_the_wrong_base_or_moment_is_not_used():
    t, llas = fixes_east(100)
    for rt, rl in (_rover_of(t, llas, base_m=30.0), _rover_of(t, llas, dt=0.5)):
        rl[:, 0] += 5.0 / M_PER_DEG_N                         # 5 m north: would show if used
        ref = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), window_end=5.0,
                              point='base_link', rover_t=rt, rover_llas=rl)
        assert np.abs(ref.pos[:, 1]).max() < 0.01


def test_standing_track_takes_the_rover_heading():
    t, llas = fixes_east(50, step_m=0.0)                     # stands the whole bag
    rt, rl = _rover_of(t, llas)
    rl[1:, 1] = np.nan                                        # one rover fix only: pair of fix 0
    ref = build_reference(t, llas, t, np.zeros((50, 2)), window_end=5.0,
                          point='base_link', rover_t=rt, rover_llas=rl)
    assert ref.pos == pytest.approx(np.tile([9.873, 0.0, -3.0], (50, 1)), abs=0.01)


def test_unknown_reference_point_is_rejected():
    t, llas = fixes_east(10)
    with pytest.raises(ValueError):
        build_reference(t, llas, [], np.zeros((0, 2)), window_end=5.0, point='rover')


def test_fix_without_a_rover_pair_holds_the_heading_of_the_nearest_pair_at_a_stop():
    t, llas = fixes_east(100)
    llas[80:] = llas[79]                                       # stands for the last 2 s
    rt, rl = _rover_of(t, llas)
    rl[:, 0] += 3.0 / M_PER_DEG_N                              # rover 3 m north: turned heading
    rl[:, 1] -= (12.436 - np.sqrt(12.436 ** 2 - 9.0)) / M_PER_DEG_E   # base stays 12.436 m
    rl[-1, :] = np.nan                                         # the last master fix has no pair
    ref = build_reference(t, llas, t, np.r_[np.tile([10.0, 0.0], (80, 1)), np.zeros((20, 2))],
                          window_end=5.0, point='base_link', rover_t=rt, rover_llas=rl)
    assert ref.pos[-1] == pytest.approx(ref.pos[-2], abs=0.01)   # no jump onto the track ahead


def test_plain_rover_fix_is_not_paired_with_a_gbas_master_track():
    t, llas = fixes_east(100, status=2)
    rt, rl = _rover_of(t, llas)
    rl[:, 3] = 2
    rl[50, 0] += 0.4 / M_PER_DEG_N                             # plain fix 0.4 m off: base in tolerance
    rl[50, 3] = 0
    ref = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), window_end=5.0,
                          point='base_link', rover_t=rt, rover_llas=rl)
    assert np.abs(ref.pos[:, 1]).max() < 0.01                  # the plain fix did not turn base_link


# the grid of /result/position (D-082): the reference goes through the core's conversion

GRID = (37, 300000.0, 6100000.0)


def test_grid_reference_is_the_fix_itself_in_the_grid():
    from tram_eval.reference import core_geo
    geo = core_geo()
    t, llas = fixes_east(100)
    ref = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), window_end=5.0, grid=GRID)
    for i in (0, 50, 99):
        assert ref.pos[i] == pytest.approx(geo.to_grid(*llas[i, :3], GRID), abs=1e-3)
    assert ref.poly == pytest.approx(ref.pos[:, :2])
    assert ref.pos_s[-1] == pytest.approx(99.0, abs=0.01)     # arc: the Doppler one, unchanged


def test_grid_reference_keeps_the_geometry_of_the_enu_one():
    """Only the frame changes: the same base_link track, distances within the UTM scale."""
    t, llas = fixes_east(100)
    rt, rl = _rover_of(t, llas, dt=0.02)
    kw = dict(window_end=5.0, point='base_link', rover_t=rt, rover_llas=rl)
    enu = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), **kw)
    grid = build_reference(t, llas, t, np.tile([10.0, 0.0], (100, 1)), grid=GRID, **kw)
    d_enu = np.hypot(*np.diff(enu.pos[:, :2], axis=0).T)
    d_grid = np.hypot(*np.diff(grid.pos[:, :2], axis=0).T)
    assert d_grid == pytest.approx(d_enu * 0.99969, rel=5e-5)
    assert grid.pos[:, 2] == pytest.approx(ALT0 - 3.0, abs=0.01)   # ellipsoidal height of base_link
    assert grid.pos_s == pytest.approx(enu.pos_s)


def test_no_fixes_in_the_grid_is_empty():
    ref = build_reference([], np.zeros((0, 4)), [], np.zeros((0, 2)), window_end=5.0, grid=GRID)
    assert len(ref.pos) == 0 and ref.origin is None
