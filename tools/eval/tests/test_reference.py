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
