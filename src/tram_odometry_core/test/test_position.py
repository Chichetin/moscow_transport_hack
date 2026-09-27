"""PathTracker: GNSS alignment in the init window, motion along the route map (D-007, D-024)."""
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from tram_odometry_core.position import PathTracker, geo
from tram_odometry_core.types import Branch, Route, load_params, load_route

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools' / 'pathgraph'))
import build_route as br  # noqa: E402  (reference ENU of the map builder and of tools/eval)

PARAMS_YAML = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
# the tests below follow the track of the master antenna itself: base_link offset zeroed; the
# offset of params.yaml (D-077) has its own tests at the end of the file
PARAMS = replace(PARAMS_YAML, position=replace(PARAMS_YAML.position, base_ahead_m=0.0,
                                               antenna_height_m=0.0))
ORIGIN = (55.8104, 37.4623, 168.0)
M_LAT = 111338.0 / 0.001 / 1000.0           # m per degree of latitude at 55.81 (approx.)
M_LON = 626.98 / 0.01                       # m per degree of longitude at 55.81 (approx.)


def _lla(e, n, alt=ORIGIN[2]):
    """Approximate inverse ENU (tests only): points are then mapped with the exact lla_to_enu."""
    return ORIGIN[0] + n / M_LAT, ORIGIN[1] + e / M_LON, alt


def _branch(lats, lons, alts):
    e, n, u = br.lla_to_enu(np.asarray(lats), np.asarray(lons), np.asarray(alts), *ORIGIN)
    s, xy = br.resample(np.column_stack([e, n]), 1.0)
    seg = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(e), np.diff(n)))])
    return Branch(s=s, x=xy[:, 0], y=xy[:, 1], z=np.interp(s, seg, u))


def _route():
    """Branch 0: 5 km west with a 10 m climb; branch 1: from its end 300 m south."""
    lat0, lon0, _ = _lla(0.0, 0.0)
    lat1, lon1, _ = _lla(-5000.0, 0.0)
    lat2, lon2, _ = _lla(-5000.0, -300.0)
    b0 = _branch(np.linspace(lat0, lat1, 500), np.linspace(lon0, lon1, 500),
                 np.linspace(168.0, 178.0, 500))
    b1 = _branch(np.linspace(lat1, lat2, 30), np.linspace(lon1, lon2, 30), np.full(30, 178.0))
    return Route(origin=ORIGIN, branches=(b0, b1))


def _grid(lat, lon, alt):
    """The published frame of the pipeline: MGRS grid of params.yaml (D-083)."""
    f = PARAMS.frames
    return geo.to_grid(lat, lon, alt, (f.grid_zone, f.grid_origin_e_m, f.grid_origin_n_m))


def _enu_bag(lat, lon, alt, origin):
    e, n, u = br.lla_to_enu(np.array([lat]), np.array([lon]), np.array([alt]), *origin)
    return float(e[0]), float(n[0]), float(u[0])


def test_load_route_reads_origin_and_branches(tmp_path):
    p = tmp_path / 'route.csv'
    p.write_text('# frame: ENU, origin_lat=55.8104, origin_lon=37.4623, origin_alt=168.0, '
                 'source=test\nbranch,s_m,x_m,y_m,z_m\n0,0.000,0.0,0.0,0.5\n0,1.000,1.0,0.0,0.6\n'
                 '1,0.000,5.0,5.0,1.0\n1,1.000,5.0,6.0,1.0\n')
    r = load_route(p)
    assert r.origin == (55.8104, 37.4623, 168.0)
    assert len(r.branches) == 2
    assert list(r.branches[0].z) == [0.5, 0.6] and list(r.branches[1].y) == [5.0, 6.0]


def test_load_route_of_the_repository_map():
    r = load_route(ROOT / 'src' / 'tram_odometry' / 'maps' / 'route.csv')
    assert len(r.branches) >= 2 and all(np.all(np.diff(b.s) > 0) for b in r.branches)


def test_not_ready_before_a_fix():
    tr = PathTracker(PARAMS, _route())
    assert not tr.ready and tr.advance(10.0) is None


def test_takes_loop_branch_that_escapes_a_dead_end_at_a_fork():
    def sampled(points):
        points = np.asarray(points, dtype=float)
        coords = [points[0]]
        for a, b in zip(points, points[1:]):
            n = int(round(np.linalg.norm(b - a)))
            coords.extend(np.linspace(a, b, n + 1)[1:])
        coords = np.asarray(coords)
        return Branch(s=np.arange(len(coords), dtype=float), x=coords[:, 0],
                      y=coords[:, 1], z=np.zeros(len(coords)))

    route = Route(
        origin=ORIGIN,
        branches=(
            sampled([(0, 0), (100, 0)]),  # continues straight into a dead end
            sampled([(50, 0), (50, -30), (90, -30), (90, 0), (70, 0), (70, 20)]),
            sampled([(70, 20), (70, 70)]),  # the loop has a forward continuation
        ),
    )
    tr = PathTracker(PARAMS, route)
    tr.on_fix(*_lla(0.0, 0.0), 2, distance=0.0)

    x, y, _, _, _ = tr.advance(60.0)  # 50 m to the fork, then 10 m onto the loop

    assert (x, y) == pytest.approx((50.0, -10.0), abs=0.1)


def test_moves_along_branch_in_the_bag_frame():
    route = _route()
    tr = PathTracker(PARAMS, route)
    start = _lla(-1000.0, 0.0, 170.0)           # 1 km along branch 0 at map height: origin
    tr.on_fix(*start, 2, distance=50.0)
    x, y, z, yaw, cov = tr.advance(50.0)
    assert math.hypot(x, y) < 0.05 and abs(z) < 0.05
    far = _lla(-4000.0, 0.0, 168.0 + 10.0 * 4000.0 / 5000.0)   # 3 km further on the branch
    x, y, z, yaw, cov = tr.advance(50.0 + 3000.0)
    fx, fy, fz = _enu_bag(*far, origin=start)
    assert math.hypot(x - fx, y - fy) < 0.5          # tangent planes 3 km apart: rotation handled
    assert abs(z - fz) < 0.3
    assert abs(math.remainder(yaw - math.pi, 2 * math.pi)) < 0.01   # heading west
    assert cov[0] > cov[1] > 0.0                     # along-track (x) uncertainty dominates


def test_continues_onto_the_next_branch_after_the_end():
    tr = PathTracker(PARAMS, _route())
    tr.on_fix(*_lla(-4990.0, 0.0), 2, distance=0.0)
    x, y, z, yaw, cov = tr.advance(110.0)            # 10 m to the end, then 100 m south
    fx, fy, _ = _enu_bag(*_lla(-5000.0, -100.0), origin=_lla(-4990.0, 0.0))
    assert math.hypot(x - fx, y - fy) < 1.0
    assert abs(math.remainder(yaw + math.pi / 2, 2 * math.pi)) < 0.05


def test_stops_at_a_dead_end():
    tr = PathTracker(PARAMS, _route())
    tr.on_fix(*_lla(-5000.0, -250.0), 2, distance=0.0)
    end = tr.advance(60.0)                       # 50 m to the end of branch 1
    assert tr.advance(500.0)[:2] == pytest.approx(end[:2])


def test_takes_loop_at_interior_fork_instead_of_terminal_spur():
    """A directed loop leaves a route branch before its represented spur ends."""
    from dataclasses import replace

    position = replace(PARAMS.position, join_m=0.5)
    params = replace(PARAMS, position=position)
    branch0 = Branch(s=np.arange(11.0), x=np.arange(11.0), y=np.zeros(11), z=np.zeros(11))
    loop_points = [(5.0, 0.0)]
    loop_points.extend((5.0, float(y)) for y in range(1, 6))
    loop_points.extend((float(x), 5.0) for x in range(4, -6, -1))
    loop_points.extend((-5.0, float(y)) for y in range(4, -1, -1))
    loop_points.extend((float(x), 0.0) for x in range(-4, 1))
    loop = np.asarray(loop_points)
    branch1 = Branch(s=np.arange(len(loop), dtype=float), x=loop[:, 0],
                     y=loop[:, 1], z=np.zeros(len(loop)))
    route = Route(origin=ORIGIN, branches=(branch0, branch1))
    tr = PathTracker(params, route)
    tr.on_fix(*_lla(0.0, 0.0), 2, distance=0.0)

    x, y, _, _, _ = tr.advance(8.0)              # fork at 5 m, then 3 m along the loop
    assert (x, y) == pytest.approx((5.0, 3.0), abs=0.1)


def test_height_follows_the_map_with_the_run_offset():
    tr = PathTracker(PARAMS, _route())
    start = _lla(-1000.0, 0.0, 170.0 + 1.5)                    # GNSS of the run 1.5 m above map
    tr.on_fix(*start, 2, distance=0.0)
    _, _, z0, _, _ = tr.advance(0.0)
    _, _, z1, _, _ = tr.advance(2000.0)
    assert z0 == pytest.approx(0.0, abs=0.05)                  # origin fix is the frame origin
    # same 1.5 m offset 2 km on (+4 m climb); ENU up of the run also drops by d^2/2R ~ 0.3 m
    _, _, fz = _enu_bag(*_lla(-3000.0, 0.0, 174.0 + 1.5), origin=start)
    assert z1 == pytest.approx(fz, abs=0.05)


def test_origin_moves_to_the_first_gbas_fix():
    tr = PathTracker(PARAMS, _route())
    tr.on_fix(*_lla(-1000.0, 3.0), 0, distance=0.0)            # plain fix, 3 m off the track
    tr.on_fix(*_lla(-1010.0, 0.0), 2, distance=10.0)           # first status 2: frame origin
    tr.on_fix(*_lla(-1020.0, 3.0), 0, distance=20.0)           # plain fixes ignored from now on
    x, y, _, _, _ = tr.advance(20.0)
    fx, fy, _ = _enu_bag(*_lla(-1020.0, 0.0), origin=_lla(-1010.0, 0.0))
    assert math.hypot(x - fx, y - fy) < 0.2


def test_bad_fixes_are_ignored():
    tr = PathTracker(PARAMS, _route())
    tr.on_fix(float('nan'), ORIGIN[1], ORIGIN[2], 2, distance=0.0)
    tr.on_fix(*_lla(-1000.0, 0.0), -1, distance=0.0)            # STATUS_NO_FIX
    assert not tr.ready


def _hdr(t):
    from types import SimpleNamespace
    sec = math.floor(t)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=round((t - sec) * 1e9)))


def _fix_msg(t, lla, status=2):
    from types import SimpleNamespace
    return '/sensing/gnss/master/fix', SimpleNamespace(
        header=_hdr(t), latitude=lla[0], longitude=lla[1], altitude=lla[2],
        status=SimpleNamespace(status=status))


def _wheels(odo, t0, t1, kmh, dt=0.1):
    from types import SimpleNamespace
    est, n = None, 0
    while t0 + n * dt <= t1 + 1e-9:
        for topic in ('/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity'):
            est = odo.step((topic, SimpleNamespace(header=_hdr(t0 + n * dt), velocity=kmh))) or est
        n += 1
    return est


def test_pipeline_follows_the_map_after_the_window():
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    start = _lla(-1000.0, 0.0, 170.0)
    odo.step(_fix_msg(0.0, start))
    est = _wheels(odo, 0.0, 100.0, 36.0)              # 10 m/s for 100 s: 1 km west
    fx, fy, _ = _grid(*_lla(-2000.0, 0.0, 172.0))
    assert est.gnss_used and math.hypot(est.x - fx, est.y - fy) < 2.0
    wx, wy, _ = _grid(*_lla(-2001.0, 0.0, 172.0))    # due west in the grid: turned by convergence
    assert abs(math.remainder(est.yaw - math.atan2(wy - fy, wx - fx), 2 * math.pi)) < 0.01


def test_pipeline_ignores_gnss_after_the_window():
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    start = _lla(-1000.0, 0.0, 170.0)
    odo.step(_fix_msg(0.0, start))
    _wheels(odo, 0.0, 10.0, 36.0)
    odo.step(_fix_msg(PARAMS.gnss.init_window_s + 5.0, _lla(-1500.0, 0.0, 171.0)))  # after window
    est = _wheels(odo, 10.1, 20.0, 36.0)
    fx, fy, _ = _grid(*_lla(-1200.0, 0.0, 170.4))
    assert math.hypot(est.x - fx, est.y - fy) < 2.0


def test_pipeline_before_the_first_fix_has_no_anchor_even_with_a_map():
    """No fix yet: where the tram is is unknown, the map's origin is kilometres from a start at
    the other terminal (#163). The line stays in local metres and is marked not absolute: the
    node holds it back inside the GNSS window, eval does not score it (D-086)."""
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    est = _wheels(odo, 0.0, 1.0, 36.0)
    assert est.position_absolute is False and not odo.position_due(est)
    assert (est.x, est.y) == pytest.approx((est.distance, 0.0))


def test_pipeline_without_gnss_publishes_local_position_after_the_window():
    """A bag without GNSS (jury_layouts runs one): no fix ever, so after the GNSS window the
    local line is due in odom -- /result/position must not stay silent (#163, D-086)."""
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    inside = _wheels(odo, 0.0, PARAMS.gnss.init_window_s - 0.5, 36.0)
    assert not inside.position_absolute and not odo.position_due(inside)
    after = _wheels(odo, PARAMS.gnss.init_window_s - 0.4, PARAMS.gnss.init_window_s + 1.0, 36.0)
    assert not after.position_absolute and odo.position_due(after)
    from types import SimpleNamespace
    late = odo.step(('/vehicle/driver_position_cmd', SimpleNamespace(
        header=_hdr(PARAMS.gnss.init_window_s - 1.0), position=0)))   # stamp rolled back into the window
    assert late is None or odo.position_due(late)


def test_pipeline_with_the_fixes_off_the_map_starts_the_line_at_the_first_fix():
    """#163: every window fix is farther than fix_gate_m from the map (a bag off the route), so
    the tracker never aligns. The straight line (D-021) starts at the first valid master fix,
    published in the grid, not at the map's origin kilometres away."""
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    off = _lla(-1000.0, 500.0, 170.0)                    # 500 m north of branch 0
    odo.step(_fix_msg(0.0, off))
    est = _wheels(odo, 0.0, 0.0, 0.0)
    fx, fy, fz = _grid(*off)
    assert not odo._tracker.ready and est.position_absolute is True
    assert math.hypot(est.x - fx, est.y - fy) < 0.1 and est.z == pytest.approx(fz, abs=0.01)


def test_pipeline_with_map_wheels_before_the_fix_then_the_map():
    """Wheels before any fix: no absolute position; the first fix the map accepts puts every
    later estimate on the map at the tram, never at the map's origin (#163, e2dcf65f)."""
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    early = _wheels(odo, 0.0, 0.3, 0.0)
    start = _lla(-1000.0, 0.0, 170.0)
    odo.step(_fix_msg(0.4, start))
    est = _wheels(odo, 0.5, 0.5, 0.0)
    fx, fy, _ = _grid(*start)
    assert early.position_absolute is False
    assert odo._tracker.ready and est.position_absolute
    assert math.hypot(est.x - fx, est.y - fy) < 12.0          # base_link ahead of master (D-077)


def test_pipeline_outlier_first_fix_then_the_map_takes_over():
    """The first valid fix is an outlier off the map: the line starts there; the next fix the
    map accepts switches to the map for good (#163)."""
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    odo.step(_fix_msg(0.0, _lla(-1000.0, 3000.0, 170.0)))    # 3 km off the route
    line = _wheels(odo, 0.1, 0.1, 0.0)
    ox, oy, _ = _grid(*_lla(-1000.0, 3000.0, 170.0))
    assert line.position_absolute and math.hypot(line.x - ox, line.y - oy) < 0.1
    start = _lla(-1000.0, 0.0, 170.0)
    odo.step(_fix_msg(0.5, start))
    est = _wheels(odo, 0.6, 3.0, 0.0)
    fx, fy, _ = _grid(*start)
    assert odo._tracker.ready and math.hypot(est.x - fx, est.y - fy) < 12.0


def test_pipeline_with_map_ignores_a_zero_lat_lon_fix_as_line_origin():
    """Trap 10 with the map: lat = lon = 0 never becomes the line's frame (#163)."""
    from types import SimpleNamespace
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    odo.step(('/sensing/gnss/master/fix', SimpleNamespace(
        header=_hdr(0.0), latitude=0.0, longitude=0.0, altitude=0.0,
        status=SimpleNamespace(status=0))))
    est = _wheels(odo, 0.1, 0.1, 0.0)
    assert est.position_absolute is False and odo._line_frame is None


def test_pipeline_without_route_keeps_the_baseline():
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS)
    est = _wheels(odo, 0.0, 10.0, 36.0)
    assert est.z == 0.0 and est.y == pytest.approx(0.0)


def _aligned_tracker():
    """A tracker aligned at 1 km along branch 0 with a good GBAS fix."""
    tr = PathTracker(PARAMS, _route())
    tr.on_fix(*_lla(-1000.0, 0.0, 170.0), 2, distance=50.0)
    return tr


def test_outlier_fix_first_in_the_window_is_not_the_origin_of_the_frame():
    """docs/data.md, trap 10: kilometres off at the start of a run, even with status 2."""
    tr = PathTracker(PARAMS, _route())
    tr.on_fix(*_lla(-1000.0, 2000.0, 170.0), 2, distance=50.0)   # 2 km off the map
    assert not tr.ready
    tr.on_fix(*_lla(-1000.0, 0.0, 170.0), 2, distance=50.0)
    x, y, z, yaw, cov = tr.advance(50.0)
    assert math.hypot(x, y) < 0.05 and abs(z) < 0.05


def test_outlier_fix_last_in_the_window_does_not_move_the_anchor():
    tr = _aligned_tracker()
    tr.on_fix(*_lla(-1000.0, 2000.0, 170.0), 2, distance=50.0)
    x, y, z, yaw, cov = tr.advance(50.0)
    assert math.hypot(x, y) < 0.05 and abs(z) < 0.05


def test_zero_lat_lon_fix_is_ignored():
    tr = PathTracker(PARAMS, _route())
    tr.on_fix(0.0, 0.0, 0.0, 0, distance=0.0)
    assert not tr.ready
    tr = _aligned_tracker()
    tr.on_fix(0.0, 0.0, 0.0, 0, distance=50.0)
    x, y, z, yaw, cov = tr.advance(50.0)
    assert math.hypot(x, y) < 0.05


def test_fix_within_the_gate_is_accepted():
    tr = PathTracker(PARAMS, _route())
    tr.on_fix(*_lla(-1000.0, PARAMS.position.fix_gate_m - 1.0, 170.0), 2, distance=50.0)
    assert tr.ready


def test_load_route_rejects_a_bad_header_and_a_one_point_branch(tmp_path):
    p = tmp_path / 'route.csv'
    p.write_text('# frame: ENU, source=test\nbranch,s_m,x_m,y_m,z_m\n0,0.0,0.0,0.0,0.0\n')
    with pytest.raises(ValueError, match='origin'):
        load_route(p)
    p.write_text('# frame: ENU, origin_lat=55.8104, origin_lon=37.4623, origin_alt=168.0\n'
                 'branch,s_m,x_m,y_m,z_m\n0,0.0,0.0,0.0,0.0\n')
    with pytest.raises(ValueError, match='branch 0'):
        load_route(p)


def _two_way_route():
    """Two tracks 8 m apart (docs/data.md: the directions of the line): branch 0 goes west
    on the northern track, branch 1 goes east on the southern one."""
    west = _branch(*(np.linspace(a, b, 200) for a, b in zip(_lla(0.0, 0.0), _lla(-2000.0, 0.0))))
    east = _branch(*(np.linspace(a, b, 200) for a, b in zip(_lla(-2000.0, -8.0), _lla(0.0, -8.0))))
    return Route(origin=ORIGIN, branches=(west, east))


def test_rover_ahead_picks_the_track_of_the_heading_over_the_nearest_one():
    """Master 3 m from the westbound track and 5 m from the eastbound one; the rover stands
    12.4 m ahead of it to the east (trap 15): the tram is on the eastbound track."""
    tr = PathTracker(PARAMS, _two_way_route())
    master = _lla(-1000.0, -3.0)
    tr.on_rover(*_lla(-1000.0 + 12.4, -3.0), 2)
    tr.on_fix(*master, 2, distance=0.0)
    x, y, _, yaw, _ = tr.advance(100.0)
    fx, fy, _ = _enu_bag(*_lla(-900.0, -8.0), origin=master)
    assert math.hypot(x - fx, y - fy) < 1.0
    assert abs(math.remainder(yaw, 2 * math.pi)) < 0.05          # heading east


def test_rover_after_master_moves_the_anchor_to_the_track_of_the_heading():
    tr = PathTracker(PARAMS, _two_way_route())
    master = _lla(-1000.0, -3.0)
    tr.on_fix(*master, 2, distance=0.0)
    tr.on_rover(*_lla(-1000.0 + 12.4, -3.0), 2)
    x, y, _, _, _ = tr.advance(100.0)
    fx, fy, _ = _enu_bag(*_lla(-900.0, -8.0), origin=master)
    assert math.hypot(x - fx, y - fy) < 1.0


def test_late_rover_along_the_anchor_does_not_undo_a_stop_snap():
    route = _two_way_route()
    route = Route(origin=route.origin, branches=route.branches, stops=((1, 1003.0),))
    tr = PathTracker(PARAMS, route)
    master = _lla(-1000.0, -8.0)                     # on the eastbound track, s = 1000
    tr.on_rover(*_lla(-1000.0 + 12.4, -8.0), 2)
    tr.on_fix(*master, 2, distance=0.0)
    assert tr.on_stop(0.0)
    snapped = tr.advance(0.0)
    tr.on_rover(*_lla(-1000.0 + 12.4, -8.0), 2)     # stamped in the window, arrives after
    assert tr.advance(0.0)[:2] == pytest.approx(snapped[:2])


def test_without_a_usable_rover_the_nearest_track_is_kept():
    """No rover, a rover on the master (no base) and an outlier rover: the nearest track."""
    master = _lla(-1000.0, -3.0)
    fx, fy, _ = _enu_bag(*_lla(-1100.0, 0.0), origin=master)
    for rover in (None, master, _lla(-1000.0 + 12.4, 2000.0), _lla(-1000.0 + 30.0, -3.0)):
        tr = PathTracker(PARAMS, _two_way_route())
        if rover is not None:
            tr.on_rover(*rover, 2)
        tr.on_fix(*master, 2, distance=0.0)
        x, y, _, _, _ = tr.advance(100.0)
        assert math.hypot(x - fx, y - fy) < 1.0, rover


def test_plain_rover_fix_next_to_a_gbas_origin_gives_no_heading():
    tr = PathTracker(PARAMS, _two_way_route())
    master = _lla(-1000.0, -3.0)
    tr.on_rover(*_lla(-1000.0 + 12.4, -3.0), 0)
    tr.on_fix(*master, 2, distance=0.0)
    x, y, _, _, _ = tr.advance(100.0)
    fx, fy, _ = _enu_bag(*_lla(-1100.0, 0.0), origin=master)
    assert math.hypot(x - fx, y - fy) < 1.0              # the nearest track, as without rover


def test_rover_fix_alone_does_not_align():
    tr = PathTracker(PARAMS, _two_way_route())
    tr.on_rover(*_lla(-1000.0, -8.0), 2)
    assert not tr.ready


def _rover_msg(t, lla, status=2):
    topic, msg = _fix_msg(t, lla, status)
    return '/sensing/gnss/rover/fix', msg


def test_pipeline_takes_the_heading_from_the_rover_in_the_window_only():
    from tram_odometry_core.pipeline import Odometry
    master = _lla(-1000.0, -3.0)
    for rover_t, expect_east in ((0.05, True), (PARAMS.gnss.init_window_s + 1.0, False)):
        odo = Odometry(PARAMS, route=_two_way_route())
        odo.step(_fix_msg(0.0, master))
        odo.step(_rover_msg(rover_t, _lla(-1000.0 + 12.4, -3.0)))
        est = _wheels(odo, rover_t, rover_t + 10.0, 36.0)        # 10 m/s for 10 s: 100 m
        assert est.distance > 90.0
        ex, ey, _ = _grid(*_lla(-1000.0 + est.distance, -8.0))    # eastbound track
        wx, wy, _ = _grid(*_lla(-1000.0 - est.distance, 0.0))     # westbound track
        assert (math.hypot(est.x - ex, est.y - ey) < 3.0) == expect_east, rover_t
        assert (math.hypot(est.x - wx, est.y - wy) < 3.0) != expect_east, rover_t


def _polyline(points):
    """Branch through the points with a 1 m step (tests of the map graph in metres)."""
    points = np.asarray(points, dtype=float)
    coords = [points[0]]
    for a, b in zip(points, points[1:]):
        n = int(round(np.linalg.norm(b - a)))
        coords.extend(np.linspace(a, b, n + 1)[1:])
    coords = np.asarray(coords)
    return Branch(s=np.arange(len(coords), dtype=float), x=coords[:, 0], y=coords[:, 1],
                  z=np.zeros(len(coords)))


def _depot_route():
    """Branch 0 runs east into a dead end; the default fork at 120 m is a loop that continues
    (branch 2); branch 3 is a dead-end side track leaving at 40 m (the depot track, #138)."""
    return Route(origin=ORIGIN, branches=(
        _polyline([(0, 0), (200, 0)]),
        _polyline([(120, 0), (120, -30), (160, -30), (160, 20)]),
        _polyline([(160, 20), (160, 120)]),
        _polyline([(40, 0), (80, -30), (80, -130)]),
    ))


def _depot_tracker():
    tr = PathTracker(PARAMS, _depot_route())
    tr.on_fix(*_lla(0.0, 0.0), 2, distance=0.0)
    return tr


def test_side_track_is_found_and_the_default_fork_kept():
    tr = _depot_tracker()
    assert tr._fork[0] == (120.0, 1)
    assert tr._side[0] == (40.0, 3)


def test_slow_tram_stays_on_the_default_path_past_the_side_track():
    tr = _depot_tracker()
    assert tr.advance(70.0, 3.0)[:2] == pytest.approx((70.0, 0.0), abs=0.1)
    assert tr.advance(130.0, 3.0)[:2] == pytest.approx((120.0, -10.0), abs=0.1)   # the loop


def test_fast_tram_past_the_side_track_start_takes_it_and_stays_on_it():
    tr = _depot_tracker()
    x, y, _, _, _ = tr.advance(70.0, 7.0)        # 30 m past the side track start
    assert math.hypot(x - 40.0, y) == pytest.approx(30.0, abs=0.1)
    assert y < -10.0                             # off branch 0, on the side track
    far = tr.advance(195.0, 2.0)                 # slow again: still there, 5 m past its dead end
    assert far[:2] == pytest.approx((80.0, -130.0), abs=0.1)


def test_fast_tram_outside_the_decision_window_keeps_the_default_path():
    tr = _depot_tracker()
    assert tr.advance(50.0, 7.0)[:2] == pytest.approx((50.0, 0.0), abs=0.1)   # 10 m < side_min_m
    tr = _depot_tracker()
    assert tr.advance(100.0, 7.0)[:2] == pytest.approx((100.0, 0.0), abs=0.1)  # 60 m > side_max_m


def test_side_switch_keeps_the_along_track_variance():
    tr = _depot_tracker()
    before = tr._var_along(70.0)
    tr.advance(70.0, 7.0)
    assert tr._var_along(70.0) == pytest.approx(before)


# base_link offset of params.yaml (D-077): the map is the master antenna track, the output is
# the front bogie pivot at rail level, 9.873 m ahead of master and 3.0 m below the antennas


def test_base_link_offset_is_the_organizers_tf():
    assert PARAMS_YAML.position.base_ahead_m == pytest.approx(9.873)
    assert PARAMS_YAML.position.antenna_height_m == pytest.approx(3.0)


def test_output_is_base_link_ahead_of_master_and_below_the_antennas():
    master = PathTracker(PARAMS, _route())
    base = PathTracker(PARAMS_YAML, _route())
    start = _lla(-1000.0, 0.0, 170.0)
    for tr in (master, base):
        tr.on_fix(*start, 2, distance=50.0)
    for distance in (50.0, 1050.0):
        xm, ym, zm, yaw_m, cov_m = master.advance(distance)
        xb, yb, zb, yaw_b, cov_b = base.advance(distance)
        assert xb - xm == pytest.approx(-9.873, abs=0.01)     # branch 0 runs west: ahead = -x
        assert yb - ym == pytest.approx(0.0, abs=0.01)
        # 3.0 m below the antenna; the map climbs 10 m per 5 km, 9.873 m ahead is 0.02 m higher
        assert zb - zm == pytest.approx(-3.0 + 10.0 * 9.873 / 5000.0, abs=0.01)
        assert abs(math.remainder(yaw_b - yaw_m, 2 * math.pi)) < 1e-3
        assert cov_b == pytest.approx(cov_m, abs=1e-3)        # the uncertainty is the anchor's


def test_base_link_ahead_follows_the_join_onto_the_next_branch():
    tr = PathTracker(PARAMS_YAML, _route())
    tr.on_fix(*_lla(-4995.0, 0.0), 2, distance=0.0)           # master 5 m before the end
    x, y, _, yaw, _ = tr.advance(0.0)                          # base_link 4.873 m down branch 1
    fx, fy, _ = _enu_bag(*_lla(-5000.0, -4.873), origin=_lla(-4995.0, 0.0))
    assert math.hypot(x - fx, y - fy) < 1.0
    assert abs(math.remainder(yaw + math.pi / 2, 2 * math.pi)) < 0.05   # heading south


def test_stop_snap_stays_on_the_master_arc_with_the_base_link_offset():
    """Stop places are where master stood: the offset must not move the snap by 9.873 m."""
    route = _route()
    stops = ((0, 1500.0),)
    master = PathTracker(PARAMS, Route(origin=route.origin, branches=route.branches, stops=stops))
    base = PathTracker(PARAMS_YAML, Route(origin=route.origin, branches=route.branches, stops=stops))
    for tr in (master, base):
        tr.on_fix(*_lla(-1000.0, 0.0, 170.0), 2, distance=0.0)
        assert tr.on_stop(505.0)                              # 5 m past the stop place
    xm, _, _, _, _ = master.advance(505.0)
    xb, _, _, _, _ = base.advance(505.0)
    assert xm == pytest.approx(_enu_bag(*_lla(-1500.0, 0.0), origin=_lla(-1000.0, 0.0, 170.0))[0],
                               abs=2.0)                       # snapped towards the place
    assert xb - xm == pytest.approx(-9.873, abs=0.01)


def test_pipeline_publishes_base_link_with_the_offset_of_params():
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS_YAML, route=_route())
    start = _lla(-1000.0, 0.0, 170.0)
    odo.step(_fix_msg(0.0, start))
    est = _wheels(odo, 0.0, 100.0, 36.0)                      # 10 m/s for 100 s: 1 km west
    fx, fy, fz = _grid(*_lla(-2009.873, 0.0, 172.0))
    assert math.hypot(est.x - fx, est.y - fy) < 2.0
    assert est.z == pytest.approx(fz - 3.0, abs=0.3)          # ellipsoidal height of base_link


def test_base_link_goes_on_past_a_dead_end_by_the_offset_at_most():
    """The map ends where master stood: the rail goes on for base_ahead_m past it."""
    tr = PathTracker(PARAMS_YAML, _route())
    start = _lla(-5000.0, -250.0)                             # branch 1 is a dead end at -300 m
    tr.on_fix(*start, 2, distance=0.0)
    for distance, south in ((45.0, 304.873), (50.0, 309.873), (500.0, 309.873)):
        x, y, _, _, _ = tr.advance(distance)
        fx, fy, _ = _enu_bag(*_lla(-5000.0, -south), origin=start)
        assert math.hypot(x - fx, y - fy) < 0.5


@pytest.mark.parametrize('speed', [math.nan, math.inf, -math.inf])
def test_non_finite_speed_never_takes_the_side_track(speed):
    tr = _depot_tracker()
    assert tr.advance(70.0, speed)[:2] == pytest.approx((70.0, 0.0), abs=0.1)


def test_running_past_the_side_dead_end_goes_back_to_the_default_path():
    """A false switch must not freeze the estimate at the dead end: the side track is 40+100 m
    long, so 30 m of path beyond its end means the tram was on the loop all along."""
    tr = _depot_tracker()
    tr.advance(70.0, 7.0)                          # switched: 30 m along the side track
    side_len = tr._s[3][-1]
    at_end = tr.advance(40.0 + side_len + 29.0, 3.0)
    assert at_end[:2] == pytest.approx((80.0, -130.0), abs=0.1)   # still waiting at its end
    back = tr.advance(40.0 + side_len + 31.0, 3.0)
    default = _depot_tracker().advance(40.0 + side_len + 31.0, 3.0)
    assert back[:2] == pytest.approx(default[:2], abs=1e-6)


@pytest.mark.parametrize('key, value', [('side_speed_mps', 0.0), ('side_min_m', 60.0),
                                        ('side_overrun_m', 0.0)])
def test_load_params_rejects_bad_side_keys(tmp_path, key, value):
    text = (ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml').read_text(encoding='utf-8')
    import re
    text = re.sub(rf'(\n\s+{key}:\s*)[-0-9.]+', rf'\g<1>{value}', text)
    path = tmp_path / 'params.yaml'
    path.write_text(text, encoding='utf-8')
    with pytest.raises(ValueError, match=key):
        load_params(path)


def test_a_new_anchor_drops_the_undo_of_an_earlier_side_switch():
    """A fix of the window re-anchors after a side switch: the switch is forgotten, a real
    fast entry into the side track is taken again, and nothing restores the old anchor."""
    tr = _depot_tracker()
    tr.advance(70.0, 7.0)                          # side switch inside the GNSS window
    tr.on_fix(*_lla(0.0, 0.0), 2, distance=70.0)   # the window re-anchors at the start
    fresh = _depot_tracker()
    # re-anchored at s = 0 with path 70: path 140 is 70 m along, as a fresh tracker at 70
    assert tr.advance(140.0, 7.0)[:2] == pytest.approx(fresh.advance(70.0, 7.0)[:2], abs=1e-6)
    assert tr._undo is not None and tr._anchor[0] == 3


def test_base_link_offset_is_map_arc_whatever_the_online_wheel_scale():
    """Two stop snaps change the online scale; base_link stays 9.873 m of map arc ahead."""
    route = _route()
    stops = ((0, 1500.0), (0, 2000.0))
    master = PathTracker(PARAMS, Route(origin=route.origin, branches=route.branches, stops=stops))
    base = PathTracker(PARAMS_YAML, Route(origin=route.origin, branches=route.branches, stops=stops))
    for tr in (master, base):
        tr.on_fix(*_lla(-1000.0, 0.0, 170.0), 2, distance=0.0)
        assert tr.on_stop(505.0) and tr.on_stop(1020.0)      # the wheels read 515 m for 500 m
        assert tr._scale != pytest.approx(1.0, abs=1e-3)
    xm, _, _, _, _ = master.advance(1300.0)
    xb, _, _, _, _ = base.advance(1300.0)
    assert xb - xm == pytest.approx(-9.873, abs=0.01)


def _depot_base_tracker():
    tr = PathTracker(PARAMS_YAML, _depot_route())
    tr.on_fix(*_lla(0.0, 0.0), 2, distance=0.0)
    return tr


def test_base_link_follows_the_side_track_once_master_takes_it():
    tr = _depot_base_tracker()
    # before the decision (master 5 m past the side start) base_link is on the default path
    assert tr.advance(45.0, 7.0)[:2] == pytest.approx((54.873, 0.0), abs=0.1)
    # master 30 m down the side track (branch 3 runs (40, 0) -> (80, -30)): base_link 39.873 m
    x, y, _, _, _ = tr.advance(70.0, 7.0)
    assert (x, y) == pytest.approx((40.0 + 0.8 * 39.873, -0.6 * 39.873), abs=0.1)


def test_base_link_goes_back_with_master_when_the_side_switch_is_undone():
    tr = _depot_base_tracker()
    tr.advance(70.0, 7.0)                                     # onto the side track
    side_len = tr._s[3][-1]
    at_end = tr.advance(40.0 + side_len + 29.0, 3.0)          # master waits at the dead end
    assert at_end[:2] == pytest.approx((80.0, -130.0 - 9.873), abs=0.1)   # base_link past it
    back = tr.advance(40.0 + side_len + 31.0, 3.0)
    default = _depot_base_tracker().advance(40.0 + side_len + 31.0, 3.0)
    assert back[:2] == pytest.approx(default[:2], abs=1e-6)


@pytest.mark.parametrize('key, value', [('base_ahead_m', -1.0), ('scale_max_dev', 1.0)])
def test_load_params_rejects_bad_base_link_keys(tmp_path, key, value):
    import re
    text = (ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml').read_text(encoding='utf-8')
    text = re.sub(rf'(\n\s+{key}:\s*)[-0-9.]+', rf'\g<1>{value}', text)
    path = tmp_path / 'params.yaml'
    path.write_text(text, encoding='utf-8')
    with pytest.raises(ValueError, match=key):
        load_params(path)
