"""PathTracker: GNSS alignment in the init window, motion along the route map (D-007, D-024)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from tram_odometry_core.position import PathTracker
from tram_odometry_core.types import Branch, Route, load_params, load_route

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools' / 'pathgraph'))
import build_route as br  # noqa: E402  (reference ENU of the map builder and of tools/eval)

PARAMS = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
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
    fx, fy, _ = _enu_bag(*_lla(-2000.0, 0.0, 172.0), origin=start)
    assert est.gnss_used and math.hypot(est.x - fx, est.y - fy) < 2.0
    assert abs(math.remainder(est.yaw - math.pi, 2 * math.pi)) < 0.01


def test_pipeline_ignores_gnss_after_the_window():
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS, route=_route())
    start = _lla(-1000.0, 0.0, 170.0)
    odo.step(_fix_msg(0.0, start))
    _wheels(odo, 0.0, 10.0, 36.0)
    odo.step(_fix_msg(PARAMS.gnss.init_window_s + 5.0, _lla(-1500.0, 0.0, 171.0)))  # after window
    est = _wheels(odo, 10.1, 20.0, 36.0)
    fx, fy, _ = _enu_bag(*_lla(-1200.0, 0.0, 170.4), origin=start)
    assert math.hypot(est.x - fx, est.y - fy) < 2.0


def test_pipeline_without_route_keeps_the_baseline():
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS)
    est = _wheels(odo, 0.0, 10.0, 36.0)
    assert est.z == 0.0 and est.y == pytest.approx(0.0)
