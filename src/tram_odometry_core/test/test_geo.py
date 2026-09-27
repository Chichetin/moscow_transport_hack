"""Output frame of /result/position: WGS84 -> flat MGRS grid 37UCB (#155, D-082)."""
import math
from pathlib import Path

import numpy as np
import pytest

from tram_odometry_core.position import geo
from tram_odometry_core.types import load_params

ROOT = Path(__file__).resolve().parents[3]
PARAMS = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
GRID = (PARAMS.frames.grid_zone, PARAMS.frames.grid_origin_e_m, PARAMS.frames.grid_origin_n_m)
# organizers, 27.09: lat, lon -> (x, y, z) of base_link; z is the output, not an input height
EXAMPLE_LL = (55.8088325462547, 37.4602768500852)
EXAMPLE_XYZ = (103501.6309, 85876.1201, 167.4109)


def test_params_grid_is_37ucb():
    assert GRID == (37, 300000.0, 6100000.0)


def test_organizers_example_xy():
    x, y, _ = geo.to_grid(*EXAMPLE_LL, 0.0, GRID)
    assert x == pytest.approx(EXAMPLE_XYZ[0], abs=0.01)
    assert y == pytest.approx(EXAMPLE_XYZ[1], abs=0.01)
    # east of the square's 100 km: no wrap into 37UDB (the example itself is past it)
    assert x > 100000.0


def test_z_is_the_ellipsoidal_height_as_given():
    assert geo.to_grid(*EXAMPLE_LL, EXAMPLE_XYZ[2], GRID)[2] == EXAMPLE_XYZ[2]


def test_utm_known_points():
    # central meridian of zone 37 (39 E): easting 500 km exactly; equator: northing 0
    e, n = geo.utm(0.0, 39.0, 37)
    assert e == pytest.approx(500000.0, abs=1e-6) and n == pytest.approx(0.0, abs=1e-6)
    # on the central meridian the northing is k0 * meridian arc; WGS84 arc to 45 N 4 984 944.378 m
    assert geo.utm(45.0, 39.0, 37)[1] == pytest.approx(0.9996 * 4984944.378, abs=0.002)


def test_ecef_geodetic_round_trip():
    rng = np.random.RandomState(155)
    for lat, lon, alt in zip(rng.uniform(55.6, 56.0, 200), rng.uniform(37.2, 37.9, 200),
                             rng.uniform(100.0, 250.0, 200)):
        back = geo.geodetic(*geo.ecef(lat, lon, alt))
        assert back[0] == pytest.approx(lat, abs=1e-10)       # ~1e-5 m
        assert back[1] == pytest.approx(lon, abs=1e-10)
        assert back[2] == pytest.approx(alt, abs=1e-4)


def test_enu_to_grid_round_trip_through_a_local_frame():
    """Points of the route area put into ENU of a run origin and back land on the direct
    conversion to under 1 mm."""
    origin = (55.8104, 37.4623, 168.0)
    rot, e0 = geo.enu_rotation(*origin[:2]), geo.ecef(*origin)
    rng = np.random.RandomState(7)
    for lat, lon, alt in zip(rng.uniform(55.79, 55.83, 100), rng.uniform(37.38, 37.55, 100),
                             rng.uniform(150.0, 200.0, 100)):
        enu = rot @ (geo.ecef(lat, lon, alt) - e0)
        via = geo.enu_to_grid(rot, e0, GRID, *enu)
        direct = geo.to_grid(lat, lon, alt, GRID)
        assert np.allclose(via, direct, atol=1e-3, rtol=0.0)


def test_pose_to_grid_heading_turns_by_convergence():
    """West of the zone meridian (39 E) meridians lean east going north: true north is 1.27 deg
    east of grid north at 37.46 E, so a northbound tram has grid yaw 90 deg - 1.27 deg."""
    origin = (*EXAMPLE_LL, 167.4)
    rot, e0 = geo.enu_rotation(*origin[:2]), geo.ecef(*origin)
    x, y, z, yaw, cov = geo.pose_to_grid(rot, e0, GRID, 0.0, 0.0, 0.0, math.pi / 2, (1.0, 4.0, 0.0))
    assert (x, y) == pytest.approx(EXAMPLE_XYZ[:2], abs=0.01)
    assert z == pytest.approx(167.4, abs=1e-4)
    gamma = math.degrees(yaw) - 90.0
    assert gamma == pytest.approx(-1.274, abs=0.01)
    # along-track variance 4 on the heading (y), cross 1: the long axis turns with the heading
    vx, vy, cxy = cov
    assert vx + vy == pytest.approx(5.0) and vx * vy - cxy * cxy == pytest.approx(4.0)
    assert vx == pytest.approx(1.0 + 3.0 * math.sin(math.radians(gamma)) ** 2)
    assert cxy == pytest.approx(-3.0 * math.sin(math.radians(gamma)) * math.cos(math.radians(gamma)))


def test_grid_distance_is_true_distance_times_utm_scale():
    """1 km east in ENU is 1 km * k * R / (R + h) in the grid (UTM scale k = 0.99971 at 1.5 deg
    off the zone meridian, reduction to the ellipsoid from h = 167 m): metric to 3.2e-4."""
    rot, e0 = geo.enu_rotation(*EXAMPLE_LL), geo.ecef(*EXAMPLE_LL, 167.4)
    a = geo.enu_to_grid(rot, e0, GRID, 0.0, 0.0, 0.0)
    b = geo.enu_to_grid(rot, e0, GRID, 1000.0, 0.0, 0.0)
    assert math.hypot(b[0] - a[0], b[1] - a[1]) == pytest.approx(999.687, abs=0.005)
