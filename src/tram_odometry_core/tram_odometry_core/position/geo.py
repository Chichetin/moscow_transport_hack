"""WGS84 geodesy for the output frame (#155, D-083): geodetic <-> ECEF, local ENU, UTM and the
flat MGRS grid of /result/position.

The organizers' frame is the plane MGRS grid of one 100 km square: UTM easting and northing of
the zone minus the corner of the square (37UCB: 300000, 6100000), height above the WGS84
ellipsoid (NavSatFix.altitude, REP 103). UTM is the Krueger series in the form of Karney (2011)
to n^4: sub-millimetre inside a zone. ECEF -> geodetic is the closed form of Heikkinen (1982).
Scalar `math` on the hot path: a few tens of microseconds per point.
"""
import math
from typing import Tuple

import numpy as np

WGS84_A = 6378137.0                       # m, semi-major axis
WGS84_F = 1.0 / 298.257223563             # flattening
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)      # first eccentricity squared
WGS84_B = WGS84_A * (1.0 - WGS84_F)       # m, semi-minor axis
UTM_K0 = 0.9996                           # scale on the central meridian
UTM_FALSE_EASTING = 500000.0              # m
UTM_ZONE_WIDTH_DEG = 6.0

_N = WGS84_F / (2.0 - WGS84_F)            # third flattening
_RECT_A = WGS84_A / (1.0 + _N) * (1.0 + _N ** 2 / 4.0 + _N ** 4 / 64.0)   # rectifying radius
_ALPHA = (_N / 2.0 - 2.0 * _N ** 2 / 3.0 + 5.0 * _N ** 3 / 16.0 + 41.0 * _N ** 4 / 180.0,
          13.0 * _N ** 2 / 48.0 - 3.0 * _N ** 3 / 5.0 + 557.0 * _N ** 4 / 1440.0,
          61.0 * _N ** 3 / 240.0 - 103.0 * _N ** 4 / 140.0,
          49561.0 * _N ** 4 / 161280.0)
_E = math.sqrt(WGS84_E2)

Grid = Tuple[int, float, float]           # UTM zone (north), easting and northing of the square corner, m


def ecef(lat: float, lon: float, alt: float) -> np.ndarray:
    """WGS84 geodetic (deg, deg, m above the ellipsoid) -> ECEF, m."""
    la, lo = math.radians(lat), math.radians(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(la) ** 2)
    return np.array([(n + alt) * math.cos(la) * math.cos(lo),
                     (n + alt) * math.cos(la) * math.sin(lo),
                     (n * (1.0 - WGS84_E2) + alt) * math.sin(la)])


def enu_rotation(lat: float, lon: float) -> np.ndarray:
    """Rows: east, north, up unit vectors in ECEF."""
    la, lo = math.radians(lat), math.radians(lon)
    return np.array([[-math.sin(lo), math.cos(lo), 0.0],
                     [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                     [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


def geodetic(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """ECEF, m -> WGS84 lat, lon (deg), height above the ellipsoid (m); Heikkinen's closed form,
    valid away from the Earth's centre (any point near the surface)."""
    a2, b2 = WGS84_A ** 2, WGS84_B ** 2
    ep2 = (a2 - b2) / b2
    p = math.hypot(x, y)
    f = 54.0 * b2 * z * z
    g = p * p + (1.0 - WGS84_E2) * z * z - WGS84_E2 * (a2 - b2)
    c = WGS84_E2 ** 2 * f * p * p / g ** 3
    s = (1.0 + c + math.sqrt(c * c + 2.0 * c)) ** (1.0 / 3.0)
    k = s + 1.0 + 1.0 / s
    pp = f / (3.0 * k * k * g * g)
    q = math.sqrt(1.0 + 2.0 * WGS84_E2 ** 2 * pp)
    r0 = (-pp * WGS84_E2 * p / (1.0 + q)
          + math.sqrt(max(a2 / 2.0 * (1.0 + 1.0 / q) - pp * (1.0 - WGS84_E2) * z * z / (q * (1.0 + q))
                          - pp * p * p / 2.0, 0.0)))
    u = math.hypot(p - WGS84_E2 * r0, z)
    v = math.sqrt((p - WGS84_E2 * r0) ** 2 + (1.0 - WGS84_E2) * z * z)
    z0 = b2 * z / (WGS84_A * v)
    h = u * (1.0 - b2 / (WGS84_A * v))
    return math.degrees(math.atan2(z + ep2 * z0, p)), math.degrees(math.atan2(y, x)), h


def utm(lat: float, lon: float, zone: int) -> Tuple[float, float]:
    """UTM easting, northing (m, northern hemisphere) of WGS84 lat, lon (deg) in `zone`."""
    phi = math.radians(lat)
    dlam = math.radians(lon - (zone * UTM_ZONE_WIDTH_DEG - 183.0))
    sin_phi = math.sin(phi)
    t = math.sinh(math.atanh(sin_phi) - 2.0 * math.sqrt(_N) / (1.0 + _N)
                  * math.atanh(2.0 * math.sqrt(_N) / (1.0 + _N) * sin_phi))
    xi = math.atan2(t, math.cos(dlam))
    eta = math.atanh(math.sin(dlam) / math.sqrt(1.0 + t * t))
    e, n = eta, xi
    for j, alpha in enumerate(_ALPHA, start=1):
        e += alpha * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
        n += alpha * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
    return UTM_FALSE_EASTING + UTM_K0 * _RECT_A * e, UTM_K0 * _RECT_A * n


def to_grid(lat: float, lon: float, alt: float, grid: Grid) -> Tuple[float, float, float]:
    """WGS84 point -> flat MGRS grid x (east), y (north) from the corner of the square, m, and
    z = height above the ellipsoid, m."""
    zone, e0, n0 = grid
    east, north = utm(lat, lon, zone)
    return east - e0, north - n0, alt


def enu_to_grid(rot: np.ndarray, ecef0: np.ndarray, grid: Grid, x: float, y: float,
                z: float) -> Tuple[float, float, float]:
    """A point of a local ENU frame (rows of `rot` at ECEF origin `ecef0`) -> MGRS grid.
    Plain floats: numpy on 3-vectors costs more than the geodesy itself (hot path)."""
    (e0, e1, e2), (n0, n1, n2), (u0, u1, u2) = rot.tolist()
    o0, o1, o2 = ecef0.tolist()
    return to_grid(*geodetic(o0 + e0 * x + n0 * y + u0 * z, o1 + e1 * x + n1 * y + u1 * z,
                             o2 + e2 * x + n2 * y + u2 * z), grid)


def pose_to_grid(rot: np.ndarray, ecef0: np.ndarray, grid: Grid, x: float, y: float, z: float,
                 yaw: float, cov):
    """(x, y, z, yaw, (var_x, var_y, cov_xy)) of a local ENU frame -> the same in the MGRS grid.
    The heading turns by the grid convergence plus the tilt of the local tangent plane (about
    -1.3 deg on the route, D-083): it is taken from a second point 1 m ahead; the covariance
    turns with it (the UTM scale, 1 - 3e-4, is left out)."""
    gx, gy, gz = enu_to_grid(rot, ecef0, grid, x, y, z)
    ax, ay, _ = enu_to_grid(rot, ecef0, grid, x + math.cos(yaw), y + math.sin(yaw), z)
    gyaw = math.atan2(ay - gy, ax - gx)
    d = gyaw - yaw
    c, s = math.cos(d), math.sin(d)
    vx, vy, cxy = cov
    return gx, gy, gz, gyaw, (c * c * vx - 2.0 * c * s * cxy + s * s * vy,
                              s * s * vx + 2.0 * c * s * cxy + c * c * vy,
                              c * s * (vx - vy) + (c * c - s * s) * cxy)
