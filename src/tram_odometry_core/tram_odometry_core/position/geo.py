"""WGS84 to the continuous grid based on MGRS square 37UCB.

The jury example uses UTM zone 37N with the CB square origin (300000, 6100000).
Keeping that origin when the tram enters DB avoids the 100 km MGRS tile wrap.
The UTM series is the standard ellipsoidal Transverse Mercator approximation;
the route is less than two degrees from zone 37's central meridian.
"""
import math

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
UTM_K0 = 0.9996
UTM_CENTRAL_MERIDIAN = math.radians(39.0)
CB_EASTING = 300000.0
CB_NORTHING = 6100000.0


def wgs84_to_mgrs_grid(lat: float, lon: float, alt: float):
    """Return continuous (x, y, z) metres relative to 37UCB, z ellipsoid height."""
    phi = math.radians(lat)
    dl = math.radians(lon) - UTM_CENTRAL_MERIDIAN
    e2 = WGS84_E2
    ep2 = e2 / (1.0 - e2)
    sin_phi, cos_phi = math.sin(phi), math.cos(phi)
    tan_phi = math.tan(phi)
    n = WGS84_A / math.sqrt(1.0 - e2 * sin_phi * sin_phi)
    t = tan_phi * tan_phi
    c = ep2 * cos_phi * cos_phi
    a = dl * cos_phi
    m = WGS84_A * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * phi
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * phi)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * phi)
        - (35 * e2**3 / 3072) * math.sin(6 * phi))
    easting = 500000.0 + UTM_K0 * n * (
        a + (1 - t + c) * a**3 / 6
        + (5 - 18*t + t*t + 72*c - 58*ep2) * a**5 / 120)
    northing = UTM_K0 * (m + n * tan_phi * (
        a*a / 2 + (5 - t + 9*c + 4*c*c) * a**4 / 24
        + (61 - 58*t + t*t + 600*c - 330*ep2) * a**6 / 720))
    return easting - CB_EASTING, northing - CB_NORTHING, alt


def ecef_to_wgs84(x: float, y: float, z: float):
    """Return (latitude degrees, longitude degrees, ellipsoid height metres)."""
    p = math.hypot(x, y)
    lon = math.atan2(y, x)
    phi = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(6):
        s = math.sin(phi)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s*s)
        alt = p / math.cos(phi) - n
        phi = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + alt)))
    s = math.sin(phi)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s*s)
    alt = p / math.cos(phi) - n
    return math.degrees(phi), math.degrees(lon), alt


class EnuGrid:
    """Project an ENU trajectory into the fixed, continuous 37UCB grid.

    The ENU origin is held for the whole run; the UTM square origin never changes.
    """

    def __init__(self, origin):
        lat, lon, alt = origin
        phi, lam = math.radians(lat), math.radians(lon)
        sin_phi, cos_phi = math.sin(phi), math.cos(phi)
        sin_lam, cos_lam = math.sin(lam), math.cos(lam)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
        self._ecef0 = np.array([(n + alt) * cos_phi * cos_lam,
                                (n + alt) * cos_phi * sin_lam,
                                (n * (1.0 - WGS84_E2) + alt) * sin_phi])
        self._enu_to_ecef = np.array([[-sin_lam, -sin_phi * cos_lam, cos_phi * cos_lam],
                                      [cos_lam, -sin_phi * sin_lam, cos_phi * sin_lam],
                                      [0.0, cos_phi, sin_phi]])

    def point(self, east: float, north: float, up: float):
        ecef = self._ecef0 + self._enu_to_ecef @ np.array([east, north, up])
        return wgs84_to_mgrs_grid(*ecef_to_wgs84(*ecef))
