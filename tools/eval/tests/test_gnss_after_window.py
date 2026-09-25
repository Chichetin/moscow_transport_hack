"""Disqualification guard (#57): GNSS after gnss.init_window_s must not change a single bit of
what pipeline.Odometry.step returns — the object both the node and tools/eval run (D-005).

Each case runs the same messages twice through a fresh Odometry: (a) with every
/sensing/gnss/* message to the end of the run, (b) with GNSS cut after the window the way
tools/eval cuts it. The Estimate sequences must be equal field by field (repr: exact floats).
"""
import dataclasses
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from tram_eval import bag
from tram_eval.bag import CMD, FRONT, MASTER_FIX, MASTER_VEL, REAR, ROVER_FIX

from test_bag import header, wheel

bag.default_odometry()   # puts tram_odometry_core on sys.path
from tram_odometry_core.pipeline import Odometry  # noqa: E402
from tram_odometry_core.types import load_params, load_route  # noqa: E402

PARAMS = load_params(str(bag.PARAMS_YAML))
ROUTE = load_route(str(bag.REPO / 'src' / 'tram_odometry' / 'maps' / PARAMS.position.map_file))
QUICK_BAG = '30639_d927f360'
M_PER_DEG_N = 111_500.0       # approx near 55.8 N: synthetic fixes land within metres of the map
M_PER_DEG_E = 62_700.0


def make_odometry(route=ROUTE, window_s=None):
    params = PARAMS if window_s is None else dataclasses.replace(
        PARAMS, gnss=dataclasses.replace(PARAMS.gnss, init_window_s=window_s))
    return Odometry(params, route=route)


def on_route(duration=60.0, speed=10.0, late_north_m=30.0):
    """Tram along branch 0 of the map at `speed`; after the window the GNSS lies: fixes
    `late_north_m` north of the track and vel pointing north, faster than ever before."""
    b = ROUTE.branches[0]
    lat0, lon0, alt0 = ROUTE.origin
    window = PARAMS.gnss.init_window_s
    msgs = []
    for k in range(int(duration * 10)):
        t = 1000.0 + 0.1 * k
        s = speed * 0.1 * k
        x, y, z = (float(np.interp(s, b.s, c)) for c in (b.x, b.y, b.z))
        dx = float(np.interp(s + 1.0, b.s, b.x)) - x
        dy = float(np.interp(s + 1.0, b.s, b.y)) - y
        late = t - 1000.0 > window
        dn = late_north_m if late else 0.0
        fix = NS(header=header(t + 0.03), latitude=lat0 + (y + dn) / M_PER_DEG_N,
                 longitude=lon0 + x / M_PER_DEG_E, altitude=alt0 + z, status=NS(status=2))
        ve, vn = (0.0, 1.5 * speed) if late else (speed * dx / math.hypot(dx, dy), speed * dy / math.hypot(dx, dy))
        msgs += [(FRONT, wheel(t, speed * 3.6)), (REAR, wheel(t + 0.01, speed * 3.6)),
                 (CMD, NS(header=header(t + 0.02), position=3)), (MASTER_FIX, fix), (ROVER_FIX, fix),
                 (MASTER_VEL, NS(header=header(t + 0.04), twist=NS(linear=NS(x=ve, y=vn, z=0.0))))]
    return msgs


def outputs(msgs, odometry, window_end=None):
    """Estimates of Odometry.step; GNSS after window_end is not fed when it is given."""
    out = []
    for topic, m in msgs:
        if window_end is not None and topic in bag.GNSS and bag.stamp(m) > window_end:
            continue
        e = odometry.step(bag.to_raw(topic, m))
        if e is not None:
            out.append(e)
    return out


def first_difference(full, cut):
    """None, or the first estimate and field that differ (repr: exact float, nan == nan)."""
    for i, (a, b) in enumerate(zip(full, cut)):
        for f in dataclasses.fields(a):
            va, vb = getattr(a, f.name), getattr(b, f.name)
            if repr(va) != repr(vb):
                return f'estimate {i}, t={a.t!r}: {f.name} {va!r} (all GNSS) != {vb!r} (GNSS cut)'
    if len(full) != len(cut):
        return f'{len(full)} estimates with GNSS after the window, {len(cut)} without'
    return None


@pytest.mark.parametrize('route', [ROUTE, None], ids=['map', 'no-map'])
def test_gnss_after_window_changes_no_bit_of_the_output(route):
    msgs = on_route()
    window_end = bag.gnss_window_end(msgs, PARAMS.gnss.init_window_s)
    full = outputs(msgs, make_odometry(route))
    cut = outputs(msgs, make_odometry(route), window_end)
    assert first_difference(full, cut) is None
    assert len(full) == 1800                              # wheels and controller, not GNSS
    assert full[-1].gnss_used                             # the window GNSS did reach the model


@pytest.mark.parametrize('route', [ROUTE, None], ids=['map', 'no-map'])
def test_the_check_catches_a_pipeline_that_reads_gnss_after_the_window(route):
    # a pipeline whose window never closes: the lying late GNSS must show up as a difference,
    # otherwise the test above would pass vacuously
    msgs = on_route()
    window_end = bag.gnss_window_end(msgs, PARAMS.gnss.init_window_s)
    diff = first_difference(outputs(msgs, make_odometry(route, window_s=1e9)),
                            outputs(msgs, make_odometry(route, window_s=1e9), window_end))
    assert diff is not None and 't=' in diff


def test_quick_bag_bit_for_bit():
    path = bag.data_dir() / QUICK_BAG
    if not (path / 'metadata.yaml').exists():
        pytest.skip('dataset not unpacked (docs/data.md)')
    msgs = bag.read_bag(path)
    window_end = bag.gnss_window_end(msgs, PARAMS.gnss.init_window_s)
    assert sum(t in bag.GNSS and bag.stamp(m) > window_end for t, m in msgs) > 1000   # real late GNSS
    full = outputs(msgs, make_odometry())
    cut = outputs(msgs, make_odometry(), window_end)
    assert first_difference(full, cut) is None
    assert len(full) > 10_000
