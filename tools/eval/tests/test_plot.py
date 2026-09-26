"""--plot: series of one run (units, error signs, controller, slip) and the PNG files."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from tram_eval import bag, cli, plot
from tram_eval.bag import CMD, FRONT, REAR
from tram_eval.reference import geodetic_to_enu

from test_bag import LAT0, LON0, M_PER_DEG_E, DeadReckoning, drive
from test_cli import short_bags  # noqa: F401  (fixture)


class Ahead(DeadReckoning):
    """Dead reckoning that reports 0.5 m/s too fast and 20 m ahead along the track (east)."""

    def step(self, raw):
        e = super().step(raw)
        return NS(t=e.t, speed=e.speed + 0.5, x=e.x + 20.0, y=e.y, slip=e.slip)


def test_series_units_error_signs_controller_and_slip():
    msgs = drive(duration=60.0, speed=10.0)
    s = plot.bag_series(msgs, 5.0, Ahead, ref_point='master')    # the double tracks master
    assert s.t0 == pytest.approx(100.0)
    for side in ('front', 'rear'):
        assert len(s.wheel_t[side]) == 600
        assert np.allclose(s.wheel_v[side], 10.0)        # m/s, not the 36 km/h of the bag
    assert len(s.cmd_t) == 600 and set(s.cmd.tolist()) == {3}
    assert np.median(s.speed_err) == pytest.approx(0.5, abs=1e-6)   # estimate - reference
    assert np.median(s.along_err) == pytest.approx(20.0, abs=1.0)   # + = estimate ahead
    assert np.max(np.abs(s.cross_err)) < 0.5
    assert s.est.slip.sum() == 600                       # the double flags every rear message
    assert s.route == []                                 # the double carries no map


def test_series_are_in_stamp_order_whatever_the_recording_order():
    msgs = drive(duration=20.0)
    late = [m for m in msgs[600:900] if m[0] in (FRONT, REAR, CMD)]   # a late burst (docs/data.md, trap 5)
    msgs = msgs[:600] + [m for m in msgs[600:] if m not in late] + late
    s = plot.bag_series(msgs, 5.0, DeadReckoning)
    for t in (s.est.t, s.wheel_t['front'], s.wheel_t['rear'], s.cmd_t):
        assert np.all(np.diff(t) >= 0)
    assert len(s.est.slip) == len(s.est.t) and s.est.slip.sum() == 200


def test_plot_bag_reports_a_failure_instead_of_raising(tmp_path, capsys):
    assert plot.plot_bag('empty', 5.0, tmp_path, make_odometry=DeadReckoning, msgs=[]) == []
    assert 'empty: plot failed' in capsys.readouterr().err


def test_route_moves_into_the_frame_of_the_reference():
    east = np.array([0.0, 100.0, 200.0])
    route = NS(origin=(LAT0, LON0 + 100.0 / M_PER_DEG_E, 150.0),
               branches=(NS(x=east, y=np.zeros(3), z=np.zeros(3)),))
    same = plot.route_in_ref_frame(NS(origin=route.origin, branches=route.branches), route.origin)
    assert np.allclose(same[0], np.stack([east, np.zeros(3)], axis=1), atol=1e-6)
    moved = plot.route_in_ref_frame(route, (LAT0, LON0, 150.0))
    assert np.allclose(moved[0][:, 0], east + 100.0, atol=0.5)   # the map origin is 100 m east of the run's
    assert np.allclose(moved[0][:, 1], 0.0, atol=0.5)
    assert plot.route_in_ref_frame(route, None) == []
    # kilometres apart the tangent planes turn by ~1e-3 rad: metres at the far end of the line
    far_origin, point = (LAT0 + 0.05, LON0 + 0.05, 160.0), (LAT0 + 0.1, LON0 + 0.1, 170.0)
    in_map = geodetic_to_enu(*point, far_origin)
    far = NS(origin=far_origin, branches=(NS(x=in_map[None, 0], y=in_map[None, 1], z=in_map[None, 2]),))
    run_origin = (LAT0, LON0, 150.0)
    assert np.allclose(plot.route_in_ref_frame(far, run_origin)[0][0], geodetic_to_enu(*point, run_origin)[:2],
                       atol=1e-3)


def test_plot_bag_writes_timeline_and_map(tmp_path):
    pytest.importorskip('matplotlib')
    files = plot.plot_bag('fake', 5.0, tmp_path, make_odometry=DeadReckoning, msgs=drive(duration=30.0))
    assert [f.name for f in files] == ['fake_timeline.png', 'fake_xy.png']
    assert all(f.stat().st_size > 10_000 for f in files)
    # a run without GNSS has no reference: the wheels and the estimate are still drawn
    wheels_only = [(t, m) for t, m in drive(duration=30.0) if t in (FRONT, REAR, CMD)]
    files = plot.plot_bag('nognss', 5.0, tmp_path, make_odometry=DeadReckoning, msgs=wheels_only)
    assert all(f.exists() for f in files)


def test_cli_plot_writes_png_per_bag(short_bags, tmp_path):  # noqa: F811
    pytest.importorskip('matplotlib')
    assert cli.main(['--bag', short_bags[0], '--jobs', '1', '--plot', '--out', str(tmp_path)]) == 0
    assert sorted(p.name for p in (tmp_path / 'plots').iterdir()) == [
        f'{short_bags[0]}_timeline.png', f'{short_bags[0]}_xy.png']
    assert bag.NOTES not in (tmp_path / 'metrics.json').read_text(encoding='utf-8')
