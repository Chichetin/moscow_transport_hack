"""Identification on synthetic data with known parameters (issue #9)."""
import os
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import drive_ident as di          # noqa: E402
import identify as ident          # noqa: E402

GRID = np.array(ident.SPEED_GRID)
C_TRUE = (0.04, 0.004, 0.0002)


def true_params(power=9.0, adhesion=1.3):
    """Traction 0.08*n capped later by power/adhesion, brake 0.1*n, flat in v."""
    rows = np.arange(ident.NOTCH_MAX + 1)[:, None] * np.ones(len(GRID))
    return {'notch_max': ident.NOTCH_MAX, 'speed_grid_mps': list(GRID),
            'traction_accel_table': list((0.08 * rows).ravel()),
            'brake_accel_table': list((0.1 * rows).ravel()),
            'adhesion_accel_mps2': adhesion, 'traction_power_w_per_kg': power, 'c': C_TRUE}


def synthetic_bag(p, delay, seconds=3000.0, grade_amp=0.02, noise=0.0, seed=0, grade_const=0.0):
    """A tram driven by random notch steps through the true model with a known delay and grade."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, seconds, di.DT)
    cmd_t = np.arange(0.0, seconds, 0.05)
    steps = np.repeat(rng.integers(-8, 16, size=len(cmd_t) // 100 + 1), 100)[:len(cmd_t)]
    notch = di.notch_at(cmd_t, steps, t - di.DT / 2 - delay)     # at the middle of step k-1 -> k
    grade = grade_const + grade_amp * np.sin(t / 60.0)
    v = np.empty(len(t))
    v[0] = 5.0
    for k in range(1, len(t)):
        acc = float(di.drive_accel(notch[k], v[k - 1], p)) - di.G * grade[k - 1]
        v[k] = min(max(v[k - 1] + acc * di.DT, 0.0), 15.5)
    v_meas = v + noise * rng.standard_normal(len(t))
    a = np.full(len(t), np.nan)
    k = int(round(di.DERIV_HALF_S / di.DT))
    a[k:-k] = (v_meas[2 * k:] - v_meas[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
    # the speed cap is not physics: samples within the derivative window of it are marked invalid
    capped = np.convolve((v >= 15.5).astype(int), np.ones(2 * k + 3, int), mode='same') > 0
    return di.BagSamples('synthetic', t, v_meas, a, np.zeros(len(t)), grade, cmd_t, steps,
                         3.6, 3.6, ~capped)


def test_resistance_is_recovered_and_non_negative():
    rng = np.random.default_rng(1)
    v = rng.uniform(1, 15, 20000)
    a = -di.resistance(v, C_TRUE) + 0.02 * rng.standard_normal(len(v))
    c = di.fit_resistance(v, a)
    assert c == pytest.approx(C_TRUE, abs=2e-3)
    c_neg = di.fit_resistance(v, -(0.05 - 0.001 * v * v))       # best unconstrained c2 < 0
    assert min(c_neg) >= 0.0


def test_blas_is_pinned_single_threaded():
    # #68: np.linalg.solve in fit_curve() below once returned a node 0.038 off from the single-
    # threaded result on CI -- roughly 11 orders of magnitude more than the ~1e-13 rounding noise
    # expected at this system's condition number (~1e3), so multi-threaded OpenBLAS (a race, or a
    # DYNAMIC_ARCH kernel bug -- not established which) is the suspect, not just "a different but
    # still correct" summation order. Root conftest.py pins the thread count before numpy is
    # imported anywhere in the session; this guards against someone removing that, and against it
    # being too late (see _NUMPY_ALREADY_IMPORTED there).
    import conftest
    assert not conftest._NUMPY_ALREADY_IMPORTED, (
        'numpy was already imported before conftest.py pinned BLAS threads -- the pin came too '
        'late to take effect (#68)')
    for var in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        assert os.environ.get(var) == '1', f'{var} must be pinned for reproducible curve fits'


def test_curve_fit_and_sparse_nodes():
    rng = np.random.default_rng(2)
    v = rng.uniform(0, 12, 5000)
    y = np.interp(v, [0, 4, 12], [1.0, 1.2, 0.6]) + 0.05 * rng.standard_normal(len(v))
    vals, counts = di.fit_curve(v, y, GRID)
    assert np.isnan(vals[GRID > 13]).all()                        # no data above 12 m/s
    ok = GRID <= 12
    assert vals[ok] == pytest.approx(np.interp(GRID[ok], [0, 4, 12], [1.0, 1.2, 0.6]), abs=0.03)


def test_fill_monotone_extends_along_speed_then_notch():
    t = np.full((4, 3), np.nan)
    t[1] = [0.2, 0.3, np.nan]            # no data at the top speed: edge value
    t[3] = [0.5, 0.2, 0.6]               # dips below the row under it: raised
    f = di.fill_monotone(t)
    assert list(f[0]) == [0, 0, 0]
    assert list(f[1]) == pytest.approx([0.2, 0.3, 0.3])
    assert list(f[2]) == pytest.approx([0.35, 0.3, 0.45])        # between rows 1 and 3
    assert (np.diff(f, axis=0) >= 0).all()


def test_drive_accel_limits_and_signs():
    p = true_params()
    assert di.drive_accel(0, 10.0, p) == pytest.approx(-di.resistance(10.0, C_TRUE))
    assert di.drive_accel(15, 0.0, p) == pytest.approx(1.2 - C_TRUE[0])          # table, power cap off at v=0
    assert di.drive_accel(15, 2.0, p) == pytest.approx(1.2 - di.resistance(2.0, C_TRUE))
    assert di.drive_accel(15, 12.0, p) == pytest.approx(9.0 / 12.0 - di.resistance(12.0, C_TRUE))
    assert di.drive_accel(-15, 5.0, p) == pytest.approx(-1.3 - di.resistance(5.0, C_TRUE))  # adhesion
    assert di.drive_accel(-40, 5.0, p) == di.drive_accel(-15, 5.0, p)                        # clipped notch
    assert di.drive_accel(3, 30.0, p) == pytest.approx(0.24 - di.resistance(30.0, C_TRUE))  # edge of grid


def test_grade_from_a_linear_ramp():
    s = np.arange(0.0, 500.0, 1.0)
    g = di.grade_along(s / 10.0, s, 0.02 * s, np.array([10.0, 25.0]))
    assert g == pytest.approx([0.02, 0.02])


def test_identify_recovers_delay_resistance_and_tables():
    p = true_params()
    bags = [synthetic_bag(p, delay=0.5, seed=s) for s in range(3)]
    res = ident.identify(bags, use_grade=True)
    assert res['delay_s'] == pytest.approx(0.5, abs=di.DT)       # resolution of the 10 Hz grid
    assert res['params']['c'] == pytest.approx(C_TRUE, abs=5e-3)
    notches = np.arange(ident.NOTCH_MAX + 1)[:, None]
    cols = (GRID >= 1.0) & (GRID <= 12.0)                         # where the drive has data
    tr = np.array(res['params']['traction_accel_table']).reshape(ident.NOTCH_MAX + 1, -1)
    effective = np.minimum(0.08 * notches, 9.0 / GRID[None, :].clip(1e-9))   # table under the power cap
    err = np.abs(tr - effective)[1:, cols]
    assert np.median(err) < 0.015 and err.max() < 0.08
    br = np.array(res['params']['brake_accel_table']).reshape(ident.NOTCH_MAX + 1, -1)
    err = np.abs(br - 0.1 * notches)[1:9, cols]                   # brake notches -1..-8 were driven
    assert np.median(err) < 0.015 and err.max() < 0.08
    adhesion = res['params']['adhesion_accel_mps2']                # never cuts a table cell
    assert adhesion >= max(tr.max(), br.max()) and adhesion == round(adhesion, 2)


@pytest.mark.parametrize('x', [1.09, 1.10, 1.11, 1.12, 1.591, 1.592])
def test_round_up_to_cent_never_undershoots(x):
    """round(x*100) first, not round(x)*100 (ревью PR #64): the latter overshoots by a
    cent for x in {1.09..1.12} on this platform's binary float rounding."""
    r = ident.round_up_to_cent(x)
    assert r >= x and r == round(r, 2) and r - x < 0.01


def test_default_fit_ignores_grade_and_absorbs_a_constant_slope():
    # the contract model_accel(notch, v) has no grade: by default the fit does not use it, so a
    # constant uphill of 1 % shows up as extra resistance g * 0.01; with --grade it is removed
    p = true_params()
    bags = [synthetic_bag(p, delay=0.3, seed=s, grade_amp=0.0, grade_const=0.01) for s in range(2)]
    c_grade = ident.identify(bags, use_grade=True)['params']['c']
    assert c_grade == pytest.approx(C_TRUE, abs=5e-3)
    for b in bags:
        b.grade[:] = np.nan                                        # without any GNSS grade...
    c0 = ident.identify(bags)['params']['c']                       # ...the default fit still runs
    assert di.resistance(5.0, c0) == pytest.approx(di.resistance(5.0, C_TRUE) + di.G * 0.01, abs=0.01)


def test_model_beats_constant_speed_on_windows():
    p = true_params()
    b = synthetic_bag(p, delay=0.3, seconds=600.0, noise=0.01)
    model, zero = di.simulate_windows(b, p, 0.3)
    assert len(model) > 50
    assert np.median(model) < 0.05 < np.median(zero)
    wrong = dict(p, c=(0.3, 0.0, 0.0))                           # 0.26 m/s^2 too much resistance
    assert np.median(di.simulate_windows(b, wrong, 0.3)[0]) > 5 * np.median(model)


def test_extract_converts_units_and_accel():
    from types import SimpleNamespace as NS

    def h(t):
        return NS(stamp=NS(sec=int(t), nanosec=int(round((t - int(t)) * 1e9))))
    msgs = []
    for k in range(300):                                          # 1 m/s^2 from 0 for 30 s, km/h in
        t = 100.0 + 0.1 * k
        kmh = 3.6 * 0.1 * k
        msgs += [(di.evbag.FRONT, NS(header=h(t), velocity=kmh)),
                 (di.evbag.REAR, NS(header=h(t + 0.01), velocity=kmh)),
                 (di.evbag.CMD, NS(header=h(t), position=7))]
    s = di.extract(msgs)
    mid = len(s.t) // 2
    assert s.v[mid] == pytest.approx(s.t[mid] - 100.0, abs=0.02)     # m/s, not km/h
    assert np.nanmedian(s.a) == pytest.approx(1.0, abs=0.01)
    assert np.isnan(s.grade).all() and np.isnan(s.ratio_front)       # no GNSS
    assert s.valid.all() and set(s.cmd_n) == {7}


def test_check_split_is_never_fitted(monkeypatch, tmp_path):
    # D-011: the check split (holdout) goes only to validate(), never to identify()
    p = true_params()
    train = [synthetic_bag(p, delay=0.3, seconds=600.0, seed=s) for s in range(2)]
    check = [synthetic_bag(p, delay=0.3, seconds=600.0, seed=9)]
    check[0].name = 'check'
    monkeypatch.setattr(ident, 'load_split', lambda split, jobs: check if split == 'holdout' else train)
    monkeypatch.setattr(di.evbag, 'out_dir', lambda: tmp_path)
    fitted = []
    real_identify = ident.identify
    monkeypatch.setattr(ident, 'identify',
                        lambda bags, **kw: fitted.extend(b.name for b in bags) or real_identify(bags, **kw))
    assert ident.main(['--check-split', 'holdout', '--no-plots', '--jobs', '1']) == 0
    assert fitted and 'check' not in fitted


def test_written_params_pass_the_contract(tmp_path):
    sys.path.insert(0, str(di.REPO / 'src' / 'tram_odometry_core'))
    from tram_odometry_core.types import load_params
    p = true_params()
    p['traction_accel_table'] = [round(x, 3) for x in p['traction_accel_table']]
    p['brake_accel_table'] = [round(x, 3) for x in p['brake_accel_table']]
    scales = {'front': {'wheel_scale': 1.0012}, 'rear': {'wheel_scale': 0.9981}}
    target = tmp_path / 'params.yaml'
    shutil.copy(ident.PARAMS_YAML, target)
    before = target.read_text(encoding='utf-8')
    ident.write_params(p, scales, target)
    loaded = load_params(target)
    assert loaded.vehicle.wheel_scale_front == 1.0012 and loaded.vehicle.wheel_scale_rear == 0.9981
    assert loaded.drive.speed_grid_mps == tuple(GRID)
    assert loaded.drive.traction_accel_table[len(GRID) * 5 + 2] == pytest.approx(0.4)
    assert (loaded.resistance.c0, loaded.resistance.c1, loaded.resistance.c2) == pytest.approx(C_TRUE)
    shutil.copy(ident.PARAMS_YAML, target)
    ident.write_params(p, {'front': {'wheel_scale': 1.0004}, 'rear': {'wheel_scale': 0.9996}}, target)
    assert load_params(target).vehicle.wheel_scale_front == load_params(ident.PARAMS_YAML).vehicle.wheel_scale_front
    after = target.read_text(encoding='utf-8')
    comments = [ln.split('#', 1)[1] for ln in before.splitlines() if '#' in ln]
    assert all(c in after for c in comments)                           # every comment kept
