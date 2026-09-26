"""Experiment #60: the F0/P fit recovers known parameters; the form model is well behaved."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import compare_forms as cf  # noqa: E402


def test_fit_force_power_recovers_the_knee():
    rng = np.random.default_rng(0)
    v = rng.uniform(0.5, 15.0, 4000)
    y = np.minimum(0.9, 6.0 / v) + 0.02 * rng.standard_normal(len(v))
    F, P, rmse = cf.fit_force_power(v, y)
    assert F == pytest.approx(0.9, abs=0.02)
    assert P == pytest.approx(6.0, abs=0.2)
    assert rmse < 0.03


def test_fit_forms_is_monotone_and_fills_missing_notches():
    rng = np.random.default_rng(1)
    v = rng.uniform(1.0, 14.0, 6000)
    n = rng.choice([-2, -1, 1, 3], len(v))                    # notch 2 traction and 3 brake missing
    true = {1: (0.4, 3.0), 3: (0.9, 7.0)}
    y = np.where(n > 0, [np.minimum(*((true[k][0], true[k][1] / vv))) if k > 0 else 0
                         for k, vv in zip(n, v)], np.where(n == -1, -0.5, -1.0))
    q = cf.fit_forms(v, np.asarray(y, float), n, notch_max=3)
    assert q['F'][2] == q['F'][1]                               # missing notch takes the lower one
    assert np.all(np.diff(q['F']) >= 0) and np.all(np.diff(q['B']) >= 0)
    assert q['B'][1] == pytest.approx(0.5, abs=1e-6) and q['B'][2] == pytest.approx(1.0, abs=1e-6)
    assert q['B'][3] == q['B'][2]                                # missing brake notch: lower one


def test_form_accel_sign_drag_and_adhesion():
    q = {'F': [0, 0.5, 1.0], 'P': [0.5, 5.0, 10.0], 'B': [0, 0.6, 3.0]}
    c = (0.01, 0.0, 0.0)
    assert cf.form_accel(0, 5.0, q, c, 1.5) == pytest.approx(-0.01)
    assert cf.form_accel(1, 2.0, q, c, 1.5) == pytest.approx(0.5 - 0.01)          # force branch
    assert cf.form_accel(2, 20.0, q, c, 1.5) == pytest.approx(0.5 - 0.01)         # power branch 10/20
    assert cf.form_accel(-2, 5.0, q, c, 1.5) == pytest.approx(-1.5 - 0.01)        # adhesion cap
    assert cf.form_accel(1, 0.0, q, c, 1.5) == pytest.approx(0.5 - 0.01)          # no P/0 blow-up


def test_ensure_utf8_stdout_survives_a_narrow_console_encoding(monkeypatch):
    """md() prints 'м/с²'; on a console whose default codepage is not UTF-8 (Windows,
    cp1251) that must not crash (#132, same class of bug as #124/#126)."""
    import io
    narrow = io.TextIOWrapper(io.BytesIO(), encoding='ascii')
    monkeypatch.setattr(cf.sys, 'stdout', narrow)
    cf.ensure_utf8_stdout()
    print('м/с²')  # raises UnicodeEncodeError on the original ascii-encoded stream
    narrow.flush()
    assert narrow.buffer.getvalue()
