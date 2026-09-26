import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools' / 'speed_residual'))

from features import FEATURES, INDEX, NAN, CorrectedOdometry, FeatureTap  # noqa: E402
from tree_model import ObliviousTrees                                      # noqa: E402
from tram_odometry_core.pipeline import Odometry                           # noqa: E402
from tram_odometry_core.types import load_params                           # noqa: E402

PARAMS = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
FRONT = '/vehicle/front_bogie_velocity'
REAR = '/vehicle/rear_bogie_velocity'
CMD = '/vehicle/driver_position_cmd'


def _hdr(t):
    sec = math.floor(t)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=round((t - sec) * 1e9)))


def wheel(topic, t, kmh):
    return topic, SimpleNamespace(header=_hdr(t), velocity=kmh)


def cmd(t, notch):
    return CMD, SimpleNamespace(header=_hdr(t), position=notch)


def cruise(t_end=5.0, kmh=36.0):
    msgs = []
    for k in range(int(t_end * 10)):
        t = 0.1 * k
        msgs += [cmd(t, 1), wheel(FRONT, t, kmh), wheel(REAR, t, kmh), cmd(t + 0.05, 1)]
    return msgs


def test_tree_model_leaf_index_and_bias(tmp_path):
    # split i sets bit i when x[f] > border; leaf values indexed by that number
    model = {'oblivious_trees': [
        {'splits': [{'float_feature_index': 0, 'border': 1.0}, {'float_feature_index': 1, 'border': 0.0}],
         'leaf_values': [0.0, 1.0, 2.0, 3.0]},
        {'splits': [{'float_feature_index': 1, 'border': 5.0}], 'leaf_values': [10.0, 20.0]},
    ], 'scale_and_bias': [0.5, [0.25]]}
    path = tmp_path / 'm.json'
    path.write_text(json.dumps(model))
    m = ObliviousTrees(path)
    assert m([2.0, -1.0]) == pytest.approx(0.5 * (1.0 + 10.0) + 0.25)
    assert m([0.0, 1.0]) == pytest.approx(0.5 * (2.0 + 10.0) + 0.25)
    assert m([2.0, 6.0]) == pytest.approx(0.5 * (3.0 + 20.0) + 0.25)


def test_features_use_only_wheel_samples_stamped_at_or_before_the_estimate():
    odo, tap = Odometry(PARAMS), FeatureTap()
    msgs = cruise(2.0) + [wheel(FRONT, 2.4, 36.5)]
    for raw in msgs:
        tap.observe(odo, odo.step(raw))
    est = odo.step(cmd(2.1, 1))       # stamped behind the newest front sample (2.4)
    x = tap.observe(odo, est)
    assert est.t == pytest.approx(2.1)
    newest, previous = tap.wheel['front'][-1], tap.wheel['front'][-2]
    assert newest[0] == pytest.approx(2.4) and previous[0] == pytest.approx(1.9)
    assert x[INDEX['wf']] == previous[1] != newest[1]
    assert x[INDEX['age_f']] == pytest.approx(2.1 - 1.9)
    assert x[INDEX['lag']] == pytest.approx(0.3)
    assert len(x) == len(FEATURES) and all(math.isfinite(v) for v in x)


def test_corrected_speed_is_clipped_and_never_negative():
    base = Odometry(PARAMS)
    up = CorrectedOdometry(Odometry(PARAMS), lambda x: 1.0, clip=0.03)
    down = CorrectedOdometry(Odometry(PARAMS), lambda x: -100.0, clip=math.inf)
    for raw in cruise():
        b, u, d = base.step(raw), up.step(raw), down.step(raw)
        if b is None:
            assert u is None and d is None
            continue
        assert u.speed == pytest.approx(b.speed + 0.03)
        assert d.speed == 0.0
        assert (u.x, u.y, u.distance) == (b.x, b.y, b.distance)   # output only: position untouched


def test_missing_history_is_the_sentinel():
    odo, tap = Odometry(PARAMS), FeatureTap()
    x = tap.observe(odo, odo.step(wheel(FRONT, 0.0, 36.0)))
    assert x[INDEX['wr']] == NAN and x[INDEX['n0']] == NAN and x[INDEX['sf1']] == NAN
