import dataclasses
from pathlib import Path

import pytest
import yaml

from tram_odometry_core import types as T
from tram_odometry_core.pipeline import Odometry

ROOT = Path(__file__).resolve().parents[3]
PARAMS_YAML = ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml'


def _yaml_leaves(d, prefix=()):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from _yaml_leaves(v, prefix + (k,))
        else:
            yield prefix + (k,), v


def _params_leaves(obj, prefix=()):
    for f in dataclasses.fields(obj):
        v = getattr(obj, f.name)
        if dataclasses.is_dataclass(v):
            yield from _params_leaves(v, prefix + (f.name,))
        else:
            yield prefix + (f.name,), v


def test_load_params_reads_every_key():
    raw = yaml.safe_load(PARAMS_YAML.read_text())['/**']['ros__parameters']
    expected = dict(_yaml_leaves(raw))
    got = dict(_params_leaves(T.load_params(PARAMS_YAML)))
    assert set(got) == set(expected)
    for key, value in expected.items():
        want = tuple(value) if isinstance(value, list) else value
        assert got[key] == want, key


def test_load_params_values_and_types():
    p = T.load_params(PARAMS_YAML)
    assert p.input.wheel_speed_scale == pytest.approx(1 / 3.6)
    assert p.gnss.init_window_s == 5.0
    assert p.position.use_map is True
    assert p.drive.notch_max == 15 and isinstance(p.drive.notch_max, int)
    assert isinstance(p.drive.traction_accel_table, tuple)
    assert p.frames.map == 'map'


def test_params_frozen():
    p = T.load_params(PARAMS_YAML)
    with pytest.raises(dataclasses.FrozenInstanceError):
        p.gnss.init_window_s = 1.0


def _write(tmp_path, mutate):
    raw = yaml.safe_load(PARAMS_YAML.read_text())
    mutate(raw['/**']['ros__parameters'])
    f = tmp_path / 'p.yaml'
    f.write_text(yaml.safe_dump(raw))
    return f


def test_load_params_missing_key_raises(tmp_path):
    f = _write(tmp_path, lambda r: r['gnss'].pop('init_window_s'))
    with pytest.raises(KeyError, match='init_window_s'):
        T.load_params(f)


def test_load_params_unknown_key_raises(tmp_path):
    f = _write(tmp_path, lambda r: r['gnss'].update(bogus=1))
    with pytest.raises(KeyError, match='bogus'):
        T.load_params(f)


def test_load_params_int_accepted_for_float(tmp_path):
    f = _write(tmp_path, lambda r: r['gnss'].update(init_window_s=5))
    assert T.load_params(f).gnss.init_window_s == 5.0


def test_inputs_are_frozen():
    w = T.WheelSample(t=1.0, bogie='front', speed=2.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        w.speed = 3.0
    for cls, args in [(T.CommandSample, (1.0, 3)),
                      (T.GnssFix, (1.0, 'master', 55.0, 37.0, 150.0, 0)),
                      (T.GnssVel, (1.0, 0.1, 0.2))]:
        assert dataclasses.fields(cls)
        assert cls(*args).t == 1.0


def test_estimate_fields_per_contract():
    names = [f.name for f in dataclasses.fields(T.Estimate)]
    assert names == ['t', 'speed', 'speed_var', 'accel', 'accel_model', 'distance',
                     'x', 'y', 'yaw', 'pos_cov', 'slip', 'gnss_used']
    names = [f.name for f in dataclasses.fields(T.SlipState)]
    assert names == ['front_trust', 'rear_trust', 'slip_front', 'slip_rear', 'adhesion_est']


def test_odometry_stub_returns_none():
    odo = Odometry(T.load_params(PARAMS_YAML))
    assert odo.step(object()) is None


@pytest.mark.parametrize('section,key,bad', [
    ('gnss', 'init_window_s', 'five'),
    ('gnss', 'init_window_s', float('nan')),
    ('gnss', 'init_window_s', True),
    ('drive', 'notch_max', 15.5),
    ('drive', 'notch_max', True),
    ('position', 'use_map', 1),
    ('gnss', 'topic_fix', 3),
    ('drive', 'traction_accel_table', []),
    ('drive', 'traction_accel_table', [0.0, 'x']),
    ('drive', 'brake_accel_table', [float('inf')]),
])
def test_load_params_bad_type_raises(tmp_path, section, key, bad):
    f = _write(tmp_path, lambda r: r[section].update({key: bad}))
    with pytest.raises(TypeError, match=key):
        T.load_params(f)


def test_load_params_wrong_layout_raises(tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text('foo: 1\n')
    with pytest.raises(KeyError, match='ros__parameters'):
        T.load_params(f)
