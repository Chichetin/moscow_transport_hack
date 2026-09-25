"""Public filter parameter and diagnostic contract for ES1."""
import dataclasses

import pytest

from tram_odometry_core import types as T

from test_types import PARAMS_YAML, _write


def test_filter_noise_parameters_are_finite_and_present():
    p = T.load_params(PARAMS_YAML)
    assert p.filter.q_accel > 0
    assert p.filter.r_wheel > 0
    assert p.filter.q_bias >= 0
    assert p.filter.initial_bias_var > 0
    assert p.filter.nis_gate > 0


@pytest.mark.parametrize('key', ['q_bias', 'initial_bias_var', 'nis_gate'])
def test_filter_parameter_is_required(tmp_path, key):
    path = _write(tmp_path, lambda p: p['filter'].pop(key))
    with pytest.raises(KeyError, match=key):
        T.load_params(path)


@pytest.mark.parametrize('key,value', [
    ('q_accel', 0.0), ('r_wheel', 0.0), ('initial_bias_var', 0.0),
    ('nis_gate', 0.0), ('q_bias', -0.1),
])
def test_filter_rejects_invalid_covariance_or_gate(tmp_path, key, value):
    path = _write(tmp_path, lambda p: p['filter'].__setitem__(key, value))
    with pytest.raises(ValueError, match=key):
        T.load_params(path)


def test_filter_diagnostic_has_measurement_identity():
    diag = T.FilterDiagnostics(t=12.5, bogie='rear', nis=4.0, accepted=False)
    assert dataclasses.is_dataclass(diag)
    assert (diag.t, diag.bogie, diag.nis, diag.accepted) == (12.5, 'rear', 4.0, False)
    with pytest.raises(dataclasses.FrozenInstanceError):
        diag.accepted = True


def test_estimate_filter_diagnostic_is_optional():
    assert dataclasses.fields(T.Estimate)[-1].name == 'filter_diagnostics'
    assert dataclasses.fields(T.Estimate)[-1].default is None
