"""Drive physics in SI units, independent of the pending train calibration."""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from tram_odometry_core import dynamics
from tram_odometry_core.types import DriveParams, ResistanceParams, load_params

ROOT = Path(__file__).resolve().parents[3]
BASE = load_params(ROOT / 'src/tram_odometry/config/params.yaml')
P = replace(BASE, drive=DriveParams(
    notch_max=2, speed_grid_mps=(0., 4., 10.),
    traction_accel_table=(0., 0., 0., 1., 2., 1., 2., 4., 2.),
    brake_accel_table=(0., 0., 0., 0.5, 1., 2., 1., 2., 4.),
    adhesion_accel_mps2=3., traction_power_w_per_kg=12., response_delay_s=0.3, use_model=True),
    resistance=ResistanceParams(c0=0.02, c1=0.01, c2=0.001))


def accel(notch, speed, params=P):
    assert hasattr(dynamics, 'model_accel'), 'drive model has not been implemented'
    return dynamics.model_accel(notch, speed, params)


@pytest.mark.parametrize('speed', [0., 2., 4., 10., 20.])
def test_neutral_only_davis_resistance(speed):
    assert accel(0, speed) == pytest.approx(-(0.02 + 0.01*speed + 0.001*speed**2))


@pytest.mark.parametrize('notch,speed,drive', [
    (1, 2., 1.5), (1, 7., 1.5), (-1, 2., -0.75),
    (-1, 7., -1.5), (2, 4., 3.), (2, 10., 1.2),
    (-2, 10., -3.), (1, 20., 0.6), (-1, 20., -2.),
    (2, 0., 2.), (-2, 0., -1.), (99, 4., 3.), (-99, 4., -2.),
])
def test_table_interpolation_limits_and_sign(notch, speed, drive):
    assert accel(notch, speed) == pytest.approx(drive - (0.02 + .01*speed + .001*speed**2))


def test_monotonic_traction_for_monotonic_surface():
    for speed in np.linspace(0., 30., 101):
        values = [accel(n, speed) for n in range(3)]
        assert values == sorted(values)


@pytest.mark.parametrize('notch', [-2, -1, 0, 1, 2])
def test_continuity_at_grid_and_limit_boundaries(notch):
    for speed in [4., 10., 12., 20.]:
        assert abs(accel(notch, speed - 1e-7) - accel(notch, speed + 1e-7)) < 1e-6
    assert abs(accel(notch, 0.) - accel(notch, 1e-7)) < 1e-6


def test_braking_is_independent_of_traction_power():
    low_power = replace(P, drive=replace(P.drive, traction_power_w_per_kg=0.1))
    assert accel(-2, 10., low_power) == accel(-2, 10.)


@pytest.mark.parametrize('speed', [-1., np.nan, np.inf, -np.inf])
def test_invalid_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        accel(1, speed)


def test_calibrated_tables_of_params_yaml():
    """D-033 numbers: neutral is drag only, full traction at 5 m/s is the table row, full
    brake is the table row capped by the adhesion limit (the limit is not below it, D-070)."""
    d = BASE.drive
    assert accel(0, 5., BASE) == pytest.approx(-BASE.resistance.c0)
    width = len(d.speed_grid_mps)
    row = d.traction_accel_table[15 * width:16 * width]
    expected = min(float(np.interp(5., d.speed_grid_mps, row)), d.adhesion_accel_mps2,
                   d.traction_power_w_per_kg / 5.)
    assert accel(15, 5., BASE) == pytest.approx(expected - BASE.resistance.c0)
    assert 0.5 < accel(15, 5., BASE) < d.adhesion_accel_mps2
    brake = float(np.interp(12., d.speed_grid_mps, d.brake_accel_table[15 * width:16 * width]))
    assert brake <= d.adhesion_accel_mps2
    assert accel(-15, 12., BASE) == pytest.approx(-brake - BASE.resistance.c0)


def test_negative_response_delay_is_rejected():
    from tram_odometry_core.types import _validate_drive
    with pytest.raises(ValueError):
        _validate_drive(replace(P.drive, response_delay_s=-0.1))

