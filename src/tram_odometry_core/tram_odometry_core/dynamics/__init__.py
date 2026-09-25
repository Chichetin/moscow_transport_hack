"""Stateless longitudinal drive model; all inputs and outputs use SI units."""
import numpy as np

from ..types import Params


def model_accel(notch: int, speed: float, params: Params) -> float:
    """Interpolate calibrated drive acceleration, limit it, then subtract drag.

    Tables and limits must satisfy the validation in ``load_params``. The caller
    supplies a finite nonnegative speed and an integer controller position.
    This is an acceleration, not a velocity integrator: at rest braking and drag
    may still return negative acceleration. The integrator enforces speed >= 0.
    Actuator lag belongs to the stateful caller, not to this pure function.
    """
    if not np.isfinite(speed) or speed < 0:
        raise ValueError('speed must be finite and nonnegative (m/s)')
    drive = params.drive
    position = min(abs(notch), drive.notch_max)
    grid = drive.speed_grid_mps
    width = len(grid)
    table = drive.traction_accel_table if notch >= 0 else drive.brake_accel_table
    start = position * width
    magnitude = float(np.interp(speed, grid, table[start:start + width]))
    magnitude = min(magnitude, drive.adhesion_accel_mps2)
    if notch > 0 and speed > 0:
        magnitude = min(magnitude, drive.traction_power_w_per_kg / speed)
    signed_drive = magnitude if notch >= 0 else -magnitude
    resistance = params.resistance
    drag = resistance.c0 + resistance.c1 * speed + resistance.c2 * speed * speed
    return float(signed_drive - drag)
