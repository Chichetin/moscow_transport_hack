"""Core types and parameters (docs/contracts.md, section 2). SI units, time in seconds.

Python 3.10 compatible (ROS 2 Humble). Imports only the standard library and PyYAML
(apt python3-yaml); the only place that reads params.yaml is `load_params`.
"""
import dataclasses
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional, Tuple, Union, get_type_hints

import yaml


@dataclass(frozen=True)
class WheelSample:            # after preprocess: m/s
    t: float
    bogie: Literal['front', 'rear']
    speed: float


@dataclass(frozen=True)
class CommandSample:
    t: float
    notch: int                # -15..15


@dataclass(frozen=True)
class GnssFix:
    t: float
    antenna: Literal['master', 'rover']
    lat: float
    lon: float
    alt: float
    status: int


@dataclass(frozen=True)
class GnssVel:
    t: float
    ve: float                 # ENU, m/s
    vn: float


Sample = Union[WheelSample, CommandSample, GnssFix, GnssVel]


@dataclass(frozen=True)
class SlipState:
    front_trust: float        # 0..1, weight of the bogie measurement in the filter
    rear_trust: float
    slip_front: bool
    slip_rear: bool
    adhesion_est: Optional[float]


@dataclass(frozen=True)
class Estimate:
    t: float                  # = t of the input that triggered the update
    speed: float              # m/s, >= 0
    speed_var: float          # (m/s)^2
    accel: float              # m/s^2, estimate
    accel_model: float        # m/s^2, drive-model prediction
    distance: float           # m, path since the start of the run
    x: float                  # m, frame map
    y: float
    yaw: float                # rad, ENU
    pos_cov: Tuple[float, float, float]   # var_x, var_y, cov_xy
    slip: SlipState
    gnss_used: bool


# --- Params: one dataclass per params.yaml section, field names == yaml keys ---

@dataclass(frozen=True)
class FramesParams:
    map: str
    base: str


@dataclass(frozen=True)
class InputParams:
    wheel_speed_scale: float
    stale_timeout_s: float
    max_wheel_accel_mps2: float


@dataclass(frozen=True)
class VehicleParams:
    mass_kg: float
    wheel_scale_front: float
    wheel_scale_rear: float


@dataclass(frozen=True)
class DriveParams:
    notch_max: int
    speed_grid_mps: Tuple[float, ...]
    traction_accel_table: Tuple[float, ...]
    brake_accel_table: Tuple[float, ...]
    adhesion_accel_mps2: float
    traction_power_w_per_kg: float


@dataclass(frozen=True)
class ResistanceParams:
    c0: float
    c1: float
    c2: float


@dataclass(frozen=True)
class FilterParams:
    rate_hz: float
    q_accel: float
    r_wheel: float


@dataclass(frozen=True)
class SlipParams:
    front_rear_threshold_mps: float
    model_residual_threshold_mps2: float


@dataclass(frozen=True)
class PositionParams:
    map_file: str
    use_map: bool


@dataclass(frozen=True)
class GnssParams:
    init_window_s: float
    topic_fix: str
    topic_vel: str


@dataclass(frozen=True)
class Params:
    frames: FramesParams
    input: InputParams
    vehicle: VehicleParams
    drive: DriveParams
    resistance: ResistanceParams
    filter: FilterParams
    slip: SlipParams
    position: PositionParams
    gnss: GnssParams


def _build(cls, raw, path):
    """Build dataclass `cls` from dict `raw`; missing and unknown keys are errors."""
    if not isinstance(raw, dict):
        raise TypeError(f'{path}: expected a mapping, got {type(raw).__name__}')
    hints = get_type_hints(cls)
    names = {f.name for f in dataclasses.fields(cls)}
    unknown = set(raw) - names
    if unknown:
        raise KeyError(f'{path}: unknown keys {sorted(unknown)}')
    kwargs = {}
    for name in sorted(names):
        key = f'{path}.{name}'
        if name not in raw:
            raise KeyError(f'{key}: missing in params')
        hint, value = hints[name], raw[name]
        if dataclasses.is_dataclass(hint):
            kwargs[name] = _build(hint, value, key)
        elif hint is float:
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                raise TypeError(f'{key}: expected finite float, got {value!r}')
            kwargs[name] = float(value)
        elif hint is int:
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f'{key}: expected int, got {value!r}')
            kwargs[name] = value
        elif hint is bool:
            if not isinstance(value, bool):
                raise TypeError(f'{key}: expected bool, got {value!r}')
            kwargs[name] = value
        elif hint is str:
            if not isinstance(value, str):
                raise TypeError(f'{key}: expected str, got {value!r}')
            kwargs[name] = value
        elif hint == Tuple[float, ...]:
            if (not isinstance(value, (list, tuple)) or not value
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                               and math.isfinite(v) for v in value)):
                raise TypeError(f'{key}: expected a non-empty list of finite floats, got {value!r}')
            kwargs[name] = tuple(float(v) for v in value)
        else:
            raise TypeError(f'{key}: unsupported field type {hint}')
    return cls(**kwargs)


def load_params(path) -> Params:
    """Read params.yaml (ROS layout `/**: ros__parameters:`) into `Params`."""
    doc = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    try:
        raw = doc['/**']['ros__parameters']
    except (KeyError, TypeError):
        raise KeyError("params file must contain '/**' -> 'ros__parameters'") from None
    params = _build(Params, raw, 'params')
    _validate_drive(params.drive)
    return params


def _validate_drive(drive: DriveParams) -> None:
    """Reject a malformed acceleration surface before the ROS node starts."""
    grid = drive.speed_grid_mps
    if len(grid) < 2 or grid[0] != 0.0 or any(b <= a for a, b in zip(grid, grid[1:])):
        raise ValueError('drive.speed_grid_mps must start at 0 and strictly increase')
    if drive.notch_max < 1:
        raise ValueError('drive.notch_max must be positive')
    if drive.adhesion_accel_mps2 <= 0:
        raise ValueError('drive.adhesion_accel_mps2 must be positive')
    if drive.traction_power_w_per_kg <= 0:
        raise ValueError('drive.traction_power_w_per_kg must be positive')
    expected = (drive.notch_max + 1) * len(grid)
    for name in ('traction_accel_table', 'brake_accel_table'):
        table = getattr(drive, name)
        if len(table) != expected or any(value < 0 for value in table):
            raise ValueError(f'drive.{name} must contain {expected} nonnegative values')
        if any(value != 0 for value in table[:len(grid)]):
            raise ValueError(f'drive.{name} must have a zero notch row')
