"""Core types and parameters (docs/contracts.md, section 2). SI units, time in seconds.

Python 3.10 compatible (ROS 2 Humble). Imports the standard library, numpy and PyYAML
(apt python3-yaml); the only place that reads params.yaml is `load_params`, the only place
that reads maps/route.csv is `load_route`.
"""
import dataclasses
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Optional, Tuple, Union, get_type_hints

import numpy as np
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
    z: float                  # m, frame map (ENU up)
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
    traction_accel_table: Tuple[float, ...]
    brake_accel_table: Tuple[float, ...]


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
    join_m: float
    along_drift_frac: float
    cross_std_m: float


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


@dataclass(frozen=True)
class Branch:                 # one directed track of maps/route.csv (docs/contracts.md §5)
    s: Any                    # (N,) m, arc length, uniform step, starts at 0
    x: Any                    # (N,) m, ENU of the map origin
    y: Any
    z: Any


@dataclass(frozen=True)
class Route:
    origin: Tuple[float, float, float]    # lat deg, lon deg, alt m of the map ENU
    branches: Tuple[Branch, ...]


def load_route(path) -> Route:
    """Read maps/route.csv: origin from the header comment, branches in file order."""
    text = Path(path).read_text()
    header = text.splitlines()[0]
    origin = tuple(float(re.search(rf'origin_{k}=([-0-9.eE+]+)', header).group(1))
                   for k in ('lat', 'lon', 'alt'))
    rows = np.loadtxt(path, delimiter=',', comments='#', skiprows=2, ndmin=2)
    branches = tuple(Branch(*(rows[rows[:, 0] == b, k].copy() for k in (1, 2, 3, 4)))
                     for b in np.unique(rows[:, 0]))
    return Route(origin=origin, branches=branches)


def load_params(path) -> Params:
    """Read params.yaml (ROS layout `/**: ros__parameters:`) into `Params`."""
    doc = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    try:
        raw = doc['/**']['ros__parameters']
    except (KeyError, TypeError):
        raise KeyError("params file must contain '/**' -> 'ros__parameters'") from None
    return _build(Params, raw, 'params')
