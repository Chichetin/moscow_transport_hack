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
class FilterDiagnostics:
    t: float                  # stamp of the wheel measurement, seconds
    bogie: Literal['front', 'rear']
    nis: float                # squared innovation divided by innovation variance
    accepted: bool            # whether the wheel measurement passed the NIS gate


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
    filter_diagnostics: Optional[FilterDiagnostics] = None


# --- Params: one dataclass per params.yaml section, field names == yaml keys ---

@dataclass(frozen=True)
class FramesParams:
    map: str
    base: str
    grid_zone: int            # UTM zone (north) of the MGRS grid of /result/position (D-083)
    grid_origin_e_m: float    # m, UTM easting of the corner of the grid square
    grid_origin_n_m: float    # m, UTM northing of the corner of the grid square


@dataclass(frozen=True)
class InputParams:
    wheel_speed_scale: float
    stale_timeout_s: float
    max_wheel_accel_mps2: float
    max_wheel_speed_mps: float
    max_stamp_jump_s: float


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
    response_delay_s: float
    use_model: bool


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
    q_bias: float
    initial_bias_var: float
    nis_gate: float
    bias_release_speed_mps: float
    pair_window_s: float
    departure_slack_mps: float
    departure_var_factor: float
    scale_min_trust: float
    scale_min_speed_mps: float
    scale_max_diff_mps: float
    scale_max_rel: float
    scale_gain: float


@dataclass(frozen=True)
class SlipParams:
    front_rear_threshold_mps: float
    model_residual_threshold_mps2: float
    noise_accel_mps2: float
    noise_hold_s: float
    freeze_min_samples: int
    freeze_dv_mps: float
    adhesion_window_s: float
    adhesion_min_accel_mps2: float
    spin_accel_mps2: float
    skid_accel_mps2: float
    readhesion_accel_mps2: float


@dataclass(frozen=True)
class PositionParams:
    map_file: str
    use_map: bool
    join_m: float
    along_drift_frac: float
    cross_std_m: float
    fix_gate_m: float
    heading_min_base_m: float
    heading_max_base_m: float
    base_ahead_m: float
    antenna_height_m: float
    side_speed_mps: float
    side_min_m: float
    side_max_m: float
    side_overrun_m: float
    anchor_std_m: float
    stop_speed_mps: float
    stop_min_s: float
    stop_snap_max_m: float
    stop_std_m: float
    scale_alpha: float
    scale_max_dev: float
    scale_min_arc_m: float
    speed_scale_prior_m: float


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
    stops: Tuple[Tuple[int, float], ...] = ()    # (branch, s) places where the tram stops


def load_route(path) -> Route:
    """Read maps/route.csv: origin from the header comment, branches in file order; stop
    places from stops.csv next to it when there is one (docs/contracts.md §5)."""
    text = Path(path).read_text(encoding='utf-8')
    header = text.splitlines()[0] if text else ''
    found = [re.search(rf'origin_{k}=([-0-9.eE+]+)', header) for k in ('lat', 'lon', 'alt')]
    if not all(found):
        raise ValueError(f'{path}: header must carry origin_lat/lon/alt (docs/contracts.md §5)')
    origin = tuple(float(m.group(1)) for m in found)
    rows = np.loadtxt(path, delimiter=',', comments='#', skiprows=2, ndmin=2)
    branches = tuple(Branch(*(rows[rows[:, 0] == b, k].copy() for k in (1, 2, 3, 4)))
                     for b in np.unique(rows[:, 0]))
    for k, b in enumerate(branches):
        if len(b.s) < 2 or not np.all(np.diff(b.s) > 0):
            raise ValueError(f'{path}: branch {k} needs >= 2 points with increasing s_m (§5)')
    stops_path = Path(path).with_name('stops.csv')
    stops = ()
    if stops_path.exists():
        rows = np.loadtxt(stops_path, delimiter=',', comments='#', skiprows=2, ndmin=2)
        stops = tuple((int(b), float(s)) for b, s in rows[:, :2])
        if any(not (0 <= b < len(branches) and 0.0 <= s <= branches[b].s[-1]) for b, s in stops):
            raise ValueError(f'{stops_path}: stop off the branches of route.csv (§5)')
    return Route(origin=origin, branches=branches, stops=stops)


def load_params(path) -> Params:
    """Read params.yaml (ROS layout `/**: ros__parameters:`) into `Params`."""
    doc = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    try:
        raw = doc['/**']['ros__parameters']
    except (KeyError, TypeError):
        raise KeyError("params file must contain '/**' -> 'ros__parameters'") from None
    params = _build(Params, raw, 'params')
    _validate_drive(params.drive)
    if not (params.input.max_wheel_speed_mps > 0):
        raise ValueError('input.max_wheel_speed_mps must be positive')
    if not (params.input.max_stamp_jump_s > 0):
        raise ValueError('input.max_stamp_jump_s must be positive')
    _validate_filter(params.filter)
    _validate_slip(params.slip)
    _validate_side(params.position)
    _validate_base_link(params.position)
    return params


def _validate_slip(slip: SlipParams) -> None:
    if not (0.0 < slip.adhesion_window_s <= 1.0):
        raise ValueError('slip.adhesion_window_s must be in (0, 1] s')
    for key in ('adhesion_min_accel_mps2', 'spin_accel_mps2', 'skid_accel_mps2'):
        if not (getattr(slip, key) > 0.0):
            raise ValueError(f'slip.{key} must be positive')
    if not (slip.readhesion_accel_mps2 >= 0.0):
        raise ValueError('slip.readhesion_accel_mps2 must be nonnegative')


def _validate_side(position: PositionParams) -> None:
    if not (position.side_speed_mps > 0):
        raise ValueError('position.side_speed_mps must be positive')
    if not (0 <= position.side_min_m <= position.side_max_m):
        raise ValueError('position.side_min_m and side_max_m must satisfy 0 <= min <= max')
    if not (position.side_overrun_m > 0):
        raise ValueError('position.side_overrun_m must be positive')


def _validate_base_link(position: PositionParams) -> None:
    """The output point of D-077: ahead of master along the track, below the antennas; the
    online wheel scale divides the offset, so it must stay away from 0."""
    if not (position.base_ahead_m >= 0):
        raise ValueError('position.base_ahead_m must be nonnegative')
    if not math.isfinite(position.antenna_height_m):
        raise ValueError('position.antenna_height_m must be finite')
    if not (0 <= position.scale_max_dev < 1):
        raise ValueError('position.scale_max_dev must satisfy 0 <= dev < 1')
    if not (position.speed_scale_prior_m > 0):
        raise ValueError('position.speed_scale_prior_m must be positive')


def _validate_filter(filt: FilterParams) -> None:
    """Reject filter covariances and gates that cannot be used safely."""
    for name in ('q_accel', 'r_wheel', 'initial_bias_var', 'nis_gate'):
        if getattr(filt, name) <= 0.0:
            raise ValueError(f'filter.{name} must be positive')
    if filt.q_bias < 0.0:
        raise ValueError('filter.q_bias must be nonnegative')
    for name in ('bias_release_speed_mps', 'pair_window_s', 'scale_min_speed_mps',
                 'scale_max_diff_mps'):
        if getattr(filt, name) <= 0.0:
            raise ValueError(f'filter.{name} must be positive')
    if filt.departure_slack_mps < 0.0:
        raise ValueError('filter.departure_slack_mps must be nonnegative')
    if filt.departure_var_factor < 1.0:
        raise ValueError('filter.departure_var_factor must be at least 1')
    for name in ('scale_min_trust', 'scale_gain'):
        if not 0.0 < getattr(filt, name) <= 1.0:
            raise ValueError(f'filter.{name} must be in (0, 1]')
    if not 0.0 < filt.scale_max_rel < 1.0:
        raise ValueError('filter.scale_max_rel must be in (0, 1)')


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
    if not (drive.response_delay_s >= 0):
        raise ValueError('drive.response_delay_s must be nonnegative')
    expected = (drive.notch_max + 1) * len(grid)
    for name in ('traction_accel_table', 'brake_accel_table'):
        table = getattr(drive, name)
        if len(table) != expected or any(value < 0 for value in table):
            raise ValueError(f'drive.{name} must contain {expected} nonnegative values')
        if any(value != 0 for value in table[:len(grid)]):
            raise ValueError(f'drive.{name} must have a zero notch row')
