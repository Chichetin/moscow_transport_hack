"""--plot: one run of a bag on one time axis, and x/y over the map (#16).

Timeline: speed (estimate, reference, both bogies as recorded), speed error, along/cross
error, driver controller, slip flags. Map: the route of maps/route.csv, reference and
estimate in the frame of the eval reference (ENU of `Reference.origin`), where the metrics
compare them; its origin follows the rule of frame `map` of the tracker (D-049, #70). For
debugging, docs/accuracy.md and the pitch; not part of
metrics.json (contracts §4). matplotlib is imported only in `render`: the dev image has
no matplotlib (docker/Dockerfile, D-039).
"""
from __future__ import annotations

import sys
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from . import bag as bagmod
from .bag import CMD, FRONT, REAR
from .metrics import Estimates, extend_track, match_nearest, project_track
from .reference import Reference, build_reference, geodetic_to_ecef

EST, REF, FRONT_C, REAR_C, SLIP_C = '#2a78d6', '#0b0b0b', '#eb6834', '#1baf7a', '#e34948'
GRID_C, MAP_C = '#d9d8d4', '#b8b7b2'


@dataclass
class Series:
    name: str
    t0: float                   # s, stamp of the first message: x axis starts here
    wheel_t: dict               # 'front'/'rear' -> (N,) s, header.stamp
    wheel_v: dict               # 'front'/'rear' -> (N,) m/s, as recorded (outliers kept)
    cmd_t: np.ndarray           # (K,) s
    cmd: np.ndarray             # (K,) controller notch, + traction, - brake
    est: Estimates
    ref: Reference
    speed_err_t: np.ndarray     # s, reference vel stamps matched to an estimate
    speed_err: np.ndarray       # m/s, estimate - reference
    pos_err_t: np.ndarray       # s, reference fix stamps matched to an estimate
    along_err: np.ndarray       # m, arc of estimate - arc of reference: + = estimate ahead
    cross_err: np.ndarray       # m, distance to the reference track
    route: list                 # (M, 2) x/y of every map branch in the frame of the reference
    crash: str | None


def wheel_speed_scale() -> float:
    """input.wheel_speed_scale of params.yaml: the core's km/h -> m/s factor (D-003)."""
    params = yaml.safe_load(bagmod.PARAMS_YAML.read_text(encoding='utf-8'))
    return float(params['/**']['ros__parameters']['input']['wheel_speed_scale'])


def enu_rotation(lat_deg: float, lon_deg: float) -> np.ndarray:
    """Rows: east, north, up unit vectors in ECEF."""
    la, lo = np.radians(lat_deg), np.radians(lon_deg)
    return np.array([[-np.sin(lo), np.cos(lo), 0.0],
                     [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)],
                     [np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]])


def route_in_ref_frame(route, origin) -> list:
    """x/y of the map branches (ENU of the map's own origin) in the ENU of `origin`; the same
    transform as PathTracker._set_origin, applied at the origin of the eval reference."""
    if route is None or origin is None:
        return []
    rot_run = enu_rotation(*origin[:2])
    a = rot_run @ enu_rotation(*route.origin[:2]).T
    c = rot_run @ (geodetic_to_ecef(*route.origin) - geodetic_to_ecef(*origin))
    return [(np.stack([b.x, b.y, b.z], axis=1) @ a.T + c)[:, :2] for b in route.branches]


def bag_series(msgs, gnss_window_s: float, make_odometry=None, name: str = '') -> Series:
    """Run the pipeline over the messages the way `bag.evaluate_bag` does and keep the series."""
    window_end = bagmod.gnss_window_end(msgs, gnss_window_s)
    odometry = (make_odometry or bagmod.default_odometry)()
    est, crash, _ = bagmod.run_pipeline(msgs, odometry, window_end)
    est, _ = bagmod.finite_only(est)
    # estimates come in recording order (late bursts, trap 5): lines and slip bands need time order
    order = np.argsort(est.t, kind='stable')
    est = Estimates(est.t[order], est.speed[order], est.pos[order], est.slip[order])
    ref = build_reference(*bagmod.reference_inputs(msgs), window_end)

    scale = wheel_speed_scale()
    wheel_t, wheel_v = {}, {}
    for side, topic in (('front', FRONT), ('rear', REAR)):
        rows = sorted(((bagmod.stamp(m), m.velocity) for tp, m in msgs if tp == topic), key=lambda r: r[0])
        wheel_t[side] = np.array([r[0] for r in rows], float)
        wheel_v[side] = np.array([r[1] for r in rows], float) * scale
    cmd = sorted(((bagmod.stamp(m), m.position) for tp, m in msgs if tp == CMD), key=lambda r: r[0])

    ri, ei = match_nearest(ref.vel_t, est.t)
    speed_err_t, speed_err = ref.vel_t[ri], est.speed[ei] - ref.speed[ri]
    ri, ei = match_nearest(ref.pos_t, est.t)
    along, cross = np.zeros(0), np.zeros(0)
    if len(ri):
        s_est, cross = project_track(*extend_track(ref.poly, ref.poly_s), est.pos[ei, :2], ref.pos_t[ri],
                                     ref.pos_s[ri[0]])
        along = s_est - ref.pos_s[ri]
    return Series(name, bagmod.stamp(msgs[0][1]), wheel_t, wheel_v,
                  np.array([c[0] for c in cmd], float), np.array([c[1] for c in cmd], float),
                  est, ref, speed_err_t, speed_err, ref.pos_t[ri], along, cross,
                  route_in_ref_frame(getattr(odometry, 'route', None), ref.origin), crash)


def _rmse(e) -> str:
    return f'{np.sqrt(np.mean(np.square(e))):.2f}' if len(e) else '—'


def _style(ax):
    ax.grid(True, color=GRID_C, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)


def render(s: Series, out_dir: Path) -> list[Path]:
    """<name>_timeline.png and <name>_xy.png in out_dir."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = s.t0
    title = (f'{s.name}: скорость RMSE {_rmse(s.speed_err)} м/с, along RMSE {_rmse(s.along_err)} м, '
             f'cross RMSE {_rmse(s.cross_err)} м' + ('  [модель упала]' if s.crash else ''))

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(14, 9),
                             gridspec_kw={'height_ratios': [3, 1.4, 1.4, 1]}, constrained_layout=True)
    ax = axes[0]
    ax.plot(s.wheel_t['front'] - t0, s.wheel_v['front'], color=FRONT_C, lw=0.8, alpha=0.8, label='передняя тележка')
    ax.plot(s.wheel_t['rear'] - t0, s.wheel_v['rear'], color=REAR_C, lw=0.8, alpha=0.8, label='задняя тележка')
    ax.plot(s.ref.vel_t - t0, s.ref.speed, color=REF, lw=1.4, label='эталон GNSS')
    ax.plot(s.est.t - t0, s.est.speed, color=EST, lw=1.4, label='оценка')
    if s.est.slip.any():
        for a in axes:
            a.fill_between(s.est.t - t0, 0, 1, where=s.est.slip, step='post', color=SLIP_C, alpha=0.15,
                           lw=0, transform=a.get_xaxis_transform(),
                           label='флаг проскальзывания' if a is ax else None)
    top = np.nanpercentile(np.concatenate([s.ref.speed, s.est.speed]), 99.5) if len(s.est.speed) else 1.0
    ax.set_ylim(-0.5, max(top, 1.0) * 1.25)     # a 50 m/s bogie outlier must not flatten the run
    ax.set_ylabel('скорость, м/с')
    ax.legend(loc='upper right', ncol=5, fontsize=8, frameon=False)
    ax.set_title(title, fontsize=10, loc='left')

    axes[1].plot(s.speed_err_t - t0, s.speed_err, color=EST, lw=0.9)
    axes[1].axhline(0, color=REF, lw=0.6)
    axes[1].set_ylabel('оценка − эталон,\nм/с')

    axes[2].plot(s.pos_err_t - t0, s.along_err, color=EST, lw=1.1, label='вдоль пути (+ впереди)')
    axes[2].plot(s.pos_err_t - t0, s.cross_err, color=FRONT_C, lw=1.1, label='поперёк пути')
    axes[2].axhline(0, color=REF, lw=0.6)
    axes[2].set_ylabel('ошибка\nположения, м')
    axes[2].legend(loc='upper left', ncol=2, fontsize=8, frameon=False)

    axes[3].step(s.cmd_t - t0, s.cmd, where='post', color=REF, lw=0.9)
    axes[3].axhline(0, color=GRID_C, lw=0.6)
    axes[3].set_ylabel('контроллер\n(+ тяга, − тормоз)')
    axes[3].set_xlabel('время от начала bag, с')
    for a in axes:
        _style(a)
    timeline = out_dir / f'{s.name}_timeline.png'
    fig.savefig(timeline, dpi=110)
    plt.close(fig)

    track = np.vstack([p for p in (s.ref.pos[:, :2], s.est.pos[:, :2]) if len(p)] or [np.zeros((1, 2))])
    lo, hi = track.min(axis=0), track.max(axis=0)
    pad = max(50.0, 0.1 * float((hi - lo).max()))
    lo, hi = lo - pad, hi + pad
    height = float(np.clip(12.0 * (hi[1] - lo[1]) / (hi[0] - lo[0]), 4.0, 12.0))   # the line runs east-west
    fig, ax = plt.subplots(figsize=(12, height), constrained_layout=True)
    for k, xy in enumerate(s.route):
        ax.plot(xy[:, 0], xy[:, 1], color=MAP_C, lw=2.5, alpha=0.6, label='карта route.csv' if k == 0 else None)
    ax.plot(s.ref.pos[:, 0], s.ref.pos[:, 1], color=REF, lw=1.2, label='эталон GNSS')
    ax.plot(s.est.pos[:, 0], s.est.pos[:, 1], color=EST, lw=1.2, label='оценка')
    if len(s.est.pos):
        ax.plot(*s.est.pos[0, :2], 'o', color=EST, ms=8, label='старт')
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x (восток), м')
    ax.set_ylabel('y (север), м')
    ax.set_title(title, fontsize=9, loc='left')
    ax.legend(loc='best', fontsize=8, frameon=False)
    _style(ax)
    xy = out_dir / f'{s.name}_xy.png'
    fig.savefig(xy, dpi=110)
    plt.close(fig)
    return [timeline, xy]


def plot_bag(path, gnss_window_s: float, out_dir: Path, make_odometry=None, msgs=None) -> list[Path]:
    """PNG files of one bag; make_odometry and msgs are for tests, as in `bag.evaluate_bag`."""
    name = Path(path).name
    try:   # metrics.json is already written: one bad bag must not take down the other plots
        msgs = bagmod.read_bag(Path(path)) if msgs is None else msgs
        return render(bag_series(msgs, gnss_window_s, make_odometry, name), Path(out_dir))
    except Exception:
        print(f'{name}: plot failed\n{traceback.format_exc(limit=3)}', file=sys.stderr)
        return []
