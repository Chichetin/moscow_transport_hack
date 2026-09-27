"""Build the route map src/tram_odometry/maps/route.csv from GNSS master of train bags (D-007).

Usage:
    .venv/bin/python tools/pathgraph/build_route.py [--data $TRAM_DATA_DIR] [--out <route.csv>]

Height z_m is ENU up of the same fixes: median over passes in 1 m bins along each branch.

Every long bag is one trip in one direction, so the route is two branches:
0 — westbound (east terminal -> east loop -> west loop), 1 — eastbound (west terminal -> east
terminal). Per branch: the longest gap-free train pass is the reference; all passes of that
direction are projected onto it and the reference moves by the median lateral offset over
passes, a few times. Where the passes since RELAID_SINCE agree on a track more than MAX_SPREAD_M
away from that median, the track was relaid and their median wins (D-096). Only train bags
(tools/eval/splits.yaml) and GBAS fixes are used.
"""
from __future__ import annotations

import argparse
import datetime
import os
import subprocess
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[2]
SPLITS = REPO / 'tools' / 'eval' / 'splits.yaml'
OUT = REPO / 'src' / 'tram_odometry' / 'maps' / 'route.csv'
MASTER_FIX = '/sensing/gnss/master/fix'

ORIGIN = (55.8104, 37.4623, 168.0)  # lat deg, lon deg, alt m: fixed map origin, east terminal
WGS84_A = 6378137.0                 # m
WGS84_E2 = 6.69437999014e-3
MAP_STATUS = 2          # NavSatFix status used: GBAS only; status 0 wanders by metres (30639)
OUTLIER_WIN = 11        # samples (~1.1 s), rolling median window
OUTLIER_M = 3.0         # m, max distance of a fix from the rolling median
MIN_SPAN_M = 1000.0     # m, east-west span for a bag to count as a pass
THIN_M = 1.0            # m, reference: drop fixes closer than this to the previous kept one
REF_SMOOTH_M = 5.0      # m, reference: moving average of coordinates (removes GNSS zig-zag)
REF_MAX_GAP_M = 40.0    # m, reference: longest allowed GNSS dropout (bridged linearly)
REF_MAX_SPEED = 20.0    # m/s, faster jumps are position steps (tram max ~16 m/s)
REF_MAX_STEP_M = 8.0    # m, reference: largest position step, must stay inside GATES_M[0]
STEP_M = 1.0            # m, output step along the branch (contract: <= 2 m)
GATES_M = (15.0, 6.0, 6.0, 6.0)  # m, per iteration: fixes farther from the branch are ignored;
                                 # the first is wide to pull reference steps back onto the passes
MIN_PASSES = 3          # passes in a bin needed to move the branch there
MAX_SPREAD_M = 1.0      # m, median |deviation| of pass offsets in a bin to move the branch:
                        # passes split between parallel tracks do not drag it midway
COVER_CELL_M = 5.0      # m, grid cell for the coverage of a track
COVER_M = 5.0           # m, a fix farther than this from every branch is uncovered
EXTRA_MIN_M = 50.0      # m, shortest uncovered piece of a pass that becomes a new branch
EXTRA_OVERLAP_M = 20.0  # m, covered track kept at both ends of a piece: joins the branches
MAX_EXTRA = 4           # extra branches (terminal tracks) at most
SHIFT_MAX_M = 10.0      # m, a piece never farther than this from a branch of its direction is
                        # that branch shifted by GNSS (a pass 5 m off under a bridge), not a track (#138)
DIR_HALF = 10           # samples (~1 s) each side: travel direction of a pass at a fix
DIR_MIN_M = 2.0         # m, shorter displacement over the window: standing, direction unknown
OFFSET_SMOOTH_M = 15.0  # m, moving average of the lateral correction along the branch
RELAID_SINCE = datetime.date(2026, 8, 10)  # UTC day of the first fix: from this day on every
                        # westbound pass drives a track ~4 m off the one of 27.07 at s ~1000-1750
                        # of branch 0 (D-096); the older majority must not keep the old track


def default_data_dir() -> Path:
    """TRAM_DATA_DIR, else <main checkout>/dataset/data (works from any worktree)."""
    env = os.environ.get('TRAM_DATA_DIR')
    if env:
        return Path(env) if Path(env).is_absolute() else REPO / env
    common = subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--path-format=absolute',
                             '--git-common-dir'], capture_output=True, text=True).stdout.strip()
    return (Path(common).parent if common else REPO) / 'dataset' / 'data'


def split_bags(path: Path, key: str) -> list[str]:
    # BaseLoader: safe_load reads 30618_68847170 as an int (YAML 1.1 allows '_' in numbers)
    return list(yaml.load(Path(path).read_text(encoding='utf-8'), Loader=yaml.BaseLoader)[key])


def _ecef(lat, lon, alt):
    la, lo = np.radians(lat), np.radians(lon)
    n = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(la) ** 2)
    return ((n + alt) * np.cos(la) * np.cos(lo), (n + alt) * np.cos(la) * np.sin(lo),
            (n * (1 - WGS84_E2) + alt) * np.sin(la))


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """WGS84 geodetic -> local ENU at (lat0, lon0, alt0), metres."""
    x, y, z = _ecef(lat, lon, alt)
    x0, y0, z0 = _ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    la, lo = np.radians(lat0), np.radians(lon0)
    e = -np.sin(lo) * dx + np.cos(lo) * dy
    n = -np.sin(la) * np.cos(lo) * dx - np.sin(la) * np.sin(lo) * dy + np.cos(la) * dz
    u = np.cos(la) * np.cos(lo) * dx + np.cos(la) * np.sin(lo) * dy + np.sin(la) * dz
    return e, n, u


def reject_outliers(xy: np.ndarray, win: int, max_dev: float) -> np.ndarray:
    """Mask of fixes within max_dev of the rolling median of their neighbours."""
    if len(xy) < win:
        return np.ones(len(xy), bool)
    pad = win // 2
    padded = np.pad(xy, ((pad, pad), (0, 0)), mode='edge')
    med = np.median(np.lib.stride_tricks.sliding_window_view(padded, win, axis=0), axis=-1)
    return np.hypot(*(xy - med).T) <= max_dev


def resample(xy: np.ndarray, step: float) -> tuple[np.ndarray, np.ndarray]:
    """Uniform resampling by arc length with spacing <= step: (s, xy)."""
    seg = np.hypot(*np.diff(xy, axis=0).T)
    xy = xy[np.concatenate([[True], seg > 0])]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    st = np.linspace(0.0, s[-1], int(np.ceil(s[-1] / step)) + 1)
    return st, np.column_stack([np.interp(st, s, xy[:, 0]), np.interp(st, s, xy[:, 1])])


def project(s: np.ndarray, poly: np.ndarray, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Arc length and signed lateral offset (left of travel is +) of pts on the polyline."""
    _, i = cKDTree(poly).query(pts)
    best_s = np.zeros(len(pts))
    best_e = np.full(len(pts), np.inf)
    for j in (np.clip(i - 1, 0, len(poly) - 2), np.clip(i, 0, len(poly) - 2)):
        a, t = poly[j], poly[j + 1] - poly[j]
        length = np.hypot(*t.T)
        rel = pts - a
        u = np.clip((rel * t).sum(1) / length ** 2, 0.0, 1.0)
        d = np.hypot(*(rel - t * u[:, None]).T)
        side = np.sign(t[:, 0] * rel[:, 1] - t[:, 1] * rel[:, 0])
        better = d < np.abs(best_e)
        best_s[better] = s[j][better] + (u * length)[better]
        best_e[better] = np.where(side == 0, 1.0, side)[better] * d[better]
    return best_s, best_e


def _moving_average(v: np.ndarray, width: int) -> np.ndarray:
    if width <= 1:
        return v
    pad = width // 2
    padded = np.pad(v, [(pad, width - 1 - pad)] + [(0, 0)] * (v.ndim - 1), mode='edge')
    kernel = np.ones(width) / width
    if v.ndim == 1:
        return np.convolve(padded, kernel, mode='valid')
    return np.column_stack([np.convolve(padded[:, k], kernel, mode='valid')
                            for k in range(v.shape[1])])


def _left_normal(poly: np.ndarray) -> np.ndarray:
    t = np.column_stack([np.gradient(poly[:, 0]), np.gradient(poly[:, 1])])
    t /= np.hypot(*t.T)[:, None]
    return np.column_stack([-t[:, 1], t[:, 0]])


def reference_track(xy: np.ndarray, step: float) -> np.ndarray:
    """One pass -> smooth polyline: thin standing jitter, resample, average coordinates."""
    kept = [xy[0]]
    for p in xy[1:]:
        if np.hypot(*(p - kept[-1])) >= THIN_M:
            kept.append(p)
    _, poly = resample(np.asarray(kept), step)
    return _moving_average(poly, max(1, int(round(REF_SMOOTH_M / step))))


def _pass_day(t: np.ndarray) -> datetime.date:
    return datetime.datetime.fromtimestamp(float(t[0]), datetime.timezone.utc).date()


def refine_branch(ref: np.ndarray, passes: list[np.ndarray], step: float, gates: tuple,
                  min_passes: int, smooth_m: float, recent: np.ndarray | None = None):
    """Move the reference by the median over passes of their mean lateral offset per bin.

    recent: mask of passes on the current track layout; in a bin where at least min_passes of
    them agree (MAX_SPREAD_M) on an offset more than MAX_SPREAD_M from the median of all passes,
    and within smooth_m of such a bin, their median (interpolated over bins where they do not
    agree) is used: the track was relaid, older passes drive a track that is gone."""
    s, poly = resample(ref, step)
    for gate in gates:
        ds = s[1] - s[0]
        per_pass = np.full((len(passes), len(s)), np.nan)
        for k, pts in enumerate(passes):
            ps, pe = project(s, poly, pts)
            ok = (np.abs(pe) <= gate) & (ps > 0) & (ps < s[-1])  # ends: clipped projections
            b = np.rint(ps[ok] / ds).astype(int)
            cnt = np.bincount(b, minlength=len(s))
            tot = np.bincount(b, weights=pe[ok], minlength=len(s))
            per_pass[k, cnt > 0] = tot[cnt > 0] / cnt[cnt > 0]
        good = np.sum(~np.isnan(per_pass), axis=0) >= min_passes
        med = np.full(len(s), np.nan)
        med[good] = np.nanmedian(per_pass[:, good], axis=0)
        good[good] = np.nanmedian(np.abs(per_pass[:, good] - med[good]), axis=0) <= MAX_SPREAD_M
        if recent is not None:
            new = per_pass[np.asarray(recent, bool)]
            agree = np.sum(~np.isnan(new), axis=0) >= min_passes
            new_med = np.full(len(s), np.nan)
            new_med[agree] = np.nanmedian(new[:, agree], axis=0)
            agree[agree] = np.nanmedian(np.abs(new[:, agree] - new_med[agree]), axis=0) <= MAX_SPREAD_M
            relaid = agree & (np.abs(new_med - med) > MAX_SPREAD_M)
            if relaid.any():
                # the zone widened by smooth_m (the turnouts) and without holes: a bin that falls
                # back to the old median (few recent passes in a 1 m bin at speed) is a spike,
                # and the moving average turns a spike into steps that zig-zag the branch
                w = int(round(smooth_m / ds))
                relaid = np.convolve(relaid, np.ones(2 * w + 1), mode='same') > 0
                med[relaid] = np.interp(np.flatnonzero(relaid), np.flatnonzero(agree), new_med[agree])
            good |= relaid
        if not good.any():
            break
        off = np.interp(np.arange(len(s)), np.flatnonzero(good), med[good])
        off = _moving_average(off, max(1, int(round(smooth_m / ds))))
        s, poly = resample(poly + _left_normal(poly) * off[:, None], step)
    return s, poly


def coverage(xy: np.ndarray) -> int:
    """Number of COVER_CELL_M grid cells a track visits (standing jitter adds nothing)."""
    return len(np.unique(np.floor(xy / COVER_CELL_M), axis=0))


def _fast_steps(t: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """step i -> i+1 is a GNSS position jump, not motion."""
    step = np.hypot(*np.diff(xy, axis=0).T)
    return (step > OUTLIER_M) & (step / np.maximum(np.diff(t), 1e-3) > REF_MAX_SPEED)


def _travel_direction(xy: np.ndarray) -> np.ndarray:
    """Unit travel direction at each fix over +-DIR_HALF samples, 0 while standing."""
    n = len(xy)
    idx = np.arange(n)
    d = xy[np.minimum(idx + DIR_HALF, n - 1)] - xy[np.maximum(idx - DIR_HALF, 0)]
    norm = np.hypot(*d.T)
    return np.where((norm >= DIR_MIN_M)[:, None], d / np.maximum(norm, 1e-9)[:, None], 0.0)


def _directed_offset(s: np.ndarray, poly: np.ndarray, xy: np.ndarray,
                     heading: np.ndarray) -> np.ndarray:
    """|lateral offset| of fixes to a branch, inf where the pass drives against it."""
    ps, off = project(s, poly, xy)
    i = np.clip(np.searchsorted(s, ps) - 1, 0, len(poly) - 2)
    tangent = poly[i + 1] - poly[i]
    against = (heading * tangent).sum(1) < 0.0
    return np.where(against, np.inf, np.abs(off))


def uncovered_pieces(t: np.ndarray, xy: np.ndarray, branches: list) -> list[np.ndarray]:
    """Runs of a pass farther than COVER_M from every branch, reached by driving (no GNSS
    jump at their edges or inside) and at least EXTRA_MIN_M long, with EXTRA_OVERLAP_M of
    covered track on both ends."""
    # a branch covers a fix only in its own direction: a single-ended tram never drives a
    # branch backwards, so a parallel track of the other direction is a track of its own (#138)
    heading = _travel_direction(xy)
    dist = np.min([_directed_offset(s, poly, xy, heading) for s, poly in branches], axis=0)
    edges = np.flatnonzero(np.diff(np.r_[0, (dist > COVER_M).astype(int), 0]))
    fast = np.r_[_fast_steps(t, xy), False]
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    pieces = []
    for a, b in zip(edges[::2], edges[1::2]):           # uncovered run xy[a:b]
        if fast[max(a - 1, 0):b].any():
            continue
        lo = np.searchsorted(cum, cum[a] - EXTRA_OVERLAP_M)
        hi = np.searchsorted(cum, cum[b - 1] + EXTRA_OVERLAP_M, side='right')
        piece = xy[lo:hi]
        if len(piece) > 1 and np.hypot(*np.diff(reference_track(piece, STEP_M), axis=0).T).sum() >= EXTRA_MIN_M:
            pieces.append(piece)
    return pieces


def _shifted_branch(piece: np.ndarray, branches: list) -> bool:
    """The piece stays within SHIFT_MAX_M of a branch of its own direction all along."""
    heading = _travel_direction(piece)
    near = np.min([_directed_offset(s, poly, piece, heading) for s, poly in branches], axis=0)
    return bool(np.max(near) <= SHIFT_MAX_M)


def add_extra_branches(branches: list, tracks: list[tuple[np.ndarray, np.ndarray]]) -> list:
    """New branches for the track the passes drive on but the branches miss (terminal tracks)."""
    branches = list(branches)
    for _ in range(MAX_EXTRA):
        pieces = [p for t, xy in tracks for p in uncovered_pieces(t, xy, branches)]
        pieces = [p for p in pieces if not _shifted_branch(p, branches)]
        if not pieces:
            break
        ref = max(pieces, key=coverage)
        branches.append(refine_branch(reference_track(ref, STEP_M), pieces, STEP_M,
                                      GATES_M[1:], MIN_PASSES, OFFSET_SMOOTH_M))
    return branches


def branch_height(s: np.ndarray, poly: np.ndarray, passes: list[tuple[np.ndarray, np.ndarray]],
                  gate: float, min_passes: int, smooth_m: float) -> np.ndarray:
    """Height along a branch: per 1 m bin the mean z of each pass within `gate` laterally,
    median over passes (>= min_passes), gaps interpolated, moving average over smooth_m."""
    ds = s[1] - s[0]
    per_pass = np.full((len(passes), len(s)), np.nan)
    for k, (xy, z) in enumerate(passes):
        ps, pe = project(s, poly, xy)
        ok = (np.abs(pe) <= gate) & (ps > 0) & (ps < s[-1])
        b = np.rint(ps[ok] / ds).astype(int)
        cnt = np.bincount(b, minlength=len(s))
        tot = np.bincount(b, weights=z[ok], minlength=len(s))
        per_pass[k, cnt > 0] = tot[cnt > 0] / cnt[cnt > 0]
    good = np.sum(~np.isnan(per_pass), axis=0) >= min_passes
    if not good.any():
        good = np.sum(~np.isnan(per_pass), axis=0) >= 1   # single-pass terminal track
    height = np.interp(np.arange(len(s)), np.flatnonzero(good),
                       np.nanmedian(per_pass[:, good], axis=0))
    return _moving_average(height, max(1, int(round(smooth_m / ds))))


def write_route(path: Path, branches: list[tuple[np.ndarray, np.ndarray, np.ndarray]],
                header: str) -> None:
    """maps/route.csv, docs/contracts.md §5."""
    lines = [f'# {header}', 'branch,s_m,x_m,y_m,z_m']
    for b, (s, xy, z) in enumerate(branches):
        lines += [f'{b},{si:.3f},{x:.3f},{y:.3f},{zi:.3f}' for si, (x, y), zi in zip(s, xy, z)]
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def read_route(path: Path) -> dict[int, tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """branch -> (s, xy, z)."""
    rows = [ln for ln in Path(path).read_text(encoding='utf-8').splitlines()
            if ln and not ln.startswith('#') and not ln.startswith('branch')]
    a = np.array([[float(v) for v in ln.split(',')] for ln in rows])
    return {int(b): (a[a[:, 0] == b, 1], a[a[:, 0] == b, 2:4], a[a[:, 0] == b, 4])
            for b in np.unique(a[:, 0])}


def read_master_fixes(bag: Path) -> np.ndarray:
    """(stamp s, lat, lon, alt, status) of GNSS master, sorted by header.stamp."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    rows = []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        conns = [c for c in reader.connections if c.topic == MASTER_FIX]
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         m.latitude, m.longitude, m.altitude, m.status.status))
    a = np.array(rows, float).reshape(-1, 5)
    return a[np.argsort(a[:, 0], kind='stable')]


def clean_track(fixes: np.ndarray, statuses: tuple[int, ...]):
    """Stamps, ENU x/y and ENU up of fixes with an accepted status, without outliers."""
    f = fixes[np.isin(fixes[:, 4], statuses) & np.all(np.isfinite(fixes[:, 1:4]), axis=1)]
    e, n, u = lla_to_enu(f[:, 1], f[:, 2], f[:, 3], *ORIGIN)
    xy = np.column_stack([e, n])
    keep = reject_outliers(xy, OUTLIER_WIN, OUTLIER_M)
    return f[keep, 0], xy[keep], u[keep]


def reference_ok(t: np.ndarray, xy: np.ndarray) -> bool:
    """A pass fit to be the reference: no long dropouts and no large position steps."""
    step = np.hypot(*np.diff(xy, axis=0).T)
    speed = step / np.maximum(np.diff(t), 1e-3)
    return bool(step.max() <= REF_MAX_GAP_M and not np.any((step > REF_MAX_STEP_M) & (speed > REF_MAX_SPEED)))


def _load_train_track(bag: Path):
    return bag.name, clean_track(read_master_fixes(bag), (MAP_STATUS,))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=Path, default=default_data_dir())
    ap.add_argument('--out', type=Path, default=OUT)
    args = ap.parse_args()
    bags = [args.data / n for n in split_bags(SPLITS, 'train')]
    with ProcessPoolExecutor(max(1, (os.cpu_count() or 2) - 2)) as ex:
        tracks = dict(ex.map(_load_train_track, bags))
    passes: dict[int, list[tuple[str, np.ndarray, np.ndarray]]] = {0: [], 1: []}
    for name, (t, xy, _) in tracks.items():
        if len(xy) > OUTLIER_WIN and abs(xy[0, 0] - xy[-1, 0]) >= MIN_SPAN_M:
            passes[0 if xy[0, 0] > xy[-1, 0] else 1].append((name, t, xy))
    branches = []
    for b in (0, 1):
        ref_name, _, ref_xy = max((p for p in passes[b] if reference_ok(p[1], p[2])),
                                  key=lambda p: coverage(p[2]))
        branches.append(refine_branch(reference_track(ref_xy, STEP_M), [xy for _, _, xy in passes[b]],
                                      STEP_M, GATES_M, MIN_PASSES, OFFSET_SMOOTH_M,
                                      np.array([_pass_day(t) >= RELAID_SINCE for _, t, _ in passes[b]])))
        print(f'branch {b}: {branches[-1][0][-1]:.0f} m, {len(passes[b])} passes, reference {ref_name}')
    usable = [tr for tr in tracks.values() if len(tr[1]) > OUTLIER_WIN]
    branches = add_extra_branches(branches, [(t, xy) for t, xy, _ in usable])
    for b, (s, poly) in enumerate(branches[2:], start=2):
        print(f'branch {b}: {s[-1]:.0f} m, extra, from ({poly[0, 0]:.0f}, {poly[0, 1]:.0f}) '
              f'to ({poly[-1, 0]:.0f}, {poly[-1, 1]:.0f})')
    branches = [(s, poly, branch_height(s, poly, [(xy, z) for _, xy, z in usable], GATES_M[-1],
                                        MIN_PASSES, OFFSET_SMOOTH_M)) for s, poly in branches]
    for b, (_, _, z) in enumerate(branches):
        print(f'branch {b}: height {z.min():.1f} .. {z.max():.1f} m')
    commit = subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--short', 'HEAD'],
                            capture_output=True, text=True).stdout.strip()
    header = (f'frame: ENU, origin_lat={ORIGIN[0]}, origin_lon={ORIGIN[1]}, origin_alt={ORIGIN[2]}, '
              f'source=GNSS master status {MAP_STATUS} of {sum(map(len, passes.values()))} train '
              f'bags (tools/eval/splits.yaml), tools/pathgraph/build_route.py @ {commit}; '
              f'branch 0 = westbound (east terminal -> west loop), '
              f'branch 1 = eastbound (west terminal -> east terminal), '
              f'branch >= 2 = terminal tracks in travel direction, start and/or end on another branch')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_route(args.out, branches, header)
    print(args.out)


if __name__ == '__main__':
    main()
