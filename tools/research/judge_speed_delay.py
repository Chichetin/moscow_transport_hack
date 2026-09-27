"""D-095: /result/velocity through the node's DelayLine against the judge's reference.

    .venv/bin/python tools/research/judge_speed_delay.py [<bag with /localization/kinematic_state>]
    .venv/bin/python tools/research/judge_speed_delay.py --holdout

The first form runs the core as tools/eval does (all inputs, and without the controller as in
the judge's image, D-093), passes the speed through DelayLine as the node does and pairs it with
the reference like coord/judge_cmp.py: nearest reference stamp within 0.05 s (the judge's slop).
`--holdout` gives the cost of the delay against the tools/eval reference (GNSS master vel, which
does not lag): median speed metrics of the holdout without and with the delay.
"""
import dataclasses
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'tools' / 'eval'))
sys.path.insert(0, str(REPO / 'src' / 'tram_odometry_core'))
from rosbags.highlevel import AnyReader  # noqa: E402
from tram_eval import bag as bagmod  # noqa: E402
from tram_odometry_core.output import DelayLine  # noqa: E402
from tram_odometry_core.types import load_params  # noqa: E402

PARAMS = load_params(REPO / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
JUDGE_BAG = ('organizers', 'check-code', 'bags', '30618_88aea4d9')   # next to dataset/data
CMD = '/vehicle/driver_position_cmd'
SLOP_S = 0.05      # the judge's ApproximateTimeSynchronizer slop


def delayed(t, v, delay):
    line = DelayLine(delay, PARAMS.input.max_stamp_jump_s)
    return np.array([line.push(float(a), float(b)) for a, b in zip(t, v)])


def judge(path: Path) -> None:
    rt, rv, rp = [], [], []
    with AnyReader([path], default_typestore=bagmod.typestore()) as r:
        con = [c for c in r.connections if c.topic == '/localization/kinematic_state']
        for c, _, raw in r.messages(connections=con):
            m = r.deserialize(raw, c.msgtype)
            rt.append(bagmod.stamp(m))
            rv.append(m.twist.twist.linear.x)
            rp.append((m.pose.pose.position.x, m.pose.pose.position.y))
    rt, rv, rp = np.array(rt), np.array(rv), np.array(rp)
    msgs = bagmod.read_bag(path)
    window_end = bagmod.gnss_window_end(msgs, bagmod.default_gnss_window())
    for mode, feed in (('all inputs', msgs), ('no controller', [x for x in msgs if x[0] != CMD])):
        est, crash, _ = bagmod.run_pipeline(feed, bagmod.default_odometry(), window_end)
        est, _ = bagmod.finite_only(est)
        i = np.clip(np.searchsorted(rt, est.t), 1, len(rt) - 1)
        j = np.where(np.abs(rt[i - 1] - est.t) < np.abs(rt[i] - est.t), i - 1, i)
        ok = np.abs(rt[j] - est.t) <= SLOP_S

        def rmse_max(v, sel=ok):
            e = v[sel] - rv[j[sel]]
            return math.sqrt(np.mean(e ** 2)), float(np.abs(e).max())

        vd = delayed(est.t, est.speed, PARAMS.output.velocity_delay_s)
        (r0, m0), (r1, m1) = rmse_max(est.speed), rmse_max(vd)
        print(f'[{mode}] crashed={crash is not None} pairs={int(ok.sum())}: speed RMSE {r0:.4f} -> '
              f'{r1:.4f} m/s, max {m0:.3f} -> {m1:.3f} (delay {PARAMS.output.velocity_delay_s} s)')
        sweep = [f'{d:.2f}:{rmse_max(delayed(est.t, est.speed, d))[0]:.4f}'
                 for d in (0.0, 0.03, 0.06, 0.08, 0.09, 0.10, 0.12, 0.15)]
        print(f'[{mode}] delay:RMSE ' + ' '.join(sweep))
        n4 = len(est.t) // 4
        quarters = [np.arange(len(est.t)) // n4 == k for k in range(4)]
        print(f'[{mode}] quarters: ' + ' '.join(
            f'{rmse_max(est.speed, ok & q)[0]:.4f}->{rmse_max(vd, ok & q)[0]:.4f}' for q in quarters))
        on_grid = ok & est.absolute
        for d in (0.0, PARAMS.output.velocity_delay_s):   # the position is NOT delayed (D-095)
            px, py = delayed(est.t, est.pos[:, 0], d), delayed(est.t, est.pos[:, 1], d)
            e = np.hypot(px - rp[j, 0], py - rp[j, 1])[on_grid]
            print(f'[{mode}] position delayed by {d:.2f} s: 2D RMSE {math.sqrt(np.mean(e ** 2)):.3f} m')


class Delayed:
    """tools/eval odometry whose speed goes through DelayLine as /result/velocity does."""

    def __init__(self):
        self.odometry = bagmod.default_odometry()
        self.line = DelayLine(PARAMS.output.velocity_delay_s, PARAMS.input.max_stamp_jump_s)

    def step(self, raw):
        est = self.odometry.step(raw)
        if est is None or not (math.isfinite(est.t) and math.isfinite(est.speed)):
            return est
        return dataclasses.replace(est, speed=self.line.push(est.t, est.speed))


def holdout() -> None:
    paths = [bagmod.data_dir() / n for n in bagmod.load_splits()['holdout']]
    window = bagmod.default_gnss_window()
    keys = ('speed_rmse', 'speed_mae', 'speed_bias_accel', 'speed_bias_brake')
    with ProcessPoolExecutor(8) as ex:
        runs = [list(ex.map(partial(bagmod.evaluate_bag, make_odometry=make), paths,
                            [window] * len(paths))) for make in (None, Delayed)]
    for k in keys:
        base, dl = ([r[k] for r in run if r.get(k) is not None] for run in runs)
        print(f'{k}: median {np.median(base):+.4f} -> {np.median(dl):+.4f} m/s ({len(paths)} bag)')


if __name__ == '__main__':
    if sys.argv[1:] == ['--holdout']:
        holdout()
    else:
        judge(Path(sys.argv[1]) if len(sys.argv) > 1 else bagmod.data_dir().parent.joinpath(*JUDGE_BAG))
