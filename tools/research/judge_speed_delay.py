"""D-095: /result/velocity through the node's DelayLine against the judge's reference.

    .venv/bin/python tools/research/judge_speed_delay.py [<bag with /localization/kinematic_state>]
    .venv/bin/python tools/research/judge_speed_delay.py --holdout

The first form feeds the bag in record order to the core as the node does (all inputs, and
without the controller as in the judge's image, D-093), passes the speed through DelayLine and
pairs outputs with the reference like the judge: message_filters ApproximateTimeSynchronizer of
Humble (per-topic queue keyed by stamp, 100 deep, the nearest unpaired stamp strictly within
0.05 s, both messages consumed), outputs arriving at their input's record time + 1 ms. With no
delay this reproduces the judge's own run (docs/verification/2026-09-27-judge-checker.md).
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
REF = '/localization/kinematic_state'
SLOP_NS = 50_000_000     # the judge's sync tolerance
QUEUE = 100              # the judge's synchronizer queue size
LATENCY_NS = 1_000_000   # an output arrives this long after its input's record time
DELAYS = (0.0, 0.03, 0.06, 0.08, 0.09, 0.10, 0.12, 0.15)


def _ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def ats_pairs(refs, results):
    """(result, reference) value pairs of the judge's synchronizer; both lists hold
    (arrival_ns, stamp_ns, value) and are merged by arrival."""
    stream = sorted([(a, 0, k, s, v) for k, (a, s, v) in enumerate(refs)]
                    + [(a, 1, k, s, v) for k, (a, s, v) in enumerate(results)],
                    key=lambda x: (x[0], x[1], x[2]))
    queues, pairs = ({}, {}), []
    for _, qi, _, s, v in stream:
        mine, other = queues[qi], queues[1 - qi]
        mine[s] = v
        while len(mine) > QUEUE:
            del mine[min(mine)]
        best = min(other, key=lambda o: abs(o - s), default=None)
        if best is None or not abs(best - s) < SLOP_NS:
            continue
        pairs.append((v, other[best]) if qi == 1 else (other[best], v))
        del mine[s]
        del other[best]
    return pairs


def judge(path: Path) -> None:
    events = []
    with AnyReader([path], default_typestore=bagmod.typestore()) as r:
        con = [c for c in r.connections if c.topic in bagmod.INPUTS + bagmod.GNSS + (REF,)]
        for c, record_ns, raw in r.messages(connections=con):
            events.append((record_ns, c.topic, r.deserialize(raw, c.msgtype)))
    ref_v = [(a, _ns(m), m.twist.twist.linear.x) for a, t, m in events if t == REF]
    ref_p = [(a, _ns(m), (m.pose.pose.position.x, m.pose.pose.position.y))
             for a, t, m in events if t == REF]
    for mode, skip in (('all inputs', ()), ('no controller', (bagmod.CMD,))):
        odometry, outs = bagmod.default_odometry(), []
        for record_ns, topic, m in events:
            if topic == REF or topic in skip or _ns(m) <= 0:
                continue
            est = odometry.step((topic, m))
            if est is not None and all(map(math.isfinite, (est.t, est.speed, est.x, est.y))):
                outs.append((record_ns + LATENCY_NS, _ns(m), est))

        def speed_error(delay):
            line = DelayLine(delay, PARAMS.input.max_stamp_jump_s)
            res = [(a, s, line.push(e.t, e.speed)) for a, s, e in outs]
            return np.array([v - r for v, r in ats_pairs(ref_v, res)])

        base, dl = speed_error(0.0), speed_error(PARAMS.output.velocity_delay_s)
        rmse = lambda e: math.sqrt(np.mean(e ** 2))   # noqa: E731
        print(f'[{mode}] pairs={len(dl)}: speed RMSE {rmse(base):.4f} -> {rmse(dl):.4f} m/s, max '
              f'{np.abs(base).max():.3f} -> {np.abs(dl).max():.3f} '
              f'(delay {PARAMS.output.velocity_delay_s} s)')
        print(f'[{mode}] delay:RMSE ' + ' '.join(f'{d:.2f}:{rmse(speed_error(d)):.4f}' for d in DELAYS))
        print(f'[{mode}] quarters: ' + ' '.join(
            f'{rmse(a):.4f}->{rmse(b):.4f}'
            for a, b in zip(np.array_split(base, 4), np.array_split(dl, 4))))
        on_grid = [(a, s, e) for a, s, e in outs if e.position_absolute]
        for delay in (0.0, PARAMS.output.velocity_delay_s):   # the position is NOT delayed
            lx, ly = (DelayLine(delay, PARAMS.input.max_stamp_jump_s) for _ in range(2))
            res = [(a, s, (lx.push(e.t, e.x), ly.push(e.t, e.y))) for a, s, e in on_grid]
            e2 = np.array([math.hypot(v[0] - r[0], v[1] - r[1]) for v, r in ats_pairs(ref_p, res)])
            print(f'[{mode}] position delayed by {delay:.2f} s: 2D RMSE {rmse(e2):.3f} m')


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
