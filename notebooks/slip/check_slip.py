"""Acceptance numbers of the slip detector (issue #12) on train bags.

    .venv/bin/python notebooks/slip/check_slip.py            # two 30639 divergence bags + 30618 train

coverage: share of frames with |front - rear| > 1 km/h (last known values, as in docs/data.md trap 8)
that carry a slip flag / on which some bogie has trust < 1 (flag or silence);
false flags: share of moving frames (a bogie > 1 km/h) with a flag on 30618 train bags.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'tools' / 'eval'))
sys.path.insert(0, str(REPO / 'src' / 'tram_odometry_core'))
from tram_eval.bag import FRONT, REAR, data_dir, default_odometry, load_splits, read_bag, stamp  # noqa: E402

KMH = 1.0
DIVERGENT = ['30639_3b3d9eb8', '30639_44226bde']    # train, front/rear > 1 km/h in 10.6 % and 1.8 %
N_HEALTHY = 12


def run(bag):
    odo, last = default_odometry(), {FRONT: None, REAR: None}
    div = div_flag = div_trust = moving = moving_flag = frames = 0
    for topic, msg in read_bag(data_dir() / bag):
        if topic in last:
            last[topic] = (stamp(msg), msg.velocity)
        est = odo.step((topic, msg))
        if est is None:
            continue
        frames += 1
        flag = est.slip.slip_front or est.slip.slip_rear
        f, r = last[FRONT], last[REAR]
        if f and r and abs(f[1] - r[1]) > KMH:
            div += 1
            div_flag += flag
            div_trust += min(est.slip.front_trust, est.slip.rear_trust) < 1.0
        if (f or r) and max(v[1] for v in (f, r) if v) > KMH:
            moving += 1
            moving_flag += flag
    return frames, div, div_flag, div_trust, moving, moving_flag


def main():
    d = df = dt = 0
    for bag in DIVERGENT:
        _, bd, bdf, bdt, m, mf = run(bag)
        print(f'{bag}: divergent frames {bd}, flag {bdf / max(bd, 1):.1%}, trust<1 {bdt / max(bd, 1):.1%}')
        d, df, dt = d + bd, df + bdf, dt + bdt
    print(f'COVERAGE (30639 train): flag {df / max(d, 1):.1%}, trust<1 {dt / max(d, 1):.1%}   (goal >= 90 %)')
    healthy = [b for b in load_splits()['train'] if b.startswith('30618')][:N_HEALTHY]
    m = mf = 0
    for bag in healthy:
        _, _, _, _, mv, mvf = run(bag)
        m, mf = m + mv, mf + mvf
    print(f'FALSE FLAGS (30618 train, {len(healthy)} bags): {mf / max(m, 1):.2%}   (goal <= 1 %)')


if __name__ == '__main__':
    main()
