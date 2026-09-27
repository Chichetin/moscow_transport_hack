"""Our pipeline against the judge's reference pose on a control bag, error by axis (#188, #186).

Usage (from the repository root):
    .venv/bin/python tools/eval/experiments/control_bag_axes.py <bag dir with /localization/kinematic_state>

The control bag of the organizers (check-code, 30618_88aea4d9) carries the judge's reference
`/localization/kinematic_state` (nav_msgs/Odometry, frame map, child base_link, 50 Hz). The
pipeline runs exactly as in tools/eval (GNSS cut after the window); every reference sample is
matched to the nearest published position within 0.05 s, like the judge's synchronizer. Printed:
x/y/z errors, along/cross split by the reference yaw, speed error, stretches with |cross| > 3 m
and how far the reference and the estimate are from the map there.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools' / 'eval'))
sys.path.insert(0, str(ROOT / 'src' / 'tram_odometry_core'))
from rosbags.highlevel import AnyReader  # noqa: E402
from tram_eval import bag as B  # noqa: E402
from tram_eval.metrics import match_nearest  # noqa: E402
from tram_odometry_core.position import geo  # noqa: E402
from tram_odometry_core.types import load_route  # noqa: E402

REF_TOPIC = '/localization/kinematic_state'
CROSS_BIG_M = 3.0


def stats(a):
    return (f'mean {np.mean(a):+.3f} median {np.median(a):+.3f} rmse {np.sqrt(np.mean(a * a)):.3f} '
            f'max|.| {np.max(np.abs(a)):.3f}')


def main():
    bagdir = Path(sys.argv[1])
    msgs, ref = [], []
    with AnyReader([bagdir], default_typestore=B.typestore()) as reader:
        for con, _, raw in reader.messages():
            m = reader.deserialize(raw, con.msgtype)
            if con.topic == REF_TOPIC:
                p, q = m.pose.pose.position, m.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                ref.append((B.stamp(m), p.x, p.y, p.z, yaw, m.twist.twist.linear.x))
            else:
                msgs.append((con.topic, m))
    ref = np.array(sorted(ref), float)
    est, crash, _ = B.run_pipeline(msgs, B.default_odometry(), B.gnss_window_end(msgs, B.default_gnss_window()))
    ok = est.absolute if est.absolute is not None else np.ones(len(est.t), bool)
    ri, ei = match_nearest(ref[:, 0], est.t[ok])
    pos, r = est.pos[ok][ei], ref[ri]
    d = pos - r[:, 1:4]
    along = d[:, 0] * np.cos(r[:, 4]) + d[:, 1] * np.sin(r[:, 4])
    cross = -d[:, 0] * np.sin(r[:, 4]) + d[:, 1] * np.cos(r[:, 4])
    print(f'crash: {crash}; pairs {len(ri)} of {len(ref)} reference samples')
    for name, a in (('x', d[:, 0]), ('y', d[:, 1]), ('z', d[:, 2]), ('along', along), ('cross', cross)):
        print(f'{name:6s} {stats(a)}')
    dist = np.linalg.norm(d, axis=1)
    print(f'3D     rmse {np.sqrt(np.mean(dist ** 2)):.3f} max {dist.max():.3f}')
    vi, vj = match_nearest(ref[:, 0], est.t)
    print(f'speed  {stats(est.speed[vj] - ref[vi, 5])}')

    route = load_route(ROOT / 'src' / 'tram_odometry' / 'maps' / 'route.csv')
    rot, e0, grid = geo.enu_rotation(*route.origin[:2]), geo.ecef(*route.origin), B.output_grid()
    branches = [np.array([geo.enu_to_grid(rot, e0, grid, float(x), float(y), float(z))[:2]
                          for x, y, z in zip(b.x[::2], b.y[::2], b.z[::2])]) for b in route.branches]

    def to_map(p):
        return min(float(np.min(np.hypot(*(b - p).T))) for b in branches)

    t = r[:, 0] - ref[0, 0]
    big = np.flatnonzero(np.abs(cross) > CROSS_BIG_M)
    print(f'|cross| > {CROSS_BIG_M} m: {len(big) / len(cross):.2%} of pairs')
    for seg in (np.split(big, np.flatnonzero(np.diff(big) > 50) + 1) if len(big) else []):
        k = seg[np.argmax(np.abs(cross[seg]))]
        mid = seg[len(seg) // 2]
        print(f'  t {t[seg[0]]:.0f}-{t[seg[-1]]:.0f} s: max cross {cross[k]:+.1f} m (along {along[k]:+.1f}); '
              f'mid: reference {to_map(r[mid, 1:3]):.2f} m from the map, estimate {to_map(pos[mid, :2]):.2f} m')


if __name__ == '__main__':
    main()
