"""Report live #200 results from multibag_live.sh; exit nonzero on a failed check."""
from __future__ import annotations

import json
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'eval'))
from tram_eval.bag import bag_reference, read_bag, stamp, typestore  # noqa: E402
from tram_eval.metrics import match_nearest  # noqa: E402

EXPECTED_RESETS = {'three': 2, 'no_gnss': 1, 'gnss_first': 1,
                   'fresh_a': 0, 'fresh_b': 0, 'fresh_c': 0}


def read_output(path: Path):
    pos, vel = [], []
    with AnyReader([path], default_typestore=typestore()) as reader:
        for conn, rx, raw in reader.messages():
            if conn.topic not in ('/result/position', '/result/velocity'):
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/result/position':
                p = msg.pose.pose.position
                pos.append((stamp(msg), rx * 1e-9, msg.header.frame_id, p.x, p.y))
            else:
                vel.append((stamp(msg), rx * 1e-9))
    return pos, vel


def rate(samples):
    times = sorted(p[1] for p in samples)
    return round((len(times) - 1) / (times[-1] - times[0]), 2) if len(times) > 1 and times[-1] > times[0] else 0.0


@lru_cache(maxsize=3)
def reference(source: str):
    messages = read_bag(Path(source))
    return bag_reference(messages, stamp(messages[0][1]) + 5.0)


def errors(pos, source):
    map_pos = [p for p in pos if p[2] == 'map']
    if not map_pos:
        return None, None
    ref = reference(source)
    est_t = np.array([p[0] for p in map_pos])
    est_xy = np.array([(p[3], p[4]) for p in map_pos])
    i_ref, i_est = match_nearest(ref.pos_t, est_t)
    if not len(i_ref):
        return None, None
    e = np.linalg.norm(ref.pos[i_ref, :2] - est_xy[i_est], axis=1)
    return round(float(np.median(e)), 2), round(float(np.percentile(e, 95)), 2)


def exits(path):
    if not path.is_file():
        return {}
    return dict(line.split('\t') for line in path.read_text().splitlines() if '\t' in line)


def main(run: Path):
    sources = json.loads((run / 'sources.json').read_text())
    plan = [line.split('\t') for line in (run / 'plan.tsv').read_text().splitlines()]
    rows = []
    print('| group/bag | frame map/odom | 2D median/p95 m | Hz position/velocity | reset group | input skipped | checks |')
    print('|---|---:|---:|---:|---:|---:|---|')
    for group, bag, expect in plan:
        stage = run / group / bag
        node_log = (run / group / 'node.log').read_text(errors='replace')
        resets = node_log.count('new bag: a fresh run')
        skipped = node_log.count('input skipped')
        pos, vel = read_output(stage / 'record') if (stage / 'record').is_dir() else ([], [])
        map_count = sum(p[2] == 'map' for p in pos)
        odom_count = sum(p[2] == 'odom' for p in pos)
        med, p95 = errors(pos, sources[bag]) if map_count else (None, None)
        hz_pos, hz_vel = rate(pos), rate(vel)
        codes = exits(stage / 'exit.tsv')
        group_codes = exits(run / group / 'exit.tsv')
        checks = []
        if not pos or not vel:
            checks.append('missing output')
        if hz_pos < 10 or hz_vel < 10:
            checks.append('rate <10 Hz')
        if expect == 'map' and (not map_count or odom_count):
            checks.append('expected map')
        if expect == 'odom' and (not odom_count or map_count):
            checks.append('expected odom')
        if len(pos) != map_count + odom_count:
            checks.append('unknown frame')
        if resets != EXPECTED_RESETS[group]:
            checks.append(f'resets {resets} != {EXPECTED_RESETS[group]}')
        if skipped:
            checks.append(f'input skipped {skipped}')
        for key in ('play_exit', 'check_exit', 'record_ready_exit'):
            if codes.get(key) != '0':
                checks.append(f'{key}={codes.get(key)}')
        if group_codes.get('ready_exit') != '0':
            checks.append('node not ready')
        if (stage / 'errors.txt').exists() or (run / group / 'errors.txt').exists():
            checks.append('process stop timeout')
        row = dict(group=group, bag=bag, expect=expect, n_map=map_count, n_odom=odom_count,
                   n_position=len(pos), n_velocity=len(vel), median_m=med, p95_m=p95,
                   hz_position=hz_pos, hz_velocity=hz_vel, resets=resets, skipped=skipped,
                   exits=codes, group_exits=group_codes, checks=checks)
        rows.append(row)
        print(f'| {group}/{bag} | {map_count}/{odom_count} | {med}/{p95} | '
              f'{hz_pos}/{hz_vel} | {resets} | {skipped} | {"ok" if not checks else "; ".join(checks)} |')
    (run / 'report.json').write_text(json.dumps(rows, indent=2, ensure_ascii=False))
    return 0 if all(not row['checks'] for row in rows) else 1


if __name__ == '__main__':
    sys.exit(main(Path(sys.argv[1])))
