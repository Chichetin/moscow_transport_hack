"""Summary of tools/submission/judge_runner.sh: one row per scenario, exit 0 only if every
scenario met its expectation.

    .venv/bin/python tools/submission/judge_runner_report.py RUN_DIR PLAN_TSV SOURCE_BAGS_JSON

RUN_DIR/bags/<bag> is the scenario input. RUN_DIR/<name>/ contains exit.tsv, times.tsv,
alive.txt, node.log, errors.txt, and record/ (the node's /result/*).
PLAN_TSV rows: name bag mode delay expect, expect is `map` (every
position absolute) or `odom` (no position absolute: a bag without GNSS, or the negative late
start of #201). SOURCE_BAGS_JSON maps a scenario bag to the full train bag it was cut from:
its GNSS master track is the reference of the 2D error (only the map positions have one).
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'eval'))

from tram_eval.bag import INPUTS, bag_reference, read_bag, stamp, typestore  # noqa: E402
from tram_eval.metrics import match_nearest  # noqa: E402

MIN_RATE_HZ = 10.0     # task.md: >= 10 Hz of each output
COVER_TOL_S = 1.0      # Allow one second for bag play and recorder shutdown timing.


def read_result(record: Path):
    """Positions (stamp, frame, x, y) and velocity stamps with receive times."""
    from rosbags.highlevel import AnyReader
    pos, vel = [], []
    with AnyReader([record], default_typestore=typestore()) as reader:
        conns = [c for c in reader.connections if c.topic in ('/result/position', '/result/velocity')]
        for conn, t_rx, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/result/position':
                p = msg.pose.pose.position
                pos.append((stamp(msg), t_rx * 1e-9, msg.header.frame_id, p.x, p.y))
            else:
                vel.append((stamp(msg), t_rx * 1e-9))
    return pos, vel


def read_input_span(bag: Path) -> tuple[float, float]:
    """First and last vehicle/controller header stamps in a scenario bag."""
    stamps = [stamp(msg) for topic, msg in read_bag(bag) if topic in INPUTS]
    return min(stamps), max(stamps)


def node_child_exit(log: Path) -> int | None:
    """Exit code of odometry_node reported by ros2 launch, if present."""
    if not log.is_file():
        return None
    content = log.read_text(encoding='utf-8')
    died = re.findall(r'process has died \[pid \d+, exit code (-?\d+)', content)
    if died:
        return int(died[-1])
    return 0 if 'process has finished cleanly' in content else None


def rate_hz(t_rx) -> float:
    """Messages per second of receive time over the span of the stream."""
    t = sorted(t_rx)
    return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0


def kv(path: Path) -> dict:
    if not path.is_file():
        return {}
    rows = [line.split('\t') for line in path.read_text(encoding='utf-8').splitlines() if line]
    return {r[0]: r[1] for r in rows if len(r) == 2}


def error_2d(pos, source: Path) -> tuple[float, float]:
    """Median and max 2D error of the map positions against the GNSS master track of `source`."""
    m = [p for p in pos if p[2] == 'map']
    if not m:
        return float('nan'), float('nan')
    msgs = read_bag(source)
    ref = bag_reference(msgs, stamp(msgs[0][1]) + 5.0)
    est_t = np.array([p[0] for p in m])
    est_xy = np.array([(p[3], p[4]) for p in m])
    i_ref, i_est = match_nearest(ref.pos_t, est_t)
    if len(i_ref) == 0:
        return float('nan'), float('nan')
    e = np.hypot(*(ref.pos[i_ref, :2] - est_xy[i_est]).T)
    return float(np.median(e)), float(np.max(e))


def scenario(run_dir: Path, row: list[str], sources: dict) -> dict:
    name, bag, mode, delay, expect = row
    d = run_dir / name
    codes, times = kv(d / 'exit.tsv'), kv(d / 'times.tsv')
    errors = (d / 'errors.txt').read_text(encoding='utf-8').strip() if (d / 'errors.txt').is_file() else ''
    alive = (d / 'alive.txt').read_text(encoding='utf-8') if (d / 'alive.txt').is_file() else ''
    out = {'name': name, 'bag': bag, 'mode': mode, 'delay_s': float(delay), 'expect': expect,
           'exit': codes, 'node_child_exit': node_child_exit(d / 'node.log'), 'errors': errors}
    t = {k: float(v) for k, v in times.items()}
    if 'node_launch' in t and 'ready' in t:
        out['launch_to_ready_s'] = round(t['ready'] - t['node_launch'], 2)
    if 'node_launch' in t and 'play' in t:
        out['play_minus_launch_s'] = round(t['play'] - t['node_launch'], 2)
    pos, vel = read_result(d / 'record') if (d / 'record').is_dir() else ([], [])
    frames = [p[2] for p in pos]
    out.update(n_map=frames.count('map'), n_odom=frames.count('odom'),
               n_other=len(frames) - frames.count('map') - frames.count('odom'),
               first_frame=frames[0] if frames else None, n_velocity=len(vel),
               rate_position_hz=round(rate_hz([p[1] for p in pos]), 2),
               rate_velocity_hz=round(rate_hz([v[1] for v in vel]), 2))
    first_in, last_in = read_input_span(run_dir / 'bags' / bag)
    coverage = {'input_first_s': first_in, 'input_last_s': last_in}
    for key, samples in (('position', pos), ('velocity', vel)):
        stamps = [sample[0] for sample in samples]
        coverage[f'{key}_first_delta_s'] = min(stamps) - first_in if stamps else None
        coverage[f'{key}_last_delta_s'] = max(stamps) - last_in if stamps else None
    out['coverage'] = coverage
    src = sources.get(bag)
    if src and out['n_map']:
        out['err2d_median_m'], out['err2d_max_m'] = (round(v, 2) for v in error_2d(pos, Path(src)))
    fails, warnings = [], []
    for key in ('play_exit', 'check_recording_exit', 'ready_exit', 'record_ready_exit'):
        if key in codes and codes[key] != '0':
            fails.append(f'{key}={codes[key]}')
    if 'node exited before stop' in alive:
        fails.append(f'нода умерла до остановки, launch exit {codes.get("node_exit")}, '
                     f'child exit {out["node_child_exit"]}')
    elif 'node running' in alive and out['node_child_exit'] not in (None, 0):
        warnings.append(f'odometry_node exit {out["node_child_exit"]} после SIGINT runner')
    if errors:
        fails.append(errors.replace('\n', '; '))
    if not vel or not pos:
        fails.append('нет /result/velocity или /result/position')
    for key in ('position', 'velocity'):
        end = coverage[f'{key}_last_delta_s']
        start = coverage[f'{key}_first_delta_s']
        if end is not None and end < -COVER_TOL_S:
            fails.append(f'покрытие /result/{key}: конец {end:+g} с от последнего входа')
        if mode == 'protocol' and key == 'velocity' and start is not None and start > COVER_TOL_S:
            fails.append(f'покрытие /result/velocity: начало {start:+g} с от первого входа')
    if min(out['rate_position_hz'], out['rate_velocity_hz']) < MIN_RATE_HZ:
        fails.append(f'частота < {MIN_RATE_HZ:g} Гц')
    if out['n_other']:
        fails.append(f'frame_id не map/odom: {out["n_other"]}')
    if expect == 'map' and out['n_odom']:
        fails.append(f'ожидался только map, odom {out["n_odom"]}')
    if expect == 'odom' and out['n_map']:
        fails.append(f'ожидался только odom (без старого якоря), map {out["n_map"]}')
    out['fails'] = fails
    out['warnings'] = warnings
    out['status'] = 'ok' if not fails else 'fail'
    return out


def main(argv) -> int:
    if len(argv) != 4:
        print(__doc__, file=sys.stderr)
        return 2
    run_dir = Path(argv[1])
    plan = [line.split('\t') for line in Path(argv[2]).read_text(encoding='utf-8').splitlines()
            if line and not line.startswith('#')]
    sources = json.loads(Path(argv[3]).read_text(encoding='utf-8'))
    rows = [scenario(run_dir, row, sources) for row in plan]
    (run_dir / 'report.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding='utf-8')
    print('| сценарий | ожидание | итог | map/odom | 1-й frame | Гц pos/vel | покрытие pos/vel: начало, конец, с | launch→ready, с | err2d med/max, м | exit play/check/launch | child exit |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        e = r['exit']
        c = r['coverage']
        def delta(value):
            return f'{value:+.3f}' if value is not None else '—'

        cover = ' / '.join(f"{delta(c[f'{key}_first_delta_s'])}/{delta(c[f'{key}_last_delta_s'])}"
                           for key in ('position', 'velocity'))
        print(f"| {r['name']} | {r['expect']} | {r['status']} | {r['n_map']}/{r['n_odom']} | "
              f"{r['first_frame']} | {r['rate_position_hz']}/{r['rate_velocity_hz']} | "
              f"{cover} | "
              f"{r.get('launch_to_ready_s', '—')} | {r.get('err2d_median_m', '—')}/{r.get('err2d_max_m', '—')} | "
              f"{e.get('play_exit')}/{e.get('check_recording_exit')}/{e.get('node_exit')} | "
              f"{r['node_child_exit']} |")
        for f in r['fails']:
            print(f'  - {r["name"]}: {f}')
        for warning in r['warnings']:
            print(f'  ! {r["name"]}: {warning}')
    return 0 if all(r['status'] == 'ok' for r in rows) else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
